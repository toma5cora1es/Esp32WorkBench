#include <wb_serial_reg.h>
#include "board_pins.h"
#include "driver/gpio.h"
#include "esp_timer.h"

static volatile uint32_t* const CMD_WORD = (volatile uint32_t*)0x50001800;
static volatile uint32_t* const TELEM    = (volatile uint32_t*)0x50001804;

struct PwmCap {
    uint8_t pin;
    volatile int64_t riseUs;
    volatile int64_t lastEdgeUs;
    volatile uint32_t highUs;
    volatile uint32_t periodUs;
};

static portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;
static PwmCap gCap[4] = {
    {SIM_MOT_LA_IN, 0, 0, 0, 0},
    {SIM_MOT_LB_IN, 0, 0, 0, 0},
    {SIM_MOT_RA_IN, 0, 0, 0, 0},
    {SIM_MOT_RB_IN, 0, 0, 0, 0}
};

static void IRAM_ATTR cap_edge(int idx) {
    PwmCap& c = gCap[idx];
    int lvl = gpio_get_level((gpio_num_t)c.pin);
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&gMux);
    if (lvl == 1) {
        if (c.riseUs > 0 && now > c.riseUs) {
            c.periodUs = (uint32_t)(now - c.riseUs);
        }
        c.riseUs = now;
    } else {
        if (c.riseUs > 0 && now > c.riseUs) {
            c.highUs = (uint32_t)(now - c.riseUs);
        }
    }
    c.lastEdgeUs = now;
    portEXIT_CRITICAL_ISR(&gMux);
}

void IRAM_ATTR isr_la() { cap_edge(0); }
void IRAM_ATTR isr_lb() { cap_edge(1); }
void IRAM_ATTR isr_ra() { cap_edge(2); }
void IRAM_ATTR isr_rb() { cap_edge(3); }

static volatile bool gTrigSeen = false;
static volatile uint16_t gTrigCount = 0;
static esp_timer_handle_t gEchoOffTimer = nullptr;

static void echo_off_cb(void* arg) {
    (void)arg;
    gpio_set_level((gpio_num_t)SIM_US_ECHO_OUT, 0);
}

void IRAM_ATTR isr_us_trig() {
    gTrigSeen = true;
    gTrigCount++;
}

static void read_pwm(int idx, uint8_t& dutyPct, uint16_t& freqHz) {
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&gMux);
    int64_t lastEdge = gCap[idx].lastEdgeUs;
    uint32_t highUs = gCap[idx].highUs;
    uint32_t periodUs = gCap[idx].periodUs;
    portEXIT_CRITICAL(&gMux);

    if (lastEdge == 0 || (now - lastEdge) > 20000) {
        dutyPct = gpio_get_level((gpio_num_t)gCap[idx].pin) ? 100 : 0;
        freqHz = 0;
        return;
    }

    if (periodUs > 0) {
        uint32_t d = (highUs * 100UL) / periodUs;
        dutyPct = (uint8_t)min((uint32_t)100, d);
        freqHz = (uint16_t)(1000000UL / periodUs);
    } else {
        dutyPct = 0;
        freqHz = 0;
    }
}

static int64_t gLastEncLUs = 0, gLastEncRUs = 0;
static uint8_t gEncStateL = 0, gEncStateR = 0;
static int16_t gEncOutL = 0, gEncOutR = 0;
static uint32_t gLastTelemMs = 0;

void setup() {
    *CMD_WORD = 0x0000015E;
    for (int i = 0; i < 4; i++) TELEM[i] = 0;

    pinMode(SIM_LINE_L_OUT, OUTPUT);
    pinMode(SIM_LINE_M_OUT, OUTPUT);
    pinMode(SIM_LINE_R_OUT, OUTPUT);
    pinMode(SIM_BTN_T1_OUT, OUTPUT);
    pinMode(SIM_BTN_T2_OUT, OUTPUT);
    pinMode(SIM_ENC_L_OUT, OUTPUT);
    pinMode(SIM_ENC_R_OUT, OUTPUT);
    pinMode(SIM_US_ECHO_OUT, OUTPUT);
    digitalWrite(SIM_US_ECHO_OUT, LOW);

    pinMode(SIM_US_TRIG_IN, INPUT_PULLDOWN);
    pinMode(SIM_MOT_LA_IN, INPUT_PULLDOWN);
    pinMode(SIM_MOT_LB_IN, INPUT_PULLDOWN);
    pinMode(SIM_MOT_RA_IN, INPUT_PULLDOWN);
    pinMode(SIM_MOT_RB_IN, INPUT_PULLDOWN);
    pinMode(SIM_LED_FL_IN, INPUT_PULLDOWN);
    pinMode(SIM_LED_FR_IN, INPUT_PULLDOWN);
    pinMode(SIM_LED_RR_IN, INPUT_PULLDOWN);
    pinMode(SIM_LED_RL_IN, INPUT_PULLDOWN);

    esp_timer_create_args_t tArgs = {};
    tArgs.callback = &echo_off_cb;
    tArgs.name = "us_echo_off";
    esp_timer_create(&tArgs, &gEchoOffTimer);

    attachInterrupt(SIM_MOT_LA_IN, isr_la, CHANGE);
    attachInterrupt(SIM_MOT_LB_IN, isr_lb, CHANGE);
    attachInterrupt(SIM_MOT_RA_IN, isr_ra, CHANGE);
    attachInterrupt(SIM_MOT_RB_IN, isr_rb, CHANGE);
    attachInterrupt(SIM_US_TRIG_IN, isr_us_trig, FALLING);

    wb_serial_reg_begin("AGV_SIM", 115200);
}

void loop() {
    wb_serial_reg_poll();

    uint32_t cmd = *CMD_WORD;
    uint8_t btns     = (cmd >> 20) & 0x0F;
    uint8_t lineMask = (cmd >> 16) & 0x0F;
    uint16_t distMm  = cmd & 0xFFFF;
    if (distMm < 40) distMm = 40;
    if (distMm > 3000) distMm = 3000;

    digitalWrite(SIM_LINE_L_OUT, (lineMask & 0x04) ? HIGH : LOW);
    digitalWrite(SIM_LINE_M_OUT, (lineMask & 0x02) ? HIGH : LOW);
    digitalWrite(SIM_LINE_R_OUT, (lineMask & 0x01) ? HIGH : LOW);
    digitalWrite(SIM_BTN_T1_OUT, (btns & 0x01) ? HIGH : LOW);
    digitalWrite(SIM_BTN_T2_OUT, (btns & 0x02) ? HIGH : LOW);

    if (gTrigSeen) {
        gTrigSeen = false;
        uint64_t echoUs = (uint64_t)((distMm * 2000UL) / 343UL);
        esp_timer_stop(gEchoOffTimer);
        gpio_set_level((gpio_num_t)SIM_US_ECHO_OUT, 1);
        esp_timer_start_once(gEchoOffTimer, echoUs);
    }

    uint32_t nowMs = millis();
    static int16_t pwmL = 0, pwmR = 0;

    if (nowMs - gLastTelemMs >= 20) {
        gLastTelemMs = nowMs;

        uint8_t dLA, dLB, dRA, dRB;
        uint16_t fLA, fLB, fRA, fRB;
        read_pwm(0, dLA, fLA);
        read_pwm(1, dLB, fLB);
        read_pwm(2, dRA, fRA);
        read_pwm(3, dRB, fRB);

        pwmL = (int16_t)dLA - (int16_t)dLB;
        pwmR = (int16_t)dRA - (int16_t)dRB;
        uint16_t freqL = (dLA >= dLB) ? fLA : fLB;
        uint16_t freqR = (dRA >= dRB) ? fRA : fRB;

        uint8_t ledMask =
            ((uint8_t)gpio_get_level((gpio_num_t)SIM_LED_FL_IN) << 0) |
            ((uint8_t)gpio_get_level((gpio_num_t)SIM_LED_FR_IN) << 1) |
            ((uint8_t)gpio_get_level((gpio_num_t)SIM_LED_RR_IN) << 2) |
            ((uint8_t)gpio_get_level((gpio_num_t)SIM_LED_RL_IN) << 3);

        TELEM[0] = ((uint32_t)(uint16_t)pwmL << 16) | (uint16_t)pwmR;
        TELEM[1] = ((uint32_t)freqL << 16) | (uint16_t)freqR;
        TELEM[2] = ((uint32_t)(ledMask & 0x0F) << 16) | (uint16_t)gTrigCount;
        TELEM[3] = ((uint32_t)(uint16_t)gEncOutL << 16) | (uint16_t)gEncOutR;
    }

    int64_t nowUs = esp_timer_get_time();
    if (abs(pwmL) >= 10) {
        int64_t halfUs = 500000LL / max(10, (abs(pwmL) * 120) / 100);
        if (nowUs - gLastEncLUs >= halfUs) {
            gLastEncLUs = nowUs;
            gEncStateL = !gEncStateL;
            digitalWrite(SIM_ENC_L_OUT, gEncStateL);
            gEncOutL += (pwmL > 0) ? 1 : -1;
        }
    }
    if (abs(pwmR) >= 10) {
        int64_t halfUs = 500000LL / max(10, (abs(pwmR) * 120) / 100);
        if (nowUs - gLastEncRUs >= halfUs) {
            gLastEncRUs = nowUs;
            gEncStateR = !gEncStateR;
            digitalWrite(SIM_ENC_R_OUT, gEncStateR);
            gEncOutR += (pwmR > 0) ? 1 : -1;
        }
    }
}