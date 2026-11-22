#include <math.h>
#include <wb_serial_reg.h>
#include "board_pins.h"
#include "driver/gpio.h"
#include "esp_timer.h"

static volatile uint32_t* const CMD_WORD = (volatile uint32_t*)0x50001800;
static volatile uint32_t* const TELEM    = (volatile uint32_t*)0x50001804;

static portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;
static volatile int32_t gEncL = 0, gEncR = 0;
static volatile int8_t  gDirL = 1, gDirR = 1;
static volatile int64_t gEchoRiseUs = 0;
static volatile uint32_t gEchoDurUs = 0;

void IRAM_ATTR isr_enc_l() {
    portENTER_CRITICAL_ISR(&gMux);
    gEncL += gDirL;
    portEXIT_CRITICAL_ISR(&gMux);
}

void IRAM_ATTR isr_enc_r() {
    portENTER_CRITICAL_ISR(&gMux);
    gEncR += gDirR;
    portEXIT_CRITICAL_ISR(&gMux);
}

void IRAM_ATTR isr_echo() {
    int lvl = gpio_get_level((gpio_num_t)AGV_PIN_US_ECHO);
    int64_t now = esp_timer_get_time();
    if (lvl == 1) {
        gEchoRiseUs = now;
    } else if (gEchoRiseUs > 0) {
        gEchoDurUs = (uint32_t)(now - gEchoRiseUs);
        gEchoRiseUs = 0;
    }
}

static void drive_motor(int pinA, int pinB, int8_t pct, volatile int8_t& dirFlag) {
    int clamped = constrain((int)pct, -100, 100);
    uint32_t maxDuty = (1UL << AGV_PWM_RES_BITS) - 1;
    uint32_t duty = (uint32_t)((abs(clamped) * (int32_t)maxDuty) / 100);

    if (clamped > 0) {
        dirFlag = 1;
        ledcWrite(pinA, duty);
        ledcWrite(pinB, 0);
    } else if (clamped < 0) {
        dirFlag = -1;
        ledcWrite(pinA, 0);
        ledcWrite(pinB, duty);
    } else {
        ledcWrite(pinA, 0);
        ledcWrite(pinB, 0);
    }
}

static float gPosX = 0.0f, gPosY = 0.0f, gTheta = 0.0f;
static int32_t gPrevEncL = 0, gPrevEncR = 0;
static int16_t gVelMms = 0;
static uint32_t gLastCtrlMs = 0, gLastTrigMs = 0;

void setup() {
    *CMD_WORD = 0x10000000;
    for (int i = 0; i < 4; i++) TELEM[i] = 0;

    pinMode(AGV_PIN_LED_FL, OUTPUT);
    pinMode(AGV_PIN_LED_FR, OUTPUT);
    pinMode(AGV_PIN_LED_RR, OUTPUT);
    pinMode(AGV_PIN_LED_RL, OUTPUT);

    pinMode(AGV_PIN_US_TRIG, OUTPUT);
    digitalWrite(AGV_PIN_US_TRIG, LOW);
    pinMode(AGV_PIN_US_ECHO, INPUT_PULLDOWN);

    pinMode(AGV_PIN_LINE_L, INPUT);
    pinMode(AGV_PIN_LINE_M, INPUT);
    pinMode(AGV_PIN_LINE_R, INPUT);
    pinMode(AGV_PIN_BTN_T1, INPUT_PULLDOWN);
    pinMode(AGV_PIN_BTN_T2, INPUT_PULLDOWN);
    pinMode(AGV_PIN_ENC_L, INPUT_PULLDOWN);
    pinMode(AGV_PIN_ENC_R, INPUT_PULLDOWN);

    ledcAttach(AGV_PIN_MOT_L_A, AGV_PWM_FREQ_HZ, AGV_PWM_RES_BITS);
    ledcAttach(AGV_PIN_MOT_L_B, AGV_PWM_FREQ_HZ, AGV_PWM_RES_BITS);
    ledcAttach(AGV_PIN_MOT_R_A, AGV_PWM_FREQ_HZ, AGV_PWM_RES_BITS);
    ledcAttach(AGV_PIN_MOT_R_B, AGV_PWM_FREQ_HZ, AGV_PWM_RES_BITS);

    attachInterrupt(AGV_PIN_ENC_L, isr_enc_l, CHANGE);
    attachInterrupt(AGV_PIN_ENC_R, isr_enc_r, CHANGE);
    attachInterrupt(AGV_PIN_US_ECHO, isr_echo, CHANGE);

    wb_serial_reg_begin("AGV_DUT", 115200);
}

void loop() {
    wb_serial_reg_poll();

    uint32_t nowMs = millis();

    if (nowMs - gLastTrigMs >= 60) {
        gLastTrigMs = nowMs;
        digitalWrite(AGV_PIN_US_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(AGV_PIN_US_TRIG, LOW);
    }

    if (nowMs - gLastCtrlMs >= 20) {
        float dt = (nowMs - gLastCtrlMs) / 1000.0f;
        gLastCtrlMs = nowMs;

        uint32_t cmd = *CMD_WORD;
        uint8_t magic = (cmd >> 28) & 0x0F;
        uint8_t mode  = (cmd >> 24) & 0x0F;
        uint8_t aux   = (cmd >> 16) & 0xFF;
        int8_t  cmdL  = (int8_t)((cmd >> 8) & 0xFF);
        int8_t  cmdR  = (int8_t)(cmd & 0xFF);

        if (magic == 0x0F) {
            portENTER_CRITICAL(&gMux);
            gEncL = 0; gEncR = 0;
            portEXIT_CRITICAL(&gMux);
            gPrevEncL = 0; gPrevEncR = 0;
            gPosX = 0.0f; gPosY = 0.0f; gTheta = 0.0f;
            *CMD_WORD = (cmd & 0x0FFFFFFFUL) | 0x10000000UL;
        }

        uint8_t lL = digitalRead(AGV_PIN_LINE_L);
        uint8_t lM = digitalRead(AGV_PIN_LINE_M);
        uint8_t lR = digitalRead(AGV_PIN_LINE_R);
        uint8_t t1 = digitalRead(AGV_PIN_BTN_T1);
        uint8_t t2 = digitalRead(AGV_PIN_BTN_T2);

        uint16_t usMm = (uint16_t)((gEchoDurUs * 343UL) / 2000UL);
        if (usMm == 0 || usMm > 3000) usMm = 3000;

        digitalWrite(AGV_PIN_LED_FL, (aux & 0x01) ? HIGH : LOW);
        digitalWrite(AGV_PIN_LED_FR, (aux & 0x01) ? HIGH : LOW);
        digitalWrite(AGV_PIN_LED_RR, (aux & 0x02) ? HIGH : LOW);
        digitalWrite(AGV_PIN_LED_RL, (aux & 0x02) ? HIGH : LOW);

        int8_t outL = 0, outR = 0;
        if (mode == 1) {
            outL = cmdL;
            outR = cmdR;
        } else if (mode == 2) {
            if (usMm > 20 && usMm < 150) {
                outL = 0; outR = 0;
            } else if (lM && !lL && !lR) {
                outL = 55; outR = 55;
            } else if (lL && !lR) {
                outL = 20; outR = 60;
            } else if (lR && !lL) {
                outL = 60; outR = 20;
            } else if (lL && lM && lR) {
                outL = 50; outR = 50;
            }
        }

        drive_motor(AGV_PIN_MOT_L_A, AGV_PIN_MOT_L_B, outL, gDirL);
        drive_motor(AGV_PIN_MOT_R_A, AGV_PIN_MOT_R_B, outR, gDirR);

        int32_t eL, eR;
        portENTER_CRITICAL(&gMux);
        eL = gEncL; eR = gEncR;
        portEXIT_CRITICAL(&gMux);

        float mPerTick = (3.1415926f * (AGV_WHEEL_DIAM_MM / 1000.0f)) / (float)AGV_TICKS_PER_REV;
        float dL = (float)(eL - gPrevEncL) * mPerTick;
        float dR = (float)(eR - gPrevEncR) * mPerTick;
        gPrevEncL = eL;
        gPrevEncR = eR;

        float dC = 0.5f * (dL + dR);
        float dTh = (dR - dL) / (AGV_WHEEL_BASE_MM / 1000.0f);
        gTheta = atan2f(sinf(gTheta + dTh), cosf(gTheta + dTh));
        gPosX += dC * cosf(gTheta);
        gPosY += dC * sinf(gTheta);
        gVelMms = (dt > 0.001f) ? (int16_t)((dC / dt) * 1000.0f) : 0;

        uint8_t mask = (lL << 2) | (lM << 1) | lR | (t1 << 3) | (t2 << 4);
        int16_t xMm = (int16_t)(gPosX * 1000.0f);
        int16_t yMm = (int16_t)(gPosY * 1000.0f);
        int16_t thMrad = (int16_t)(gTheta * 1000.0f);

        TELEM[0] = ((uint32_t)(uint16_t)(int16_t)eL << 16) | (uint16_t)(int16_t)eR;
        TELEM[1] = ((uint32_t)usMm << 16) | ((uint32_t)mask << 8) | mode;
        TELEM[2] = ((uint32_t)(uint16_t)xMm << 16) | (uint16_t)yMm;
        TELEM[3] = ((uint32_t)(uint16_t)thMrad << 16) | (uint16_t)gVelMms;
    }
}