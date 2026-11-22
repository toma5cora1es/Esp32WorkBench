#include <math.h>
#include <stdio.h>
#include <wb_serial_reg.h>
#include <wb_espnow_link.h>
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
static volatile uint32_t gLastCmdMs = 0;
static volatile bool gReplyPending = false;
static volatile int64_t gPendingEchoUs = 0;
static volatile uint32_t gPendingSeq = 0;

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

static void on_master_cmd(const WbEspNowPkt& pkt, const uint8_t* srcMac) {
    (void)srcMac;
    *CMD_WORD = pkt.word0;
    gLastCmdMs = millis();
    gPendingEchoUs = pkt.echoTxUs;
    gPendingSeq = pkt.seq;
    gReplyPending = true;
}

static uint16_t gUsHist[3] = {3000, 3000, 3000};
static uint8_t  gUsIdx = 0;

static uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    return b;
}

static int apply_calibration(int8_t reqPct, int minPwm, int trimPct) {
    if (reqPct == 0) return 0;
    int sign = (reqPct > 0) ? 1 : -1;
    int mag = constrain(abs((int)reqPct), 1, 100);
    int span = max(0, AGV_MAX_SAFE_PWM - minPwm);
    int pwm = minPwm + (mag * span) / 100;
    pwm = (pwm * trimPct) / 100;
    return sign * constrain(pwm, 0, AGV_MAX_SAFE_PWM);
}

static int slew_step(int cur, int target) {
    if (target > cur) return min(target, cur + AGV_SLEW_PCT_TICK);
    if (target < cur) return max(target, cur - AGV_SLEW_PCT_TICK);
    return target;
}

static void write_motor_pwm(int pinA, int pinB, int pct, volatile int8_t& dirFlag) {
    uint32_t maxDuty = (1UL << AGV_PWM_RES_BITS) - 1;
    uint32_t duty = (uint32_t)((abs(pct) * (int32_t)maxDuty) / 100);
    if (pct > 0) {
        dirFlag = 1;
        ledcWrite(pinA, duty);
        ledcWrite(pinB, 0);
    } else if (pct < 0) {
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
static float gDistWin[5] = {0};
static uint8_t gWinIdx = 0;
static int16_t gCurPwmL = 0, gCurPwmR = 0, gVelMms = 0;
static uint32_t gLastCtrlMs = 0, gLastTrigMs = 0, gLastUsbCmd = 0x10000000;
static char gNodeTag[48];

void setup() {
    *CMD_WORD = 0x10000000;
    gLastUsbCmd = 0x10000000;
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

    const uint8_t peerMac[6] = AGV_PEER_MAC;
    wb_espnow_begin(AGV_ESPNOW_CH, peerMac, on_master_cmd);

    snprintf(gNodeTag, sizeof(gNodeTag), "AGV_SLAVE_%s", WiFi.macAddress().c_str());
    wb_serial_reg_begin(gNodeTag, AGV_SERIAL_BAUD);
}

void loop() {
    wb_serial_reg_poll();

    uint32_t nowMs = millis();
    uint32_t cmd = *CMD_WORD;
    if (cmd != gLastUsbCmd) {
        gLastUsbCmd = cmd;
        gLastCmdMs = nowMs;
    }

    if (nowMs - gLastTrigMs >= AGV_US_POLL_MS) {
        gLastTrigMs = nowMs;
        digitalWrite(AGV_PIN_US_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(AGV_PIN_US_TRIG, LOW);

        uint16_t rawMm = (uint16_t)((gEchoDurUs * 343UL) / 2000UL);
        if (rawMm < 20 || rawMm > 3000) rawMm = 3000;
        gUsHist[gUsIdx] = rawMm;
        gUsIdx = (gUsIdx + 1) % 3;
    }

    if (nowMs - gLastCtrlMs >= 20) {
        gLastCtrlMs = nowMs;

        uint8_t magic = (cmd >> 28) & 0x0F;
        uint8_t mode  = (cmd >> 24) & 0x0F;
        uint8_t aux   = (cmd >> 16) & 0xFF;
        int8_t  reqL  = (int8_t)((cmd >> 8) & 0xFF);
        int8_t  reqR  = (int8_t)(cmd & 0xFF);

        if (magic == 0x0F) {
            portENTER_CRITICAL(&gMux);
            gEncL = 0; gEncR = 0;
            portEXIT_CRITICAL(&gMux);
            gPrevEncL = 0; gPrevEncR = 0;
            gPosX = 0.0f; gPosY = 0.0f; gTheta = 0.0f;
            for (int i = 0; i < 5; i++) gDistWin[i] = 0.0f;
            *CMD_WORD = (cmd & 0x0FFFFFFFUL) | 0x10000000UL;
            gLastUsbCmd = *CMD_WORD;
        }

        uint8_t lL = digitalRead(AGV_PIN_LINE_L);
        uint8_t lM = digitalRead(AGV_PIN_LINE_M);
        uint8_t lR = digitalRead(AGV_PIN_LINE_R);
        uint8_t t1 = digitalRead(AGV_PIN_BTN_T1);
        uint8_t t2 = digitalRead(AGV_PIN_BTN_T2);

        uint16_t usMm = median3(gUsHist[0], gUsHist[1], gUsHist[2]);
        bool linkExpired = (gLastCmdMs == 0) || ((nowMs - gLastCmdMs) > AGV_WATCHDOG_MS);
        bool obstacleFront = (usMm > 20 && usMm < AGV_AUTO_BRAKE_MM);

        digitalWrite(AGV_PIN_LED_FL, (aux & 0x01) ? HIGH : LOW);
        digitalWrite(AGV_PIN_LED_FR, (aux & 0x01) ? HIGH : LOW);
        digitalWrite(AGV_PIN_LED_RR, (aux & 0x02) ? HIGH : LOW);
        digitalWrite(AGV_PIN_LED_RL, (aux & 0x02) ? HIGH : LOW);

        int tgtPwmL = 0, tgtPwmR = 0;
        uint8_t activeFsm = mode;

        if (linkExpired || t1 == HIGH || mode == 0) {
            activeFsm = 0;
            tgtPwmL = 0;
            tgtPwmR = 0;
        } else if (mode == 1) {
            if (obstacleFront && (reqL + reqR) > 0) {
                activeFsm = 4;
                tgtPwmL = 0;
                tgtPwmR = 0;
            } else {
                tgtPwmL = apply_calibration(reqL, AGV_MIN_PWM_L, AGV_TRIM_L_PCT);
                tgtPwmR = apply_calibration(reqR, AGV_MIN_PWM_R, AGV_TRIM_R_PCT);
            }
        }

        if (activeFsm == 0) {
            gCurPwmL = 0;
            gCurPwmR = 0;
        } else {
            gCurPwmL = (int16_t)slew_step(gCurPwmL, tgtPwmL);
            gCurPwmR = (int16_t)slew_step(gCurPwmR, tgtPwmR);
        }

        write_motor_pwm(AGV_PIN_MOT_L_A, AGV_PIN_MOT_L_B, gCurPwmL, gDirL);
        write_motor_pwm(AGV_PIN_MOT_R_A, AGV_PIN_MOT_R_B, gCurPwmR, gDirR);

        int32_t eL, eR;
        portENTER_CRITICAL(&gMux);
        eL = gEncL; eR = gEncR;
        portEXIT_CRITICAL(&gMux);

        float mPerEdge = (3.1415926f * (AGV_WHEEL_DIAM_MM / 1000.0f)) / (float)AGV_EDGES_PER_REV;
        float dL = (float)(eL - gPrevEncL) * mPerEdge;
        float dR = (float)(eR - gPrevEncR) * mPerEdge;
        gPrevEncL = eL;
        gPrevEncR = eR;

        float dC = 0.5f * (dL + dR);
        float dTh = (dR - dL) / (AGV_WHEEL_BASE_MM / 1000.0f);
        gTheta = atan2f(sinf(gTheta + dTh), cosf(gTheta + dTh));
        gPosX += dC * cosf(gTheta);
        gPosY += dC * sinf(gTheta);

        gDistWin[gWinIdx] = dC;
        gWinIdx = (gWinIdx + 1) % 5;
        float winDist = gDistWin[0] + gDistWin[1] + gDistWin[2] + gDistWin[3] + gDistWin[4];
        gVelMms = (int16_t)((winDist / 0.100f) * 1000.0f);

        uint8_t mask = (lL << 2) | (lM << 1) | lR | (t1 << 3) | (t2 << 4);
        int16_t xMm = (int16_t)(gPosX * 1000.0f);
        int16_t yMm = (int16_t)(gPosY * 1000.0f);
        int16_t thMrad = (int16_t)(gTheta * 1000.0f);

        TELEM[0] = ((uint32_t)(uint16_t)(int16_t)eL << 16) | (uint16_t)(int16_t)eR;
        TELEM[1] = ((uint32_t)usMm << 16) | ((uint32_t)mask << 8) | activeFsm;
        TELEM[2] = ((uint32_t)(uint16_t)xMm << 16) | (uint16_t)yMm;
        TELEM[3] = ((uint32_t)(uint16_t)thMrad << 16) | (uint16_t)gVelMms;
    }

    if (gReplyPending) {
        gReplyPending = false;
        WbEspNowPkt rep = {};
        rep.seq = gPendingSeq;
        rep.word0 = *CMD_WORD;
        for (int i = 0; i < 4; i++) rep.telem[i] = TELEM[i];
        rep.echoTxUs = gPendingEchoUs;
        wb_espnow_send(rep);
    }
}