#include <stdio.h>
#include <wb_serial_reg.h>
#include <wb_espnow_link.h>
#include "board_pins.h"
#include "esp_timer.h"

static volatile uint32_t* const REG_CMD        = (volatile uint32_t*)0x50001800;
static volatile uint32_t* const REG_TELEM      = (volatile uint32_t*)0x50001804;
static volatile uint32_t* const REG_LINK_RTT   = (volatile uint32_t*)0x50001814;
static volatile uint32_t* const REG_LINK_AGE   = (volatile uint32_t*)0x50001818;
static volatile uint32_t* const REG_LINK_STATS = (volatile uint32_t*)0x5000181C;

static portMUX_TYPE gBridgeMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t gLastRxMs = 0;
static uint32_t gTxSeq = 0;
static uint32_t gLastCmdSent = 0;
static uint32_t gLastSendMs = 0;
static char gNodeTag[48];

static void on_slave_reply(const WbEspNowPkt& pkt, const uint8_t* srcMac) {
    (void)srcMac;
    int64_t nowUs = esp_timer_get_time();
    portENTER_CRITICAL(&gBridgeMux);
    for (int i = 0; i < 4; i++) {
        REG_TELEM[i] = pkt.telem[i];
    }
    if (pkt.echoTxUs > 0 && nowUs >= pkt.echoTxUs) {
        *REG_LINK_RTT = (uint32_t)(nowUs - pkt.echoTxUs);
    }
    gLastRxMs = millis();
    portEXIT_CRITICAL(&gBridgeMux);
}

void setup() {
    *REG_CMD = 0x10000000;
    for (int i = 0; i < 4; i++) REG_TELEM[i] = 0;
    *REG_LINK_RTT = 0;
    *REG_LINK_AGE = 9999;
    *REG_LINK_STATS = 0;

    const uint8_t peerMac[6] = AGV_PEER_MAC;
    wb_espnow_begin(AGV_ESPNOW_CH, peerMac, on_slave_reply);

    snprintf(gNodeTag, sizeof(gNodeTag), "AGV_MASTER_%s", WiFi.macAddress().c_str());
    wb_serial_reg_begin(gNodeTag, AGV_SERIAL_BAUD);
}

void loop() {
    wb_serial_reg_poll();

    uint32_t nowMs = millis();
    uint32_t cmd = *REG_CMD;

    if (cmd != gLastCmdSent || (nowMs - gLastSendMs >= 30)) {
        gLastCmdSent = cmd;
        gLastSendMs = nowMs;

        WbEspNowPkt pkt = {};
        pkt.seq = ++gTxSeq;
        pkt.word0 = cmd;
        pkt.echoTxUs = esp_timer_get_time();
        wb_espnow_send(pkt);
    }

    portENTER_CRITICAL(&gBridgeMux);
    uint32_t lastRx = gLastRxMs;
    portEXIT_CRITICAL(&gBridgeMux);

    *REG_LINK_AGE = (lastRx == 0) ? 9999 : (nowMs - lastRx);
    *REG_LINK_STATS = ((_wb_tx_ok & 0xFFFFUL) << 16) | (_wb_tx_fail & 0xFFFFUL);
}