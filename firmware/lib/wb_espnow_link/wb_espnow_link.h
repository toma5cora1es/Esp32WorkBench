#pragma once
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

struct __attribute__((packed)) WbEspNowPkt {
    uint32_t seq;
    uint32_t word0;
    uint32_t telem[4];
    int64_t  echoTxUs;
};

typedef void (*WbEspNowRxFn)(const WbEspNowPkt& pkt, const uint8_t* srcMac);

static WbEspNowRxFn _wb_user_rx = nullptr;
static volatile uint32_t _wb_tx_ok = 0;
static volatile uint32_t _wb_tx_fail = 0;
static uint8_t _wb_peer_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

inline void _wb_on_sent(const wifi_tx_info_t* tx_info, esp_now_send_status_t status) {
    (void)tx_info;
    if (status == ESP_NOW_SEND_SUCCESS) {
        _wb_tx_ok++;
    } else {
        _wb_tx_fail++;
    }
}

inline void _wb_on_recv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (len != (int)sizeof(WbEspNowPkt) || _wb_user_rx == nullptr) {
        return;
    }
    WbEspNowPkt pkt;
    memcpy(&pkt, data, sizeof(pkt));
    _wb_user_rx(pkt, info->src_addr);
}

inline bool wb_espnow_begin(uint8_t channel, const uint8_t peerMac[6], WbEspNowRxFn rxCb) {
    memcpy(_wb_peer_mac, peerMac, 6);
    _wb_user_rx = rxCb;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        return false;
    }

    esp_now_register_send_cb(_wb_on_sent);
    esp_now_register_recv_cb(_wb_on_recv);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, _wb_peer_mac, 6);
    peer.channel = channel;
    peer.encrypt = false;
    peer.ifidx = WIFI_IF_STA;
    return esp_now_add_peer(&peer) == ESP_OK;
}

inline bool wb_espnow_send(const WbEspNowPkt& pkt) {
    return esp_now_send(_wb_peer_mac, (const uint8_t*)&pkt, sizeof(pkt)) == ESP_OK;
}