#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "esp_log.h"

static char _wb_rx_buf[128];
static size_t _wb_rx_len = 0;
static const char* _wb_node_tag = "WB_SERIAL_REG";

inline void wb_serial_reg_begin(const char* nodeTag = "WB_SERIAL_REG", uint32_t baud = 115200) {
    _wb_node_tag = nodeTag;
    esp_log_level_set("*", ESP_LOG_NONE);
    if (baud > 0) {
        Serial.begin(baud);
    }
    while (Serial.available()) {
        Serial.read();
    }
    Serial.printf("ESP32_READY %s\n", _wb_node_tag);
}

inline void _wb_exec_cmd(char* line) {
    while (*line == ' ' || *line == '\t') {
        line++;
    }
    if (*line == '\0') {
        return;
    }

    if (strcmp(line, "STATUS") == 0) {
        Serial.printf("ESP32_READY %s\n", _wb_node_tag);
        return;
    }

    if ((line[0] == 'R' || line[0] == 'r') && line[1] == ' ') {
        uint32_t addr = (uint32_t)strtoul(line + 2, nullptr, 16);
        uint32_t val = *(volatile uint32_t*)addr;
        Serial.printf("%08X\n", (unsigned int)val);
        return;
    }

    if ((line[0] == 'W' || line[0] == 'w') && line[1] == ' ') {
        char* endPtr = nullptr;
        uint32_t addr = (uint32_t)strtoul(line + 2, &endPtr, 16);
        uint32_t val = (uint32_t)strtoul(endPtr, nullptr, 16);
        *(volatile uint32_t*)addr = val;
        uint32_t rb = *(volatile uint32_t*)addr;
        Serial.printf("%08X\n", (unsigned int)rb);
        return;
    }

    if ((line[0] == 'D' || line[0] == 'd') && line[1] == ' ') {
        char* endPtr = nullptr;
        uint32_t addr = (uint32_t)strtoul(line + 2, &endPtr, 16);
        int count = (int)strtol(endPtr, nullptr, 10);
        if (count < 1) count = 1;
        if (count > 64) count = 64;
        for (int i = 0; i < count; i++) {
            uint32_t val = *(volatile uint32_t*)(addr + (uint32_t)(i * 4));
            if (i > 0) Serial.print(' ');
            Serial.printf("%08X", (unsigned int)val);
        }
        Serial.print('\n');
        return;
    }

    Serial.printf("ERR_UNKNOWN_CMD %s\n", line);
}

inline void wb_serial_reg_poll() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            _wb_rx_buf[_wb_rx_len] = '\0';
            _wb_exec_cmd(_wb_rx_buf);
            _wb_rx_len = 0;
        } else if (_wb_rx_len < sizeof(_wb_rx_buf) - 1) {
            _wb_rx_buf[_wb_rx_len++] = c;
        }
    }
}