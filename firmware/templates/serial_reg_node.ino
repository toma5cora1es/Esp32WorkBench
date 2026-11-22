#include <wb_serial_reg.h>

void setup() {
    wb_serial_reg_begin("SERIAL_REG_NODE", 115200);
}

void loop() {
    wb_serial_reg_poll();
}