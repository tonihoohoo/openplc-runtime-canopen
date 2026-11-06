// SPDX-License-Identifier: Apache-2.0
// Spike: TWAI in QEMU. Boot-up 0x705, heartbeat 0x705 every 100 ms, RPDO 0x205 echoed on 0x185.
#include "driver/twai.h"

static uint32_t last_hb = 0;

static void send_frame(uint32_t id, const uint8_t *data, uint8_t len) {
    twai_message_t m = {};
    m.identifier = id;
    m.data_length_code = len;
    for (uint8_t i = 0; i < len; i++) m.data[i] = data[i];
    esp_err_t e = twai_transmit(&m, pdMS_TO_TICKS(10));
    if (e != ESP_OK) Serial.printf("tx %03lx err %d\n", (unsigned long)id, e);
}

void setup() {
    Serial.begin(115200);
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4, TWAI_MODE_NORMAL);
    g.rx_queue_len = 32;
#ifdef SPIKE_IRAM
    g.intr_flags = ESP_INTR_FLAG_IRAM;
#endif
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    Serial.printf("install %d\n", twai_driver_install(&g, &t, &f));
    Serial.printf("start %d\n", twai_start());
    const uint8_t boot = 0x00;
    send_frame(0x705, &boot, 1);
    Serial.println("spike ready");
}

void loop() {
    twai_message_t m;
    while (twai_receive(&m, 0) == ESP_OK) {
        if (m.identifier == 0x205) send_frame(0x185, m.data, m.data_length_code);
    }
    uint32_t now = millis();
    if (now - last_hb >= 100) {
        last_hb = now;
        const uint8_t state = 0x7F;
        send_frame(0x705, &state, 1);
    }
    delay(1);
}
