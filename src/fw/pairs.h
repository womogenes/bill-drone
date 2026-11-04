// Which drone talks to which controller (fw/drone/main/radio.c, fw/controller/native/main/relay.c): each board finds its own station MAC here and answers only its partner, so one image serves every drone and every controller. The drone's MAC is also its ID in the recordings. A new drone or controller is a new line; a board that is not listed stops at boot and prints its MAC.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_mac.h"

typedef struct {
    uint8_t drone[6];
    uint8_t controller[6];
} pair_t;

static const pair_t pairs[] = {
    {{0x20, 0x50, 0x0d, 0x8b, 0x7e, 0x68}, {0x04, 0xb2, 0x47, 0x54, 0x80, 0x4c}},
};

// This board's station MAC into own and its partner's into partner; is_drone says which side of the table this board is on.
static inline void pair_find(bool is_drone, uint8_t own[6], uint8_t partner[6]) {
    esp_read_mac(own, ESP_MAC_WIFI_STA);
    for (size_t i = 0; i < sizeof(pairs) / sizeof(pairs[0]); i++) {
        if (!memcmp(is_drone ? pairs[i].drone : pairs[i].controller, own, 6)) {
            memcpy(partner, is_drone ? pairs[i].controller : pairs[i].drone, 6);
            return;
        }
    }
    printf("This %s (%02x:%02x:%02x:%02x:%02x:%02x) is not in fw/pairs.h: add it with its partner and flash again\n", is_drone ? "drone" : "controller", own[0], own[1], own[2], own[3], own[4], own[5]);
    abort();
}
