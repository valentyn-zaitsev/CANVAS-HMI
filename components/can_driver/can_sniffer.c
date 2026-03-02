#include "can_sniffer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static sniffer_state_t state;

void can_sniffer_init(void) {
    memset(&state, 0, sizeof(state));
}

void can_sniffer_record(uint32_t id, const uint8_t *data, uint8_t dlc) {
    state.total_msgs++;

    // Find existing entry
    for (int i = 0; i < state.num_ids; i++) {
        if (state.entries[i].id == id) {
            state.entries[i].count++;
            state.entries[i].last_dlc = dlc;
            state.entries[i].last_tick = xTaskGetTickCount();
            memcpy(state.entries[i].last_data, data, dlc > 8 ? 8 : dlc);
            return;
        }
    }

    // New ID
    if (state.num_ids < SNIFFER_MAX_IDS) {
        sniffer_entry_t *e = &state.entries[state.num_ids];
        e->id = id;
        e->count = 1;
        e->last_dlc = dlc;
        e->last_tick = xTaskGetTickCount();
        memcpy(e->last_data, data, dlc > 8 ? 8 : dlc);
        state.num_ids++;
    }
}

const sniffer_state_t *can_sniffer_get_state(void) {
    return &state;
}

void can_sniffer_reset(void) {
    memset(&state, 0, sizeof(state));
}
