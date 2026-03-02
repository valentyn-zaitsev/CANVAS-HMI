#ifndef CAN_SNIFFER_H
#define CAN_SNIFFER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SNIFFER_MAX_IDS 64

typedef struct {
    uint32_t id;
    uint32_t count;
    uint8_t last_data[8];
    uint8_t last_dlc;
    uint32_t last_tick;
} sniffer_entry_t;

typedef struct {
    sniffer_entry_t entries[SNIFFER_MAX_IDS];
    int num_ids;
    uint32_t total_msgs;
} sniffer_state_t;

void can_sniffer_init(void);
void can_sniffer_record(uint32_t id, const uint8_t *data, uint8_t dlc);
const sniffer_state_t *can_sniffer_get_state(void);
void can_sniffer_reset(void);

#ifdef __cplusplus
}
#endif

#endif // CAN_SNIFFER_H
