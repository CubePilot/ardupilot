
#include "AP_Periph.h"
#include <canard.h>
#include <AP_HAL_ChibiOS/CANIface.h>
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>
#include <uavcan/protocol/dynamic_node_id/Allocation.h>

#if !defined(HAL_BOARD_AP_PERIPH_HEREPRO)
#ifndef HAL_CAN_POOL_SIZE
#define HAL_CAN_POOL_SIZE 4000
#endif
static CanardInstance canard;
static uint32_t canard_memory_pool[HAL_CAN_POOL_SIZE/sizeof(uint32_t)];
#ifndef HAL_CAN_DEFAULT_NODE_ID
#define HAL_CAN_DEFAULT_NODE_ID CANARD_BROADCAST_NODE_ID
#endif
uint8_t PreferredNodeID = HAL_CAN_DEFAULT_NODE_ID;
uint8_t transfer_id;

static ChibiOS::CANIface can_iface(0);

void AP_Periph_FW::can_start()
{
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_INITIALIZATION;
    node_status.uptime_sec = AP_HAL::millis() / 1000U;

    can_iface.init(1000000, AP_HAL::CANIface::NormalMode);

    canardInit(&canard, (uint8_t *)canard_memory_pool, sizeof(canard_memory_pool),
               onTransferReceived, shouldAcceptTransfer, NULL);

    if (PreferredNodeID != CANARD_BROADCAST_NODE_ID) {
        canardSetLocalNodeID(&canard, PreferredNodeID);
    }

    // wait for dynamic node ID allocation
    can_wait_node_id();
}

void AP_Periph_FW::processTx(void)
{
    static uint8_t fail_count;
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
        AP_HAL::CANFrame txmsg {};
        txmsg.dlc = txf->data_len;
        memcpy(txmsg.data, txf->data, 8);
        txmsg.id = (txf->id | AP_HAL::CANFrame::FlagEFF);
        // push message with 1s timeout
        if (can_iface.send(txmsg, AP_HAL::micros64() + 1000000, 0) > 0) {
            canardPopTxQueue(&canard);
            fail_count = 0;
        } else {
            // just exit and try again later. If we fail 8 times in a row
            // then start discarding to prevent the pool filling up
            if (fail_count < 8) {
                fail_count++;
            } else {
                canardPopTxQueue(&canard);
            }
            return;
        }
    }
}

void AP_Periph_FW::processRx(void)
{
    AP_HAL::CANFrame rxmsg;
    while (true) {
        bool read_select = true;
        bool write_select = false;
        can_iface.select(read_select, write_select, nullptr, 0);
        if (!read_select) {
            break;
        }
        CanardCANFrame rx_frame {};

        //palToggleLine(HAL_GPIO_PIN_LED);
        uint64_t timestamp;
        AP_HAL::CANIface::CanIOFlags flags;
        can_iface.receive(rxmsg, timestamp, flags);
        memcpy(rx_frame.data, rxmsg.data, 8);
        rx_frame.data_len = rxmsg.dlc;
        rx_frame.id = rxmsg.id;
        canardHandleRxFrame(&canard, &rx_frame, timestamp);
    }
}

int16_t AP_Periph_FW::canard_broadcast(uint64_t data_type_signature,
                                    uint16_t data_type_id,
                                    uint8_t priority,
                                    const void* payload,
                                    uint16_t payload_len)
{
    return canardBroadcast(&canard,
                    data_type_signature,
                    data_type_id,
                    &transfer_id,
                    priority,
                    payload,
                    payload_len);
}

uint16_t AP_Periph_FW::pool_peak_percent(void)
{
    const CanardPoolAllocatorStatistics stats = canardGetPoolAllocatorStatistics(&canard);
    const uint16_t peak_percent = (uint16_t)(100U * stats.peak_usage_blocks / stats.capacity_blocks);
    return peak_percent;
}

void AP_Periph_FW::cleanup_stale_trx(uint64_t &timestamp_usec)
{
    canardCleanupStaleTransfers(&canard, timestamp_usec);
}


/*
  wait for dynamic allocation of node ID
 */
void AP_Periph_FW::can_wait_node_id(void)
{
    uint8_t node_id_allocation_transfer_id = 0;
    const uint32_t led_pattern = 0xAAAA;
    uint8_t led_idx = 0;
    uint32_t last_led_change = AP_HAL::millis();
    const uint32_t led_change_period = 50;

    while (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID)
    {
        printf("Waiting for dynamic node ID allocation... (pool %u)\n", pool_peak_percent());

        stm32_watchdog_pat();
        uint32_t now = AP_HAL::millis();

        send_next_node_id_allocation_request_at_ms =
            now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
            get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

        while (((now=AP_HAL::millis()) < send_next_node_id_allocation_request_at_ms) &&
               (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID))
        {
            processTx();
            processRx();
            canardCleanupStaleTransfers(&canard, AP_HAL::micros64());
            stm32_watchdog_pat();

            if (now - last_led_change > led_change_period) {
                // blink LED in recognisable pattern while waiting for DNA
                palWriteLine(HAL_GPIO_PIN_LED, (led_pattern & (1U<<led_idx))?1:0);
                led_idx = (led_idx+1) % 32;
                last_led_change = now;
            }
        }


        if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID)
        {
            break;
        }

        // Structure of the request is documented in the DSDL definition
        // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
        uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
        allocation_request[0] = (uint8_t)(PreferredNodeID << 1U);

        if (node_id_allocation_unique_id_offset == 0)
        {
            allocation_request[0] |= 1;     // First part of unique ID
        }

        uint8_t my_unique_id[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH];
        readUniqueID(my_unique_id);

        static const uint8_t MaxLenOfUniqueIDInRequest = 6;
        uint8_t uid_size = (uint8_t)(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH - node_id_allocation_unique_id_offset);
        if (uid_size > MaxLenOfUniqueIDInRequest)
        {
            uid_size = MaxLenOfUniqueIDInRequest;
        }

        // Paranoia time
        assert(node_id_allocation_unique_id_offset < UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH);
        assert(uid_size <= MaxLenOfUniqueIDInRequest);
        assert(uid_size > 0);
        assert((uid_size + node_id_allocation_unique_id_offset) <= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH);

        memmove(&allocation_request[1], &my_unique_id[node_id_allocation_unique_id_offset], uid_size);

        // Broadcasting the request
        const int16_t bcast_res = canardBroadcast(&canard,
                                                  UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                                                  UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                                                  &node_id_allocation_transfer_id,
                                                  CANARD_TRANSFER_PRIORITY_LOW,
                                                  &allocation_request[0],
                                                  (uint16_t) (uid_size + 1));
        if (bcast_res < 0)
        {
            printf("Could not broadcast ID allocation req; error %d\n", bcast_res);
        }

        // Preparing for timeout; if response is received, this value will be updated from the callback.
        node_id_allocation_unique_id_offset = 0;
    }

    printf("Dynamic node ID allocation complete [%d]\n", canardGetLocalNodeID(&canard));
}


void AP_Periph_FW::handle_allocation_response(CanardInstance* ins, CanardRxTransfer* transfer)
{
    // Rule C - updating the randomized time interval
    send_next_node_id_allocation_request_at_ms =
        AP_HAL::millis() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID)
    {
        printf("Allocation request from another allocatee\n");
        node_id_allocation_unique_id_offset = 0;
        return;
    }

    // Copying the unique ID from the message
    static const uint8_t UniqueIDBitOffset = 8;
    uint8_t received_unique_id[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH];
    uint8_t received_unique_id_len = 0;
    for (; received_unique_id_len < (transfer->payload_len - (UniqueIDBitOffset / 8U)); received_unique_id_len++) {
        assert(received_unique_id_len < UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH);
        const uint8_t bit_offset = (uint8_t)(UniqueIDBitOffset + received_unique_id_len * 8U);
        (void) canardDecodeScalar(transfer, bit_offset, 8, false, &received_unique_id[received_unique_id_len]);
    }

    // Obtaining the local unique ID
    uint8_t my_unique_id[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH];
    readUniqueID(my_unique_id);

    // Matching the received UID against the local one
    if (memcmp(received_unique_id, my_unique_id, received_unique_id_len) != 0) {
        printf("Mismatching allocation response\n");
        node_id_allocation_unique_id_offset = 0;
        return;         // No match, return
    }

    if (received_unique_id_len < UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH) {
        // The allocator has confirmed part of unique ID, switching to the next stage and updating the timeout.
        node_id_allocation_unique_id_offset = received_unique_id_len;
        send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;

        printf("Matching allocation response: %d\n", received_unique_id_len);
    } else {
        // Allocation complete - copying the allocated node ID from the message
        uint8_t allocated_node_id = 0;
        (void) canardDecodeScalar(transfer, 0, 7, false, &allocated_node_id);
        assert(allocated_node_id <= 127);

        canardSetLocalNodeID(ins, allocated_node_id);
        printf("Node ID allocated: %d\n", allocated_node_id);
    }
}
#else
uint16_t AP_Periph_FW::pool_peak_percent() { return 0; }
void AP_Periph_FW::processTx() {}
void AP_Periph_FW::processRx() {}
void AP_Periph_FW::cleanup_stale_trx(unsigned long long&) {}
int16_t AP_Periph_FW::canard_broadcast(unsigned long long, unsigned short, unsigned char, void const*, unsigned short) { return 0; }
void AP_Periph_FW::handle_allocation_response(CanardInstance*, CanardRxTransfer*) {}
void AP_Periph_FW::can_start() {}
#endif