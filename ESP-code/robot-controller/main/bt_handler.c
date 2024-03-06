#define BTSTACK_FILE__ "bt_handler.c"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
 
#include "btstack.h"

// include FreeRTOS headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define RFCOMM_SERVER_CHANNEL 1
#define HEARTBEAT_PERIOD_MS 1000

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static uint16_t rfcomm_channel_id;
static uint8_t  spp_service_buffer[150];
static btstack_packet_callback_registration_t hci_event_callback_registration;

static SemaphoreHandle_t lineBufferMutex;

static char lineBuffer[1024];

extern TaskHandle_t xMotorTaskHandle;
extern TaskHandle_t xSensorTaskHandle;

extern float ledBrightness;
extern float rightMotorSpeed;
extern float leftMotorSpeed;
extern float weaponMotorSpeed;

static void spp_service_setup(void){

    // register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    l2cap_init();

#ifdef ENABLE_BLE
    // Initialize LE Security Manager. Needed for cross-transport key derivation
    sm_init();
#endif

    // create an RFCOMM server
    rfcomm_init();
    rfcomm_register_service(packet_handler, RFCOMM_SERVER_CHANNEL, 0xffff);  // reserved channel, mtu limited by l2cap

    // init SDP, create record for SPP and register with SDP
    sdp_init();
    memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
    // creates a service record for the SPP service - controller connects to this and forms a serial connection
    spp_create_sdp_record(spp_service_buffer, sdp_create_service_record_handle(), RFCOMM_SERVER_CHANNEL, "Robot");
    btstack_assert(de_get_len( spp_service_buffer) <= sizeof(spp_service_buffer));
    sdp_register_service(spp_service_buffer);
}

// packet_handler
// Callback for bluetooth messages received
// HCI events are for the bluetooth setup and RFCOMM events are for the data
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);

    bd_addr_t event_addr;
    uint8_t   rfcomm_channel_nr;
    uint16_t  mtu;

    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {
                case HCI_EVENT_PIN_CODE_REQUEST:
                    // inform about pin code request, occurs during pairing
                    // doesn't actually use 0000, but a random pin
                    printf("Pin code request - using '0000'\n");
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;

                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    // ssp: inform about user confirmation request
                    printf("SSP User Confirmation Request with numeric value '%06"PRIu32"'\n", little_endian_read_32(packet, 8));
                    printf("SSP User Confirmation Auto accept\n");
                    break;

                case RFCOMM_EVENT_INCOMING_CONNECTION:
                    // creates a new RFCOMM channel for the incoming connection
                    rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
                    rfcomm_channel_nr = rfcomm_event_incoming_connection_get_server_channel(packet);
                    rfcomm_channel_id = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
                    printf("RFCOMM channel %u requested for %s\n", rfcomm_channel_nr, bd_addr_to_str(event_addr));
                    rfcomm_accept_connection(rfcomm_channel_id);
                    break;
               
                case RFCOMM_EVENT_CHANNEL_OPENED:
                    if (rfcomm_event_channel_opened_get_status(packet)) {
                        printf("RFCOMM channel open failed, status 0x%02x\n", rfcomm_event_channel_opened_get_status(packet));
                    } else {
                        rfcomm_channel_id = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                        mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
                        printf("RFCOMM channel open succeeded. New RFCOMM Channel ID %u, max frame size %u\n", rfcomm_channel_id, mtu);
                    }
                    break;
                case RFCOMM_EVENT_CAN_SEND_NOW:
                    // only send the data if we've checked the controller is ready
                    // takes mutex to prevent sensor task from writing to the buffer during send
                    xSemaphoreTake(lineBufferMutex, portMAX_DELAY);
                    rfcomm_send(rfcomm_channel_id, (uint8_t*) lineBuffer, (uint16_t) strlen(lineBuffer));
                    xSemaphoreGive(lineBufferMutex);
                    break;
             
                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    printf("RFCOMM channel closed\n");
                    rfcomm_channel_id = 0;
                    break;
                
                default:
                    break;
            }
            break;

        case RFCOMM_DATA_PACKET:
            // packet is array of characters in utf-8
            // data split by " ", and the first character is the command

            int lower = 0, upper = 0;
            // iterate through the packet and split the data until the end
            while (upper < size) {
                if (packet[upper] == ' '){
                    packet[upper] = '\0';
                    switch (packet[lower]) {
                        case 'Z': // LED brightness
                            ledBrightness = atof((char *)packet + lower + 1);
                            break;
                        case 'R': // Right motor speed
                            rightMotorSpeed = atof((char *)packet + lower + 1);
                            break;
                        case 'L': // Left motor speed
                            leftMotorSpeed = atof((char *)packet + lower + 1);
                            break;
                        case 'W': // Weapon motor speed
                            weaponMotorSpeed = atof((char *)packet + lower + 1);
                            break;
                        default:
                            break;
                    }
                    lower = upper + 1;
                    upper = lower;
                }
                upper++;
            }

            // resume the motor task, so it can process the new data
            vTaskResume(xMotorTaskHandle);
            break;

        default:
            break;
    }
}

int btstack_main(int argc, const char * argv[]);
int btstack_main(int argc, const char * argv[]){
    (void)argc;
    (void)argv;

    // do the initial setup for the spp service
    spp_service_setup();

    // set the device as discoverable
    gap_discoverable_control(1);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_set_local_name("Robot 00:00:00:00:00:00");

    hci_power_control(HCI_POWER_ON);
    
    return 0;
}

