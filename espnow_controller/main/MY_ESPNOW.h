#include <stdint.h>
#include <string.h>
#include "esp_now.h"


#ifndef MY_ESPNOW_H
#define MY_ESPNOW_H

/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
    #define ESPNOW_WIFI_MODE WIFI_MODE_STA
    #define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
    #define ESPNOW_WIFI_MODE WIFI_MODE_AP
    #define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

#define ESPNOW_QUEUE_SIZE 6

// macro used to determine if given MAC address is the broadcast address
#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

enum {
    ESPNOW_DATA_BROADCAST,
    ESPNOW_DATA_UNICAST,
    ESPNOW_DATA_MAX, // implicitly 2
};

// defines event IDs for send and receive callbacks
typedef enum {
    ESPNOW_SEND_CB,
    ESPNOW_RECV_CB,
} espnow_event_id_t;

// struct for send callback info (MAC of recipient and send status)
typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN]; // note that ESP_NOW_ETH_ALEN is defined by esp-idf (i think 6)
    esp_now_send_status_t status; // esp_now_send_status_t is a typedef enum defined by esp-idf 
} espnow_event_send_cb_t;

// struct for receive callback info (MAC of sender, data sent to us and the length)
typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN]; // note that ESP_NOW_ETH_ALEN is defined by esp-idf (i think 6)
    uint8_t* data;
    int data_len;
} espnow_event_recv_cb_t;

// union (makes all members share the same memory location) of send and receive callback info
typedef union {
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
} espnow_event_info_t;

// struct for event info (id and general information)
typedef struct {
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

// struct for received data from controller
typedef struct {
  u_int8_t xValuePack;
  u_int8_t yValuePack;
  u_int8_t buttonState;
  u_int8_t togSwitchVal;
} ESDA_controller_struct;

/* User defined field of ESPNOW data 
- This is the structure of the data that will be sent or received */
typedef struct {
    uint8_t type;                         //Broadcast or unicast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;                     //Sequence number of ESPNOW data.
    uint16_t crc;                         //CRC16 value of ESPNOW data.
    // uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    ESDA_controller_struct sendData;       // Data to be sent
    // uint8_t payload[0];                   //Real payload of ESPNOW data.
} __attribute__((packed)) espnow_data_t;    // gcc specific atribute (__attribute__((packed))) prevents compiler from adding padding bytes between members of a structure

/* Parameters of sending ESPNOW data.
- Used to set up the paramteters for sending data
- Use buffer member to send the espnow_data_t data */
typedef struct {
    bool unicast;                         //Send unicast ESPNOW data.
    bool broadcast;                       //Send broadcast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    // uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint16_t count;                       //Total count of unicast ESPNOW data to be sent.
    uint16_t delay;                       //Delay between sending two ESPNOW data, unit: ms.
    int len;                              //Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                      //Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   //MAC address of destination device.
} espnow_send_param_t;

#endif