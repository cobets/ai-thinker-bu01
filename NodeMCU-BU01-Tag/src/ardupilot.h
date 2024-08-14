#include <Arduino.h>
#include "SoftwareSerial.h"

#define MSG_HEADER          0x01
#define MSGID_BEACON_CONFIG 0x02
#define MSGID_BEACON_DIST   0x03
#define MSGID_POSITION      0x04

// structure for messages uploaded to ardupilot
union beacon_config_msg {
    struct {
        uint8_t beacon_id;
        uint8_t beacon_count;
        int32_t x;
        int32_t y;
        int32_t z;
    } info;
    uint8_t buf[14];
};
union beacon_distance_msg {
    struct {
        uint8_t beacon_id;
        uint32_t distance;
    } info;
    uint8_t buf[5];
};
union vehicle_position_msg {
    struct {
        int32_t x;
        int32_t y;
        int32_t z;
        int16_t position_error;
    } info;
    uint8_t buf[14];
};
////////////////////////////////////////////////

// Serial Output to Flight controller
SoftwareSerial fcboardSerial(PB8, PB9); // rx, tx

void send_message(uint8_t msg_id, uint8_t data_len, uint8_t data_buf[])
{
    // sanity check
    if (data_len == 0) {
        return;
    }

    // message is buffer length + 1 (for checksum)
    uint8_t msg_len = data_len+1;

    // calculate checksum and place in last element of array
    uint8_t checksum = 0;
    checksum ^= msg_id;
    checksum ^= msg_len;
    for (uint8_t i=0; i<data_len; i++) {
        checksum = checksum ^ data_buf[i];
    }

    // send message
    int16_t num_sent = 0;
    num_sent += fcboardSerial.write(MSG_HEADER);
    num_sent += fcboardSerial.write(msg_id);
    num_sent += fcboardSerial.write(msg_len);
    num_sent += fcboardSerial.write(data_buf, data_len);
    num_sent += fcboardSerial.write(&checksum, 1);
    fcboardSerial.flush();
}

// send all beacon config to ardupilot
void send_beacon_config(int aNumAnchors, float aAnchorMatrix[][3])
{
    beacon_config_msg msg;
    msg.info.beacon_count = aNumAnchors;
    for (uint8_t i = 0; i < aNumAnchors; i++) {
        msg.info.beacon_id = i;
        msg.info.x = aAnchorMatrix[i][0] * 1000;
        msg.info.y = aAnchorMatrix[i][1] * 1000;
        msg.info.z = aAnchorMatrix[i][2] * 1000;
        send_message(MSGID_BEACON_CONFIG, sizeof(msg.buf), msg.buf);
    }
    Serial1.println("Sent anchor info");
}

// send a beacon's distance to ardupilot
void send_beacon_distance(uint8_t beacon_id, uint32_t distance_mm)
{
    beacon_distance_msg msg;
    msg.info.beacon_id = beacon_id;
    msg.info.distance = distance_mm;
    send_message(MSGID_BEACON_DIST, sizeof(msg.buf), msg.buf);
}

// send vehicle's position to ardupilot
void send_vehicle_position(int32_t x, int32_t y, float pos_error)
{
    // sanity check position
    if (x == 0 || y == 0) {
        return;
    }

    vehicle_position_msg msg;
    
    msg.info.x = x;
    msg.info.y = y;
    msg.info.z = 0;
    msg.info.position_error = pos_error * 1000;
    send_message(MSGID_POSITION, sizeof(msg.buf), msg.buf);
}

void print_tab()
{  
    Serial.print("\t");
}

// print coordinates to the serial monitor
void print_coordinates(int32_t x, int32_t y, float pos_error)
{  
    Serial.print("Pos x:");
    Serial.print(x);
    print_tab();
    Serial.print("y:");
    Serial.print(y);
    Serial.print(" pos err:");
    Serial.print(pos_error);
    Serial.println(); 
}

// get position of tag
void get_position(float aTagPosition[2], float aTagPositionError)
{
    int32_t lx = aTagPosition[0] * 1000;
    int32_t ly = aTagPosition[1] * 1000;

    // display position
    print_coordinates(lx, ly, aTagPositionError);
    
    // send to ardupilot
    send_vehicle_position(lx, ly, aTagPositionError);
} 

// get ranges for each anchor
void get_ranges(int aNumAnchors, float aAnchorDistance[])
{
    // get range for each anchor
    for (uint8_t i=0; i<aNumAnchors; i++) {
        // send info to ardupilot
        send_beacon_distance(i, aAnchorDistance[i] * 1000);
    }
}

void sendToArduPilot(
    int aNumAnchors,
    float aAnchorMatrix[][3],
    float aTagPosition[2],
    float aTagPositionError,
    bool aTagPositionDetected,
    float aAnchorDistance[])
{     
    static uint32_t loop_start = 0;
    static uint8_t stage = 0;   // 0 = initialisation, 1 = normal flight
    static uint16_t beacon_sent_count = 0;
    static uint32_t beacon_sent_time = 0;

    // initialise start time
    if (loop_start == 0) {
        loop_start = millis();
    }

    // advance to normal flight stage after 1min
    if (stage == 0) {
        uint32_t time_diff =  (millis() - loop_start);
        if (time_diff > 60000) {
            stage = 1;
            Serial.println("Stage1");
        }
    }

    // slow down counter
    static uint8_t counter = 0;
    counter++;
    if (counter >= 20) {
        counter = 0;
    }

    // during stage 0 (init) send position and beacon config as quickly as possible
    // during stage 1 send about every 2 seconds
    if (stage == 0 || counter == 0) {
        send_beacon_config(aNumAnchors, aAnchorMatrix);
        if (aTagPositionDetected) {
            get_position(aTagPosition, aTagPositionError);
        }
        if (beacon_sent_count > 0 && beacon_sent_time != 0) {
            uint32_t time_diff = millis() - beacon_sent_time;
            float hz = (float)beacon_sent_count / (time_diff / 1000.0f);
            Serial.print("Beacon hz:");
            Serial.println(hz);
        }
        beacon_sent_count = 0;
        beacon_sent_time = millis();
    }

    // send beacon distances
    get_ranges(aNumAnchors, aAnchorDistance);
    beacon_sent_count++; 
}
