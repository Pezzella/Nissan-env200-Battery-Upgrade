#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include "../leaf-can-bridge-3-port-env200/canframe.h"

static const uint8_t crctable[256] = {
0,133,143,10,155,30,20,145,179,54,60,185,40,173,167,34,
227,102,108,233,120,253,247,114,80,213,223,90,203,78,68,193,
67,198,204,73,216,93,87,210,240,117,127,250,107,238,228,97,
160,37,47,170,59,190,180,49,19,150,156,25,136,13,7,130,
134,3,9,140,29,152,146,23,53,176,186,63,174,43,33,164,
101,224,234,111,254,123,113,244,214,83,89,220,77,200,194,71,
197,64,74,207,94,219,209,84,118,243,249,124,237,104,98,231,
38,163,169,44,189,56,50,183,149,16,26,159,14,139,129,4,
137,12,6,131,18,151,157,24,58,191,181,48,161,36,46,171,
106,239,229,96,241,116,126,251,217,92,86,211,66,199,205,72,
202,79,69,192,81,212,222,91,121,252,246,115,226,103,109,232,
41,172,166,35,178,55,61,184,154,31,21,144,1,132,142,11,
15,138,128,5,148,17,27,158,188,57,51,182,39,162,168,45,
236,105,99,230,119,242,248,125,95,218,208,85,196,65,75,206,
76,201,195,70,215,82,88,221,255,122,112,245,100,225,235,110,
175,42,32,165,52,177,187,62,28,153,147,22,135,2,8,141
};

static void calc_crc8(can_frame_t *frame){
    uint8_t crc = 0;
    for(int i=0;i<7;i++){
        crc = crctable[(crc ^ frame->data[i]) & 0xFF];
    }
    frame->data[7] = crc;
}

static void patch_1DB(can_frame_t *frame, uint16_t main_battery_soc,
                      uint16_t main_pack_gids, uint16_t extender_full_gids,
                      uint16_t GIDS, int quick_charging, int slow_charging){
    if(slow_charging) return;
    uint8_t raw_soc = frame->data[4] & 0x7F;
    if(quick_charging && raw_soc <= 90){
        uint32_t main_full_gids = main_battery_soc ?
            ((uint32_t)main_pack_gids * 100) / main_battery_soc : main_pack_gids;
        uint32_t total_full_gids = main_full_gids + extender_full_gids;
        uint16_t total_soc = total_full_gids ?
            ((uint32_t)GIDS * 100) / total_full_gids : main_battery_soc;
        if(total_soc <= 90){
            frame->data[4] = (frame->data[4] & 0x80) | (total_soc & 0x7F);
            calc_crc8(frame);
        }
    }
}

int main(void){
    can_frame_t frame = {.can_id = 0x1DB, .can_dlc = 8, .data = {0}};
    frame.data[4] = 70; // OEM SOC 70%
    calc_crc8(&frame);
    patch_1DB(&frame, 70, 300, 375, 603, 1, 0);
    assert((frame.data[4] & 0x7F) == 75);
    uint8_t crc = frame.data[7];
    calc_crc8(&frame);
    assert(frame.data[7] == crc);

    can_frame_t frame2 = {.can_id = 0x1DB, .can_dlc = 8, .data = {0}};
    frame2.data[4] = 95; // above 90%, should not patch
    calc_crc8(&frame2);
    patch_1DB(&frame2, 95, 300, 375, 760, 1, 0);
    assert((frame2.data[4] & 0x7F) == 95);

    can_frame_t frame3 = {.can_id = 0x1DB, .can_dlc = 8, .data = {0}};
    frame3.data[4] = 70; // slow charging, should not patch
    calc_crc8(&frame3);
    patch_1DB(&frame3, 70, 300, 375, 603, 1, 1);
    assert((frame3.data[4] & 0x7F) == 70);
    printf("1DB tests passed\n");
    return 0;
}

