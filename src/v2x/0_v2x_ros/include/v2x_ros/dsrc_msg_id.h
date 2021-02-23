#ifndef __DSRC_MSG_ID_H__
#define __DSRC_MSG_ID_H__

namespace dsrc_msg_id
{
    enum dsrc_msg_ids
    {
        //
        // DER forms,
        // All DER forms are now retired and not to be used
        //
        RESERVED_MESSAGE_ID_D = 0,
        ALA_CARTE_MESSAGE_D,               // 1
        BASIC_SAFETY_MESSAGE_D,            // 2
        BASIC_SAFETY_MESSAGE_VERBOSE_D,    // 3
        COMMON_SAFETY_REQUEST_D,           // 4
        EMERGENCY_VEHICLE_ALERT_D,         // 5
        INTERSECTION_COLLISION_D,          // 6
        MAP_DATA_D,                        // 7
        NMEA_CORRECTIONS_D,                // 8
        PROBE_DATA_MANAGEMENT_D,           // 9
        PROBE_VEHICLE_DATA_D,              // 10
        ROAD_SIDE_ALERT_D,                 // 11
        RTCM_CORRECTIONS_D,                // 12
        SIGNAL_PHASE_AND_TIMING_MESSAGE_D, // 13
        SIGNAL_REQUEST_MESSAGE_D,          // 14
        SIGNAL_STATUS_MESSAGE_D,           // 15
        TRAVELER_INFORMATION_D,            // 16
        UPER_FRAME_D,                      // 17
        //
        // UPER forms
        //
        MAP_DATA = 18,                        // MAP
        SIGNAL_PHASE_AND_TIMING_MESSAGE = 19, // SPAT
        // Above two entries were adopted in the 2015-04 edition
        // Message assignments added in 2015 follow below
        BASIC_SAFETY_MESSAGE = 20,    // BSM
        COMMON_SAFETY_REQUEST = 21,   // CSR
        EMERGENCY_VEHICLE_ALERT = 22, // EVA
        INTERSECTION_COLLISION = 23,  // ICA
        NMEA_CORRECTIONS = 24,        // NMEA
        PROBE_DATA_MANAGEMENT = 25,   // PDM
        PROBE_VEHICLE_DATA = 26,      // PVD
        ROAD_SIDE_ALERT = 27,         // RSA
        RTCM_CORRECTIONS = 28,        // RTCM
        SIGNAL_REQUEST_MESSAGE = 29,  // SRM
        SIGNAL_STATUS_MESSAGE = 30,   // SSM
        TRAVELER_INFORMATION = 31,    // TIM
        PERSONAL_SAFETY_MESSAGE = 32, // PSM
        //
        // The Below values are reserved for local message testing use
        //
        TEST_MESSAGE00 = 240, // Hex 0xF0
        TEST_MESSAGE01,       // 241
        TEST_MESSAGE02,       // 242
        TEST_MESSAGE03,       // 243
        TEST_MESSAGE04,       // 244
        TEST_MESSAGE05,       // 245
        TEST_MESSAGE06,       // 246
        TEST_MESSAGE07,       // 247
        TEST_MESSAGE08,       // 248
        TEST_MESSAGE09,       // 249
        TEST_MESSAGE10,       // 250
        TEST_MESSAGE11,       // 251
        TEST_MESSAGE12,       // 252
        TEST_MESSAGE13,       // 253
        TEST_MESSAGE14,       // 254
        TEST_MESSAGE15        // 255
    };
}

#endif