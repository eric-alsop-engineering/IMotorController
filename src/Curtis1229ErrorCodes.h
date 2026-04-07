/*
  Curtis1229ErrorCodes.h — Fault code lookup for Curtis 1229 motor controllers.

  Maps Curtis-specific fault codes (from 1229 OS 1.8 Fault Status Mapping)
  to short human-readable strings suitable for EVE display status messages.

  Fault codes are Curtis-specific — NOT standard CANopen EMCY codes.
  See Firmware/Documentation/Curtis1229/1229_Fault_Status_Mapping.xlsx for
  the complete mapping with status register bit positions.

  Created April 7, 2026.
  Copyright 2026 Best Tugs, LLC
*/
#ifndef CURTIS1229_ERROR_CODES_H
#define CURTIS1229_ERROR_CODES_H

#include <Arduino.h>

// ─── Curtis 1229 Fault Codes (3100R / decimal) ─────────────────────────────
// From 1229_Fault_Status_Mapping.xlsx and 53129_1229_OS 1.8_RevA.pdf

#define CURTIS_FAULT_HW_FAILSAFE                1
#define CURTIS_FAULT_PLD_CLOCK_FAIL              2
#define CURTIS_FAULT_CALIBRATION_RESET           9
#define CURTIS_FAULT_MAIN_BRAKE_OVERCURRENT     10
#define CURTIS_FAULT_MAIN_DRIVER_OPEN_DRAIN     11
#define CURTIS_FAULT_EMR_REDUNDANCY             12
#define CURTIS_FAULT_EEPROM_FAILURE             13
#define CURTIS_FAULT_PUSH_OVERVOLTAGE           14
#define CURTIS_FAULT_MAIN_CONTACTOR_DROPPED     15
#define CURTIS_FAULT_CURRENT_SENSOR             16
#define CURTIS_FAULT_MAIN_CONTACTOR_WELDED      17
#define CURTIS_FAULT_ENCODER                    18
#define CURTIS_FAULT_PDO_TIMEOUT                19
#define CURTIS_FAULT_SUPERVISOR_COMMS           20
#define CURTIS_FAULT_SUPERVISOR_WATCHDOG        21
#define CURTIS_FAULT_SUPERVISOR_POT1            22
#define CURTIS_FAULT_SUPERVISOR_POT2            23
#define CURTIS_FAULT_SUPERVISOR_POT3            24
#define CURTIS_FAULT_SUPERVISOR_POTH            25
#define CURTIS_FAULT_SUPERVISOR_SW1             26
#define CURTIS_FAULT_SUPERVISOR_SW2             27
#define CURTIS_FAULT_SUPERVISOR_SW3             28
#define CURTIS_FAULT_SUPERVISOR_SW4             29
#define CURTIS_FAULT_SUPERVISOR_SW5             30
#define CURTIS_FAULT_SUPERVISOR_KSI_VOLTAGE     31
#define CURTIS_FAULT_SUPERVISOR_MOTOR_SPEED     32
#define CURTIS_FAULT_SUPERVISOR_DIR_CHECK       33
#define CURTIS_FAULT_EXTERNAL_SUPPLY            34
#define CURTIS_FAULT_EMBRAKE_OVERCURRENT        35
#define CURTIS_FAULT_EMBRAKE_OPEN_DRAIN         36
#define CURTIS_FAULT_EMBRAKE_DRIVER_ON          37
#define CURTIS_FAULT_EM_BRAKE_FAILED_TO_SET     38
#define CURTIS_FAULT_POT1_WIPER                 41
#define CURTIS_FAULT_POT2_WIPER                 42
#define CURTIS_FAULT_POT3_WIPER                 43
#define CURTIS_FAULT_DRIVER_SHORTED             70
#define CURTIS_FAULT_DRIVER3_FAULT              71
#define CURTIS_FAULT_DRIVER3_OVERCURRENT        72
#define CURTIS_FAULT_DRIVER4_FAULT              73
#define CURTIS_FAULT_DRIVER4_OVERCURRENT        74
#define CURTIS_FAULT_DRIVER5_FAULT              75
#define CURTIS_FAULT_DRIVER5_OVERCURRENT        76
#define CURTIS_FAULT_DRIVER6_FAULT              77
#define CURTIS_FAULT_DRIVER6_OVERCURRENT        78
#define CURTIS_FAULT_CORRELATION                79
#define CURTIS_FAULT_HPD_SEQUENCING             80
#define CURTIS_FAULT_PARAMETER_CHANGE           81
#define CURTIS_FAULT_NV_MEMORY                  82
#define CURTIS_FAULT_MOTOR_TEMP_HOT_CUTBACK     90
#define CURTIS_FAULT_MOTOR_OPEN                 92
#define CURTIS_FAULT_CONTROLLER_OVERCURRENT     93
#define CURTIS_FAULT_VBAT_TOO_HIGH              94
#define CURTIS_FAULT_CONTROLLER_UNDERTEMP_CB    95
#define CURTIS_FAULT_STALL_DETECTED             96
#define CURTIS_FAULT_OVERTEMP_CUTBACK           97
#define CURTIS_FAULT_OVERVOLTAGE_CUTBACK        98
#define CURTIS_FAULT_UNDERVOLTAGE_CUTBACK       99
#define CURTIS_FAULT_SEVERE_UNDERVOLTAGE        50
#define CURTIS_FAULT_CONTROLLER_SEVERE_UNDERTEMP 52
#define CURTIS_FAULT_CONTROLLER_SEVERE_OVERTEMP  53
#define CURTIS_FAULT_PRECHARGE_FAILED           54

// ─── Fault code → short display string ─────────────────────────────────────
// Returns a short string suitable for EVE StatusBar display.
// Strings are stored in flash (PROGMEM not needed on Teensy 4.1 / ARM).

inline const char* curtis1229FaultString(uint16_t faultCode)
{
    switch (faultCode)
    {
    case 0:  return "No Fault";
    case CURTIS_FAULT_HW_FAILSAFE:             return "HW FAILSAFE";
    case CURTIS_FAULT_PLD_CLOCK_FAIL:           return "PLD CLOCK FAIL";
    case CURTIS_FAULT_CALIBRATION_RESET:        return "CALIBRATION RESET";
    case CURTIS_FAULT_MAIN_BRAKE_OVERCURRENT:   return "MAIN BRAKE OVERCUR";
    case CURTIS_FAULT_MAIN_DRIVER_OPEN_DRAIN:   return "MAIN DRV OPEN";
    case CURTIS_FAULT_EMR_REDUNDANCY:           return "EMR REDUNDANCY";
    case CURTIS_FAULT_EEPROM_FAILURE:           return "EEPROM FAILURE";
    case CURTIS_FAULT_PUSH_OVERVOLTAGE:         return "PUSH OVERVOLTAGE";
    case CURTIS_FAULT_MAIN_CONTACTOR_DROPPED:   return "CONTACTOR DROPPED";
    case CURTIS_FAULT_CURRENT_SENSOR:           return "CURRENT SENSOR";
    case CURTIS_FAULT_MAIN_CONTACTOR_WELDED:    return "CONTACTOR WELDED";
    case CURTIS_FAULT_ENCODER:                  return "ENCODER FAULT";
    case CURTIS_FAULT_PDO_TIMEOUT:              return "PDO TIMEOUT";
    case CURTIS_FAULT_SUPERVISOR_COMMS:         return "SUPERVISOR COMMS";
    case CURTIS_FAULT_SUPERVISOR_WATCHDOG:      return "SUPERVISOR WDT";
    case CURTIS_FAULT_SUPERVISOR_POT1:          return "SUPERVISOR POT1";
    case CURTIS_FAULT_SUPERVISOR_POT2:          return "SUPERVISOR POT2";
    case CURTIS_FAULT_SUPERVISOR_POT3:          return "SUPERVISOR POT3";
    case CURTIS_FAULT_SUPERVISOR_POTH:          return "SUPERVISOR POTH";
    case CURTIS_FAULT_SUPERVISOR_SW1:           return "SUPERVISOR SW1";
    case CURTIS_FAULT_SUPERVISOR_SW2:           return "SUPERVISOR SW2";
    case CURTIS_FAULT_SUPERVISOR_SW3:           return "SUPERVISOR SW3";
    case CURTIS_FAULT_SUPERVISOR_SW4:           return "SUPERVISOR SW4";
    case CURTIS_FAULT_SUPERVISOR_SW5:           return "SUPERVISOR SW5";
    case CURTIS_FAULT_SUPERVISOR_KSI_VOLTAGE:   return "SUPERVISOR KSI";
    case CURTIS_FAULT_SUPERVISOR_MOTOR_SPEED:   return "SUPERVISOR SPEED";
    case CURTIS_FAULT_SUPERVISOR_DIR_CHECK:     return "SUPERVISOR DIR";
    case CURTIS_FAULT_EXTERNAL_SUPPLY:          return "EXT SUPPLY FAULT";
    case CURTIS_FAULT_EMBRAKE_OVERCURRENT:      return "EMBRAKE OVERCUR";
    case CURTIS_FAULT_EMBRAKE_OPEN_DRAIN:       return "EMBRAKE OPEN DRN";
    case CURTIS_FAULT_EMBRAKE_DRIVER_ON:        return "EMBRAKE DRV ON";
    case CURTIS_FAULT_EM_BRAKE_FAILED_TO_SET:   return "EMBRAKE NO SET";
    case CURTIS_FAULT_POT1_WIPER:               return "POT1 WIPER";
    case CURTIS_FAULT_POT2_WIPER:               return "POT2 WIPER";
    case CURTIS_FAULT_POT3_WIPER:               return "POT3 WIPER";
    case CURTIS_FAULT_DRIVER_SHORTED:           return "DRIVER SHORTED";
    case CURTIS_FAULT_DRIVER3_FAULT:            return "DRIVER3 FAULT";
    case CURTIS_FAULT_DRIVER3_OVERCURRENT:      return "DRIVER3 OVERCUR";
    case CURTIS_FAULT_DRIVER4_FAULT:            return "DRIVER4 FAULT";
    case CURTIS_FAULT_DRIVER4_OVERCURRENT:      return "DRIVER4 OVERCUR";
    case CURTIS_FAULT_DRIVER5_FAULT:            return "DRIVER5 FAULT";
    case CURTIS_FAULT_DRIVER5_OVERCURRENT:      return "DRIVER5 OVERCUR";
    case CURTIS_FAULT_DRIVER6_FAULT:            return "DRIVER6 FAULT";
    case CURTIS_FAULT_DRIVER6_OVERCURRENT:      return "DRIVER6 OVERCUR";
    case CURTIS_FAULT_CORRELATION:              return "CORRELATION";
    case CURTIS_FAULT_HPD_SEQUENCING:           return "HPD SEQUENCING";
    case CURTIS_FAULT_PARAMETER_CHANGE:         return "PARAM CHANGE";
    case CURTIS_FAULT_NV_MEMORY:                return "NV MEMORY FAULT";
    case CURTIS_FAULT_MOTOR_TEMP_HOT_CUTBACK:   return "MOTOR TEMP HIGH";
    case CURTIS_FAULT_MOTOR_OPEN:               return "MOTOR OPEN";
    case CURTIS_FAULT_CONTROLLER_OVERCURRENT:   return "CTRL OVERCURRENT";
    case CURTIS_FAULT_VBAT_TOO_HIGH:            return "VBAT TOO HIGH";
    case CURTIS_FAULT_CONTROLLER_UNDERTEMP_CB:  return "CTRL UNDERTEMP";
    case CURTIS_FAULT_STALL_DETECTED:           return "STALL DETECTED";
    case CURTIS_FAULT_OVERTEMP_CUTBACK:         return "OVERTEMP CUTBACK";
    case CURTIS_FAULT_OVERVOLTAGE_CUTBACK:      return "OVERVOLT CUTBACK";
    case CURTIS_FAULT_UNDERVOLTAGE_CUTBACK:     return "UNDERVOLT CUTBACK";
    case CURTIS_FAULT_SEVERE_UNDERVOLTAGE:      return "SEVERE UNDERVOLT";
    case CURTIS_FAULT_CONTROLLER_SEVERE_UNDERTEMP: return "SEVERE UNDERTEMP";
    case CURTIS_FAULT_CONTROLLER_SEVERE_OVERTEMP:  return "SEVERE OVERTEMP";
    case CURTIS_FAULT_PRECHARGE_FAILED:         return "PRECHARGE FAILED";
    default:                                    return "UNKNOWN FAULT";
    }
}

// ─── EMCY status byte → fault code mapping tables ──────────────────────────
// EMCY payload bytes 3-7 are status register bitmasks.
// For category 0x1000: bytes 3-7 = Status1-Status5, each byte has 8 fault bits.
// For category 0x1001: bytes 3-7 = Status6-Status10 (Status9/10 are reserved).
// Each entry maps [register_index][bit_position] → Curtis fault code.
// From 1229_Fault_Status_Mapping.xlsx.

// Category 0x1000 — Status1 through Status5 (5 bytes × 8 bits = 40 faults)
static const uint8_t EMCY_CAT_1000_FAULT_MAP[5][8] = {
    // Status1 (byte 3): bits 0-7
    { 17, 14, 15, 41, 42, 43, 13, 80 },
    // Status2 (byte 4): bits 0-7
    { 50, 19, 99, 98, 97, 52, 53, 54 },
    // Status3 (byte 5): bits 0-7
    { 93, 16, 90, 81, 92, 74, 18, 96 },
    // Status4 (byte 6): bits 0-7
    { 12, 72, 38, 71, 73, 75, 77, 70 },
    // Status5 (byte 7): bits 0-7
    { 35, 36, 37, 79, 10, 11, 76, 78 },
};

// Category 0x1001 — Status6 through Status10 (5 bytes × 8 bits = 40 faults)
// Status9 and Status10 are reserved (all zeros).
static const uint8_t EMCY_CAT_1001_FAULT_MAP[5][8] = {
    // Status6 (byte 3): bits 0-7
    { 0, 20, 21, 22, 23, 24, 25, 26 },  // bit 0 = PUSH_SW_ACTIVE (code 0, informational)
    // Status7 (byte 4): bits 0-7
    { 27, 28, 29, 30, 31, 32, 33, 34 },
    // Status8 (byte 5): bits 0-7
    {  1,  9,  2, 95, 82, 94, 49,  0 },  // bit 7 = LAST_FAULT (code 0, sentinel)
    // Status9 (byte 6): all reserved
    {  0,  0,  0,  0,  0,  0,  0,  0 },
    // Status10 (byte 7): all reserved
    {  0,  0,  0,  0,  0,  0,  0,  0 },
};

// ─── Decode first active fault code from EMCY status bytes ─────────────────
// Scans status bitmask bytes from an EMCY message and returns the first
// non-zero fault code found. Returns 0 if no faults are set.

inline uint16_t curtis1229DecodeFaultFromEMCY(uint16_t errorCategory, const uint8_t statusBytes[5])
{
    const uint8_t (*faultMap)[8] = nullptr;

    if (errorCategory == 0x1000)
        faultMap = EMCY_CAT_1000_FAULT_MAP;
    else if (errorCategory == 0x1001)
        faultMap = EMCY_CAT_1001_FAULT_MAP;
    else
        return 0;  // Unknown category (0x6200 user faults handled separately)

    for (int byteIdx = 0; byteIdx < 5; byteIdx++)
    {
        uint8_t statusByte = statusBytes[byteIdx];
        if (statusByte == 0) continue;  // Skip empty registers quickly

        for (int bit = 0; bit < 8; bit++)
        {
            if (statusByte & (1 << bit))
            {
                uint8_t code = faultMap[byteIdx][bit];
                if (code != 0) return (uint16_t)code;
            }
        }
    }
    return 0;
}

// ─── Fault severity classification ─────────────────────────────────────────
// Returns true if the fault triggers a full shutdown (critical).
// Cutback faults (limited current) are warnings, not critical.

inline bool curtis1229FaultIsCritical(uint16_t faultCode)
{
    switch (faultCode)
    {
    case CURTIS_FAULT_HW_FAILSAFE:
    case CURTIS_FAULT_PLD_CLOCK_FAIL:
    case CURTIS_FAULT_CALIBRATION_RESET:
    case CURTIS_FAULT_MAIN_BRAKE_OVERCURRENT:
    case CURTIS_FAULT_MAIN_DRIVER_OPEN_DRAIN:
    case CURTIS_FAULT_EMR_REDUNDANCY:
    case CURTIS_FAULT_EEPROM_FAILURE:
    case CURTIS_FAULT_CURRENT_SENSOR:
    case CURTIS_FAULT_ENCODER:
    case CURTIS_FAULT_PDO_TIMEOUT:
    case CURTIS_FAULT_SUPERVISOR_COMMS:
    case CURTIS_FAULT_SUPERVISOR_WATCHDOG:
    case CURTIS_FAULT_SUPERVISOR_POT1:
    case CURTIS_FAULT_SUPERVISOR_POT2:
    case CURTIS_FAULT_SUPERVISOR_POT3:
    case CURTIS_FAULT_SUPERVISOR_POTH:
    case CURTIS_FAULT_SUPERVISOR_SW1:
    case CURTIS_FAULT_SUPERVISOR_SW2:
    case CURTIS_FAULT_SUPERVISOR_SW3:
    case CURTIS_FAULT_SUPERVISOR_SW4:
    case CURTIS_FAULT_SUPERVISOR_SW5:
    case CURTIS_FAULT_SUPERVISOR_KSI_VOLTAGE:
    case CURTIS_FAULT_SUPERVISOR_MOTOR_SPEED:
    case CURTIS_FAULT_SUPERVISOR_DIR_CHECK:
    case CURTIS_FAULT_EXTERNAL_SUPPLY:
    case CURTIS_FAULT_SEVERE_UNDERVOLTAGE:
    case CURTIS_FAULT_CONTROLLER_SEVERE_UNDERTEMP:
    case CURTIS_FAULT_CONTROLLER_SEVERE_OVERTEMP:
    case CURTIS_FAULT_PRECHARGE_FAILED:
    case CURTIS_FAULT_CONTROLLER_OVERCURRENT:
    case CURTIS_FAULT_STALL_DETECTED:
    case CURTIS_FAULT_DRIVER_SHORTED:
        return true;
    default:
        return false;
    }
}

#endif // CURTIS1229_ERROR_CODES_H
