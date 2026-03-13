#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// ===================================================================================
// 1. CAN ARBITRATION IDs (Network Dictionary)
// ===================================================================================
// Lower ID = Higher Priority on the physical wire. 
// Critical control (like braking/throttle) should always have lower IDs than telemetry.
typedef enum {
    CAN_ID_PEDAL            = 0x110, // Priority 1: Motion Control
    CAN_ID_AUX_CTRL         = 0x210, // Priority 2: Body/Lights/Wipers
    CAN_ID_PWR_MONITOR_780  = 0x310, // Priority 3: INA780 (High Current Traction)
    CAN_ID_PWR_MONITOR_740  = 0x311, // Priority 3: INA740 (Low Current Aux)
    CAN_ID_PWR_ENERGY       = 0x312, // Priority 4: Energy Accumulator Stats
    CAN_ID_DASH_STAT        = 0x400  // Priority 5: Dashboard UI Updates
} CanMsgID_t;

// Force the compiler to pack these structs exactly as defined, without adding invisible 
// padding bytes. This ensures the wire format perfectly matches the struct format.
#pragma pack(push, 1) 

// ===================================================================================
// A. PEDAL PAYLOAD (1 Byte)
// ===================================================================================
typedef struct {
    uint8_t rawData; 
} PedalPayload;

// Packs a 0-100% float and a brake boolean into a single, efficient byte.
static inline void PedalPayload_set(PedalPayload* p, float throttlePercent, bool brakePressed) {
    if (throttlePercent > 100.0f) throttlePercent = 100.0f;
    if (throttlePercent < 0.0f)   throttlePercent = 0.0f;
    uint8_t t_bits = (uint8_t)((throttlePercent * 127.0f) / 100.0f);
    p->rawData = (t_bits << 1) | (brakePressed ? 1 : 0);
}

// Unpacks the byte back into a 0-100% float for the motor controller.
static inline float PedalPayload_getThrottle(const PedalPayload* p) {
    return (float)(p->rawData >> 1) * 0.7874f; 
}

static inline bool PedalPayload_isBrakePressed(const PedalPayload* p) {
    return (p->rawData & 0x01);
}

// ===================================================================================
// B. AUX CONTROL PAYLOAD (1 Byte)
// ===================================================================================
// Uses bit-fields to pack 8 boolean states into a single byte for lights/accessories.
typedef struct {
    uint8_t raw;
} AuxControlPayload;

#define AUX_LEFT_TURN    (1 << 0)
#define AUX_RIGHT_TURN   (1 << 1)
#define AUX_BRAKE_LIGHT  (1 << 2)
#define AUX_HEADLIGHTS   (1 << 3)
#define AUX_HAZARDS      (1 << 4)
#define AUX_HORN         (1 << 5)
#define AUX_WIPERS       (1 << 6)

static inline void AuxControlPayload_clear(AuxControlPayload* p) { p->raw = 0; }
static inline void AuxControlPayload_set(AuxControlPayload* p, uint8_t flag)    { p->raw |=  flag; }
static inline void AuxControlPayload_clear_flag(AuxControlPayload* p, uint8_t flag) { p->raw &= ~flag; }
static inline bool AuxControlPayload_get(const AuxControlPayload* p, uint8_t flag)  { return (p->raw & flag) != 0; }

// ===================================================================================
// C. POWER PAYLOAD (4 Bytes)
// ===================================================================================
// Transmits the raw ADC registers from the INA sensors to avoid doing floating-point
// math on the sender node. The receiver handles the conversion to physical values.
typedef struct {
    uint16_t rawVolts; 
    int16_t  rawAmps;  
} PowerPayload;

#define POWER_V_SCALE       0.003125f 
#define POWER_I_780_SCALE   0.0024f   
#define POWER_I_740_SCALE   0.0012f   

static inline void PowerPayload_setRaw(PowerPayload* p, uint16_t raw_volts, int16_t raw_amps) {
    p->rawVolts = (uint16_t)raw_volts;
    p->rawAmps  = (int16_t)raw_amps; 
}

static inline float PowerPayload_getVoltage(const PowerPayload* p) { return (float)p->rawVolts * POWER_V_SCALE; }
static inline float PowerPayload_getCurrent_780(const PowerPayload* p) { return (float)p->rawAmps * POWER_I_780_SCALE; }
static inline float PowerPayload_getCurrent_740(const PowerPayload* p) { return (float)p->rawAmps * POWER_I_740_SCALE; }

// ===================================================================================
// D. ENERGY PAYLOAD (40 Bits / 5 Bytes)
// ===================================================================================
// The INA sensors use a 40-bit energy accumulator. Standard C variables are 32 or 64 bits.
// We manually pack only the 5 active bytes to save 3 bytes of valuable CAN bus bandwidth.
typedef struct {
    uint8_t rawEnergy[5]; 
} EnergyPayload;

#define ENERGY_LSB_JOULES_780 0.00768f
#define ENERGY_LSB_JOULES_740 0.00384f 

// Slices the 64-bit variable into 5 bytes (Little Endian)
static inline void EnergyPayload_setRaw(EnergyPayload* p, uint64_t raw_energy_reg) {
    p->rawEnergy[0] = (uint8_t)(raw_energy_reg & 0xFF);         
    p->rawEnergy[1] = (uint8_t)((raw_energy_reg >> 8) & 0xFF);  
    p->rawEnergy[2] = (uint8_t)((raw_energy_reg >> 16) & 0xFF); 
    p->rawEnergy[3] = (uint8_t)((raw_energy_reg >> 24) & 0xFF); 
    p->rawEnergy[4] = (uint8_t)((raw_energy_reg >> 32) & 0xFF); 
}

// Rebuilds the 5 bytes back into a 64-bit integer for math operations
static inline double EnergyPayload_getJoules_780(const EnergyPayload* p) {
    uint64_t rawVal = 0;
    rawVal |= (uint64_t)p->rawEnergy[0];
    rawVal |= ((uint64_t)p->rawEnergy[1] << 8);
    rawVal |= ((uint64_t)p->rawEnergy[2] << 16);
    rawVal |= ((uint64_t)p->rawEnergy[3] << 24);
    rawVal |= ((uint64_t)p->rawEnergy[4] << 32);
    return (double)rawVal * ENERGY_LSB_JOULES_780;
}

static inline double EnergyPayload_getJoules_740(const EnergyPayload* p) {
    uint64_t rawVal = 0;
    rawVal |= (uint64_t)p->rawEnergy[0];
    rawVal |= ((uint64_t)p->rawEnergy[1] << 8);
    rawVal |= ((uint64_t)p->rawEnergy[2] << 16);
    rawVal |= ((uint64_t)p->rawEnergy[3] << 24);
    rawVal |= ((uint64_t)p->rawEnergy[4] << 32);
    return (double)rawVal * ENERGY_LSB_JOULES_740;
}

#pragma pack(pop)