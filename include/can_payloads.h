#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// =============================================================================
// can_payloads.h — CAN wire format definitions.
//
// This is the single source of truth for every frame that travels on the bus:
//   - CAN arbitration IDs (the "network dictionary")
//   - Packed payload structs (the wire format)
//   - Inline encode/decode helpers (no floating-point on the sender node)
//
// Rules:
//   - Never add runtime state here. Runtime state lives in can_state.h.
//   - All structs are #pragma pack(1). What you see is what goes on the wire.
//   - All encode/decode helpers are static inline — zero code size cost if unused.
// =============================================================================

// -----------------------------------------------------------------------------
// 1. CAN Arbitration IDs (Network Dictionary)
// Lower ID = Higher Priority on the physical wire.
// Critical control (throttle/braking) must always have lower IDs than telemetry.
// -----------------------------------------------------------------------------
typedef enum {
    CAN_ID_PEDAL            = 0x110, // Priority 1: Motion Control
    CAN_ID_AUX_CTRL         = 0x210, // Priority 2: Body / Lights / Wipers
    CAN_ID_PWR_MONITOR_780  = 0x310, // Priority 3: INA780 (High-Current Traction)
    CAN_ID_PWR_MONITOR_740  = 0x311, // Priority 3: INA740 (Low-Current Aux)
    CAN_ID_PWR_ENERGY       = 0x312, // Priority 4: Energy Accumulator Stats
    CAN_ID_DASH_STAT        = 0x400  // Priority 5: Dashboard UI Updates
} CanMsgID_t;

// Force the compiler to pack structs exactly as defined — no invisible padding.
// This guarantees the wire format perfectly matches the struct layout.
#pragma pack(push, 1)

// -----------------------------------------------------------------------------
// A. PEDAL PAYLOAD (2 Bytes / 16 Bits)
// Layout: [15] Brake Flag | [14:0] 15-bit Throttle (0–32767)
// Moving Brake to MSB allows for instant safety checks via bit-masking.
// -----------------------------------------------------------------------------
typedef struct {
    uint16_t pedalData;
} PedalPayload;

static inline void PedalPayload_set(PedalPayload *p, float throttlePercent, bool brakePressed)
{
    // [1] Robust Clamping
    if (throttlePercent > 100.0f) throttlePercent = 100.0f;
    if (throttlePercent < 0.0f)   throttlePercent = 0.0f;  

    // [2] Accurate 15-bit Scaling (0 - 32767)
    // Adding 0.5f prevents truncation errors (e.g., 99.9% becoming 99%).
    uint16_t t_bits = (uint16_t)((throttlePercent * 327.67f) + 0.5f);

    // [3] Safe Bit Packing
    // Masking throttle with 0x7FFF ensures it CANNOT bleed into the MSB.
    uint16_t brake_bit = (brakePressed ? 0x8000 : 0x0000);
    p->pedalData = brake_bit | (t_bits & 0x7FFF);
}

static inline void PedalPayload_setRaw(PedalPayload *p, uint16_t raw) {
    p->pedalData = raw;
}

static inline float PedalPayload_getThrottle(const PedalPayload *p)
{
    // Mask out the MSB (Brake) and divide by the 15-bit scale constant.
    return (float)(p->pedalData & 0x7FFF) * (100.0f / 32767.0f); 
}

static inline bool PedalPayload_isBrakePressed(const PedalPayload *p)
{
    // Quick check of bit 15.
    return (p->pedalData & 0x8000) != 0; 
}

// -----------------------------------------------------------------------------
// B. AUX CONTROL PAYLOAD (1 Byte)
// 8 boolean accessory states packed into one byte via bitfields.
// Transmitted both periodically (heartbeat) and on-change by the sender node.
// -----------------------------------------------------------------------------
typedef union {
    struct {
        uint8_t left_turn   : 1; // Bit 0
        uint8_t right_turn  : 1; // Bit 1
        uint8_t brake_light : 1; // Bit 2
        uint8_t headlights  : 1; // Bit 3
        uint8_t hazards     : 1; // Bit 4
        uint8_t horn        : 1; // Bit 5
        uint8_t wipers      : 1; // Bit 6
        uint8_t reserved    : 1; // Bit 7 — reserved for future use
    };
    uint8_t rawAux;
} AuxControlPayload;

// -----------------------------------------------------------------------------
// C. POWER PAYLOAD (4 Bytes)
// Transmits raw ADC registers from INA780/INA740 sensors.
// Floating-point conversion is done by the receiver, not the sender.
// -----------------------------------------------------------------------------
typedef struct {
    uint16_t rawVolts;
    int16_t  rawAmps;
} PowerPayload;

#define POWER_V_SCALE       0.003125f   // V per LSB
#define POWER_I_780_SCALE   0.0024f     // A per LSB (INA780 high-current)
#define POWER_I_740_SCALE   0.0012f     // A per LSB (INA740 low-current)

static inline void PowerPayload_setRaw(PowerPayload *p, uint16_t raw_volts, int16_t raw_amps)
{
    p->rawVolts = raw_volts;
    p->rawAmps  = raw_amps;
}

static inline float PowerPayload_getVoltage(const PowerPayload *p)
{
    return (float)p->rawVolts * POWER_V_SCALE;
}

static inline float PowerPayload_getCurrent_780(const PowerPayload *p)
{
    return (float)p->rawAmps * POWER_I_780_SCALE;
}

static inline float PowerPayload_getCurrent_740(const PowerPayload *p)
{
    return (float)p->rawAmps * POWER_I_740_SCALE;
}

// -----------------------------------------------------------------------------
// D. ENERGY PAYLOAD (5 Bytes / 40 Bits)
// The INA sensors use a 40-bit hardware energy accumulator. We manually pack
// only the 5 active bytes (little-endian) to save 3 bytes of CAN bandwidth
// compared to a naively-cast uint64_t.
// -----------------------------------------------------------------------------
typedef struct {
    uint8_t rawEnergy[5];
} EnergyPayload;

#define ENERGY_LSB_JOULES_780   0.00768f    // J per LSB (INA780)
#define ENERGY_LSB_JOULES_740   0.00384f    // J per LSB (INA740)

// Pack a 64-bit register value into 5 bytes, little-endian.
static inline void EnergyPayload_setRaw(EnergyPayload *p, uint64_t raw_energy_reg)
{
    p->rawEnergy[0] = (uint8_t)(raw_energy_reg        & 0xFF);
    p->rawEnergy[1] = (uint8_t)((raw_energy_reg >>  8) & 0xFF);
    p->rawEnergy[2] = (uint8_t)((raw_energy_reg >> 16) & 0xFF);
    p->rawEnergy[3] = (uint8_t)((raw_energy_reg >> 24) & 0xFF);
    p->rawEnergy[4] = (uint8_t)((raw_energy_reg >> 32) & 0xFF);
}

// Rebuild 5 bytes into a 64-bit value, then apply the sensor-specific scale.
static inline double EnergyPayload_getJoules_780(const EnergyPayload *p)
{
    uint64_t raw = 0;
    raw |= (uint64_t)p->rawEnergy[0];
    raw |= ((uint64_t)p->rawEnergy[1] <<  8);
    raw |= ((uint64_t)p->rawEnergy[2] << 16);
    raw |= ((uint64_t)p->rawEnergy[3] << 24);
    raw |= ((uint64_t)p->rawEnergy[4] << 32);
    return (double)raw * ENERGY_LSB_JOULES_780;
}

static inline double EnergyPayload_getJoules_740(const EnergyPayload *p)
{
    uint64_t raw = 0;
    raw |= (uint64_t)p->rawEnergy[0];
    raw |= ((uint64_t)p->rawEnergy[1] <<  8);
    raw |= ((uint64_t)p->rawEnergy[2] << 16);
    raw |= ((uint64_t)p->rawEnergy[3] << 24);
    raw |= ((uint64_t)p->rawEnergy[4] << 32);
    return (double)raw * ENERGY_LSB_JOULES_740;
}

#pragma pack(pop)
