#include "can_state.h"

static portMUX_TYPE s_energy_mux = portMUX_INITIALIZER_UNLOCKED;

// =============================================================================
// can_state.c — Global state instance definitions.
//
// All structs are zero-initialized by the C runtime before main() runs.
// last_rx_tick == 0 on boot, so staleness checks work correctly from the start:
// any non-zero tick after boot indicates at least one frame has been received.
// =============================================================================

CanStatePedal_t  g_can_pedal  = {0};
CanStateAux_t    g_can_aux    = {0};
CanStatePwr780_t g_can_pwr780 = {0};
CanStatePwr740_t g_can_pwr740 = {0};
CanStateEnergy_t g_can_energy = {0};
CanStateDash_t   g_can_dash   = {0};

void can_state_get_energy_raw(EnergyPayload *dest) {
    // This lock prevents the can_manager from updating the 
    // bytes while we are reading them.
    portENTER_CRITICAL(&s_energy_mux);
    memcpy(dest->rawEnergy, g_can_energy.data.rawEnergy, 5);
    portEXIT_CRITICAL(&s_energy_mux);
}