/*

    Simple Satellite Operations  beacon_series.c

    Copyright (C) 2026  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "beacon_series.h"

#include "beacon_cts1.h"

#include <limits.h>
#include <math.h>
#include <string.h>

// Shorthand for the two structs' member offsets, so the table below
// reads as field names rather than as numbers and the compiler is the
// one that works out where anything sits.
#define B(member)  offsetof(COMMS_beacon_basic_packet_t, member)
#define X(member)  offsetof(COMMS_beacon_extended_packet_t, member)

// The extended beacon duplicates the basic one's fields, so an offset
// taken from either struct has to land in the same place. This is the
// same guarantee beacon_cts1.h asserts on the size of the shared part,
// spelled out for the handful of fields the table reaches through the
// basic struct.
_Static_assert(B(eps_battery_voltage_mV) == X(eps_battery_voltage_mV),
               "the two beacon structs disagree on where the battery voltage is");
_Static_assert(B(obc_temperature_cC) == X(obc_temperature_cC),
               "the two beacon structs disagree on where the OBC temperature is");
_Static_assert(B(end_message) == X(end_message),
               "the two beacon structs disagree on where the shared part ends");

const bs_field_t BEACON_SERIES_FIELDS[] = {
    // ---- power, from the part both beacons carry ----
    { "batt_v",       "battery voltage",        "V",
      B(eps_battery_voltage_mV),    BS_U16, 0.001, 0, BS_SENT_ZERO,
      BS_NEEDS_NOTHING, BS_GROUP_POWER },
    { "batt_pct",     "battery charge",         "%",
      B(eps_battery_percent),       BS_U8,  1.0,   0, BS_SENT_ZERO,
      BS_NEEDS_NOTHING, BS_GROUP_POWER },
    { "pcu_in",       "solar power in",         "W",
      B(eps_total_pcu_power_input_cW),  BS_I32, 0.01, 0, BS_SENT_EPS_I32,
      BS_NEEDS_NOTHING, BS_GROUP_POWER },
    { "pcu_out",      "power out of the PCU",   "W",
      B(eps_total_pcu_power_output_cW), BS_I32, 0.01, 0, BS_SENT_EPS_I32,
      BS_NEEDS_NOTHING, BS_GROUP_POWER },
    { "pcu_avg_in",   "solar power in, mean",   "W",
      B(eps_total_avg_pcu_power_input_cW),  BS_I32, 0.01, 0, BS_SENT_EPS_I32,
      BS_NEEDS_NOTHING, BS_GROUP_POWER },
    { "pcu_avg_out",  "power out, mean",        "W",
      B(eps_total_avg_pcu_power_output_cW), BS_I32, 0.01, 0, BS_SENT_EPS_I32,
      BS_NEEDS_NOTHING, BS_GROUP_POWER },
    { "eps_faults",   "EPS fault count",        "",
      B(eps_total_fault_count),     BS_I32, 1.0,   0, BS_SENT_NEG1,
      BS_NEEDS_NOTHING, BS_GROUP_POWER },
    { "eps_mode",     "EPS mode",               "",
      B(eps_mode_enum),             BS_U8,  1.0,   0, BS_SENT_U8_MAX,
      BS_NEEDS_NOTHING, BS_GROUP_POWER },

    // ---- temperatures ----
    { "batt_t0",      "battery temperature 0",  "C",
      B(eps_battery_temperature_0_cC), BS_I16, 0.01, 0, BS_SENT_TEMP_I16,
      BS_NEEDS_NOTHING, BS_GROUP_THERMAL },
    { "batt_t1",      "battery temperature 1",  "C",
      B(eps_battery_temperature_1_cC), BS_I16, 0.01, 0, BS_SENT_TEMP_I16,
      BS_NEEDS_NOTHING, BS_GROUP_THERMAL },
    { "obc_t",        "OBC temperature",        "C",
      B(obc_temperature_cC),        BS_I32, 0.01,  0, BS_SENT_TEMP_I32,
      BS_NEEDS_NOTHING, BS_GROUP_THERMAL },
    { "mpi_t",        "MPI temperature, last active", "C",
      X(mpi_last_temperature_C),    BS_I8,  1.0,   1, BS_SENT_MPI_TEMP,
      BS_NEEDS_NOTHING, BS_GROUP_THERMAL },

    // ---- housekeeping ----
    { "uptime",       "OBC uptime",             "h",
      B(uptime_ms),                 BS_U32, 1.0 / 3600000.0, 0, BS_SENT_NONE,
      BS_NEEDS_NOTHING, BS_GROUP_HOUSEKEEPING },
    { "since_uplink", "time since last uplink", "h",
      B(duration_since_last_uplink_ms), BS_U32, 1.0 / 3600000.0, 0,
      BS_SENT_NONE, BS_NEEDS_NOTHING, BS_GROUP_HOUSEKEEPING },
    { "eps_uptime",   "EPS uptime",             "h",
      B(eps_uptime_sec),            BS_U32, 1.0 / 3600.0, 0, BS_SENT_NONE,
      BS_NEEDS_NOTHING, BS_GROUP_HOUSEKEEPING },
    { "beacon_count", "beacons since boot",     "",
      B(total_beacon_count_since_boot), BS_U32, 1.0, 0, BS_SENT_NONE,
      BS_NEEDS_NOTHING, BS_GROUP_HOUSEKEEPING },
    { "tcmd_queued",  "telecommands queued",    "",
      B(total_tcmd_queued_count),   BS_U16, 1.0,   0, BS_SENT_NONE,
      BS_NEEDS_NOTHING, BS_GROUP_HOUSEKEEPING },
    { "tcmd_pending", "telecommands pending",   "",
      B(pending_queued_tcmd_count), BS_U16, 1.0,   0, BS_SENT_NONE,
      BS_NEEDS_NOTHING, BS_GROUP_HOUSEKEEPING },
    { "op_state",     "operation state",        "",
      B(cts1_operation_state),      BS_U8,  1.0,   0, BS_SENT_NONE,
      BS_NEEDS_NOTHING, BS_GROUP_HOUSEKEEPING },
    { "antenna",      "active antenna",         "",
      B(active_rf_switch_antenna),  BS_U8,  1.0,   0, BS_SENT_NONE,
      BS_NEEDS_NOTHING, BS_GROUP_HOUSEKEEPING },
    { "osc",          "active oscillator",      "MHz",
      X(obc_active_oscillator_MHz), BS_U8,  1.0,   1, BS_SENT_NONE,
      BS_NEEDS_NOTHING, BS_GROUP_HOUSEKEEPING },
    { "obc_batt_v",   "battery voltage, OBC ADC", "V",
      X(obc_adc_battery_voltage_mV), BS_I16, 0.001, 1, BS_SENT_NONE,
      BS_NEEDS_NOTHING, BS_GROUP_HOUSEKEEPING },

    // ---- the solar array, channel by channel (extended only) ----
    { "solar0_v",     "solar channel 0 in",     "V",
      X(eps_pcu_ch0_volt_in_mppt_mV), BS_I16, 0.001, 1, BS_SENT_EPS_I16,
      BS_NEEDS_NOTHING, BS_GROUP_SOLAR },
    { "solar0_i",     "solar channel 0 current in", "mA",
      X(eps_pcu_ch0_curr_in_mppt_mA), BS_I16, 1.0,   1, BS_SENT_EPS_I16,
      BS_NEEDS_NOTHING, BS_GROUP_SOLAR },
    { "solar1_v",     "solar channel 1 in",     "V",
      X(eps_pcu_ch1_volt_in_mppt_mV), BS_I16, 0.001, 1, BS_SENT_EPS_I16,
      BS_NEEDS_NOTHING, BS_GROUP_SOLAR },
    { "solar1_i",     "solar channel 1 current in", "mA",
      X(eps_pcu_ch1_curr_in_mppt_mA), BS_I16, 1.0,   1, BS_SENT_EPS_I16,
      BS_NEEDS_NOTHING, BS_GROUP_SOLAR },
    { "solar2_v",     "solar channel 2 in",     "V",
      X(eps_pcu_ch2_volt_in_mppt_mV), BS_I16, 0.001, 1, BS_SENT_EPS_I16,
      BS_NEEDS_NOTHING, BS_GROUP_SOLAR },
    { "solar2_i",     "solar channel 2 current in", "mA",
      X(eps_pcu_ch2_curr_in_mppt_mA), BS_I16, 1.0,   1, BS_SENT_EPS_I16,
      BS_NEEDS_NOTHING, BS_GROUP_SOLAR },
    { "solar3_v",     "solar channel 3 in",     "V",
      X(eps_pcu_ch3_volt_in_mppt_mV), BS_I16, 0.001, 1, BS_SENT_EPS_I16,
      BS_NEEDS_NOTHING, BS_GROUP_SOLAR },
    { "solar3_i",     "solar channel 3 current in", "mA",
      X(eps_pcu_ch3_curr_in_mppt_mA), BS_I16, 1.0,   1, BS_SENT_EPS_I16,
      BS_NEEDS_NOTHING, BS_GROUP_SOLAR },
    { "net_batt_w",   "net battery power, mean", "W",
      X(eps_total_avg_net_battery_power_cW), BS_I16, 0.01, 1, BS_SENT_EPS_I16,
      BS_NEEDS_NOTHING, BS_GROUP_SOLAR },
    { "distrib_w",    "power distributed, mean", "W",
      X(eps_total_avg_power_distributed_cW), BS_I16, 0.01, 1, BS_SENT_EPS_I16,
      BS_NEEDS_NOTHING, BS_GROUP_SOLAR },
    { "pack_status",  "battery pack status",    "",
      X(eps_battery_pack_status_bitfield), BS_U16, 1.0, 1, BS_SENT_PACK,
      BS_NEEDS_NOTHING, BS_GROUP_SOLAR },

    // ---- the ADCS (extended only) ----
    { "est_mode",     "estimation mode",        "",
      X(adcs_current_state_1), BS_ADCS_EST_MODE,  1.0, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ADCS },
    { "ctrl_mode",    "control mode",           "",
      X(adcs_current_state_1), BS_ADCS_CTRL_MODE, 1.0, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ADCS },
    { "run_mode",     "ADCS run mode",          "",
      X(adcs_current_state_1), BS_ADCS_RUN_MODE,  1.0, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ADCS },
    { "sun_up",       "Sun above the horizon",  "",
      X(adcs_current_state_1), BS_ADCS_SUN_UP,    1.0, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ADCS },
    { "adcs_faults",  "ADCS flags set",         "",
      X(adcs_current_state_1), BS_ADCS_FAULT_COUNT, 1.0, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ADCS },
    { "mag_x",        "magnetic field X, body", "uT",
      X(adcs_magnetic_field_x_T_en8), BS_I16, 0.01, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ADCS },
    { "mag_y",        "magnetic field Y, body", "uT",
      X(adcs_magnetic_field_y_T_en8), BS_I16, 0.01, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ADCS },
    { "mag_z",        "magnetic field Z, body", "uT",
      X(adcs_magnetic_field_z_T_en8), BS_I16, 0.01, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ADCS },
    { "mag_norm",     "field magnitude",        "uT",
      X(adcs_magnetic_field_x_T_en8), BS_MAG_MAGNITUDE, 0.01, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ADCS },
    { "css1",         "coarse sun sensor 1",    "",
      X(adcs_raw_css_1), BS_U8, 1.0, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ADCS },
    { "css2",         "coarse sun sensor 2",    "",
      X(adcs_raw_css_2), BS_U8, 1.0, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ADCS },
    { "css3",         "coarse sun sensor 3",    "",
      X(adcs_raw_css_3), BS_U8, 1.0, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ADCS },
    { "css4",         "coarse sun sensor 4",    "",
      X(adcs_raw_css_4), BS_U8, 1.0, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ADCS },
    { "css5",         "coarse sun sensor 5",    "",
      X(adcs_raw_css_5), BS_U8, 1.0, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ADCS },
    { "css6",         "coarse sun sensor 6",    "",
      X(adcs_raw_css_6), BS_U8, 1.0, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ADCS },
    { "css7",         "coarse sun sensor 7",    "",
      X(adcs_raw_css_7), BS_U8, 1.0, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ADCS },
    { "css9",         "coarse sun sensor 9",    "",
      X(adcs_raw_css_9), BS_U8, 1.0, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ADCS },

    // ---- how it was turning and where it was facing ----
    { "rate_norm",    "body rate, MEMS",        "deg/s",
      X(adcs_angular_rate_norm_cdeg_per_sec), BS_U16, 0.01, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ATTITUDE },
    { "rate_x",       "estimated rate X",       "deg/s",
      X(adcs_estimated_rate_x_cdeg_per_sec), BS_I16, 0.01, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ATTITUDE },
    { "rate_y",       "estimated rate Y",       "deg/s",
      X(adcs_estimated_rate_y_cdeg_per_sec), BS_I16, 0.01, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ATTITUDE },
    { "rate_z",       "estimated rate Z",       "deg/s",
      X(adcs_estimated_rate_z_cdeg_per_sec), BS_I16, 0.01, 1, BS_SENT_NONE,
      BS_NEEDS_ADCS, BS_GROUP_ATTITUDE },
    { "roll",         "roll",                   "deg",
      X(adcs_estimated_roll_angle_cdeg),  BS_I16, 0.01, 1, BS_SENT_NONE,
      BS_NEEDS_ATTITUDE, BS_GROUP_ATTITUDE },
    { "pitch",        "pitch",                  "deg",
      X(adcs_estimated_pitch_angle_cdeg), BS_I16, 0.01, 1, BS_SENT_NONE,
      BS_NEEDS_ATTITUDE, BS_GROUP_ATTITUDE },
    { "yaw",          "yaw",                    "deg",
      X(adcs_estimated_yaw_angle_cdeg),   BS_I16, 0.01, 1, BS_SENT_NONE,
      BS_NEEDS_ATTITUDE, BS_GROUP_ATTITUDE },
};

const size_t BEACON_SERIES_FIELD_N =
    sizeof BEACON_SERIES_FIELDS / sizeof BEACON_SERIES_FIELDS[0];

#undef B
#undef X

const char *beacon_series_group_name(bs_group_t g)
{
    switch (g) {
        case BS_GROUP_POWER:        return "Power";
        case BS_GROUP_THERMAL:      return "Temperature";
        case BS_GROUP_SOLAR:        return "Solar array";
        case BS_GROUP_HOUSEKEEPING: return "Housekeeping";
        case BS_GROUP_ADCS:         return "ADCS";
        case BS_GROUP_ATTITUDE:     return "Attitude";
        default:                    return "?";
    }
}

// The plain integer reads. Little-endian throughout.
static double read_u(const uint8_t *p, int n)
{
    uint64_t v = 0;
    for (int i = n - 1; i >= 0; i--) v = (v << 8) | p[i];
    return (double) v;
}

static double read_i(const uint8_t *p, int n)
{
    uint64_t v = 0;
    for (int i = n - 1; i >= 0; i--) v = (v << 8) | p[i];
    // Sign-extend from n bytes.
    const uint64_t sign = (uint64_t) 1 << (n * 8 - 1);
    if (v & sign) return (double) ((int64_t) (v | ~(sign * 2 - 1)));
    return (double) v;
}

// Is the raw value one of the subsystem's "no reading" markers?
static int is_sentinel(bs_sentinel_t kind, double raw)
{
    switch (kind) {
        case BS_SENT_NONE:   return 0;
        case BS_SENT_EPS_I16: return raw == -9999.0;
        case BS_SENT_EPS_I32: return raw == -99999.0;
        case BS_SENT_TEMP_I16:
            // The flight firmware pegs a battery thermistor it cannot
            // read at INT16_MAX; the blob leaves it at -9999.
            return raw == (double) INT16_MAX || raw == (double) INT16_MIN
                || raw == -9999.0;
        case BS_SENT_TEMP_I32:
            return raw == (double) INT32_MAX || raw == (double) INT32_MIN;
        case BS_SENT_MPI_TEMP:
            return raw == -99.0 || raw == -98.0 || raw == -97.0;
        case BS_SENT_PACK:   return raw == 65535.0;
        case BS_SENT_ZERO:   return raw == 0.0;
        case BS_SENT_NEG1:   return raw == -1.0;
        case BS_SENT_U8_MAX: return raw == 255.0;
    }
    return 0;
}

int beacon_series_value(const bs_field_t *f,
                        const uint8_t *payload, size_t len,
                        double *out)
{
    if (f == NULL || payload == NULL || out == NULL) return 0;

    const int is_ext   = beacon_is_extended(payload, len);
    const int is_basic = beacon_is_basic(payload, len);
    if (!is_ext && !is_basic) return 0;
    if (f->ext_only && !is_ext) return 0;

    // No bounds check past this point, and it is the two tests above
    // that make one unnecessary. Both sniffs are length-gated -- a
    // basic beacon is 130 bytes or 134 with the CSP CRC32 trailer, an
    // extended one 198 or 202 -- and every offset in the table comes
    // from a struct through offsetof. So a field that is not ext_only
    // ends by byte 130 and any accepted payload is at least that long,
    // and an ext_only field ends by byte 198 on a payload the ext_only
    // test has already established is at least 198.
    //
    // The ext_only test earns its keep on the 134-byte basic beacon in
    // particular: the extended beacon's first three extra fields start
    // at byte 130, exactly where that packet's CRC32 trailer sits, so
    // without it the oscillator frequency and the OBC's battery reading
    // would be read out of a checksum.
    const uint8_t *p = payload + f->off;

    // Preconditions that depend on the ADCS having answered.
    if (f->needs != BS_NEEDS_NOTHING) {
        COMMS_beacon_extended_packet_t b;
        memcpy(&b, payload, sizeof b);
        beacon_ext_adcs_state_t st;
        beacon_ext_adcs_state(b.adcs_current_state_1, &st);
        if (!st.reported) return 0;
        if (f->needs == BS_NEEDS_ATTITUDE
            && !beacon_ext_attitude_is_valid(payload, len)) return 0;
    }

    double raw = 0.0;
    switch (f->type) {
        case BS_U8:  raw = read_u(p, 1); break;
        case BS_I8:  raw = read_i(p, 1); break;
        case BS_U16: raw = read_u(p, 2); break;
        case BS_I16: raw = read_i(p, 2); break;
        case BS_U32: raw = read_u(p, 4); break;
        case BS_I32: raw = read_i(p, 4); break;

        case BS_MAG_MAGNITUDE: {
            const double x = read_i(p, 2);
            const double y = read_i(p + 2, 2);
            const double z = read_i(p + 4, 2);
            raw = sqrt(x * x + y * y + z * z);
            break;
        }

        case BS_ADCS_EST_MODE:
        case BS_ADCS_CTRL_MODE:
        case BS_ADCS_RUN_MODE:
        case BS_ADCS_SUN_UP:
        case BS_ADCS_FAULT_COUNT: {
            beacon_ext_adcs_state_t st;
            beacon_ext_adcs_state(p, &st);
            if (!st.reported) return 0;
            if (f->type == BS_ADCS_EST_MODE)      raw = st.estimation_mode;
            else if (f->type == BS_ADCS_CTRL_MODE) raw = st.control_mode;
            else if (f->type == BS_ADCS_RUN_MODE)  raw = st.run_mode;
            else if (f->type == BS_ADCS_SUN_UP)    raw = st.sun_above_local_horizon;
            else {
                // Every flag the ADCS reports as wrong, counted. One
                // series that says "something is off" is more use on a
                // four-month plot than forty that each say which.
                const int flags[] = {
                    st.cubesense1_comm_error, st.cubesense2_comm_error,
                    st.cubecontrol_signal_comm_error,
                    st.cubecontrol_motor_comm_error,
                    st.cubewheel1_comm_error, st.cubewheel2_comm_error,
                    st.cubewheel3_comm_error, st.cubestar_comm_error,
                    st.magnetometer_range_error,
                    st.cam1_sram_overcurrent_detected,
                    st.cam1_3v3_overcurrent_detected,
                    st.cam1_sensor_busy_error, st.cam1_sensor_detection_error,
                    st.sun_sensor_range_error,
                    st.cam2_sram_overcurrent_detected,
                    st.cam2_3v3_overcurrent_detected,
                    st.cam2_sensor_busy_error, st.cam2_sensor_detection_error,
                    st.nadir_sensor_range_error, st.rate_sensor_range_error,
                    st.wheel_speed_range_error, st.coarse_sun_sensor_error,
                    st.startracker_match_error,
                    st.startracker_overcurrent_detected,
                };
                int n = 0;
                for (size_t i = 0; i < sizeof flags / sizeof flags[0]; i++)
                    n += (flags[i] != 0);
                raw = n;
            }
            break;
        }
        default: return 0;
    }

    if (is_sentinel(f->sentinel, raw)) return 0;
    *out = raw * f->scale;
    return 1;
}

const bs_field_t *beacon_series_find(const char *key)
{
    if (key == NULL) return NULL;
    for (size_t i = 0; i < BEACON_SERIES_FIELD_N; i++) {
        if (strcmp(BEACON_SERIES_FIELDS[i].key, key) == 0)
            return &BEACON_SERIES_FIELDS[i];
    }
    return NULL;
}
