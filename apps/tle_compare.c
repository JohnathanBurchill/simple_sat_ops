/*

    Simple Satellite Operations  tle_compare.c

    Live side-by-side comparison of two or more catalog objects from one
    TLE file. Built to answer a single question after a deployment: which
    of the several catalog entries is actually ours, when one decoy sits
    very close to where we think our satellite is.

    For each named object it shows the same ephemeris simple_sat_ops
    shows (az, el, range, range-rate, sub-point, Doppler) plus the next
    pass to the nearest second, and stacks the objects one line below the
    other inside each section so the numbers line up and the differences
    jump out. A SEPARATION block reduces it to the handful of numbers a
    real pass can confirm: the angular gap on the sky, the range and
    Doppler differences, and how far apart the two AOS times fall.

    Read-only. No hardware, no IPC, no transmit. Build needs ncurses and
    the SGP4SDP4 library; it does not need ALSA or UHD, so it runs on any
    host that can build next_in_queue.

    Copyright (C) 2026  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#include "prediction.h"

#include <sgp4sdp4.h>
#include <ncurses.h>

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define MAX_OBJECTS        8
#define LIGHT_KM_S         299792.458
#define GM_KM3_S2          398600.4418   // Earth standard gravitational parameter
#define EARTH_RADIUS_KM    6378.137      // WGS84 equatorial radius
#define DEFAULT_FREQ_MHZ   436.150
#define DEFAULT_WINDOW_MIN 1440.0   // 24 h forward search for the next pass
#define NAME_COL           16       // table label width
#define PASS_REFRESH_S     10       // recompute next-pass cadence (seconds)

typedef struct {
    char         name[64];      // requested name prefix, matched in the file
    prediction_t pred;          // observer + TLE + per-tick outputs
    int          loaded;        // load_tle succeeded

    // Live, refreshed every second.
    double az, el, range_km, rr_km_s, lat, lon, alt_km, speed_km_s;
    double doppler_hz;

    // Next pass, refreshed on the PASS_REFRESH_S cadence.
    int    has_pass;            // a pass was found in the window
    int    in_pass;             // currently above the horizon
    double aos_jul, los_jul;    // crossings, to ~1 s
    double max_el, aos_az, dur_min;

    // select_ephemeris() converts the TLE units in place and must run
    // exactly once. We run it once at setup, capture the converted
    // elements and the deep-space verdict here, and restore from this
    // copy before each propagation so switching between objects never
    // re-converts (and thus corrupts) the elements.
    tle_t  tle_ready;
    int    deep_space;

    // Orbit shape from the elements (constant for the loaded TLE),
    // computed once at setup. Apogee/perigee as altitudes above the
    // mean Earth radius (standard catalog convention).
    double apogee_km, perigee_km;

    // TLE epoch as a Julian date (the instant the elements are valid).
    double epoch_jul;

    // Worst-case on-sky angle (deg) from object 0 over object 0's next
    // pass; the pointing error if we tracked object 0 but this object
    // was really our bird. -1 until computed (object 0 itself: unused).
    double pass_max_sep_deg;

    // Signed along-track time offset vs object 0 (seconds): the time for
    // this object to reach object 0's current track point (+ = trails,
    // - = leads), min and max sampled over object 0's full orbit.
    double dtsec_min, dtsec_max;
    int    dtsec_valid;

    // Multi-day drift of the orbit-mean along-track offset: a linear fit
    // over a window centred on now (-trend_days .. +trend_days). _start
    // is the fitted offset trend_days ago, _now at present, _end
    // trend_days ahead, _rate the slope (s/day).
    double trend_start_s, trend_now_s, trend_end_s, trend_rate_s_per_day;
    int    trend_valid;
} obj_t;

static volatile sig_atomic_t g_quit = 0;
static void on_signal(int sig) { (void) sig; g_quit = 1; }

// Window (days) for the along-track drift trend; set from --trend-days.
static double g_trend_days = 3.0;

// ---- time helpers ----------------------------------------------------------

static double now_jul_utc(void)
{
    struct tm utc;
    struct timeval tv;
    UTC_Calendar_Now(&utc, &tv);
    return Julian_Date(&utc, &tv);
}

static time_t jul_to_unix(double jul_utc)
{
    return (time_t) ((jul_utc - 2440587.5) * 86400.0 + 0.5);
}

static void fmt_clock_local(double jul_utc, char *buf, size_t n)
{
    time_t t = jul_to_unix(jul_utc);
    struct tm lt;
    if (localtime_r(&t, &lt) == NULL) { snprintf(buf, n, "--:--:--"); return; }
    strftime(buf, n, "%H:%M:%S %Z", &lt);
}

static void fmt_utc_datetime(double jul_utc, char *buf, size_t n)
{
    time_t t = jul_to_unix(jul_utc);
    struct tm ut;
    if (gmtime_r(&t, &ut) == NULL) { snprintf(buf, n, "----------"); return; }
    strftime(buf, n, "%Y-%m-%d %H:%M:%SZ", &ut);
}

static void fmt_hms(double secs, char *buf, size_t n)
{
    long s = (long) (secs + 0.5);
    // Clamp after the cast so the compiler can bound the field widths
    // (it cannot track ranges across a double-to-long cast). 0..99:59:59.
    if (s < 0L) s = 0L;
    if (s > 359999L) s = 359999L;
    long h = s / 3600; s %= 3600;
    long m = s / 60;   s %= 60;
    snprintf(buf, n, "%02ld:%02ld:%02ld", h, m, s);
}

static void fmt_ms(double secs, char *buf, size_t n)
{
    long s = (long) (secs + 0.5);
    if (s < 0L) s = 0L;                     // bound both ends so the
    if (s > 59999L) s = 59999L;             // field widths are provable (999:59)
    long m = s / 60; s %= 60;
    snprintf(buf, n, "%ld:%02ld", m, s);
}

// ---- geometry --------------------------------------------------------------

// Angular separation between two look directions given as (az, el) in
// degrees, returned in degrees. This is the single most telling number
// when two objects might be the same: when it is small they are in the
// same patch of sky right now.
static double separation_deg(double az1, double el1, double az2, double el2)
{
    double a1 = az1 * M_PI / 180.0, e1 = el1 * M_PI / 180.0;
    double a2 = az2 * M_PI / 180.0, e2 = el2 * M_PI / 180.0;
    double x1 = cos(e1) * cos(a1), y1 = cos(e1) * sin(a1), z1 = sin(e1);
    double x2 = cos(e2) * cos(a2), y2 = cos(e2) * sin(a2), z2 = sin(e2);
    double d = x1 * x2 + y1 * y2 + z1 * z2;
    if (d > 1.0) d = 1.0;
    if (d < -1.0) d = -1.0;
    return acos(d) * 180.0 / M_PI;
}

// ---- propagation -----------------------------------------------------------

// One-time per object: convert the loaded elements with select_ephemeris
// (which rewrites the tle units in place and decides SGP4 vs SDP4), then
// stash the converted copy and the deep-space verdict. Must be called
// exactly once after load_tle, before any prep_object/propagation.
static void setup_object(obj_t *o)
{
    ClearFlag(ALL_FLAGS);
    select_ephemeris(&o->pred.satellite_ephem.tle);
    o->deep_space = isFlagSet(DEEP_SPACE_EPHEM_FLAG) ? 1 : 0;
    o->tle_ready  = o->pred.satellite_ephem.tle;

    // Apogee / perigee from the mean elements. select_ephemeris has left
    // xno in rad/min; the Kepler semi-major axis from the mean motion is
    // a = (GM / n^2)^(1/3) with n in rad/s. Report apogee/perigee as
    // altitudes above the mean Earth radius: a(1 +/- e) - R_earth.
    double e = o->tle_ready.eo;
    double n_rad_s = o->tle_ready.xno / 60.0;
    if (n_rad_s > 0.0) {
        double a_km = cbrt(GM_KM3_S2 / (n_rad_s * n_rad_s));
        o->apogee_km  = a_km * (1.0 + e) - EARTH_RADIUS_KM;
        o->perigee_km = a_km * (1.0 - e) - EARTH_RADIUS_KM;
    }

    // tle.epoch is the raw YYDDD.ddddd field (select_ephemeris leaves it
    // alone); Julian_Date_of_Epoch turns it into a Julian date.
    o->epoch_jul = Julian_Date_of_Epoch(o->tle_ready.epoch);
}

// sgp4sdp4 keeps the chosen ephemeris and its init state in module-level
// globals, so when more than one satellite shares the process we must
// reset before propagating each one. We restore the once-converted
// elements and re-assert the deep-space flag WITHOUT calling
// select_ephemeris again (it is not idempotent: a second call would
// re-scale the elements and produce a garbage orbit). ClearFlag also
// drops SGP4/SDP4_INITIALIZED so the propagator re-derives its constants
// for this object.
static void prep_object(obj_t *o)
{
    o->pred.satellite_ephem.tle = o->tle_ready;
    ClearFlag(ALL_FLAGS);
    if (o->deep_space) SetFlag(DEEP_SPACE_EPHEM_FLAG);
}

static double el_at(prediction_t *p, double jul)
{
    update_satellite_position(p, jul);
    return p->satellite_ephem.elevation;
}

// Bisect a bracketed horizon crossing down to ~0.5 s. el(t_lo) and
// el(t_hi) are assumed to straddle zero.
static double refine_cross(prediction_t *p, double t_lo, double t_hi)
{
    double e_lo = el_at(p, t_lo);
    for (int i = 0; i < 48 && (t_hi - t_lo) * 86400.0 > 0.5; ++i) {
        double tm = 0.5 * (t_lo + t_hi);
        double e = el_at(p, tm);
        if ((e < 0.0) == (e_lo < 0.0)) { t_lo = tm; e_lo = e; }
        else                           { t_hi = tm; }
    }
    return 0.5 * (t_lo + t_hi);
}

// Find the current or next pass, AOS/LOS to the second. prep_object()
// must have run for this object first. Leaves the prediction propagated
// at an arbitrary time; the caller restores live state on the next tick.
static void find_pass(obj_t *o, double jul_now, double window_min)
{
    prediction_t *p = &o->pred;
    o->has_pass = 0;
    o->in_pass = 0;

    const double step = 30.0 / 86400.0;          // 30 s coarse step
    const double end  = jul_now + window_min / 1440.0;

    double e_now = el_at(p, jul_now);
    double aos = 0.0, los = 0.0;

    if (e_now >= 0.0) {
        // Already up. AOS is in the past; walk back to it (cap 40 min).
        o->in_pass = 1;
        double tb = jul_now, eb = e_now;
        double back_limit = jul_now - 40.0 / 1440.0;
        while (eb >= 0.0 && tb > back_limit) { tb -= step; eb = el_at(p, tb); }
        aos = (eb < 0.0) ? refine_cross(p, tb, tb + step) : tb;

        double tf = jul_now, ef = e_now;
        while (ef >= 0.0 && tf < end) { tf += step; ef = el_at(p, tf); }
        los = (ef < 0.0) ? refine_cross(p, tf - step, tf) : tf;
        o->has_pass = 1;
    } else {
        // Below horizon: scan forward for the next AOS.
        double t = jul_now, e_prev = e_now;
        while (t < end) {
            double e = el_at(p, t + step);
            if (e_prev < 0.0 && e >= 0.0) {
                aos = refine_cross(p, t, t + step);
                double tf = aos, ef = 1.0;
                while (ef >= 0.0 && tf < end) { tf += step; ef = el_at(p, tf); }
                los = (ef < 0.0) ? refine_cross(p, tf - step, tf) : tf;
                o->has_pass = 1;
                break;
            }
            e_prev = e;
            t += step;
        }
        if (!o->has_pass) return;
    }

    // AOS azimuth and the pass peak, sampled at 5 s.
    el_at(p, aos);
    o->aos_az = p->satellite_ephem.azimuth;
    double max_el = -90.0;
    for (double tt = aos; tt <= los; tt += 5.0 / 86400.0) {
        double e = el_at(p, tt);
        if (e > max_el) max_el = e;
    }
    o->aos_jul  = aos;
    o->los_jul  = los;
    o->max_el   = max_el;
    o->dur_min  = (los - aos) * 1440.0;
}

// Maximum topocentric (look-vector) angular separation between two
// objects over [j0, j1], in degrees. This is the on-sky angle the
// antenna would have to swing through if it tracked one object while
// the other was the real target — i.e. the worst-case pointing error
// over the pass. Both predictions are left propagated at an arbitrary
// time; the caller restores live state next tick.
static double max_sep_over(obj_t *a, obj_t *b, double j0, double j1)
{
    double maxsep = -1.0;
    for (double t = j0; t <= j1; t += 5.0 / 86400.0) {   // 5 s steps
        prep_object(a);
        update_satellite_position(&a->pred, t);
        double az1 = a->pred.satellite_ephem.azimuth;
        double el1 = a->pred.satellite_ephem.elevation;
        prep_object(b);
        update_satellite_position(&b->pred, t);
        double az2 = b->pred.satellite_ephem.azimuth;
        double el2 = b->pred.satellite_ephem.elevation;
        double s = separation_deg(az1, el1, az2, el2);
        if (s > maxsep) maxsep = s;
    }
    return maxsep;
}

// Fill each subsequent object's worst-case on-sky separation from the
// first object across the FIRST object's next pass. Requires find_pass
// to have run for all objects already. -1 means not computed.
static void compute_pass_separations(obj_t *objs, int n)
{
    for (int i = 1; i < n; ++i) objs[i].pass_max_sep_deg = -1.0;
    if (n < 2 || !objs[0].loaded || !objs[0].has_pass) return;
    for (int i = 1; i < n; ++i) {
        if (!objs[i].loaded) continue;
        objs[i].pass_max_sep_deg =
            max_sep_over(&objs[0], &objs[i], objs[0].aos_jul, objs[0].los_jul);
    }
}

// ECI position + velocity (km, km/s) at a Julian date, straight from
// SGP4/SDP4. We call the propagator directly rather than going through
// update_satellite_position because that routine overwrites the
// satellite position vector with the observer's (it reuses the field),
// and the along-track time offset below needs the true satellite state.
static void sat_state(obj_t *o, double jul, vector_t *pos, vector_t *vel)
{
    prep_object(o);
    double jul_epoch = Julian_Date_of_Epoch(o->tle_ready.epoch);
    double tsince = (jul - jul_epoch) * 1440.0;
    if (o->deep_space) SDP4(tsince, &o->pred.satellite_ephem.tle, pos, vel);
    else               SGP4(tsince, &o->pred.satellite_ephem.tle, pos, vel);
    Convert_Sat_State(pos, vel);            // canonical units -> km, km/s
}

// Signed along-track time offset (seconds) of object b relative to a at
// one instant: the dt that brings b (moving along its velocity) closest
// to a's current position. dt = (r_a - r_b) . v_b / |v_b|^2. Positive
// means b must advance to reach a's track point, i.e. b trails a.
static double along_track_dtsec(const vector_t *ra,
                                const vector_t *rb, const vector_t *vb)
{
    double vv = vb->x * vb->x + vb->y * vb->y + vb->z * vb->z;
    if (vv <= 0.0) return 0.0;
    double dot = (ra->x - rb->x) * vb->x
               + (ra->y - rb->y) * vb->y
               + (ra->z - rb->z) * vb->z;
    return dot / vv;
}

// For each subsequent object, the min and max signed along-track time
// offset vs object 0, sampled across object 0's full orbital period.
static void compute_orbit_dtsec(obj_t *objs, int n, double jul_now)
{
    for (int i = 1; i < n; ++i) objs[i].dtsec_valid = 0;
    if (n < 2 || !objs[0].loaded) return;
    double xno = objs[0].tle_ready.xno;             // rad/min
    if (!(xno > 0.0)) return;
    double period_days = (2.0 * M_PI / xno) / 1440.0;

    const int N = 600;
    double step = period_days / (double) N;
    for (int i = 1; i < n; ++i) {
        if (!objs[i].loaded) continue;
        double mn = 1e30, mx = -1e30;
        for (int k = 0; k <= N; ++k) {
            double t = jul_now + (double) k * step;
            vector_t pa, va, pb, vb;
            sat_state(&objs[0], t, &pa, &va);
            sat_state(&objs[i], t, &pb, &vb);
            double dt = along_track_dtsec(&pa, &pb, &vb);
            if (dt < mn) mn = dt;
            if (dt > mx) mx = dt;
        }
        objs[i].dtsec_min = mn;
        objs[i].dtsec_max = mx;
        objs[i].dtsec_valid = (mx >= mn);
    }
}

// Mean signed along-track offset (seconds) of b vs a over one orbit of
// a starting at jul. Averaging across the orbit removes the within-orbit
// wobble and leaves the secular offset that drifts day to day.
static double orbit_mean_dtsec(obj_t *a, obj_t *b, double jul, double period_days)
{
    const int K = 48;
    double sum = 0.0;
    for (int k = 0; k < K; ++k) {
        double t = jul + period_days * (double) k / (double) K;
        vector_t pa, va, pb, vb;
        sat_state(a, t, &pa, &va);
        sat_state(b, t, &pb, &vb);
        sum += along_track_dtsec(&pa, &pb, &vb);
    }
    return sum / (double) K;
}

// For each subsequent object, linear-fit the orbit-mean along-track
// offset vs object 0 over a window centred on now (-days .. +days), so
// we can see whether it has been and is drifting apart or closing on
// average. Fills trend_start/now/end/rate.
static void compute_alongtrack_trend(obj_t *objs, int n, double jul_now, double days)
{
    for (int i = 1; i < n; ++i) objs[i].trend_valid = 0;
    if (n < 2 || !objs[0].loaded || days <= 0.0) return;
    double xno = objs[0].tle_ready.xno;             // rad/min
    if (!(xno > 0.0)) return;
    double period_days = (2.0 * M_PI / xno) / 1440.0;

    // Sample the actual orbit-mean offset at -days, now, and +days rather
    // than least-squares fitting: for a fast-drifting object the offset is
    // not linear over the window, and a fit intercept would disagree with
    // the instantaneous value. The endpoints give an honest average rate.
    for (int i = 1; i < n; ++i) {
        if (!objs[i].loaded) continue;
        double ys = orbit_mean_dtsec(&objs[0], &objs[i], jul_now - days, period_days);
        double yn = orbit_mean_dtsec(&objs[0], &objs[i], jul_now,        period_days);
        double ye = orbit_mean_dtsec(&objs[0], &objs[i], jul_now + days, period_days);
        objs[i].trend_start_s        = ys;
        objs[i].trend_now_s          = yn;
        objs[i].trend_end_s          = ye;
        objs[i].trend_rate_s_per_day = (ye - ys) / (2.0 * days);
        objs[i].trend_valid          = 1;
    }
}

// Verdict over the trend window from its endpoints. A sign flip means the
// objects pass through coincidence inside the window ("crossing");
// otherwise compare the magnitude of the offset at the two ends.
static const char *trend_word(double start_s, double end_s)
{
    if ((start_s > 0.0) != (end_s > 0.0)
        && fabs(start_s) > 1.0 && fabs(end_s) > 1.0)
        return "crossing";
    double d = fabs(end_s) - fabs(start_s);
    if (d >  1.0) return "separating";
    if (d < -1.0) return "closing";
    return "stable";
}

// Propagate one object to jul_now and store the live ephemeris + Doppler.
// prep_object() must have run for this object first.
static void compute_live(obj_t *o, double jul_now, double freq_hz)
{
    update_satellite_position(&o->pred, jul_now);
    ephemeres_t *e = &o->pred.satellite_ephem;
    o->az = e->azimuth;       o->el = e->elevation;
    o->range_km = e->range_km; o->rr_km_s = e->range_rate_km_s;
    o->lat = e->latitude;     o->lon = e->longitude;
    o->alt_km = e->altitude_km; o->speed_km_s = e->speed_km_s;
    o->doppler_hz = -(e->range_rate_km_s / LIGHT_KM_S) * freq_hz;
}

// ---- drawing ---------------------------------------------------------------

static void draw(obj_t *objs, int n, double jul_now, double freq_hz,
                 const char *tle_path, double lat, double lon, double alt_m)
{
    char utc_s[32], loc_s[40];
    time_t tnow = jul_to_unix(jul_now);
    struct tm ug;
    gmtime_r(&tnow, &ug);
    strftime(utc_s, sizeof utc_s, "%Y-%m-%d %H:%M:%SZ", &ug);
    fmt_clock_local(jul_now, loc_s, sizeof loc_s);

    erase();
    int row = 0;
    mvprintw(row++, 0, "tle_compare    UTC %s    local %s", utc_s, loc_s);
    mvprintw(row++, 0, "TLE %s    obs %.4f, %.4f  %.0f m    Doppler @ %.4f MHz",
             tle_path, lat, lon, alt_m, freq_hz / 1e6);
    row++;

    // --- TLE EPOCH ---
    mvprintw(row++, 0, "TLE EPOCH        %-20s %8s", "epoch (UTC)", "age (d)");
    for (int i = 0; i < n; ++i) {
        obj_t *o = &objs[i];
        if (!o->loaded) { row++; continue; }
        char ep[24];
        fmt_utc_datetime(o->epoch_jul, ep, sizeof ep);
        mvprintw(row++, 0, "%d %-*.*s %-20s %8.1f",
                 i + 1, NAME_COL, NAME_COL, o->name, ep, jul_now - o->epoch_jul);
    }
    row++;

    // --- SKY NOW ---
    mvprintw(row++, 0, "SKY NOW          %8s %7s %10s %11s",
             "az", "el", "range km", "rr km/s");
    for (int i = 0; i < n; ++i) {
        obj_t *o = &objs[i];
        if (!o->loaded) { mvprintw(row++, 0, "%d %-*s  (not found)", i + 1, NAME_COL, o->name); continue; }
        mvprintw(row++, 0, "%d %-*.*s %8.2f %7.2f %10.1f %11.3f",
                 i + 1, NAME_COL, NAME_COL, o->name,
                 o->az, o->el, o->range_km, o->rr_km_s);
    }
    row++;

    // --- SUB-POINT ---
    mvprintw(row++, 0, "SUB-POINT        %8s %7s %10s %11s",
             "lat", "lon", "alt km", "speed km/s");
    for (int i = 0; i < n; ++i) {
        obj_t *o = &objs[i];
        if (!o->loaded) { row++; continue; }
        mvprintw(row++, 0, "%d %-*.*s %8.2f %7.2f %10.1f %11.3f",
                 i + 1, NAME_COL, NAME_COL, o->name,
                 o->lat, o->lon, o->alt_km, o->speed_km_s);
    }
    row++;

    // --- ORBIT (apogee / perigee altitude) ---
    mvprintw(row++, 0, "ORBIT            %11s %11s", "apogee km", "perigee km");
    for (int i = 0; i < n; ++i) {
        obj_t *o = &objs[i];
        if (!o->loaded) { row++; continue; }
        mvprintw(row++, 0, "%d %-*.*s %11.1f %11.1f",
                 i + 1, NAME_COL, NAME_COL, o->name, o->apogee_km, o->perigee_km);
    }
    row++;

    // --- DOPPLER ---
    mvprintw(row++, 0, "DOPPLER          %10s %14s",
             "shift kHz", "downlink MHz");
    for (int i = 0; i < n; ++i) {
        obj_t *o = &objs[i];
        if (!o->loaded) { row++; continue; }
        mvprintw(row++, 0, "%d %-*.*s %10.3f %14.5f",
                 i + 1, NAME_COL, NAME_COL, o->name,
                 o->doppler_hz / 1e3, (freq_hz + o->doppler_hz) / 1e6);
    }
    row++;

    // --- NEXT PASS ---
    mvprintw(row++, 0, "NEXT PASS        %12s %12s %8s %7s %8s",
             "AOS local", "in / LOS in", "max el", "asc az", "dur m:s");
    for (int i = 0; i < n; ++i) {
        obj_t *o = &objs[i];
        if (!o->loaded) { row++; continue; }
        if (!o->has_pass) {
            mvprintw(row++, 0, "%d %-*.*s   (no pass in window)",
                     i + 1, NAME_COL, NAME_COL, o->name);
            continue;
        }
        char aos_c[40], cd[16], dur[16];
        fmt_clock_local(o->aos_jul, aos_c, sizeof aos_c);
        fmt_ms(o->dur_min * 60.0, dur, sizeof dur);
        if (o->in_pass) {
            fmt_hms((o->los_jul - jul_now) * 86400.0, cd, sizeof cd);
            mvprintw(row++, 0, "%d %-*.*s %12s %12s %8.1f %7.1f %8s  IN PASS",
                     i + 1, NAME_COL, NAME_COL, o->name,
                     aos_c, cd, o->max_el, o->aos_az, dur);
        } else {
            fmt_hms((o->aos_jul - jul_now) * 86400.0, cd, sizeof cd);
            mvprintw(row++, 0, "%d %-*.*s %12s %12s %8.1f %7.1f %8s",
                     i + 1, NAME_COL, NAME_COL, o->name,
                     aos_c, cd, o->max_el, o->aos_az, dur);
        }
    }
    row++;

    // --- SEPARATION vs object 1 ---
    if (n >= 2 && objs[0].loaded) {
        obj_t *a = &objs[0];
        mvprintw(row++, 0, "SEPARATION vs 1  %8s %12s %10s %11s %10s",
                 "now deg", "pass-max deg", "range km", "doppler kHz", "AOS dsec");
        for (int i = 1; i < n; ++i) {
            obj_t *b = &objs[i];
            if (!b->loaded) { row++; continue; }
            double sep = separation_deg(a->az, a->el, b->az, b->el);
            double drange = b->range_km - a->range_km;
            double ddopp = (b->doppler_hz - a->doppler_hz) / 1e3;
            char aosd[16] = "--", pmax[16] = "--";
            if (a->has_pass && b->has_pass) {
                snprintf(aosd, sizeof aosd, "%+.0f",
                         (b->aos_jul - a->aos_jul) * 86400.0);
            }
            if (b->pass_max_sep_deg >= 0.0) {
                snprintf(pmax, sizeof pmax, "%.3f", b->pass_max_sep_deg);
            }
            mvprintw(row++, 0, "%d %-*.*s %8.3f %12s %10.1f %11.3f %10s",
                     i + 1, NAME_COL, NAME_COL, b->name,
                     sep, pmax, drange, ddopp, aosd);
        }
        row++;

        // --- ALONG-TRACK time offset over object 0's full orbit ---
        mvprintw(row++, 0, "ALONG-TRACK vs 1 %12s %12s   s, + trails / - leads",
                 "dtsec min", "dtsec max");
        for (int i = 1; i < n; ++i) {
            obj_t *b = &objs[i];
            if (!b->loaded) { row++; continue; }
            if (b->dtsec_valid)
                mvprintw(row++, 0, "%d %-*.*s %12.1f %12.1f",
                         i + 1, NAME_COL, NAME_COL, b->name,
                         b->dtsec_min, b->dtsec_max);
            else
                mvprintw(row++, 0, "%d %-*.*s %12s %12s",
                         i + 1, NAME_COL, NAME_COL, b->name, "--", "--");
        }
        row++;

        // --- DRIFT: along-track offset trend centred on now (+/- days) ---
        char dtitle[40], hstart[24], hend[24];
        snprintf(dtitle, sizeof dtitle, "DRIFT vs 1 (%.1fd)", 2.0 * g_trend_days);
        snprintf(hstart, sizeof hstart, "-%gd s", g_trend_days);
        snprintf(hend,   sizeof hend,   "+%gd s", g_trend_days);
        mvprintw(row++, 0, "%-17s%8s %8s %9s %8s %s",
                 dtitle, hstart, "now s", "rate s/d", hend, "trend");
        for (int i = 1; i < n; ++i) {
            obj_t *b = &objs[i];
            if (!b->loaded) { row++; continue; }
            if (b->trend_valid)
                mvprintw(row++, 0, "%d %-*.*s %8.1f %8.1f %9.2f %8.1f %s",
                         i + 1, NAME_COL, NAME_COL, b->name,
                         b->trend_start_s, b->trend_now_s, b->trend_rate_s_per_day,
                         b->trend_end_s, trend_word(b->trend_start_s, b->trend_end_s));
            else
                mvprintw(row++, 0, "%d %-*.*s %8s",
                         i + 1, NAME_COL, NAME_COL, b->name, "--");
        }
        row++;
    }

    mvprintw(row++, 0, "q quit    (live 1 Hz, next pass every %d s)", PASS_REFRESH_S);
    refresh();
}

// Plain-text equivalent of draw() for --once mode: one snapshot to stdout,
// no ncurses. Useful over a plain SSH session and for logging / scripting.
static void print_text(obj_t *objs, int n, double jul_now, double freq_hz,
                       const char *tle_path, double lat, double lon, double alt_m)
{
    char utc_s[32], loc_s[40];
    time_t tnow = jul_to_unix(jul_now);
    struct tm ug;
    gmtime_r(&tnow, &ug);
    strftime(utc_s, sizeof utc_s, "%Y-%m-%d %H:%M:%SZ", &ug);
    fmt_clock_local(jul_now, loc_s, sizeof loc_s);

    printf("tle_compare    UTC %s    local %s\n", utc_s, loc_s);
    printf("TLE %s    obs %.4f, %.4f  %.0f m    Doppler @ %.4f MHz\n\n",
           tle_path, lat, lon, alt_m, freq_hz / 1e6);

    printf("TLE EPOCH        %-20s %8s\n", "epoch (UTC)", "age (d)");
    for (int i = 0; i < n; ++i) {
        obj_t *o = &objs[i];
        if (!o->loaded) continue;
        char ep[24];
        fmt_utc_datetime(o->epoch_jul, ep, sizeof ep);
        printf("%d %-*.*s %-20s %8.1f\n",
               i + 1, NAME_COL, NAME_COL, o->name, ep, jul_now - o->epoch_jul);
    }

    printf("\nSKY NOW          %8s %7s %10s %11s\n", "az", "el", "range km", "rr km/s");
    for (int i = 0; i < n; ++i) {
        obj_t *o = &objs[i];
        if (!o->loaded) { printf("%d %-*s  (not found)\n", i + 1, NAME_COL, o->name); continue; }
        printf("%d %-*.*s %8.2f %7.2f %10.1f %11.3f\n",
               i + 1, NAME_COL, NAME_COL, o->name, o->az, o->el, o->range_km, o->rr_km_s);
    }
    printf("\nSUB-POINT        %8s %7s %10s %11s\n", "lat", "lon", "alt km", "speed km/s");
    for (int i = 0; i < n; ++i) {
        obj_t *o = &objs[i];
        if (!o->loaded) continue;
        printf("%d %-*.*s %8.2f %7.2f %10.1f %11.3f\n",
               i + 1, NAME_COL, NAME_COL, o->name, o->lat, o->lon, o->alt_km, o->speed_km_s);
    }
    printf("\nORBIT            %11s %11s\n", "apogee km", "perigee km");
    for (int i = 0; i < n; ++i) {
        obj_t *o = &objs[i];
        if (!o->loaded) continue;
        printf("%d %-*.*s %11.1f %11.1f\n",
               i + 1, NAME_COL, NAME_COL, o->name, o->apogee_km, o->perigee_km);
    }
    printf("\nDOPPLER          %10s %14s\n", "shift kHz", "downlink MHz");
    for (int i = 0; i < n; ++i) {
        obj_t *o = &objs[i];
        if (!o->loaded) continue;
        printf("%d %-*.*s %10.3f %14.5f\n",
               i + 1, NAME_COL, NAME_COL, o->name,
               o->doppler_hz / 1e3, (freq_hz + o->doppler_hz) / 1e6);
    }
    printf("\nNEXT PASS        %12s %12s %8s %7s %8s\n",
           "AOS local", "in / LOS in", "max el", "asc az", "dur m:s");
    for (int i = 0; i < n; ++i) {
        obj_t *o = &objs[i];
        if (!o->loaded) continue;
        if (!o->has_pass) { printf("%d %-*.*s   (no pass in window)\n",
                                   i + 1, NAME_COL, NAME_COL, o->name); continue; }
        char aos_c[40], cd[16], dur[16];
        fmt_clock_local(o->aos_jul, aos_c, sizeof aos_c);
        fmt_ms(o->dur_min * 60.0, dur, sizeof dur);
        fmt_hms(((o->in_pass ? o->los_jul : o->aos_jul) - jul_now) * 86400.0, cd, sizeof cd);
        printf("%d %-*.*s %12s %12s %8.1f %7.1f %8s%s\n",
               i + 1, NAME_COL, NAME_COL, o->name, aos_c, cd,
               o->max_el, o->aos_az, dur, o->in_pass ? "  IN PASS" : "");
    }
    if (n >= 2 && objs[0].loaded) {
        obj_t *a = &objs[0];
        printf("\nSEPARATION vs 1  %8s %12s %10s %11s %10s\n",
               "now deg", "pass-max deg", "range km", "doppler kHz", "AOS dsec");
        for (int i = 1; i < n; ++i) {
            obj_t *b = &objs[i];
            if (!b->loaded) continue;
            char aosd[16] = "--", pmax[16] = "--";
            if (a->has_pass && b->has_pass)
                snprintf(aosd, sizeof aosd, "%+.0f", (b->aos_jul - a->aos_jul) * 86400.0);
            if (b->pass_max_sep_deg >= 0.0)
                snprintf(pmax, sizeof pmax, "%.3f", b->pass_max_sep_deg);
            printf("%d %-*.*s %8.3f %12s %10.1f %11.3f %10s\n",
                   i + 1, NAME_COL, NAME_COL, b->name,
                   separation_deg(a->az, a->el, b->az, b->el), pmax,
                   b->range_km - a->range_km,
                   (b->doppler_hz - a->doppler_hz) / 1e3, aosd);
        }
        printf("\nALONG-TRACK vs 1 %12s %12s   (s, + trails / - leads)\n",
               "dtsec min", "dtsec max");
        for (int i = 1; i < n; ++i) {
            obj_t *b = &objs[i];
            if (!b->loaded) continue;
            if (b->dtsec_valid)
                printf("%d %-*.*s %12.1f %12.1f\n",
                       i + 1, NAME_COL, NAME_COL, b->name, b->dtsec_min, b->dtsec_max);
            else
                printf("%d %-*.*s %12s %12s\n",
                       i + 1, NAME_COL, NAME_COL, b->name, "--", "--");
        }
        char dtitle[40], hstart[24], hend[24];
        snprintf(dtitle, sizeof dtitle, "DRIFT vs 1 (%.1fd)", 2.0 * g_trend_days);
        snprintf(hstart, sizeof hstart, "-%gd s", g_trend_days);
        snprintf(hend,   sizeof hend,   "+%gd s", g_trend_days);
        printf("\n%-17s%8s %8s %9s %8s %s\n",
               dtitle, hstart, "now s", "rate s/d", hend, "trend");
        for (int i = 1; i < n; ++i) {
            obj_t *b = &objs[i];
            if (!b->loaded) continue;
            if (b->trend_valid)
                printf("%d %-*.*s %8.1f %8.1f %9.2f %8.1f %s\n",
                       i + 1, NAME_COL, NAME_COL, b->name,
                       b->trend_start_s, b->trend_now_s, b->trend_rate_s_per_day,
                       b->trend_end_s, trend_word(b->trend_start_s, b->trend_end_s));
            else
                printf("%d %-*.*s %8s\n", i + 1, NAME_COL, NAME_COL, b->name, "--");
        }
    }
}

// ---- main ------------------------------------------------------------------

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [options] <name1> <name2> [name3 ...]\n"
        "\n"
        "Compare the live ephemeris and next pass of two or more catalog\n"
        "objects from one TLE file, stacked for easy comparison. Names are\n"
        "matched as case-sensitive prefixes against the TLE name lines.\n"
        "\n"
        "Options:\n"
        "  --tle <path>       TLE file (default: $HOME/.local/state/simple_sat_ops/active.tle)\n"
        "  --lat <deg>        Observer latitude  (default: RAO, %.4f)\n"
        "  --lon <deg>        Observer longitude (default: RAO, %.4f)\n"
        "  --alt <m>          Observer altitude metres (default: RAO, %.0f)\n"
        "  --freq-mhz <MHz>   Carrier for the Doppler column (default: %.3f)\n"
        "  --window-min <min> Forward search window for the next pass (default: %.0f)\n"
        "  --trend-days <d>   Half-window (days) for the drift trend, centred on now,\n"
        "                     so the trend spans -d..+d (default: %.1f, i.e. %.0f d total)\n"
        "  --once             Print one snapshot as plain text and exit (no UI).\n"
        "  --help             This text.\n",
        prog, RAO_LATITUDE, RAO_LONGITUDE, RAO_ALTITUDE,
        DEFAULT_FREQ_MHZ, DEFAULT_WINDOW_MIN, g_trend_days, 2.0 * g_trend_days);
}

// Pull the value for a "--flag value" pair, or NULL if missing.
static const char *take_value(int argc, char **argv, int *i)
{
    if (*i + 1 >= argc) return NULL;
    return argv[++(*i)];
}

int main(int argc, char **argv)
{
    char tle_path[512] = {0};
    double lat = RAO_LATITUDE, lon = RAO_LONGITUDE, alt_m = RAO_ALTITUDE;
    double freq_hz = DEFAULT_FREQ_MHZ * 1e6;
    double window_min = DEFAULT_WINDOW_MIN;
    int once = 0;

    obj_t objs[MAX_OBJECTS];
    memset(objs, 0, sizeof objs);
    int n = 0;

    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i];
        const char *v = NULL;
        if (strcmp(a, "--help") == 0 || strcmp(a, "-h") == 0) {
            usage(argv[0]); return EXIT_SUCCESS;
        } else if (strcmp(a, "--tle") == 0 && (v = take_value(argc, argv, &i))) {
            snprintf(tle_path, sizeof tle_path, "%s", v);
        } else if (strncmp(a, "--tle=", 6) == 0) {
            snprintf(tle_path, sizeof tle_path, "%s", a + 6);
        } else if (strcmp(a, "--lat") == 0 && (v = take_value(argc, argv, &i))) {
            lat = atof(v);
        } else if (strncmp(a, "--lat=", 6) == 0) { lat = atof(a + 6);
        } else if (strcmp(a, "--lon") == 0 && (v = take_value(argc, argv, &i))) {
            lon = atof(v);
        } else if (strncmp(a, "--lon=", 6) == 0) { lon = atof(a + 6);
        } else if (strcmp(a, "--alt") == 0 && (v = take_value(argc, argv, &i))) {
            alt_m = atof(v);
        } else if (strncmp(a, "--alt=", 6) == 0) { alt_m = atof(a + 6);
        } else if (strcmp(a, "--freq-mhz") == 0 && (v = take_value(argc, argv, &i))) {
            freq_hz = atof(v) * 1e6;
        } else if (strncmp(a, "--freq-mhz=", 11) == 0) { freq_hz = atof(a + 11) * 1e6;
        } else if (strcmp(a, "--window-min") == 0 && (v = take_value(argc, argv, &i))) {
            window_min = atof(v);
        } else if (strncmp(a, "--window-min=", 13) == 0) { window_min = atof(a + 13);
        } else if (strcmp(a, "--trend-days") == 0 && (v = take_value(argc, argv, &i))) {
            g_trend_days = atof(v);
        } else if (strncmp(a, "--trend-days=", 13) == 0) { g_trend_days = atof(a + 13);
        } else if (strcmp(a, "--once") == 0) { once = 1;
        } else if (a[0] == '-' && a[1] != '\0' && !(a[0] == '-' && a[1] >= '0' && a[1] <= '9')) {
            fprintf(stderr, "Unknown option: %s\n", a);
            usage(argv[0]); return EXIT_FAILURE;
        } else {
            if (n >= MAX_OBJECTS) {
                fprintf(stderr, "Too many objects (max %d)\n", MAX_OBJECTS);
                return EXIT_FAILURE;
            }
            snprintf(objs[n].name, sizeof objs[n].name, "%s", a);
            n++;
        }
    }

    if (n < 2) {
        fprintf(stderr, "Need at least two object names to compare.\n\n");
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    if (tle_path[0] == '\0' && tle_default_path(tle_path, sizeof tle_path) != 0) {
        fprintf(stderr, "No --tle given and $HOME-based default path unavailable.\n");
        return EXIT_FAILURE;
    }

    // Load each object's elements. A missing object is non-fatal: it is
    // shown as "(not found)" so a typo in one name does not blank the run.
    int any_loaded = 0;
    for (int i = 0; i < n; ++i) {
        objs[i].pred.tles_filename = tle_path;
        objs[i].pred.satellite_ephem.name = objs[i].name;
        objs[i].pred.observer_ephem.position_geodetic.lat = lat * M_PI / 180.0;
        objs[i].pred.observer_ephem.position_geodetic.lon = lon * M_PI / 180.0;
        objs[i].pred.observer_ephem.position_geodetic.alt = alt_m / 1000.0;
        objs[i].loaded = (load_tle(&objs[i].pred) == 0);
        if (objs[i].loaded) setup_object(&objs[i]);
        any_loaded |= objs[i].loaded;
    }
    if (!any_loaded) {
        fprintf(stderr, "None of the named objects were found in %s\n", tle_path);
        return EXIT_FAILURE;
    }

    // Non-interactive snapshot: compute once, print plain text, exit.
    if (once) {
        double jul_now = now_jul_utc();
        for (int i = 0; i < n; ++i) {
            if (!objs[i].loaded) continue;
            prep_object(&objs[i]);
            compute_live(&objs[i], jul_now, freq_hz);
            prep_object(&objs[i]);
            find_pass(&objs[i], jul_now, window_min);
        }
        compute_pass_separations(objs, n);
        compute_orbit_dtsec(objs, n, jul_now);
        compute_alongtrack_trend(objs, n, jul_now, g_trend_days);
        print_text(objs, n, jul_now, freq_hz, tle_path, lat, lon, alt_m);
        return EXIT_SUCCESS;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    initscr();
    cbreak();
    noecho();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);

    time_t last_live = 0, last_pass = 0;
    while (!g_quit) {
        int ch = getch();
        if (ch == 'q' || ch == 'Q') break;

        time_t wall = time(NULL);
        double jul_now = now_jul_utc();

        if (wall != last_live) {
            last_live = wall;
            for (int i = 0; i < n; ++i) {
                if (!objs[i].loaded) continue;
                prep_object(&objs[i]);
                compute_live(&objs[i], jul_now, freq_hz);
            }
        }

        if (last_pass == 0 || wall - last_pass >= PASS_REFRESH_S) {
            last_pass = wall;
            for (int i = 0; i < n; ++i) {
                if (!objs[i].loaded) continue;
                prep_object(&objs[i]);
                find_pass(&objs[i], jul_now, window_min);
            }
            compute_pass_separations(objs, n);
            compute_orbit_dtsec(objs, n, jul_now);
            compute_alongtrack_trend(objs, n, jul_now, g_trend_days);
        }

        draw(objs, n, jul_now, freq_hz, tle_path, lat, lon, alt_m);
        napms(200);
    }

    endwin();
    return EXIT_SUCCESS;
}
