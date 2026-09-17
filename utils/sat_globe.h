/*

    Simple Satellite Operations  utils/sat_globe.h

    The ephemeris globe panel the raylib viewers put beside their data: the
    Earth as it was at a moment of the mission, NASA's Blue Marble wrapped on a
    sphere and lit from where the Sun actually stood, with a stretch of the
    satellite's ground track drawn across it and a dot where the satellite was
    at the moment on screen. Drag it to turn it, scroll to zoom.

    The orbit comes out of the packet DB's own tle table -- whichever element
    set's epoch is closest to the moment being shown -- and is propagated by the
    same SGP4 path next_in_queue uses.

    Shared by utils/mpi_viewer.c (the globe under the experiment list, tracking
    the recording) and utils/frontiersat_camera_viewer.c (the globe under the
    capture list, tracking the picture).

    Using it, in outline:

        globe_t g = {0};
        g.zoom = 1.0;
        globe_set_font(font, font_loaded, spacing);   // optional
        globe_load_map(&g, "my_viewer");
        globe_load_tles(db_path);
        ...
        globe_input(&g, x, y, w, h);                  // before drawing
        if (strcmp(g.track_key, key) != 0)
            globe_set_track(&g, key, t0_ms, t1_ms);
        globe_draw(&g, x, y, w, h, now_ms, "Ground track");
        ...
        globe_free(&g);

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

#ifndef SAT_GLOBE_H
#define SAT_GLOBE_H

#include <raylib.h>

#include "prediction.h"

// The map is stored as an equirectangular image, so a level is just a smaller
// copy of it: the whole 360 degrees of longitude spans about 2*pi*R pixels
// across the middle of a disc of radius R, and picking the level nearest that
// width keeps the globe from shimmering when it is small.
#define GLOBE_MAP_LEVELS  5
#define GLOBE_MAP_FILE    "nasa_blue_marble_2048.png"

// How much of the panel the globe fills at zoom 1, and the zoom limits.
#define GLOBE_FILL        0.47
#define GLOBE_ZOOM_MIN    1.0
#define GLOBE_ZOOM_MAX    16.0

// How long after the last turn or zoom the disc is drawn again at the screen's
// own resolution. Long enough that a flick of the wheel counts as one movement.
#define GLOBE_SETTLE      0.20f

// How bright the unlit side is drawn, and the sine of the half-width of the
// soft terminator (0.10 is about six degrees of Sun elevation either way,
// roughly civil twilight).
#define GLOBE_NIGHT       0.24
#define GLOBE_TERMINATOR  0.10

// Samples along the ground track, and the panel geometry: the title row above
// the disc, the whole panel's height, and the least it can shrink to before
// the window is too short to show it at all.
#define GLOBE_TRACK_PTS   240
#define GLOBE_TITLE_H     22
#define GLOBE_PANEL_H     360
#define GLOBE_PANEL_MIN_H 190

// Earth's equatorial radius, for putting the track at its real altitude above
// the unit sphere. sgp4sdp4's own xkmper, without pulling in its constants.
#define GLOBE_EARTH_KM    6378.135

typedef struct {
    unsigned char *rgb;
    int w, h;
} maplevel_t;

typedef struct {
    float lat, lon, alt_km;
} subpoint_t;

typedef struct {
    // The world map, and the same map halved a few times over.
    maplevel_t lv[GLOBE_MAP_LEVELS];
    int        nlv;

    // Where the camera looks, and how much of the panel the disc fills.
    double lat0, lon0, zoom;

    // Where the middle of the disc sits, as an offset from the middle of the
    // panel. Turning and zooming are about the satellite rather than about the
    // panel, which means holding its dot still and letting the globe move
    // under it -- this is how far it has moved.
    double ox, oy;

    // The satellite's own point, as of the last frame drawn: the thing turning
    // and zooming pivot on. Read a frame late, which no drag can see.
    double anchor_lat, anchor_lon, anchor_alt;
    int    have_anchor;

    // The rendered disc, and what it was rendered for.
    Color    *pix;
    Texture2D tex;
    int       tex_w, tex_h;
    int       valid;
    double    r_lat0, r_lon0, r_zoom, r_sunlat, r_sunlon, r_ox, r_oy;

    // The orbit being shown: one converted element set, propagated for every
    // sample. track_key is the caller's name for whatever the track was built
    // for -- an experiment, a picture -- and is how it notices a change.
    int        have_tle;
    tle_t      tle;
    char       track_key[40];
    char       tle_note[64];
    subpoint_t trk[GLOBE_TRACK_PTS];
    int        ntrk;

    // The sub-solar point, Earth-fixed, in the middle of that stretch of track.
    double sun_lat, sun_lon;

    // A drag that began on the globe keeps turning it until the button is let
    // go, the same way a whereogram scrub does.
    int     drag;
    int     drag_pivot;   // this drag turns about the satellite, not the middle
    Vector2 drag_at;

    // Seconds left of the coarse render the disc falls back to while it is
    // being turned or zoomed. At zero it is drawn once more at the screen's
    // own resolution, r_ss texture pixels to the panel pixel.
    float   settle;
    int     r_ss;

    // Set when the caller restored a saved view for the track being resumed,
    // so the first track built does not frame itself over it.
    int     keep_view;
} globe_t;

// Draw the panel's text with the caller's font. Without this the panel falls
// back to raylib's built-in bitmap font.
void globe_set_font(Font f, int loaded, float spacing);

// Load the equirectangular world map and halve it a few times over. tool names
// the program in the warning when the map is nowhere to be found, which leaves
// the globe drawn as a plain sphere rather than not drawn at all. Returns 0 if
// the map was not found.
int globe_load_map(globe_t *g, const char *tool);

// Load FrontierSat's element sets out of the packet DB's tle table, so a moment
// can be matched to the one closest to it in time. Returns how many came back;
// call it again to pick up element sets added since.
int globe_load_tles(const char *db_path);

// Point the globe at a stretch of track: pick the element set whose epoch is
// closest to it, walk the ground track from t0_ms to t1_ms, work out where the
// Sun was in the middle of it, and turn the globe to face the middle of the
// track. key is stored in track_key.
void globe_set_track(globe_t *g, const char *key, double t0_ms, double t1_ms);

// The whole view back to how it first opens -- framed on the track and zoomed
// out to the whole Earth.
void globe_reset_view(globe_t *g);

// The sub-satellite point at a moment, from the element set already loaded.
// Returns 0 if there is no element set.
int globe_subpoint(const globe_t *g, double unix_ms,
                   double *lat, double *lon, double *alt_km);

// Drag turns the globe, the wheel (or two fingers) zooms it. Call it with the
// panel's own rectangle, before drawing.
void globe_input(globe_t *g, int x, int y, int w, int h);

// The panel: the lit Earth, the ground track, and a dot where the satellite was
// at now_ms.
void globe_draw(globe_t *g, int x, int y, int w, int h, double now_ms,
                const char *title);

void globe_free(globe_t *g);

#endif // SAT_GLOBE_H
