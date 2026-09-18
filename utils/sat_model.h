/*

    Simple Satellite Operations  utils/sat_model.h

    FrontierSat itself, drawn small on the globe when the view is close
    enough in to make room for it, turned the way the satellite was
    turned. Zoomed out, the satellite is a dot; zoomed in, the dot
    becomes a body you can read an attitude off -- which face is looking
    down, which way it is flying, where the camera is pointed.

    The shape is built here rather than loaded from the CAD. Three
    reasons. The body axes are exact by construction, and the whole
    point of drawing it is to say which way those axes point -- a model
    imported with its own arbitrary origin and up-axis would have to be
    aligned by hand, and would be wrong in a way nothing on screen could
    reveal. It needs no asset in the repository and no licence note. And
    at the size this is drawn, a few dozen flat faces carry everything a
    reader can see anyway.

    What it is, from CTS-SAT-1's design (the interactive model at
    https://www.calgarytospace.ca/cts-sat-1, and the team's own reading
    of it):

      - a 3U CubeSat, 10 x 10 x 34 cm, solar cells on the four long
        faces;
      - the MPI's entrance slit across the ram face, so the long axis
        lies along the direction of flight (+X in the body frame);
      - the colour camera -- the one whose pictures
        frontiersat_camera_viewer shows -- as a barrel on the nadir face
        (+Z), beside one of the two ADCS cameras;
      - the second ADCS camera opposite it, on the zenith face (-Z);
      - the deployable composite lattice boom stowed as a roll on the
        nadir face beside the camera. It has not been deployed, so it is
        drawn rolled.

    NOT TO SCALE, and deliberately so: a 34 cm satellite at the globe's
    own scale is a ten-thousandth of a pixel. It is drawn at a fixed size
    on screen whatever the zoom, as a symbol of where the satellite is
    and how it is turned. The caller says so on screen.

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

#ifndef SAT_MODEL_H
#define SAT_MODEL_H

#include "sat_globe.h"

// The satellite's real size, in metres. The long axis runs +X (the ram
// face, where the MPI slit is); the other two are the 10 cm sides.
#define SAT_MODEL_LENGTH_M 0.34
#define SAT_MODEL_WIDTH_M  0.10

// How many pixels the long axis spans on screen. Fixed, so the model
// stays readable at every zoom -- it is a symbol, not a scale drawing.
#define SAT_MODEL_PX 72.0

// Draw it. `at` is where the satellite is, in the globe's own world
// units (Earth radius 1), Earth-fixed. bx / by / bz are the body axes in
// that same frame -- unit vectors, right-handed. `sun` is the Earth-fixed
// direction of the Sun, for lighting the faces the way the Earth beneath
// is lit; pass NULL for flat shading.
//
// `alpha` fades the whole thing in as the view zooms toward it. `solid`
// draws the faces filled, for an attitude the satellite actually
// reported; 0 draws it as an outline, for one assumed rather than
// measured, so the two can never be mistaken for each other.
void sat_model_draw(const globe_proj_t *pr,
                    const double at[3],
                    const double bx[3], const double by[3],
                    const double bz[3],
                    const double sun[3],
                    unsigned char alpha, int solid);

#endif // SAT_MODEL_H
