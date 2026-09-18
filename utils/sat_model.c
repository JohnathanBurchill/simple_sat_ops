/*

    Simple Satellite Operations  utils/sat_model.c

    See sat_model.h for what is drawn and why it is built here rather
    than loaded from the CAD.

    How it is drawn: the model is a handful of boxes and flat patches in
    body coordinates, in metres. Each vertex is turned into the globe's
    Earth-fixed world frame through the three body axes the caller
    supplies, scaled so the satellite spans a fixed number of pixels,
    placed at the satellite's own point, and projected by the globe's own
    orthographic projection -- the same one the ground track goes
    through, so the model lands exactly where the dot would have.

    There is no depth buffer and none is wanted. The body is convex, so
    dropping the faces that point away from the viewer leaves only faces
    that cannot overlap each other; the few pieces that stand off the
    body (the camera barrel, the boom roll) are drawn after it, farthest
    first. That is enough at this size and costs nothing.

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

#include "sat_model.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

// Half-extents of the body, in metres.
#define HX (SAT_MODEL_LENGTH_M * 0.5)
#define HY (SAT_MODEL_WIDTH_M  * 0.5)
#define HZ (SAT_MODEL_WIDTH_M  * 0.5)

// The palette. Structure is the anodised aluminium of the rails and end
// plates; cells are the blue of the solar panels; the lenses are dark so
// they read as openings rather than as fittings.
//
// Brighter than the satellite really is, and on purpose: this is drawn
// 70-odd pixels across on a panel whose background is nearly black and
// whose Earth is half in shadow. At the true colours it came out as a
// dark smudge. Contrast between the faces is what carries an attitude at
// this size, so the palette is chosen for that.
static const Color C_STRUCTURE = { 205, 208, 214, 255 };
static const Color C_CELLS     = {  58,  80, 140, 255 };
static const Color C_NADIR     = { 150, 156, 168, 255 };  // the camera face
static const Color C_LENS      = {  16,  18,  28, 255 };
static const Color C_CAMERA    = {  70,  74,  86, 255 };
static const Color C_BOOM      = { 116, 104,  88, 255 };
static const Color C_SLIT      = {  12,  12,  18, 255 };

// The outline each visible face is drawn with, over its fill: it stands
// in for the structural rails and the panel edges, and is most of what
// makes the body read as a solid at this size.
static const Color C_EDGE      = { 225, 230, 240, 110 };

// One flat face: four corners in body metres, a colour, and how far it
// is pushed off the surface it sits on so a patch does not fight with
// the face under it.
typedef struct {
    double  v[4][3];
    Color   c;
} quad_t;

// Enough for the body, three barrels, the boom roll and the patches.
#define MAX_QUADS 64

typedef struct {
    quad_t q[MAX_QUADS];
    int    n;
} mesh_t;

static void add_quad(mesh_t *m, Color c,
                     const double a[3], const double b[3],
                     const double d[3], const double e[3])
{
    if (m->n >= MAX_QUADS) return;
    quad_t *q = &m->q[m->n++];
    memcpy(q->v[0], a, 3 * sizeof(double));
    memcpy(q->v[1], b, 3 * sizeof(double));
    memcpy(q->v[2], d, 3 * sizeof(double));
    memcpy(q->v[3], e, 3 * sizeof(double));
    q->c = c;
}

// A box from its centre and half-extents, six faces, each its own
// colour so the long sides can be solar cells and the ends structure.
// The winding is consistent (counter-clockwise seen from outside), which
// is what lets the back faces be dropped later.
// nadir, when not the transparent black below, colours the +Z face on
// its own: the face the camera looks out of is the one a reader most
// wants to pick out, so it does not share the solar cells' colour.
static void add_box(mesh_t *m, const double ctr[3], double hx, double hy,
                    double hz, Color ends, Color sides, Color nadir)
{
    const double x0 = ctr[0] - hx, x1 = ctr[0] + hx;
    const double y0 = ctr[1] - hy, y1 = ctr[1] + hy;
    const double z0 = ctr[2] - hz, z1 = ctr[2] + hz;
    if (nadir.a == 0) nadir = sides;
    #define V(X, Y, Z) (const double[3]){ X, Y, Z }
    // +X and -X: the ram and anti-ram ends.
    add_quad(m, ends,  V(x1,y0,z0), V(x1,y1,z0), V(x1,y1,z1), V(x1,y0,z1));
    add_quad(m, ends,  V(x0,y1,z0), V(x0,y0,z0), V(x0,y0,z1), V(x0,y1,z1));
    // +Y and -Y.
    add_quad(m, sides, V(x1,y1,z0), V(x0,y1,z0), V(x0,y1,z1), V(x1,y1,z1));
    add_quad(m, sides, V(x0,y0,z0), V(x1,y0,z0), V(x1,y0,z1), V(x0,y0,z1));
    // +Z (nadir, the camera face) and -Z (zenith).
    add_quad(m, nadir, V(x0,y0,z1), V(x1,y0,z1), V(x1,y1,z1), V(x0,y1,z1));
    add_quad(m, sides, V(x1,y0,z0), V(x0,y0,z0), V(x0,y1,z0), V(x1,y1,z0));
    #undef V
}

// A flat patch lying on a face, pushed a millimetre clear of it: the
// MPI's slit, a camera's lens. `n` is which way the face points.
static void add_patch(mesh_t *m, Color c, const double ctr[3],
                      const double n[3], const double u[3],
                      const double vv[3], double hu, double hv)
{
    const double lift = 0.002;
    double o[3];
    for (int i = 0; i < 3; i++) o[i] = ctr[i] + n[i] * lift;
    double a[3], b[3], d[3], e[3];
    for (int i = 0; i < 3; i++) {
        a[i] = o[i] - u[i] * hu - vv[i] * hv;
        b[i] = o[i] + u[i] * hu - vv[i] * hv;
        d[i] = o[i] + u[i] * hu + vv[i] * hv;
        e[i] = o[i] - u[i] * hu + vv[i] * hv;
    }
    add_quad(m, c, a, b, d, e);
}

// The satellite, in body metres. +X is the ram face and the MPI slit,
// +Z the nadir face with the colour camera, an ADCS camera and the
// stowed boom, -Z the zenith face with the other ADCS camera.
static void build_frontiersat(mesh_t *m)
{
    m->n = 0;
    const double origin[3] = { 0, 0, 0 };

    // The 3U body: solar cells down the long faces, structure at the two
    // ends, and the nadir face in its own colour because it is the one
    // carrying the camera.
    add_box(m, origin, HX, HY, HZ, C_STRUCTURE, C_CELLS, C_NADIR);

    // The MPI's entrance slit, standing vertically across the ram face.
    const double xface[3] = { HX, 0, 0 };
    const double nx[3] = { 1, 0, 0 }, ny[3] = { 0, 1, 0 }, nz[3] = { 0, 0, 1 };
    add_patch(m, C_SLIT, xface, nx, ny, nz, 0.008, HZ * 0.78);

    // The colour camera on the nadir face, a barrel standing off it,
    // with a dark lens on its outer end. Placed toward the ram end,
    // beside the boom, as the model on the team's site has it.
    const double cam_c[3] = { HX * 0.45, 0.0, HZ + 0.011 };
    add_box(m, cam_c, 0.018, 0.018, 0.013, C_CAMERA, C_CAMERA, C_CAMERA);
    const double cam_face[3] = { cam_c[0], cam_c[1], cam_c[2] + 0.011 };
    add_patch(m, C_LENS, cam_face, nz, nx, ny, 0.014, 0.014);

    // The ADCS cameras: one beside the colour camera on the nadir face,
    // one opposite it on the zenith face.
    const double adcs_n[3] = { -HX * 0.10, HY * 0.5, HZ + 0.008 };
    add_box(m, adcs_n, 0.012, 0.012, 0.009, C_CAMERA, C_CAMERA, C_CAMERA);
    const double adcs_n_face[3] = { adcs_n[0], adcs_n[1], adcs_n[2] + 0.008 };
    add_patch(m, C_LENS, adcs_n_face, nz, nx, ny, 0.009, 0.009);

    const double adcs_z[3] = { -HX * 0.10, HY * 0.5, -HZ - 0.008 };
    add_box(m, adcs_z, 0.012, 0.012, 0.009, C_CAMERA, C_CAMERA, C_CAMERA);
    const double nzm[3] = { 0, 0, -1 };
    const double adcs_z_face[3] = { adcs_z[0], adcs_z[1], adcs_z[2] - 0.008 };
    add_patch(m, C_LENS, adcs_z_face, nzm, nx, ny, 0.009, 0.009);

    // The deployable composite lattice boom, stowed: a roll lying across
    // the nadir face, behind the camera. It has not been deployed, so
    // this is the whole of it.
    const double boom_c[3] = { -HX * 0.45, 0.0, HZ + 0.013 };
    add_box(m, boom_c, 0.034, HY * 0.84, 0.016, C_BOOM, C_BOOM, C_BOOM);
}

// ---- drawing ---------------------------------------------------------------

static void cross3(const double a[3], const double b[3], double o[3])
{
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}

static double dot3(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// A face ready to draw: its four corners on the panel, how far away it
// is, and what colour it came out.
typedef struct {
    Vector2 p[4];
    double  depth;
    Color   c;
} face_t;

static int face_cmp(const void *a, const void *b)
{
    const double da = ((const face_t *) a)->depth;
    const double db = ((const face_t *) b)->depth;
    // Farthest first, so nearer faces are drawn over them.
    if (da < db) return -1;
    if (da > db) return 1;
    return 0;
}

void sat_model_draw(const globe_proj_t *pr,
                    const double at[3],
                    const double bx[3], const double by[3],
                    const double bz[3],
                    const double sun[3],
                    unsigned char alpha, int solid)
{
    if (pr == NULL || alpha == 0) return;

    mesh_t mesh;
    build_frontiersat(&mesh);

    // Metres to the globe's world units, chosen so the long axis spans
    // SAT_MODEL_PX pixels whatever the zoom. pr->R is one Earth radius
    // in pixels, so 1/R is a pixel in world units.
    if (pr->R <= 0.0) return;
    const double scale = (SAT_MODEL_PX / pr->R) / SAT_MODEL_LENGTH_M;

    face_t out[MAX_QUADS];
    int nout = 0;

    for (int i = 0; i < mesh.n; i++) {
        const quad_t *q = &mesh.q[i];

        // Body metres into the Earth-fixed world, through the body axes.
        double w[4][3];
        for (int k = 0; k < 4; k++) {
            for (int c = 0; c < 3; c++) {
                w[k][c] = at[c] + scale * (q->v[k][0] * bx[c]
                                         + q->v[k][1] * by[c]
                                         + q->v[k][2] * bz[c]);
            }
        }

        // The face's normal, from two of its edges. The winding is
        // consistent, so this points out of the body.
        double e1[3], e2[3], nrm[3];
        for (int c = 0; c < 3; c++) {
            e1[c] = w[1][c] - w[0][c];
            e2[c] = w[3][c] - w[0][c];
        }
        cross3(e1, e2, nrm);
        const double nl = sqrt(dot3(nrm, nrm));
        if (nl <= 0.0) continue;
        for (int c = 0; c < 3; c++) nrm[c] /= nl;

        // Drop the faces pointing away from the viewer. pr->o is the
        // direction out of the panel, so a face is towards us when its
        // normal leans that way.
        const double facing = dot3(nrm, pr->o);
        if (facing <= 0.0) continue;

        // Lit the way the Earth under it is lit, with enough ambient
        // that a face in shadow is still a face rather than a hole.
        double shade = 1.0;
        if (sun != NULL) {
            const double s = dot3(nrm, sun);
            shade = 0.52 + 0.48 * (s > 0.0 ? s : 0.0);
        }
        // A touch of extra light on the faces square to the viewer, so
        // the body reads as a solid rather than as a flat outline.
        shade *= 0.82 + 0.18 * facing;

        face_t *f = &out[nout++];
        double depth = 0.0;
        for (int k = 0; k < 4; k++) {
            f->p[k] = globe_project_world(pr, w[k], NULL);
            depth += dot3(w[k], pr->o);
        }
        f->depth = depth * 0.25;
        f->c = (Color){ (unsigned char) (q->c.r * shade),
                        (unsigned char) (q->c.g * shade),
                        (unsigned char) (q->c.b * shade),
                        alpha };
    }

    qsort(out, (size_t) nout, sizeof out[0], face_cmp);

    for (int i = 0; i < nout; i++) {
        const face_t *f = &out[i];
        if (solid) {
            // raylib wants its triangles wound one way round to fill
            // them; the projection can flip a face either way, so draw
            // both windings and let the one that is wrong contribute
            // nothing.
            DrawTriangle(f->p[0], f->p[1], f->p[2], f->c);
            DrawTriangle(f->p[0], f->p[2], f->p[1], f->c);
            DrawTriangle(f->p[0], f->p[2], f->p[3], f->c);
            DrawTriangle(f->p[0], f->p[3], f->p[2], f->c);
            // The rails and panel edges, near enough: an outline on each
            // face is what makes the body read as a solid at seventy
            // pixels rather than as a flat patch of colour.
            const Color edge = { C_EDGE.r, C_EDGE.g, C_EDGE.b,
                                 (unsigned char) (C_EDGE.a * alpha / 255) };
            for (int k = 0; k < 4; k++)
                DrawLineEx(f->p[k], f->p[(k + 1) % 4], 1.0f, edge);
        } else {
            // Attitude assumed rather than measured: an outline, so it
            // can never be read as the real thing.
            const Color line = { f->c.r, f->c.g, f->c.b,
                                 (unsigned char) (alpha * 3 / 4) };
            for (int k = 0; k < 4; k++)
                DrawLineEx(f->p[k], f->p[(k + 1) % 4], 1.0f, line);
        }
    }
}
