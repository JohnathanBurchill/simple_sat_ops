// sso_operator.h — high-level helpers layered on sso_ipc.
//
// tx_frame_sdr and b210_rx_tx both need the same dance: connect to
// simple_sat_ops.sock, send `hello`, wait for `welcome`, verify the
// running operator's Unix user matches our own, fish out the
// pass-folder path. This module factors that into one call.

#ifndef SSO_OPERATOR_H
#define SSO_OPERATOR_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SSO_OP_OK = 0,
    SSO_OP_NO_OPERATOR = 1,    // simple_sat_ops not running
    SSO_OP_MISMATCH    = 2,    // operator's user != $USER
    SSO_OP_PROTOCOL    = 3,    // unexpected wire response
} sso_operator_status_t;

// Connect to simple_sat_ops.sock, do the hello/welcome handshake, and
// verify the operator's Unix user matches getenv("USER").
//
// role: "external" (typical for tx_frame_sdr / b210_rx_tx operator
// mode) or "viewer".
//
// out_pass_folder / out_operator_user are filled from the welcome reply
// when successful (may be empty strings if the server is in operator
// mode but hasn't computed a pass folder yet).
//
// Returns 0 on success (SSO_OP_OK), non-zero on each failure mode
// listed in sso_operator_status_t. Negative on transport error.
int sso_operator_verify(const char *role,
                         char *out_pass_folder, size_t out_pass_folder_size,
                         char *out_operator_user, size_t out_operator_user_size);

// One-shot publisher: connect to simple_sat_ops.sock, send the event,
// close. Used by tx_frame_sdr to push tx-command-sent without keeping
// a long-lived connection. Returns 0 on success, -1 on error.
struct sso_event;
typedef struct sso_event sso_event_t_fwd;
#include "sso_ipc.h"  // for sso_event_t definition
int sso_operator_publish(const sso_event_t *evt);

#ifdef __cplusplus
}
#endif

#endif
