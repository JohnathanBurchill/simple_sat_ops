#include "sso_operator.h"

#include "sso_audit.h"
#include "sso_ipc.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef struct {
    int got_welcome;
    char pass_folder[256];
    char operator_user[64];
} verify_ctx_t;

static void on_event_verify(sso_ipc_client_t *cli,
                             const sso_event_t *evt, void *user) {
    (void) cli;
    verify_ctx_t *ctx = (verify_ctx_t *) user;
    if (evt->type == SSO_EVT_WELCOME) {
        ctx->got_welcome = 1;
        snprintf(ctx->pass_folder, sizeof(ctx->pass_folder), "%s",
                 evt->pass_folder);
        snprintf(ctx->operator_user, sizeof(ctx->operator_user), "%s",
                 evt->operator_user);
    }
}

int sso_operator_verify(const char *role,
                         char *out_pass_folder, size_t out_pass_folder_size,
                         char *out_operator_user, size_t out_operator_user_size) {
    if (out_pass_folder && out_pass_folder_size) out_pass_folder[0] = '\0';
    if (out_operator_user && out_operator_user_size) out_operator_user[0] = '\0';

    sso_ipc_client_t *cli = sso_ipc_client_connect("simple_sat_ops");
    if (!cli) return SSO_OP_NO_OPERATOR;

    verify_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    sso_ipc_client_on_event(cli, on_event_verify, &ctx);

    sso_event_t hello;
    sso_event_init(&hello, SSO_EVT_HELLO);
    const char *user = sso_unix_user();
    snprintf(hello.from, sizeof(hello.from), "%s", user);
    snprintf(hello.user, sizeof(hello.user), "%s", user);
    snprintf(hello.role, sizeof(hello.role), "%s",
             (role && role[0]) ? role : "external");

    char buf[2048];
    if (sso_event_encode(&hello, buf, sizeof(buf)) != 0
        || sso_ipc_client_send(cli, buf) != 0) {
        sso_ipc_client_close(cli);
        return -1;
    }

    // Wait up to ~2 s for the welcome event.
    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    deadline.tv_sec += 2;
    while (!ctx.got_welcome) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long ms = (deadline.tv_sec - now.tv_sec) * 1000
                  + (deadline.tv_nsec - now.tv_nsec) / 1000000;
        if (ms <= 0) break;
        int rc = sso_ipc_client_step(cli, (int) ms);
        if (rc != 0) break;
    }

    int result;
    if (!ctx.got_welcome) {
        result = SSO_OP_PROTOCOL;
    } else if (ctx.operator_user[0] == '\0') {
        result = SSO_OP_NO_OPERATOR;
    } else if (strcmp(ctx.operator_user, user) != 0) {
        result = SSO_OP_MISMATCH;
    } else {
        result = SSO_OP_OK;
    }

    if (out_pass_folder && out_pass_folder_size) {
        snprintf(out_pass_folder, out_pass_folder_size, "%s", ctx.pass_folder);
    }
    if (out_operator_user && out_operator_user_size) {
        snprintf(out_operator_user, out_operator_user_size, "%s",
                 ctx.operator_user);
    }

    sso_ipc_client_close(cli);
    return result;
}

int sso_operator_publish(const sso_event_t *evt) {
    if (!evt) return -1;
    sso_ipc_client_t *cli = sso_ipc_client_connect("simple_sat_ops");
    if (!cli) return -1;
    char buf[4096];
    if (sso_event_encode(evt, buf, sizeof(buf)) != 0) {
        sso_ipc_client_close(cli);
        return -1;
    }
    int rc = sso_ipc_client_send(cli, buf);
    // Give the server a moment to drain.
    sso_ipc_client_step(cli, 100);
    sso_ipc_client_close(cli);
    return rc;
}
