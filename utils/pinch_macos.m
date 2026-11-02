/*

    Simple Satellite Operations  utils/pinch_macos.m

    macOS pinch-gesture shim for the raylib viewers. raylib/GLFW
    don't forward NSEventTypeMagnify (trackpad pinch) to the C event
    loop, so we install an NSEvent local monitor that accumulates the
    magnification delta into a global the viewer reads & resets
    every frame.

    Copyright (C) 2026  Johnathan K Burchill  --  GPLv3 or later.
*/

#import <AppKit/AppKit.h>

// Updated by the magnify-event handler installed below. Read & reset
// from the C frame loop. NSEvent.magnification is signed and roughly
// in [-1, +1] cumulative for a comfortable pinch, with each event
// carrying a delta on the order of 0.01..0.05.
// Intentionally a plain float, not _Atomic: the handler runs on the main
// thread's event loop and the frame loop reads it on the same thread, so
// there's no real cross-thread race; a missed/duplicated delta would only
// be a one-frame zoom hiccup. Promote to _Atomic if it ever moves threads.
float g_sso_pinch_delta = 0.0f;

static id g_pinch_monitor = nil;

void sso_install_pinch_monitor(void)
{
    if (g_pinch_monitor != nil) return;
    g_pinch_monitor =
        [NSEvent addLocalMonitorForEventsMatchingMask:NSEventMaskMagnify
            handler:^NSEvent *(NSEvent *event) {
                g_sso_pinch_delta += (float) event.magnification;
                return event;  // pass through (GLFW ignores it anyway)
            }];
}
