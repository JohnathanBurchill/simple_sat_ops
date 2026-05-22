/*

    Simple Satellite Operations  utils/iq_annotator_macos.m

    macOS-specific pinch-gesture shim for iq_annotator. raylib/GLFW
    don't forward NSEventTypeMagnify (trackpad pinch) to the C event
    loop, so we install an NSEvent local monitor that accumulates the
    magnification delta into a global. iq_annotator reads & resets it
    every frame.

    Copyright (C) 2026  Johnathan K Burchill  --  GPLv3 or later.
*/

#import <AppKit/AppKit.h>

// Updated by the magnify-event handler installed below. Read & reset
// from the C frame loop. NSEvent.magnification is signed and roughly
// in [-1, +1] cumulative for a comfortable pinch, with each event
// carrying a delta on the order of 0.01..0.05.
float g_iq_annotator_pinch_delta = 0.0f;

static id g_pinch_monitor = nil;

void iq_annotator_install_pinch_monitor(void)
{
    if (g_pinch_monitor != nil) return;
    g_pinch_monitor =
        [NSEvent addLocalMonitorForEventsMatchingMask:NSEventMaskMagnify
            handler:^NSEvent *(NSEvent *event) {
                g_iq_annotator_pinch_delta += (float) event.magnification;
                return event;  // pass through (GLFW ignores it anyway)
            }];
}
