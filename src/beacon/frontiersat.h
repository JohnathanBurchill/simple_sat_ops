// frontiersat.h — single source of truth for the FrontierSat link
// constants now that radio.h is gone. The B210-side tools
// (tx_frame_sdr, b210_rx_capture, b210_rx_tx) included radio.h only
// for FRONTIERSAT_CARRIER_HZ; this header replaces that.

#ifndef FRONTIERSAT_H
#define FRONTIERSAT_H

// Simplex carrier: uplink and downlink share 436.150 MHz.
#define FRONTIERSAT_CARRIER_HZ 436150000.0

#endif
