/*

    Simple Satellite Operations  frontiersat.h

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

// frontiersat.h — single source of truth for the FrontierSat link
// constants now that radio.h is gone. The B210-side tools
// (tx_frame_sdr, b210_rx_capture, b210_rx_tx) included radio.h only
// for FRONTIERSAT_CARRIER_HZ; this header replaces that.

#ifndef FRONTIERSAT_H
#define FRONTIERSAT_H

// Simplex carrier: uplink and downlink share 436.150 MHz.
#define FRONTIERSAT_CARRIER_HZ 436150000.0

#endif
