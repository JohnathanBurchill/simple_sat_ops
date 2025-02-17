# Simple Satellite Operations tools

A set of tools dedicated to predicting and tracking satellite passes near a
ground station.

## simple_sat_ops

Custom drivers control the SPIG Rot2Prog antenna rotator and Icom IC-9700
transceiver for satellite communications over UHF and VHF. It works for us,
your mileage may vary.

![A radio demo gif](demo/simple_sat_ops_demo_radio_only_20250217.gif)

## next_in_queue

Prints upcoming overpasses of satellites from a file of TLEs using a SGP4SDP4
library.

![A demo without hardware gif](demo/simple_sat_ops_demo_no_hardware_20250127.gif)

## lifetime 

Estimates the lifetime of a satellite from TLE the alone. Inaccurate: this is
a toy "what if?" calculation. In SGP4SDP4, orbit decay is apparently modelled
empirically based on measurement of the rate of change of the mean anomaly and
its rate of change. See the sgp4sdp4 source code and related references for
details.

