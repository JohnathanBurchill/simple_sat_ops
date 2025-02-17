# Simple Satellite Operations tools

A set of tools dedicated to predicting and tracking satellite passes near a
ground station using an ICOM IC-9700 transceiver and a SPIG Rot2Prog antenna
rotator controller. 

The aim is twofold: 1) demonstrate reliable and fast operation through a
simple terminal interface, and 2) learn by doing. The software suits needs;
your mileage may vary. 

Based on the [sgp4sdp4 C library](https://github.com/KJ7LNW/sgp4sdp4). Many
thanks to [@KJ7LNW](https://github.com/KJ7LNW) for making this available.

**CAUTION**: the sgp4sdp4 change log date is from 2001. We have not verified
that this version of sgp4sdp4 is consistent with that recommended by [Valado
et al.
2006](https://celestrak.org/publications/AIAA/2006-6753/AIAA-2006-6753-Rev3.pdf).

Thanks go to the University of Calgary [Rothney Astrophysical
Observatory](https://science.ucalgary.ca/rothney-observatory) staff and the
[CalgaryToSpace](https://www.calgarytospace.ca) RF communications lead (as of
Feb 2025) for logistics and technical support while testing this software on
the FrontierSat ground station equipment.

## simple_sat_ops

Custom drivers control the SPIG Rot2Prog antenna rotator and Icom IC-9700
transceiver for satellite communications over UHF and VHF. 

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

# Installation

We use ``cmake``:

```bash
mkdir build
cd build
cmake ..
make install
```

This installs the binaries to ``$HOME/bin``. Adapt CMakeLists.txt to suit your needs.

Use whatever build system you prefer, or use one-liners like this:

```bash
gcc -o simple_sat_ops main.c prediction.c radio.c antenna_rotator.c -lncurses -lsgp4sdp4
gcc -o next_in_queue next_in_queue.c prediction.c -lsgp4sdp4
gcc -o lifetime lifetime.c prediction.c -lsgp4sdp4
```

If you don't know what `libsgp4sdp4` is, look it up 🙂.

