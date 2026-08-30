# assets

Files the GUI tools look for at run time, next to the binary, in
`$XDG_DATA_HOME/simple_sat_ops/`, in `~/.local/share/simple_sat_ops/`, or in
this directory. Nothing here is installed by `make install`; a tool that cannot
find a file falls back (the font to raylib's built-in one, the world map to a
plain blue sphere).

- `SourceCodePro-Regular.ttf` — the UI font for `mpi_viewer`,
  `decode_inspector`, `frontiersat_camera_viewer` and `live_waterfall`.
  Source Code Pro, Adobe, SIL Open Font License 1.1.

- `nasa_blue_marble_2048.png` — the world map `mpi_viewer` wraps on its globe.
  NASA Earth Observatory / Visible Earth, "Blue Marble: Land Surface, Shallow
  Water, and Shaded Topography", <https://visibleearth.nasa.gov/images/57752>.
  Public domain (NASA imagery carries no copyright). Reduced here from the
  published 2048 x 1024 JPEG to a 256-colour PNG, which the globe samples on
  the CPU; equirectangular, longitude 180 W at the left edge.
