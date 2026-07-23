Place optional preloaded map files in this directory:

- `occupancy.bin`
- `esdf.bin`

The filenames and directory can be changed with the `esdf_map.preload_*`
parameters. An empty `esdf_map.preload_map_directory` disables preloading.

## Source resolution

The current binary format stores voxel-center coordinates in world space but
does not store the source voxel size. Set
`esdf_map.preload_source_resolution` to the resolution used when these files
were saved.

`esdf_map.resolution` remains the target map resolution and is also used as the
RViz `CUBE_LIST` cube edge length. If the source and target grids differ,
`occupancy.bin` is conservatively resampled and the target ESDF is rebuilt.

## Binary identification

The fixed magic strings identify the file type: `ASR_OCC_BIN` for occupancy and
`ASR_ESDF_BIN` for ESDF. The following `uint32_t version` field independently
identifies the binary layout; the current supported version is `1`.

