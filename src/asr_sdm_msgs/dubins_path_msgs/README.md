# dubins_path_msgs

Interfaces for [`dubins3d_planner`](../dubins3d_planner/README.md), the 3D
Dubins planner for a vehicle with bounded pitch and yaw rates.

## Messages

**`Configuration`** — a pose of the vehicle. Position plus a right-handed
orthonormal frame, because a heading alone is not enough: the roll angle decides
which osculating sphere is which.

| Field | Type | Meaning |
| --- | --- | --- |
| `position` | `geometry_msgs/Point` | position |
| `tangent` | `geometry_msgs/Vector3` | direction of travel (T) |
| `tangent_normal` | `geometry_msgs/Vector3` | wing direction (t) |
| `surface_normal` | `geometry_msgs/Vector3` | osculating surface normal, `T x t = u` |

**`PathSample`** — one sample along a path, with the same four fields, so the
full attitude including roll is available to a controller or visualiser.

**`DubinsPath`** — `path_type` (the winning construction, for example
`cyc_inner` or `plane_left_right`), `length` in metres, and the `samples`
ordered from the initial to the final configuration.

## Services

**`PlanPath`** — plans between two configurations.

The request carries the two endpoints plus optional per-call overrides of the
planner settings: `pitch_radius`, `yaw_radius`, `tight_turn_radius`,
`location_samples`, `heading_samples` and `sample_spacing`. A non-positive value
means "use the node parameter", so a request may override none, some or all of
them.

The response reports `success` with a human-readable `message` and the resulting
`path`. For diagnostics it also lists the best length found for *every*
construction in `candidate_types` and `candidate_lengths`; infinite entries mean
that construction is infeasible for the query. `planning_time` is the wall-clock
time of the search in seconds.

## License

Apache-2.0.
