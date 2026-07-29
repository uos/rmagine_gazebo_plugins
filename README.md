[[Code](https://github.com/uos/rmagine_gazebo_plugins)] [[Wiki](https://github.com/uos/rmagine_gazebo_plugins/wiki)]

> [!IMPORTANT]
> The original package targets Gazebo Classic. This repository also contains an in-progress Gazebo Harmonic (`gz-sim8`) port for ROS 2 Jazzy.

> [!NOTE]
> A Gazebo Harmonic (gz-sim8) port is in progress. Build only the ported targets with:
> `colcon build --packages-select rmagine_gazebo_plugins --cmake-args -DRMAGINE_GZSIM_PORT=ON --merge-install --symlink-install`
> To build the Classic plugins instead:
> `colcon build --packages-select rmagine_gazebo_plugins --cmake-args -DRMAGINE_GZSIM_PORT=OFF --merge-install --symlink-install`

## Gazebo Harmonic Port Status

This repository currently contains two tracks:

- The original Gazebo Classic implementation.
- A new Gazebo Harmonic (`gz-sim8`) port under `include/rmagine_gazebo_plugins/gz` and `src/gz`.

Current Harmonic status:

- Working:
  - `rmagine_embree_map_system`
  - `rmagine_embree_sensor_system`
  - `rmagine_optix_map_system` (GPU, requires an OptiX SDK at build time -- see below)
  - `rmagine_optix_sensor_system` (GPU)
  - Static Embree map build from a minimal gz-sim world
  - Static OptiX (GPU) map build from a minimal gz-sim world, runtime-verified
    against a real NVIDIA GPU (see "OptiX / GPU Harmonic Port" below)
  - ROS 2 publication of `sensor_msgs/msg/LaserScan` on `scan` (Spherical only)
  - ROS 2 publication of `sensor_msgs/msg/PointCloud2` on `points`
  - Selectable sensor model via `model_type` SDF element: `spherical`
    (default), `pinhole`, `o1dn`, `ondn` -- see "Non-spherical sensor
    models (Pinhole/O1Dn/OnDn)" below
- Partially working:
  - Baseline and dynamic validation worlds for the Embree CPU path
  - RViz visualization works when using `sensor_link` as the fixed frame
  - RViz visualization with `world` as the fixed frame is currently best handled with an external static TF for the example world
- Missing / not yet ported:
  - A robust TF integration strategy for the gz-sim path
  - Full parity with the Gazebo Classic implementation

The existing baseline/dynamic fixtures are now wired into `colcon test`
(`ament_add_test`, see `CMakeLists.txt`) — `colcon test --packages-select
rmagine_gazebo_plugins` runs `embree_fixture_baseline`/`embree_fixture_dynamic`
as real, aggregated tests (`colcon test-result` picks them up), not just a
hand-run or CI-script invocation.

The current manual validation procedure is documented in [TESTING.md](TESTING.md).

## Migration State Assessment

The package is **partially migrated** to ROS 2 Jazzy + Gazebo Harmonic (`gz-sim8`).

What exists today is a new Harmonic Embree path alongside the original Classic code. That path is real, builds cleanly, and has been validated in small dedicated worlds, but it is **not yet equivalent to a full project-level migration** of `rmagine_gazebo_plugins`.

Current migration stage:

- Ported and usable in controlled validation worlds
- Not yet complete for general project integration
- Still missing major Classic-to-Harmonic parity items

### What Has Already Been Ported

#### Harmonic-specific runtime path

The following new gz-sim code exists under `include/rmagine_gazebo_plugins/gz` and `src/gz`:

- `rmagine_embree_map_system`
- `rmagine_embree_sensor_system`
- `rmagine_optix_map_system` (GPU)
- `rmagine_optix_sensor_system` (GPU)
- `map_registry` / `optix_map_registry`
- `test_box_mover_system`

This gives the package a native Harmonic-side implementation for:

- building an Embree map from gz-sim visuals
- publishing ROS 2 `LaserScan` and `PointCloud2`
- refreshing the sensor simulator when the map changes
- driving a deterministic moving obstacle for validation (`test_box_mover_system`
  supports both linear oscillation, via `axis`/`amplitude`/`period`, and
  angular oscillation, via `angular_axis`/`angular_amplitude`/`period` —
  the latter added for `radarays_gazebo_plugins`'s multi-object dynamic
  scenario; both default to off, so existing worlds using only the linear
  params are unaffected)

#### OptiX / GPU Harmonic port

`rmagine_optix_map_system` and `rmagine_optix_sensor_system` mirror the
Embree pair above (own `OptixMapRegistry` singleton, same SDF param
surface), swapped to `rmagine::OptixMap`/`OptixScene`/`SphereSimulatorOptix`.
They only build if `rmagine` was itself built with a working `rmagine::optix`
target, which additionally needs, beyond the CUDA toolkit:

- an NVIDIA GPU with a real display driver (`nvidia-smi` working)
- the OptiX **SDK** (headers only -- `optix.h` etc; the runtime library,
  `libnvoptix.so.1`, ships with the driver already). Not apt-installable;
  download from NVIDIA (developer login required) and point
  `find_package(OptiX)` (see `rmagine/src/rmagine_optix/cmake/FindOptiX.cmake`)
  at it via `$OPTIX_INCLUDE_DIR`, `/opt/optix/include`, or `~/optix/include`.
- **SDK/driver ABI match**: OptiX SDK 9.1.0 (ABI 118) failed with
  `OPTIX_ERROR_UNSUPPORTED_ABI_VERSION` against driver 580.173.02 on the
  machine this was built on -- that driver's OptiX runtime component simply
  doesn't support that new an ABI yet, even though the driver itself is
  recent. **OptiX SDK 7.5.0 (ABI 60) worked**, matching this codebase's own
  hardcoded driver-compatibility table in `OptixContext.cpp` (which only
  lists entries up to 7.5.0) -- this code was written/tested against the
  7.x SDK generation. If OptiX init fails with an ABI error, try an older
  SDK before assuming the GPU/driver itself is the problem.

Three real bugs were found and fixed while getting this working, verified
against the actual GPU (not just a compile check):

1. `SphereSimulatorOptix`'s single-Transform *returning*
   `simulate<ResT>(Tbm)` convenience overload segfaults (crashes inside
   `SphericalModel::getHeight()` with a bad `this` -- the result bundle is
   never actually sized before the kernel launch). The pre-sized void
   `simulate<ResT>(Tbm, ret)` overload -- the one rmagine's own OptiX test
   suite (`tests/optix/simulation_spherical.cpp`) exercises -- works
   correctly and is what `rmagine_optix_sensor_system` uses.
2. **Box/sphere/cylinder primitives silently render at unit size**,
   regardless of their configured SDF dimensions. Root cause: the map
   system does `instance->setScale(shape_size); instance->apply();` to size
   the shared unit-mesh instance, then later calls
   `geom->setTransformAndScale(M)` for the world pose -- but
   `setTransformAndScale()` **decomposes its matrix into both the
   transform AND the scale**, clobbering the shape size back to `(1,1,1)`
   (the `M` here is built from a pose and a hardcoded identity scale, since
   the SDF `<scale>` tag isn't read separately). Confirmed with a
   standalone repro outside gz-sim: a 0.2 box and a 1.0 box raytraced to
   the *identical* hit distance until fixed. Fix (applied in
   `rmagine_optix_map_system.cpp`): use `setTransform(T)` instead, which
   only touches the transform and leaves the previously-set scale alone.
   **This same bug was present in `rmagine_embree_map_system.cpp` (the CPU
   path) too** -- confirmed by code inspection, and fixed there as well
   with the identical one-line change. It was silent in the existing
   baseline/dynamic worlds only because their boxes happen to be sized
   `1x1x1`/`0.2x0.2x0.2` in ways that don't expose it. Rather than assume a
   fixture re-capture was needed, checked what every fixture-backing world
   actually feeds through the map: every traced shape in every existing
   world (`rmagine_gazebo_plugins`'s baseline/dynamic, `radarays_gazebo_plugins`'s
   static/dynamic/dynamic_multi) is already unit-scaled, so the fix is
   mathematically a no-op for all of them -- confirmed empirically by
   rerunning all 5 fixtures against the fix; all passed unchanged, no
   re-capture needed. See `MIGRATION_HANDOFF.md`, item #19.
3. **`rmagine_optix_map_system`'s MESH case crashed the first time it was
   ever actually exercised** -- every world tested through item #18/#19
   above only used `<box>` geometry, so this code path (mesh geometry, as
   opposed to a primitive) had never really run until `radarays_ros`'s GPU
   radar work (see item #21) needed a real mesh. The MESH case adds the
   `OptixMesh` **directly** to the top-level scene, no instance wrapping --
   unlike the box/sphere/cylinder cases just above it, which already go
   through `geom_scene->instantiate()`. That direct-add is broken in this
   rmagine/OptiX version: `simulate()`/`move_waves()`/`signal_shader()` all
   run and `cudaDeviceSynchronize()` cleanly, but the very next
   device-to-host copy throws `cudaErrorIllegalAddress`. Root-caused with a
   standalone repro outside gz-sim entirely (rules out any gz-sim/cross-.so
   involvement) that narrowed it down to exactly this one difference:
   wrapping the same mesh in an `OptixInst` (`mesh->makeScene()` ->
   `commit()` -> `instantiate()`), matching what `rmagine::import_optix_map()`
   and the primitive-shape cases already do, fixes it outright. Fixed in
   `rmagine_optix_map_system.cpp`'s MESH case accordingly.

Runtime-verified end-to-end: `worlds/gz_optix_baseline.sdf` (a straight port
of `gz_embree_baseline.sdf` to the OptiX systems) launched under
`gz sim -s -r --headless-rendering`, `/scan` and `/points` publish over
ROS 2, and the GPU-computed hit distance on the fixed `1x1x1` box (76/400
finite hits, `2.500m`-`2.549m`) matches the analytically expected front-face
distance exactly.

There is now also `worlds/gz_optix_dynamic.sdf`, a straight port of
`gz_embree_dynamic.sdf` (same `test_box_mover_system`-driven oscillating
box, `axis: 1 0 0`, `amplitude: 0.75`, `period: 6.0`, swapped to the OptiX
map/sensor systems). Runtime-verified over two full oscillation periods:
the reported min hit range swept smoothly `1.75m -> 3.28m -> 1.75m`,
tracking the box's `+-0.75m` motion around its `2.5m` center distance with
no jumps, stalls, or NaN gaps -- the map rebuild + simulator refresh cycle
(`RmagineOptixMapSystem`'s `HasSceneChanged()` -> `RmagineOptixSensorSystem`
picking up the new map revision) works correctly on the GPU path, not just
statically.

#### Non-spherical sensor models (Pinhole/O1Dn/OnDn)

`rmagine_embree_sensor_system` and `rmagine_optix_sensor_system` were
hardcoded to `SphereSimulatorEmbree`/`SphereSimulatorOptix` +
`SphericalModel` -- any rotating-2D-lidar-style sensor, but nothing
camera-shaped or with a custom ray pattern. All four of rmagine's sensor
models (`SphericalModel`, `PinholeModel`, `O1DnModel`, `OnDnModel`) share
the same `getWidth()`/`getHeight()`/`getBufferId()`/`getDirection()`/
`getOrigin()`/`range` interface, and all four simulator classes
(`{Sphere,Pinhole,O1Dn,OnDn}Simulator{Embree,Optix}`) share the same
`setTsb()`/`setModel()`/`simulate()` shape -- so both systems now select
one of the four via a new `model_type` SDF element (`spherical`, the
default -- unchanged behavior; `pinhole`; `o1dn`; `ondn`), and dispatch
through shared templated helpers (new `sensor_model_config.hpp/.cpp` for
SDF/YAML parsing into a `SensorModelConfig`, new
`sensor_model_publish.hpp` for the generic `PointCloud2`/`LaserScan`
publish + debug-log code, `LaserScan` skipped at compile time via
`if constexpr` for anything that isn't `SphericalModel`, since only a
single-row spherical scan has a flat-scan equivalent).

New SDF elements, on top of the existing `map_key`/`frame`/`topic_scan`/
`topic_points`/`update_rate`/`debug`/`range_min`/`range_max` (all
unchanged):

- `model_type`: `spherical` (default) / `pinhole` / `o1dn` / `ondn`.
  Unknown values fall back to `spherical` (logged, not fatal).
- Pinhole: `pinhole_width`/`pinhole_height` (pixels), `pinhole_hfov`
  (radians; `pinhole_vfov` optional, derived from the aspect ratio if
  omitted).
- O1Dn/OnDn: `rays_file`, a YAML file with a shared per-pixel-ray schema
  (`width`/`height`/`rays: [{origin: [x,y,z], dir: [x,y,z]}, ...]`,
  row-major). O1Dn uses only the first ray's `origin` (one shared origin,
  N directions); OnDn uses each ray's own `origin` (fully arbitrary,
  non-central ray set) -- the same file can be reused between the two by
  just switching `model_type`, since O1Dn simply ignores every origin but
  the first. There's no reasonable small set of SDF scalars for an
  arbitrary ray pattern, unlike Pinhole's closed-form FOV/resolution, so a
  file was used instead (same rationale as `radarays_ros`'s
  `materials_file`).

A real bug surfaced during runtime verification, not just compile-testing:
the shared `PointCloud2` publish code (carried over from the
Spherical-only original) computed each point as `getDirection(vid, hid) *
range` without adding `getOrigin(vid, hid)`. Invisible for
Spherical/Pinhole (`getOrigin()` is always `{0,0,0}` for both -- a single
shared origin at the sensor frame's own origin), but silently wrong for
O1Dn/OnDn, whose rays can originate away from the sensor frame's origin --
points would silently collapse onto the wrong location. Fixed by adding
the origin back in.

**Runtime-verified**, CPU/Embree and GPU/OptiX both, against the existing
`target_box` fixture-backing worlds (`gz_embree_baseline.sdf`/
`gz_optix_baseline.sdf`, box front face at `2.5m`):

- Spherical (the default, unchanged): both existing `colcon test` fixtures
  (`embree_fixture_baseline`/`embree_fixture_dynamic`) still pass with
  byte-identical results after this refactor -- confirms no regression.
- Pinhole: a 20x20, narrow-FOV (`0.2` rad) camera aimed at the box hit it
  on all 400/400 rays at `2.500m`-`2.525m`, on both Embree and OptiX.
  Matches the analytically expected front-face distance.
- O1Dn: a single forward ray (shared origin `(0,0,0)`) hit the box at
  exactly `2.500m`.
- OnDn: first with two rays sharing the same trivial origin/direction as
  the O1Dn case (confirms basic plumbing, but doesn't exercise what makes
  OnDn different from O1Dn) -- then with two rays given genuinely
  different origins, one offset `2m` forward along the ray's own travel
  direction (`origin (2,0.3,0)`, `dir (1,0,0)`) and one not (`origin
  (0,-0.3,0)`, same `dir`). The first ray correctly reported hitting the
  box after traveling only `0.5m` (not `2.5m`), and the published
  `PointCloud2` points landed at the true hit locations, `(2.5, 0.3, 0)`
  and `(2.5, -0.3, 0)` respectively -- confirms the per-ray origin is both
  used in the actual raycast and correctly reflected in the published
  point, not just carried through as inert metadata (this is also what
  caught the `getOrigin()` publish bug above -- the degenerate
  same-origin OnDn test alone would not have).

This coverage is now permanent, not just a one-off manual check: three new
`colcon test` fixtures (`embree_fixture_pinhole`/`embree_fixture_o1dn`/
`embree_fixture_ondn`, alongside the pre-existing
`embree_fixture_baseline`/`embree_fixture_dynamic` -- 5 total,
`colcon test --packages-select rmagine_gazebo_plugins`), each with its own
committed world (`worlds/gz_embree_{pinhole,o1dn,ondn}.sdf`) and fixture
JSON (`testdata/embree_harmonic/{pinhole,o1dn,ondn}_fixture.json`). The
OnDn fixture is the discriminating two-different-origins case above, not
the degenerate one, so it actually guards the `getOrigin()` regression.

Extending `scripts/embree_fixture_harness.py` for this needed two real
fixes, not just new data files -- the harness had two assumptions baked in
from only ever having Spherical worlds to deal with:

- `capture_world()` unconditionally waited for both `/scan` and `/points`
  before proceeding, but Pinhole/O1Dn/OnDn worlds never publish `/scan` (no
  flat-scan equivalent -- see `PublishLaserScanIfApplicable`) and would
  simply time out. Fixed with a small `EXPECTS_SCAN` dict keyed by world
  name.
- `compare_capture()` special-cased exactly two worlds by name
  (`if world == "baseline": ... else: ...`, the `else` branch assuming
  Classic's "dynamic" motion-span checks). Generalized to key every
  assertion off whether its field is *present* in that world's own fixture
  JSON `expected` dict instead -- new worlds opt into only the checks that
  apply to them (e.g. Pinhole/O1Dn/OnDn's fixtures have no `"scan"` key at
  all, and their `points` blocks skip the motion-span checks that only
  make sense for a moving scene) rather than needing another per-world
  `if` branch in the harness itself.
- O1Dn/OnDn also needed their `rays_file` SDF element resolved to a real
  installed filesystem path at launch time (a committed `.sdf` can't
  contain a build/install-tree-specific absolute path) -- the committed
  world files carry a `RAYS_FILE_PLACEHOLDER` token, substituted by
  `resolved_world_path()` before `gz sim` ever sees the file.

#### Plane/heightmap/ignore_link fixtures (regression pass, 2026-07-29)

A full clean rebuild + `colcon test` regression pass found that the plane,
heightmap, and per-link-ignore geometry support added earlier in this
migration had **no permanent test coverage at all** -- they were only ever
verified via ad-hoc scratchpad SDF worlds that don't survive between
sessions (confirmed: they were gone by the next session). Closed the same
way as the Pinhole/O1Dn/OnDn gap above -- three more `colcon test`
fixtures, same harness, same pattern:

- `embree_fixture_plane` (`worlds/gz_embree_plane.sdf`) -- a vertical wall
  plane with a non-default normal (`-1 0 0`, not the primitive default
  `+Z`), so it actually exercises both plane geometry support and the
  `HasSceneChanged()` non-identity-rotation fix (previously an infinite
  rebuild loop for exactly this case). `center_value` expected ~3.0m.
- `embree_fixture_heightmap` (`worlds/gz_embree_heightmap.sdf` +
  `testdata/embree_harmonic/heightmap_fixture.png`, a committed
  deterministic grayscale pyramid gradient) -- exercises
  `BuildHeightmapEmbreeMesh()`'s image-to-mesh triangulation.
  `center_value` expected ~3.0m (sensor at z=1 hits the rising terrain
  partway across it, once height crosses z=1).
- `embree_fixture_ignore_link` (`worlds/gz_embree_ignore_link.sdf`) --
  **falsifiable**, not just crash-freedom: `multi_link_target::shadow_link`
  (a closer box, in the map plugin's `<ignore_link>` list) sits directly in
  front of `multi_link_target::real_link` (the real target). If
  `ignore_link` regresses, `shadow_link` occludes `real_link` and
  `center_value` reads ~1.8m instead of ~2.5m -- a tight expected range
  around 2.5m catches that regression directly instead of only checking
  that the node didn't crash.

All three, alongside the existing five, run under `colcon test
--packages-select rmagine_gazebo_plugins` (8 total). The heightmap world
needed the same placeholder-resolution treatment as O1Dn/OnDn's
`rays_file` -- `HEIGHTMAP_URI_PLACEHOLDER`, resolved by the same
`resolved_world_path()`, generalized to loop over both placeholder kinds
instead of hardcoding just `rays_file`.

#### OptiX/GPU fixtures (2026-07-29, later same day)

Closed the OptiX-side gap flagged above. `rmagine_gazebo_plugins` gained
4 GPU fixtures under `colcon test --packages-select rmagine_gazebo_plugins`
(12 total now, was 8):

- `optix_fixture_baseline`/`optix_fixture_dynamic` -- `gz_optix_baseline.sdf`/
  `gz_optix_dynamic.sdf` already existed on disk (identical scenes to their
  embree_baseline/embree_dynamic counterparts, only the plugin filenames
  differ) but were never wired into anything at all. Reused the exact same
  expected numeric ranges as the CPU fixtures -- same deterministic
  ray-triangle geometry either way, confirmed to match exactly.
- `optix_fixture_noise` (`worlds/gz_optix_noise.sdf`, new) -- falsifiable
  check on the noise models themselves: `optix_baseline`'s own
  `center_value_stddev` is ~0 (deterministic), so a `center_value_stddev_min`
  well above that floor proves the `<noise type="gaussian">` element is
  actually being applied, not just present in the SDF without effect.
- `optix_fixture_multi` (`worlds/gz_optix_multi.sdf`, new) -- falsifiable
  check on multi-topic fan-out: the extra `scan2`/`points2` topics must
  ALSO receive messages (verified: matched the primary topics' count
  exactly, 21/21), not just the primary `topic_scan`/`topic_points`.
  Needed a small harness extension (`WORLD_EXTRA_TOPICS`, extra
  subscriptions in `TopicCollector`) since this is the first fixture that
  needs to check more than the two hardcoded topic names.

`radarays_gazebo_plugins` gained its own first GPU fixture,
`radarays_fixture_egomotion_gpu` -- `gz_egomotion_radar_gpu.sdf` (the
moving-*sensor* scenario, Tier1 #1, distinct from `dynamic_cpu`'s
moving-*target* scenario) already existed but was never wired in either.
Falsifiable motion checks (`center_value_span_min`,
`center_column_peak_row_span_min`) prove the sensor's own motion actually
shows up in the output (real observed swing: center_value 0.17-2.16m,
peak row span ~55 of 1024), not just "some nonzero data".

#### Mesh-by-URI caching fixture -- and a real bug it found (2026-07-29)

Closed the last item above. Didn't fit the capture/compare-JSON pattern
the other fixtures use (it's about an internal load *count*, not
published topic values), so it's a standalone script instead:
`embree_fixture_mesh_cache` runs `check_mesh_cache_fixture.py` against a
new world, `worlds/gz_embree_mesh_cache.sdf` -- three static entities
sharing one mesh file (`testdata/embree_harmonic/box_test.ply`, a
self-contained unit box, no external dataset dependency), plus an
unrelated moving box to force many rebuild passes over the capture
window. It greps the map system's debug log for a `(cache miss)` marker
(added for this -- there was no way to observe cache hits/misses at all
before) and checks the count stays at exactly 1, shared across all 3
entities and every rebuild pass, not just some nonzero-but-uncounted
"seems to work".

**First run found a real, previously-undiscovered bug, not just a gap**:
count came back 3 initially, growing to 383 within seconds. Root cause in
`rmagine_embree_map_system.cpp`'s (and identically,
`rmagine_optix_map_system.cpp`'s) `BuildStaticMap()`: the cache-hit
condition required `within_pass_mesh_uris_.insert(key).second` in
addition to the key being cached -- meant to guard against a
same-pass "population race" between two entities loading a brand-new key
at the same time. But `BuildStaticMap()` runs single-threaded (confirmed
by inspection -- no threads/async anywhere near it), so that race never
existed, and the guard's only real effect was: whenever 2+ entities share
an already-cached mesh in the same rebuild pass, only the *first* one
gets the legitimate hit -- every other one falls through and reloads the
file from disk (re-running Assimp) again, every single pass, forever.
This silently defeated the entire point of mesh-by-URI caching for
exactly the case it exists for (multiple entities sharing one mesh file)
-- and would keep doing so on every future clean rebuild, invisibly,
since nothing was measuring it. Fixed on both backends: cache hits now
depend only on whether the key is in `mesh_cache_`, full stop --
`within_pass_mesh_uris_` was dead weight after that (removed from both
`.cpp`/`.hpp` pairs). Re-verified: count is 1, stays at 1 across ~26s and
dozens of forced rebuild passes.

Full count across all four packages as of this pass: **66 tests, 0
failures** (`rmagine`: 30, `rmagine_gazebo_plugins`: 13,
`radarays_gazebo_plugins`: 4, `radarays_ros`: 1). Every Classic-parity
gap originally flagged in the systematic audit now has automated
regression coverage; GPU CI going live and the optimizer's richer-scene
work remain the only genuinely blocked/deferred items (need repo access
and a real labeled scene respectively, neither available this session).

#### Build and package integration

The package already supports a gated Harmonic build path:

- `RMAGINE_GZSIM_PORT=ON`

The current build metadata is set up for:

- `gz-sim8`
- `gz-plugin2`
- `gz-transport13`
- `gz-common5`
- `gz-math7`
- `gz-msgs10`
- `gz-rendering8`

ROS 2 dependencies used by the new path include:

- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `tf2_ros`
- Python-side evaluation tooling dependencies

#### Validation tooling

There is now a Harmonic-specific validation layer:

- `gz_embree_baseline.sdf`
- `gz_embree_dynamic.sdf`
- `gz_embree_example.sdf` as a compatibility alias of `dynamic`
- stored fixtures:
  - `testdata/embree_harmonic/baseline_fixture.json`
  - `testdata/embree_harmonic/dynamic_fixture.json`
- capture / compare harness:
  - `capture_embree_fixture`
  - `compare_embree_fixture`

### What Has Only Been Validated in Toy Worlds

These are the things we have evidence for, but only in the controlled validation setup, not yet in the actual simulation stack.

#### Static sensing correctness

In `gz_embree_baseline.sdf`, the Harmonic Embree path has been validated to:

- publish `scan` at about `5 Hz`
- publish `points` at about `5 Hz`
- produce stable finite scan hits
- produce finite point cloud values
- match the simple box geometry numerically

#### Dynamic obstacle responsiveness

In `gz_embree_dynamic.sdf`, the Harmonic Embree path has been validated to:

- detect a moving obstacle
- rebuild the map as the obstacle moves
- refresh the simulator against the new map revision
- change `/scan` over time consistently with obstacle motion
- change `/points` over time consistently with obstacle motion

#### Documentation and repeatable checks

The repo now documents a repeatable Harmonic validation flow in:

- `README.md`
- `TESTING.md`

That validation is still explicitly based on **minimal synthetic worlds**, not on the real robot / vehicle / environment stack.

### What Remains Missing Before This Is Considered Migrated for the Actual Project

#### 1. Real project integration

The biggest missing step is using the new Harmonic Embree path inside the **actual target simulation**, not just the toy worlds.

That includes:

- mounting the sensor in the real robot / vessel / vehicle model
- validating frame semantics in the real model hierarchy
- validating the actual environment geometry, not only one moving box
- confirming the expected ROS 2 topics and downstream consumers still work

#### 2. Full Classic feature parity

The Harmonic path does **not** yet represent full migration parity with the old Classic implementation.

Still missing or incomplete:

- OptiX / GPU path: static and dynamic-scene baselines both done and
  GPU-verified; the primitive-shape scale bug (see above) is fixed on both
  the GPU and CPU/Embree sides now, verified against all existing fixtures
  with no re-capture needed (see `MIGRATION_HANDOFF.md`, item #19)
- ~~non-spherical sensor models (Pinhole/O1Dn/OnDn)~~ -- done, see
  "Non-spherical sensor models (Pinhole/O1Dn/OnDn)" above
- confidence that Classic-era assumptions in the rest of the package are replaced cleanly

A dedicated audit (diffing this package's current `src/gz/*` Harmonic
System code against the Classic plugin code still built via
`RMAGINE_GZSIM_PORT=OFF`, and against `origin/noetic` for anything the
bundled Classic code might itself be missing) found 9 concrete gaps,
roughly in order of how likely they are to matter for a real sensor
(radar/LiDAR mounted on a moving vehicle) rather than a toy validation
world:

1. ~~**PointCloud2 only ever carries x/y/z.**~~ -- done, both CPU/Embree
   and GPU/OptiX. `PublishPointCloud()` (`sensor_model_publish.hpp`) now
   always adds a `ring` field (the row/`vid` index -- well-defined
   identically for every model type, so unlike `LaserScan` it isn't
   gated to Spherical only), plus optional `normal_x/y/z`/`obj_id`/
   `face_id` fields when the caller provides them via a new
   `PointCloudExtras` struct. Both sensor systems now request
   `Bundle<Ranges, Normals, ObjectIds, FaceIds>` instead of just
   `Ranges` (Embree computes these as part of the same intersection
   query; OptiX needs an extra explicit VRAM->RAM download per field,
   same pattern as `ranges` already used).

   **Runtime-verified**, both backends, against the plane-geometry test
   world above: real, physically-plausible values throughout -- `ring=0`
   (correct, single-row Spherical model); `face_id` alternating `0`/`1`
   exactly matching the ground plane's own 2-triangle mesh
   (`genPlane()`'s `faces[0]={1,0,3}`/`faces[1]={3,2,1}`); `obj_id=0`
   constant (correct, one object in the scene); consistent unit-length
   surface normals across the flat plane. All 16 existing `colcon test`
   fixtures still pass unchanged (their comparison harness only reads
   x/y/z by name, so the new fields are transparently ignored there).
2. ~~**Plane geometry is silently unsupported**~~ -- done, both
   CPU/Embree and GPU/OptiX. rmagine already ships `EmbreePlane`/
   `OptixPlane` primitives (unit 1x1 quad in the XY plane, normal +Z) --
   wired into both map systems' `IsSupportedGeometry()`/`GeometryKey()`/
   primitive-construction switch, same pattern as Box/Sphere/Cylinder.
   SDF planes can specify an arbitrary `<normal>` (default `0 0 1`), so an
   extra rotation from +Z to that normal is composed onto the entity's
   world pose before instantiation (`gz::math::Quaterniond::SetFrom2Axes`).

   Runtime-verified, both backends: a ground plane 3m below a
   straight-down-facing sensor hit at exactly 3.000-3.004m; a *vertical
   wall* plane (`<normal>1 0 0</normal>`, the non-trivial rotation case) 5m
   from a sensor looking along +X hit at exactly 5.001-5.006m on both
   Embree and OptiX.

   Found and fixed a real bug doing this: `HasSceneChanged()` (the
   per-tick check for whether the map needs rebuilding) computes each
   visual's pose independently from `BuildStaticMap()`, and didn't apply
   the same plane-normal rotation -- so for any plane with a non-default
   normal, the comparison permanently disagreed with what was actually
   stored, causing an infinite rebuild loop (every single tick, confirmed
   at ~1000/sec in the vertical-wall test). Invisible for a default-normal
   ground plane (identity rotation, so the mismatch was always zero) --
   only the vertical-wall test caught it. Fixed by applying the identical
   rotation in `HasSceneChanged()` too, on both the Embree and OptiX map
   systems. All 5 existing `colcon test` fixtures still pass unchanged.
3. ~~**Heightmap geometry is unsupported**~~ -- done, both CPU/Embree and
   GPU/OptiX. Classic's own conversion
   (`embree_conversions.cpp`/`optix_conversions.cpp`) used
   `gazebo::common::HeightmapDataLoader`, a Gazebo-Classic-only API with
   no gz-sim equivalent, so this reimplements the image-to-mesh
   conversion directly instead of porting that code: loads the
   heightmap's grayscale image via `gz::common::Image` (still available,
   newer version), builds a triangulated grid mesh from it (height =
   pixel value x the heightmap's own `<size>` Z-scale), and hands that to
   `EmbreeMesh`/`OptixMesh` the same way a regular mesh file would be
   (OptiX side wrapped in an `OptixInst` too, same requirement as the
   existing MESH case). Resolution is capped at 100x100 regardless of the
   source image's own resolution -- a first working implementation, not
   configurable yet.

   **Runtime-verified**, both backends: a synthetic 32x32 heightmap image
   (a flat square plateau in the center, value 255, against a 0
   background) with `<size>10 10 2</size>` under a straight-down-facing
   sensor 5m up hit at exactly `3.000-3.001m` (5m - 2m plateau height)
   directly above the plateau, and exactly `5.000-5.001m` (ground level,
   no elevation) 4m off to the side -- confirms the actual elevation
   *shape* came through correctly, not just a flat placeholder mesh at
   the wrong height. All 16 existing `colcon test` fixtures still pass
   unchanged.
4. ~~**Visual-level `<scale>` is hardcoded to identity**~~ -- investigated,
   turned out **not to be a real gap**: SDF's `<visual>` element has no
   `<scale>` concept at all in Harmonic/gz-sim (checked against
   sdformat's own `visual.sdf` schema across every format version, and
   `sdf::Visual`'s C++ API -- no `Scale()` method exists). Classic's
   `gazebo::msgs::Visual` had its own protobuf `scale` field with no SDF
   equivalent; that's a Classic-specific concept that doesn't carry over,
   not something Harmonic dropped. A mesh's own `<geometry><mesh><scale>`
   (a *different*, geometry-level scale) was already read and applied
   correctly the whole time. Corrected the stale, misleading code comment
   that prompted this finding in the first place.
5. ~~**Per-link self-tagging ignore mechanism lost.**~~ -- functionally
   closed, both CPU/Embree and GPU/OptiX, though **not** via true
   self-tagging like Classic's embedded `<rmagine_ignore/>`: checked
   gz-sim's ECS directly first (`Link`/`Model`/`Visual` components are all
   empty markers -- confirmed via their component data types -- with no
   mechanism to preserve an arbitrary custom SDF tag per-link/model), so
   Classic's exact architecture genuinely doesn't port. Instead extended
   the *existing* `ignore_model` mechanism (an externally-configured list
   on the map system's own SDF) with a new `ignore_link` element
   (`"model_name::link_name"`, repeatable, same pattern) -- same
   functional outcome (ignore one link of a multi-link model without
   ignoring the whole model), different mechanism (external
   configuration vs. self-tagging).

   **Runtime-verified**: a two-link model with a closer "ignored" box (at
   1.5m, would normally be the hit) and a farther "visible" box (at
   4.5m) behind it -- with `<ignore_link>obstacle::ignored_link</ignore_link>`
   configured, the ray passed straight through the closer box and hit the
   farther one instead (`4.500-4.501m`, not `1.500m`). All 16 existing
   `colcon test` fixtures still pass unchanged.
6. ~~**OptiX noise models are completely missing on the GPU sensor
   path.**~~ -- done. All three of rmagine's existing CUDA noise classes
   (`GaussianNoiseCuda`/`UniformDustNoiseCuda`/`RelGaussianNoiseCuda`,
   unchanged since Classic) wired into `RmagineOptixSensorSystem` via a
   repeated `<noise type="gaussian|uniform_dust|rel_gaussian">` SDF
   element, applied directly to `res_gpu.ranges` (in VRAM) right after
   `simulate()`, before download -- same ordering Classic used. Flat
   under the plugin itself rather than nested under a `<ray>` wrapper
   like Classic did, to match every other SDF element this plugin
   already uses (`min_angle`/`max_angle`/`samples`/etc are all flat, no
   `<ray>` nesting -- introducing Classic's nesting for just this one
   element would be inconsistent). CPU/Embree was never in scope here --
   Classic's own noise support was OptiX-only too.

   **Runtime-verified**: a gaussian noise element (`stddev: 0.05`) on the
   ground-plane test world turned the previously rock-solid exact
   `3.000-3.004m` range into a realistic scatter (`2.919m-3.157m` across
   several ticks) still centered on the true `3.0m` distance -- clearly
   different behavior from the no-noise case, not a no-op. All 16
   existing `colcon test` fixtures (none of which configure noise) still
   pass unchanged.
7. **Full-scene rebuild on any change** -- **partially addressed**, both
   CPU/Embree and GPU/OptiX, not fully. True incremental add/remove/
   transform/scale diffing like Classic's `UpdateState` (persistent
   per-geometry identity across rebuilds, updating only what actually
   changed instead of tearing down and rebuilding the whole scene) is a
   large architectural rewrite -- out of proportion with this being a
   Tier 3 item, and risky to attempt this late in a long session against
   an otherwise-stable, well-tested system. Scoped down instead to the
   single biggest cost driver the full-rebuild-every-time design has:
   **mesh-by-URI caching**, keyed by `(resolved uri, mesh scale)`. The
   already-cached primitive shapes (box/sphere/cylinder/plane) never paid
   this cost; ordinary mesh files did -- re-read and re-parsed via
   Assimp/gz-common on *every single rebuild*, even one triggered by an
   unrelated entity elsewhere in the scene that has nothing to do with
   that mesh. Now cached and reused across rebuilds the same way
   primitives already are: a cached base scene, instantiated fresh per
   use (needed for correctness -- a raw, non-instanced geometry ties its
   transform to the geometry object itself, so directly sharing one
   across multiple differently-posed entities would silently corrupt
   whichever was set last). A small within-pass dedup guard falls back
   to a fresh load if the exact same mesh key legitimately appears more
   than once in a single rebuild pass, rather than risk a subtly wrong
   shared-cache-population race for that rarer case.

   **Runtime-verified**, both backends, under sustained repeated-rebuild
   load: a world with the real `avz_no_roof.stl` mesh (the same one used
   throughout this whole migration) plus an *unrelated* moving box
   elsewhere in the scene, forcing a full map rebuild on every tick.
   CPU: the existing `radarays_fixture_dynamic_cpu`/
   `radarays_fixture_dynamic_multi_cpu` `colcon test` fixtures already
   exercise exactly this (repeated rebuilds while a static mesh map stays
   cached) and still pass with their committed numeric thresholds
   unchanged. GPU (no equivalent fixture exists yet to lean on): a
   purpose-built test ran 524 consecutive rebuilds, and every single one
   produced the identical, correct raycast result (`min=0.712,
   max=9.139`) -- confirms the cached, reused geometry stays valid and
   uncorrupted across hundreds of rebuild cycles, not just the first one.
8. ~~**No Gazebo-native mesh loader fallback**~~ -- done, both CPU/Embree
   and GPU/OptiX. Assimp stays the primary loader (it's the
   well-exercised, verified-working path for every mesh this workspace
   actually uses), but now falls back to `gz::common::MeshManager` (still
   available in Harmonic, just a newer version -- lives in gz-common's
   `graphics` component, a separate library from the `Image`-carrying
   base component already used for heightmaps, which is why this needed
   its own `find_package`/link addition) if Assimp fails to parse the
   file. Order flipped from Classic's own GAZEBO-then-INTERNAL, for the
   reason above.

   **Verified with a standalone test program** (not the full gz-sim
   pipeline -- Assimp's format support is broad enough that naturally
   forcing it to fail on a real, valid mesh file wasn't practical to
   arrange, so this proves the actual conversion logic directly instead,
   the same root-cause-repro approach used earlier in this migration for
   OptiX bugs): loaded a hand-written unit-cube `.obj` via
   `gz::common::MeshManager` using the exact vertex/index-extraction code
   from `BuildEmbreeMeshFromGzCommon`, built a real `EmbreeMesh` from it,
   and raycast against it -- hit at exactly `4.5` (the analytically
   correct front-face distance). All 16 existing `colcon test` fixtures
   still pass unchanged (all use Assimp-parseable files, so the fallback
   path isn't exercised by them, only the primary path -- confirms no
   regression there).
9. ~~**Multi-topic/multi-message-type fan-out lost.**~~ -- done, both
   CPU/Embree and GPU/OptiX, for the two message types Harmonic actually
   supports (`LaserScan`/`PointCloud2` -- Classic's third type,
   `sensor_msgs/PointCloud`, is long deprecated in ROS 2 with no
   meaningful use here, so it wasn't reintroduced). A new repeatable
   `<output><topic>.../topic><type>scan|points</type></output>` SDF
   element adds arbitrary extra publishers alongside the existing
   `topic_scan_`/`topic_points_` default (unchanged, still the first/only
   output if no `<output>` elements are given) -- `PublishLaserScanIfApplicable`/
   `PublishPointCloud` (`sensor_model_publish.hpp`) now take a *vector* of
   publishers instead of one, building each message once and publishing
   it to every configured output of that type.

   **Runtime-verified**: added a second `points` output
   (`<output><topic>points_extra</topic><type>points</type></output>`)
   alongside the existing default `/points` topic -- both received the
   identical message count (33/33) over the same observation window,
   confirming genuine simultaneous fan-out, not just the default topic
   renamed. All 16 existing `colcon test` fixtures (none of which
   configure extra outputs) still pass unchanged.

New capabilities Harmonic has that Classic didn't: named/versioned
`MapRegistry` map keys (multiple independent maps + staleness detection,
vs. Classic's one implicit global map per world), and an explicit
human-readable reason string for every detected scene change (useful for
debugging, `HasSceneChanged()`).

See MIGRATION_HANDOFF.md's cross-package gap list for how this fits
against the other 2 ROS-coupled packages' own gaps and a proposed tackle
order.

#### 3. TF and frame-tree strategy for real use

The toy-world validation shows sensor data is correct in `sensor_link`, but a project-grade TF strategy is still not fully settled.

Still missing:

- robust `world` / robot / sensor frame integration for the real system
- confirmation of how TF should be owned in the actual migrated stack
- validation in RViz and downstream tools using the real project frame tree

#### 4. Performance and scaling validation

The dynamic toy world proves correctness at small scale, but not performance suitability for the real project.

Still missing:

- validation on real scene complexity
- acceptable rebuild / update cost under real workloads
- a decision on whether current full-scene rebuild behavior is sufficient
- confirmation that dynamic obstacle handling scales enough for the target use case

#### 5. Automated migration-grade testing

The fixture harness now runs as real `colcon test` tests (`ament_add_test`,
see `CMakeLists.txt`) — `colcon test --packages-select
rmagine_gazebo_plugins` executes `embree_fixture_baseline`/`embree_fixture_dynamic`
and `colcon test-result` aggregates them. That's still not the same thing
as a full automated CI test suite for the migrated package.

Still missing:

- actually triggering these on a real CI runner (the GitHub Actions
  workflow exists and calls `ci_build_and_test.sh`, but has never run on a
  real runner — see `MIGRATION_HANDOFF.md`, "What Has Already Been Done"
  #15's honesty note)
- a broader regression matrix
- tests tied to real project scenarios, not only the toy validation worlds

### Practical Conclusion

The Harmonic Embree CPU path is now good enough to treat as a **working port prototype**.

The package should **not yet be considered fully migrated for the real project** until it has been validated in the actual simulation stack.

The next migration milestone should move from toy-world validation to **real model / real environment integration**, not add more toy-world complexity first.

## Gazebo Harmonic (gz-sim) Validation Worlds

There are now two explicit validation worlds:

### 1. Baseline world

Use this first to validate the sensing baseline with one fixed box and no moving geometry:

```console
gz sim -v 4 $(ros2 pkg prefix rmagine_gazebo_plugins)/share/rmagine_gazebo_plugins/worlds/gz_embree_baseline.sdf
```

Expected behavior:
- `/scan` should contain finite hits on the fixed box.
- `/points` should contain finite points, not only NaNs.
- The center region of the scan should be near the front face distance of the box.

### 2. Dynamic world

Use this after the baseline world behaves correctly:

```console
gz sim -v 4 $(ros2 pkg prefix rmagine_gazebo_plugins)/share/rmagine_gazebo_plugins/worlds/gz_embree_dynamic.sdf
```

Expected behavior:
- `target_box` moves deterministically along the x axis.
- The map system reports pose-change rebuilds.
- The sensor system refreshes when the map revision changes.
- `/scan` and `/points` should vary over time with the moving box.

For backward compatibility, `worlds/gz_embree_example.sdf` mirrors the dynamic world and should be kept identical to it.

Topics for both worlds:
- `scan` (`sensor_msgs/msg/LaserScan`)
- `points` (`sensor_msgs/msg/PointCloud2`)

Current Harmonic validation files:

- `worlds/gz_embree_baseline.sdf`
- `worlds/gz_embree_dynamic.sdf`
- `worlds/gz_embree_example.sdf` (dynamic-world compatibility alias)

## Stored Topic Fixtures

The Harmonic Embree port now includes a small fixture harness for repeatable topic evaluation.

Committed reference fixtures live under:

- `testdata/embree_harmonic/baseline_fixture.json`
- `testdata/embree_harmonic/dynamic_fixture.json`

Use the installed entrypoints with `ros2 run`:

```console
ros2 run rmagine_gazebo_plugins capture_embree_fixture baseline
ros2 run rmagine_gazebo_plugins compare_embree_fixture baseline
```

and:

```console
ros2 run rmagine_gazebo_plugins capture_embree_fixture dynamic
ros2 run rmagine_gazebo_plugins compare_embree_fixture dynamic
```

By default, captures are written to:

- `/tmp/rmagine_embree_baseline_capture.json`
- `/tmp/rmagine_embree_dynamic_capture.json`

The capture tool launches the requested world headlessly, waits for `/scan` and `/points`, records a short fixed sample window, and writes a compact JSON result containing:

- scan and point-cloud publish-rate estimates
- finite-hit counts
- representative center-beam and point-cloud probe series
- a small representative sample slice

The compare tool checks the latest capture against the committed fixture with tolerant, world-specific rules:

- baseline expects stable finite hits near the fixed box distance
- dynamic expects time-varying probe metrics as the box moves

# rmagine_gazebo_plugins (WIP)

Range sensor plugins for Gazebo using the sensor simulation library [rmagine](https://github.com/uos/rmagine). 
With rmagine's OptiX backend it is possible to simulate depth sensor data directly on your RTX graphics card. With Embree backend you can simulate any provided sensor online on your CPU.
Embree and OptiX are libraries for raytracing and build BVH acceleration structures on the scene for faster ray traversals.
After building these acceleration structures, you can simulate depth sensors on CPU or GPU without getting perfomance issues even in large Gazebo worlds.

Youtube-Video:

<div align="center">
<a href="http://www.youtube.com/watch?feature=player_embedded&v=IOrBxiW0AmY
" target="_blank" >
  <img src="https://i.ytimg.com/vi/IOrBxiW0AmY/maxresdefault.jpg" 
  alt="Rmagine Gazebo Plugin YT Video" width="80%" style="max-width: 500px" height="auto" border="10" />
</a>
</div>



## Examples

After compiling 

### `example.launch.xml`

```console
ros2 launch rmagine_gazebo_plugins example.launch.xml
```

Simulates a 3d lidar at 20hz on Embree backend.  
To use OptiX backend, run

```console
ros2 launch rmagine_gazebo_plugins example.launch.xml rmagine:=optix
```

Open RViz set fixed frame to `base_footprint` and visualize topic `laser3d/pcl`.

### `rotating_scanner.launch.xml`

```console
ros2 launch rmagine_gazebo_plugins rotating_laser.launch.xml
```

or with OptiX backend

```console
ros2 launch rmagine_gazebo_plugins rotating_laser.launch.xml rmagine:=optix
```

Open RViz set fixed frame to `base_footprint` and visualize topic `laser2d/scan`.
In Gazebo-GUI find the `laser2d` link at model `robot_sensor`.
To let the scanner rotate go to Gazebo-GUI:
1. Right-click on the `laser2d` link at model `robot_sensor`
2. Click "Apply Force/Torque"
3. Set Torque to y=0.5 
4. Click "Apply Torque"

Now the scanner cylinder should rotate in Gazebo as well as in RViz.


## Usage

### 1. Installation

#### Rmagine

Follow instructions of Rmagine library installation. Compile with Embree or OptiX backends for CPU or GPU support respectively.
You can clone rmagine into your ROS workspace's src folder: `colcon_ws/src`.

#### Compilation
Clone this repository to your ROS workspace (src folder), `colcon_ws/src`:

```console
git clone git@github.com:uos/rmagine_gazebo_plugins.git
```

In `colcon_ws` folder compile with

```console
colcon build
```

Depending on which backends were installed during Rmagine installation the following plugins are built:

1. Embree
    - World-Plugins: `rmagine_embree_map_gzplugin`
    - Sensor-Plugins: `rmagine_embree_spherical`
2. OptiX
    - World-Plugins: `rmagine_optix_map_gzplugin`
    - Sensor-Plugins: `rmagine_optix_spherical`


### 2. Sensor Registration

The rmagine sensors are implemented as new gazebo sensors. They need to be registered first. To do that, you need to add `librmagine_embree_sensors_gzregister.so` or `librmagine_optix_sensors_gzregister.so` to the arguments of the gazebo execution call.


**Embree Example**

1. Gazebo executable:

```console
gazebo -s librmagine_embree_sensors_gzregister.so
```

2. ROS 2 launch file

```xml
<executable cmd="gzserver -s librmagine_embree_sensors_gzregister.so ..." />
<executable cmd="gzclient" />
```

> [!NOTE]
> See launch files in this repository for more details.

### 3. Map Plugins

Embree sensor plugins require one Embree map plugin running.
OptiX sensor plugins require one OptiX map plugin running.
In world-files the map plugins can be enabled as follows:

```xml
<world>
...

<!-- Embree Map Plugin -->
<plugin name='rmagine_embree_map' filename='librmagine_embree_map_gzplugin.so'>
</plugin>

<!-- Optix Map Plugin -->
<plugin name='rmagine_optix_map' filename='librmagine_optix_map_gzplugin.so'>
</plugin>

</world>
```

The map plugins construct a acceleration structure over the Gazebo scene.
As soon as the gazebo scene changes, the acceleration structure is updated accordingly.
Some other examples are located in the worlds folder.

To increase the performance sdf entities can be marked to be ignored by the map plugins.
For example, if you know that your 3D lidar never scans the robot it is attached to, you may consider excluding the entire robot of the map plugins.

To achieve that in world-files just add an `rmagine_ignore` tag to the model:

```xml
<world>

<!-- Exclude single model from map -->
<model name='plane1_model'>
    <rmagine_ignore/>
    ...
</model>

<!-- Or exclude single link from map -->
<model name="plane2_model">
  ...
  <link name="plane2_link">
    ...
    <rmagine_ignore/>
  </link>

</model>

</world>
```

How to add ignores in urdf-files will be explained in the next section.

### 4. Sensors

**2D Laser**

```xml
<gazebo reference="laser2d">
    <sensor type="rmagine_embree_spherical" name="laser2d">
      <pose>0 0 0 0 0 0</pose>
      <always_on>true</always_on>
      <update_rate>60</update_rate>

      <ray>
        <scan>
          <horizontal>
            <min_angle>${-M_PI}</min_angle>
            <increment>${1.0 * M_PI / 180.0}</increment>
            <samples>360</samples>
          </horizontal>
        </scan>

        <range>
          <min>0.0</min>
          <max>10.0</max>
        </range>

        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
        
      </ray>
    </sensor>
</gazebo>
```

**3D Laser**

```xml
<gazebo reference="laser3d">
    <sensor type="rmagine_embree_spherical" name="laser3d">
      <pose>0 0 0 0 0 0</pose>
      <always_on>true</always_on>
      <update_rate>60</update_rate>

      <ray>
        <scan>
          <horizontal>
            <min_angle>${-M_PI}</min_angle>
            <increment>${1.0 * M_PI / 180.0}</increment>
            <samples>360</samples>
          </horizontal>
          <vertical>
            <min_angle>${-60.0 * M_PI / 180.0}</min_angle>
            <increment>${1.0 * M_PI / 180.0}</increment>
            <samples>120</samples>
          </vertical>
        </scan>

        <range>
          <min>0.0</min>
          <max>80.0</max>
        </range>

        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
    </sensor>
</gazebo>
```

As in world-files, ignores can be added to URDF files:

```xml
<robot>
...

<!-- Ignore the entire robote-->
<gazebo>
    <rmagine_ignore/>
</gazebo>

<!-- Ignore a link. Useful if you want to ignore the scanner visual -->
<gazebo reference="my_scanner_link">
    <rmagine_ignore/>
</gazebo>
</robot>
```

### 5. Noise

Currently noise models are implemented as preprocessing steps directly on the simulated ranges data. Any of the following noise models can be chained to generate complex combined noise models.

1. Gaussian Noise

Apply gaussian noise $N(\mu, \sigma)$ to simulated ranges.

| Parameter |  Description  |
|:---------:|:-------------:|
| `mean` | Mean $\mu$ of normal distributed noise |
| `stddev` | standard deviation $\sigma$ of normal distributed noise |

Example:

```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.01</stddev>
</noise>
```

2. Relative Gaussian Noise

Apply gaussian noise $N(\mu, \sigma_r)$ to simulated ranges. Here, the standard deviation varies depending on distance.


| Parameter |  Description  |
|:---------:|:-------------:|
| `mean` | Mean $\mu$ of normal distributed noise |
| `stddev` | standard deviation $\sigma$ of normal distributed noise |
| `range_exp` | range exponent $c$ to compute range based stddev: $ \sigma_r = \sigma \cdot r^{c} $ |


Example:

```xml
<noise>
  <type>rel_gaussian</type>
  <mean>0.0</mean>
  <stddev>0.002</stddev>
  <range_exp>1.0</range_exp>
</noise>
```

3. Uniform Dust Noise

Apply uniform dust noise to simulated ranges. Assuming some small particles could be hit by the range sensor that are not modeled by the scene, use this noise type. 

Parameters:

| Parameter |  Description  |
|:---------:|:-------------:|
| `hit_prob` | Probability of a ray hitting a particle in one meter free space. |
| `return_prob` | Probability of a ray hitting dust returns to sender depending on particle distance |

Example:

```xml
<noise>
  <type>uniform_dust</type>
  <hit_prob>0.0000001</hit_prob>
  <return_prob>0.5</return_prob>
</noise> 
```


**Noise Chaining**

Example of using the gaussian model first and the uniform dust model second:

```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.002</stddev>
</noise>

<noise>
  <type>uniform_dust</type>
  <hit_prob>0.0000001</hit_prob>
  <return_prob>0.5</return_prob>
</noise> 
```


## ROS Plugin

This plugin generates ROS-messages of the simulated data and writes them to specified ROS-topics.
The following ROS-Adapter are available dependend on your sensor type:


`librmagine_optix_ros_gzplugin.so`
- sensor types: `rmagine_optix_spherical`

`librmagine_embree_ros_gzplugin.so`
- sensor types: `rmagine_embree_spherical`



Supported `output` messages are:
- `sensor_msgs/msg/LaserScan`
- `sensor_msgs/msg/PointCloud`
- `sensor_msgs/msg/PointCloud2`

Examples - this time using OptiX.

**2D Laser**

```xml
<gazebo reference="laser2d">
    <sensor type="rmagine_optix_spherical" name="laser2d">
      <pose>0 0 0 0 0 0</pose>
      <always_on>true</always_on>
      <update_rate>60</update_rate>

      <ray>
        <scan>
          <horizontal>
            <min_angle>${-M_PI}</min_angle>
            <increment>${1.0 * M_PI / 180.0}</increment>
            <samples>360</samples>
          </horizontal>
        </scan>

        <range>
          <min>0.0</min>
          <max>10.0</max>
        </range>

        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
        
      </ray>

      <plugin name="rmagine_ros_laser2d" filename="librmagine_optix_ros_gzplugin.so">
          <frame>laser2d</frame>
          <outputs>
            <output name="scan">
              <msg>sensor_msgs/LaserScan</msg>
              <topic>laser2d/scan</topic>
            </output>

            <output name="pcl">
              <msg>sensor_msgs/PointCloud</msg>
              <topic>laser2d/pcl</topic>
            </output>
          </outputs>
      </plugin>
    </sensor>
</gazebo>
```

**3D Laser**

```xml
<gazebo reference="laser3d">
    <sensor type="rmagine_optix_spherical" name="laser3d">
      <pose>0 0 0 0 0 0</pose>
      <always_on>true</always_on>
      <update_rate>60</update_rate>

      <ray>
        <scan>
          <horizontal>
            <min_angle>${-M_PI}</min_angle>
            <increment>${1.0 * M_PI / 180.0}</increment>
            <samples>360</samples>
          </horizontal>
          <vertical>
            <min_angle>${-60.0 * M_PI / 180.0}</min_angle>
            <increment>${1.0 * M_PI / 180.0}</increment>
            <samples>120</samples>
          </vertical>
        </scan>

        <range>
          <min>0.0</min>
          <max>80.0</max>
        </range>

        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>

      <plugin name="rmagine_ros_laser3d" filename="librmagine_optix_ros_gzplugin.so">
          <frame>laser3d</frame>
          <outputs>
            <output name="pcl">
              <msg>sensor_msgs/PointCloud</msg>
              <topic>laser3d/pcl</topic>
            </output>

            <output name="pcl2">
              <msg>sensor_msgs/PointCloud2</msg>
              <topic>laser3d/pcl2</topic>
            </output>
          </outputs>
      </plugin>
    </sensor>
</gazebo>
```

## Work in Progress

This is a pre-release. There is still some work to do for the first stable release:

- Implemented: SphericalModel. TODO: PinholeModel, O1DnModel, OnDnModel
  (this refers to the original Gazebo Classic plugins in this section --
  the new Harmonic/gz-sim port further up (`rmagine_embree_sensor_system`/
  `rmagine_optix_sensor_system`) already supports all four via `model_type`)
- Tests: More tests on different devices. Let me know, if you had problems integrating the rmagine_gazebo_plugins into your project.

Nice-to-Have:
- Add segmenting functionallity: Store labeled sensor data from a list of poses in a commonly used file format

Known Issues:

- Sometimes the Gazebo simulation needs to be started twice in order to get everything started (blocking threads?)
- "Core dumped" on exit:
```bash
[Dbg] [rmagine_embree_map_gzplugin.cpp:52] [RmagineEmbreeMap] Destroyed.
terminate called after throwing an instance of 'boost::wrapexcept<boost::lock_error>'
terminate called recursively
  what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
Aborted (core dumped)
```


### Known Bug-Fixes:

On my system, Gazebo finds all rmagine libraries automatically. If that is not the case for you, try appending your ROS workspace `your_ws` to the Gazebo search pathes:

```console
export GAZEBO_PLUGIN_PATH=~/your_ws/devel/lib:$GAZEBO_PLUGIN_PATH
```
