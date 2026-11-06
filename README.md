# blender_rosbag_importer

Blender add-on that imports ROS bags (ROS1 `.bag`, ROS2 `.db3`/`.mcap`, via
[`rosbags`](https://pypi.org/project/rosbags/)) as animated scenes:

- `/tf` and `/tf_static` become keyframed Empties, one per child frame, parented to
  each other to reproduce the TF tree.
- Point cloud topics (`sensor_msgs/PointCloud2`) become ONE native Blender Point
  Cloud object per topic, parented to the Empty matching the message's
  `header.frame_id`. Its geometry is rewritten in place as the timeline scrubs (a
  `frame_change_pre` handler swaps in whichever scan(s) bracket the current frame)
  rather than keyframing per-point data or stacking one object per scan. Consecutive
  scans crossfade into each other via a Geometry Nodes modifier rather than
  snapping - see "Point cloud transitions & color" below.
- Grid map topics (`grid_map_msgs/GridMap`, from ETH/ANYbotics' `grid_map` ROS
  package - the standard elevation-map message in legged-robotics work) become ONE
  animated heightfield Mesh object per topic, parented the same way and updated the
  same way (rewritten in place every frame, not one object per scan) - see "Grid map
  heightfields" below.
- Image topics (`sensor_msgs/Image` or `CompressedImage`) are extracted to a numbered
  frame sequence on disk and loaded as a Blender image sequence on an Image Empty,
  best-effort parented to the TF frame matching the topic's name prefix. Held at
  whichever message's frame most recently passed via a `frame_change_pre` handler,
  exactly like point clouds/grid maps, rather than trusting Blender's own sequence
  player to advance one file per Blender frame - see "Fixed: image pink flash /
  cross-modality desync" below for why that used to be a real, visible bug.
- Playback is time-aligned to start at frame 0, anchored to the earliest message
  among the selected topics - ONE shared `TimeAlignment` instance, used by every
  importer, so "frame N" means the same real bag timestamp for every modality.

## Layout

The add-on is the **inner** `blender_rosbag_importer/` package; `build.py` zips it up
with its dependencies.

```
blender_rosbag_importer/          # repo root
├── blender_rosbag_importer/      # <- the actual Blender add-on package
│   ├── blender_manifest.toml      extension metadata (build.py appends wheels/platform)
│   ├── __init__.py                bl_info + register/unregister
│   ├── preferences.py             "Install Dependencies" button (see below)
│   ├── bag_reader.py              rosbags wrapper - no bpy import, unit-testable
│   ├── pointcloud2_decode.py      PointCloud2 -> numpy (no bpy import)
│   ├── point_cloud_cache.py       decode + cache a PointCloud2 topic to .npz (no bpy import)
│   ├── grid_map_decode.py         GridMap -> numpy heightfield (no bpy import)
│   ├── grid_map_cache.py          decode + cache a GridMap topic to .npz (no bpy import)
│   ├── image_decode.py            Image/CompressedImage -> numpy/bytes (no bpy import)
│   ├── tf_tree.py                 /tf + /tf_static -> per-frame transform time series
│   ├── time_alignment.py          bag timestamp -> Blender frame number
│   ├── frame_bracket.py           shared hold-last-scan/frame logic (no bpy import)
│   ├── import_point_clouds.py     bpy: TF empties + PointCloud objects
│   ├── point_cloud_nodes.py       bpy: GN crossfade + color-field modifier
│   ├── import_grid_map.py         bpy: animated heightfield Mesh objects
│   ├── grid_map_nodes.py          bpy: GN holes + color-source modifier
│   ├── color_ramps.py             bpy: shared rainbow Color Ramp builder
│   ├── import_images.py           bpy: frame extraction + animated Image Empty
│   ├── properties.py              RosbagTopicEntry PropertyGroup (per-topic checkbox row)
│   ├── ui_list.py                 UIList rendering the topic checklist
│   └── operators.py               IMPORT_OT_rosbag - the actual "File > Import > ROS Bag"
└── build.py                       builds the per-platform extension zips into dist/
```

The bag-parsing/TF/time-alignment/decode/cache/`frame_bracket` modules never import
`bpy` (nor, as of `point_cloud_cache.py`/`grid_map_cache.py`, actually require
`rosbags` to be installed just to *import* the module - only a
`TYPE_CHECKING`-guarded hint uses `bag_reader.BagReader`, so
they can be exercised with a plain fake reader double), so they can be run with
plain `python3` outside Blender. Only
the `import_*`/`operators`/`properties`/`ui_list`/`preferences`/`point_cloud_nodes`/
`grid_map_nodes`/`color_ramps`/`materials` modules touch `bpy`.

## Installing

Blender 4.2+. Grab the zip for your platform (build it with `python build.py`, which
writes `dist/blender_rosbag_importer-<platform>.zip` for windows-x64, linux-x64,
macos-arm64 and macos-x64) and drag it onto a Blender window, or use Edit >
Preferences > Get Extensions > Install from Disk. `rosbags` and its dependencies
ship inside the zip as wheels, so there's nothing else to install.

Then File > Import > ROS Bag (.bag/.db3/.mcap).

Installing the inner `blender_rosbag_importer/` folder directly (legacy add-on
install) still works, but it has no bundled wheels: open the add-on's preferences
and click **Install Dependencies** to pip-install `rosbags` into a `vendor/` folder
inside the add-on.

## Usage notes

- The topic checklist appears in the file browser's side panel once you've picked a
  bag file. Point cloud, grid map, and image topics default to selected; everything
  else defaults to skipped. Each row's "Import As" dropdown can override the
  guessed kind.
- `Point Cloud Stride` / `Grid Map Stride` / `Image Stride` import every Nth message
  - use this to keep large bags tractable. `Max Points/Scan` decimates each
  individual point cloud message (0 = no limit); there's no equivalent for grid maps
  since their cell count is fixed by the map's own `resolution`/`length_x`/`length_y`.
- A point cloud or grid map topic publishing under more than one `frame_id`, or a TF
  frame whose parent changes over time, logs a warning to Blender's console rather
  than failing the import. A grid map topic whose grid size (rows/cols) changes
  partway through the recording logs a warning too, and scans that don't match the
  first one's size are skipped (the mesh's vertex/face topology is only built once,
  from the first scan - see "Grid map heightfields" below).

## Point cloud transitions & color

Each point-cloud topic gets a single Point Cloud object with a shared "ROSBag
Crossfade" Geometry Nodes modifier (`point_cloud_nodes.py`), which does two things:

- **Crossfade instead of a hard cut.** Consecutive scans generally have different
  point counts with no stable per-point correspondence (ROS doesn't attach
  persistent point IDs), so a literal per-point position lerp between "point i of
  scan A" and "point i of scan B" isn't meaningful - it would slide unrelated points
  around rather than tracking a surface. Instead, the object always holds BOTH the
  outgoing and incoming scan's points at once (tagged with `_pc_src`/`_pc_rand`
  attributes), and the modifier's `Blend Factor` input - driven every frame from
  Python, smoothstep-eased between the two scans' frames - stochastically deletes a
  growing fraction of the outgoing scan's points while revealing a growing fraction
  of the incoming scan's, i.e. a per-point dissolve/density crossfade rather than a
  geometric blend. **Tradeoff:** the point count (and its attribute memory) roughly
  doubles for the whole span between any two scans' frames, not just briefly at the
  boundary - lower `Max Points/Scan` if that makes playback choppy.
- **Live color source switch, by field name.** The modifier's `Color Field` STRING
  input names which per-point attribute drives the displayed color: empty (`""`,
  the default absent an override) means the baseline - the real per-point RGB baked
  in at import (or its grayscale-from-intensity fallback, or nothing) - any other
  value names one of the FLOAT attributes import_point_clouds.py wrote for every
  extra PointCloud2 field the topic actually published (`intensity`, `ring`, `time`,
  or anything else - not a fixed list), remapped by `Field Min`/`Field Max` into 0-1
  and fed through the same blue-cyan-green-yellow-red rainbow Color Ramp grid maps
  use for elevation (see "Grid map heightfields" below) - one shared visual language
  for both features. All of Blend Factor/Color Field/Field Min/Field Max are
  per-object modifier inputs, editable any time from the modifier's panel in
  Properties (Color Field is a plain text box - type any attribute name into it) -
  no re-import needed. Each topic defaults `Color Field` to `"intensity"` only when
  it has no real color field AND actually published an `intensity` field (real RGB
  still wins by default whenever a topic has one, same priority as before this was
  generalized past a fixed RGB/intensity toggle), and seeds Min/Max from that
  field's topic-wide observed range. **Retyping `Color Field` to a different
  attribute does NOT auto-adjust Min/Max to match it** - they're left as whatever
  they were, so the ramp can look over/under-saturated until Min/Max are adjusted by
  hand to the new field's real range (a deliberate scope limit, not a bug - see
  `point_cloud_nodes.py`'s docstring for why a live-computed alternative wasn't used
  instead). The ramp's color stops are a node-internal datablock (Blender has no
  "ramp" socket type), so they're shared across every point-cloud topic and edited
  by opening a Geometry Nodes editor on any imported point cloud object rather than
  from the modifier panel.

## Grid map heightfields

Each grid map topic gets a single Mesh object with a shared "ROSBag GridMap
Coloring" Geometry Nodes modifier (`grid_map_nodes.py`). Unlike point clouds,
consecutive `GridMap` scans on one topic share the exact same cell grid (same
`resolution`/`length_x`/`length_y`, assumed constant per topic), so there's a
stable per-vertex correspondence from message to message - no crossfade is needed
or implemented; each scan just replaces the previous one outright (the timeline
still animates - the mesh is rewritten every frame the bracketing scan changes -
it just doesn't ease between two scans the way point clouds do). Since the vertex
grid is stable, a literal per-vertex Z lerp between scans *would* be meaningful
here if smoothing were ever wanted - noted as a possible future enhancement, not
implemented since it wasn't asked for.

- **Mesh construction.** `import_grid_map.py` builds the full `rows x cols` grid of
  vertices (one per cell, at its center) and `(rows-1) x (cols-1)` quads ONCE at
  import time; every frame, only per-vertex attributes (Z position, color,
  validity, and every other layer) are rewritten via `foreach_set`, mirroring the
  point-cloud path. Cell `(0, 0)` sits at the map's `+x, +y` corner; `info.pose` (the
  map's own local offset/rotation, which can vary per scan - e.g. a robot-centered
  local elevation map that recenters as it walks) is applied as the object's own
  `location`/`rotation_quaternion` every frame, on top of parenting to the TF empty
  matching `header.frame_id`.
- **Holes for invalid cells.** A cell invalid in the elevation layer, or in any of
  `basic_layers` (or just the elevation layer itself if `basic_layers` is empty), is
  tagged with a `_valid` attribute the GN modifier reads to `Delete Geometry` that
  vertex - which also removes every quad that touched it, leaving a real gap rather
  than a `z = 0` plateau or an arbitrary color.
- **Color.** Z comes from the `elevation` layer, or the first entry of
  `basic_layers` if there's no layer literally named `elevation`. If a packed
  "color" layer exists (grid_map's analogue of PointCloud2's packed RGB field -
  same bit-packing convention), it's preferred; otherwise (or via the modifier's
  live `Use Elevation Color` boolean, togglable any time after import - a plain
  on/off toggle rather than the point cloud's by-name `Color Field`, since a grid
  map only ever has one non-color layer worth switching to) the `elevation`
  attribute is remapped by `Elevation Min`/`Elevation Max` through a Color Ramp
  pre-populated with an
  explicit 5-stop blue -> cyan -> green -> yellow -> red rainbow, approximating
  `grid_map_rviz_plugin`'s default low-to-high height coloring (not claimed
  pixel-identical - see that module's docstring). Each topic defaults to "packed
  color" only if it ever published one, "Elevation ramp" otherwise, with Min/Max
  seeded from that topic's observed elevation range (over valid cells only).
- **The rolling/circular buffer - unverified.** grid_map internally stores each
  layer as a fixed-size buffer that's logically "rolled" in place rather than
  copied whenever the map shifts (e.g. a local elevation map recentering under a
  walking robot); `outer_start_index`/`inner_start_index` on the message say which
  buffer cell holds logical cell `(0, 0)`. **This add-on's unwrap logic
  (`grid_map_decode._unwrap_layer`) is based on recollection of `grid_map_ros`'s
  conversion code, not a live source check or a real sample message** - no network
  access was available to re-verify against the ROS package, and no bag containing
  `grid_map_msgs/GridMap` was found anywhere on this machine (searched `D:\`,
  `Desktop`, `Documents`, `Downloads`, including a promisingly-named
  `Downloads/elevation_map.bag`, which turned out to carry no GridMap topic at all).
  If this assumption is wrong, the symptom is a heightfield that looks torn/shifted
  along one or both axes wherever a real bag's roll offset isn't `(0, 0)` - which
  may not show up at all for a map that's never rolled far. See
  `grid_map_decode.py`'s module docstring for exactly what's assumed and where to
  fix it if a real grid_map bag ever contradicts it.

## Fixed: image pink flash / cross-modality desync

Two bugs were reported from playing back an imported bag: some frames flashed
Blender's classic pink/magenta "missing texture" color instead of holding the last
valid data, and images and point clouds didn't reflect the same real-world bag
timestamp at a given Blender frame (e.g. "at 1 second").

- **Root cause (both bugs): `import_images.py` only wrote correctly-named files at
  import time, then left Blender's own image-sequence player to display them - and
  that player advances exactly one file per Blender frame, which has nothing to do
  with a topic's actual (irregular) message timing.** The files were also named by
  their Blender frame number (e.g. `frame_000042.png`), but Blender's own sequence
  file discovery doesn't scan the directory for whatever exists - after loading the
  first file, it looks for the NEXT one by incrementing that file's numeric suffix
  by exactly 1 per Blender frame. Since messages don't land on every single Blender
  frame, almost every one of those consecutively-guessed filenames didn't exist on
  disk - a genuine missing-file load failure, i.e. the pink flash. And since the
  sequence was just playing "file 1, file 2, file 3, ..." one per Blender frame
  regardless of what frame number each file was actually meant for, images drifted
  out of sync with point clouds/grid maps (which DO correctly hold their data
  against the real, shared `TimeAlignment`-derived frame numbers) rather than
  showing the same instant.
- **Fix.** Frames are now named by their plain 1-based position in the sequence
  (always consecutive, so Blender's file discovery always finds a real file - no
  more pink), and a `frame_change_pre` handler drives `image_user.frame_offset`
  every frame from the exact same hold-last-value bisect logic
  (`frame_bracket.bracket()`) point clouds and grid maps already used, computed
  against each frame's real `align.frame_of(...)` Blender frame number. This makes
  images hold-and-switch exactly like point clouds/grid maps do, instead of
  independently free-running through their own file list.
- **`frame_bracket.py` (new, no bpy import).** The hold-the-first-scan-before-its-
  frame / hold-the-last-scan-forever-after-its-frame logic that used to be three
  separate `bisect_right(...) - 1` snippets (one per importer) is now one shared,
  shared function (`bracket()`), used identically by
  `import_point_clouds.py`, `import_grid_map.py`, and `import_images.py`. Code
  review of the point-cloud and grid-map handlers found their hold-first/hold-last
  behavior was already correct before this change (verified by tracing
  `bisect_right`'s return value for a `scene_frame` before the first entry and at/
  after the last one) - consolidating them here doesn't fix a bug in those two, it
  just removes the risk of the three copies silently drifting apart in the future.
- **`TimeAlignment` sharing was already correct.** `operators.py` builds exactly ONE
  `TimeAlignment`, anchored at `reader.start_time_ns(selected_names)` - the earliest
  timestamp across every SELECTED topic, not just one - and passes that SAME
  instance to every cache/importer call. Every timestamp-to-frame conversion in the
  codebase (`point_cloud_cache.py`, `grid_map_cache.py`, `import_images.py`,
  `import_point_clouds.py`'s TF empties) goes through that one instance's
  `frame_of()`, so this was not a source of the desync. `frame_of()`'s rounding
  (Python's built-in `round()`, round-half-to-even) is applied identically
  everywhere too, so while an exact-half-frame timestamp can round down or up
  depending on the parity of the integer part, every modality computes the exact
  same frame number for the exact same timestamp regardless - not a source of
  cross-modality drift either. The full explanation for the reported desync is the image bug above.

## Known limitations (v0.1)

- Raw (uncompressed) `sensor_msgs/Image` support covers `rgb8`/`bgr8`/`mono8`/`mono16`
  only; other encodings are skipped with a warning.
- No `PointCloud2` or `GridMap` topic existed in any bag found on this machine - that
  path has no end-to-end real-bag run, and neither Geometry Nodes modifier
  (`point_cloud_nodes.py`, `grid_map_nodes.py`) has been exercised inside Blender at
  all (this environment has no Blender install) - see "Verifying in Blender" below
  for what to check by hand. TF-empty creation and basic (non-animated) image
  extraction *were* both run end-to-end against a real bag in an earlier session,
  before the image pink-flash/desync fix above existed - that fix's actual
  `image_user.frame_offset`/`frame_change_pre` behavior has NOT been exercised
  inside Blender, same as everything else in this add-on that
  touches `bpy`.
- The GridMap circular-buffer unwrap convention is unverified - see "Grid map
  heightfields" above.

## Verifying in Blender

Neither Geometry Nodes node tree (`point_cloud_nodes.py`, `grid_map_nodes.py`) nor
the image-sequence `image_user.frame_offset` mechanism (`import_images.py`) has been
run inside Blender - all are built entirely from Python against the documented 4.2
API. To check them by hand after installing the add-on:

**Point cloud crossfade:**

1. Import a bag with a `PointCloud2` topic that has an `intensity` field (and
   ideally a second scalar field like `ring`, to exercise an arbitrary field name,
   not just the default), keeping it checked as "Point Cloud".
2. Select the imported point cloud object; in the Modifier Properties tab you
   should see a "ROSBag Crossfade" Geometry Nodes modifier with four inputs: Blend
   Factor, Color Field (a text box), Field Min, Field Max.
3. Scrub the timeline across a scan boundary - the cloud should visibly dissolve
   from one scan into the next instead of popping, and `Blend Factor` in the
   modifier panel should animate from 0 to 1 across that span (it's set from Python
   each frame, not a driver, so it won't show a driver's purple/keyframe coloring -
   that's expected).
4. For a topic with a real color field, confirm `Color Field` defaults to empty and
   the cloud shows its real RGB. For one with no real color field but an
   `intensity` field, confirm `Color Field` defaults to `"intensity"` and the cloud
   shows the blue-to-red rainbow ramp (not the old black-to-white grayscale -
   that's an intentional visual change, not a bug).
5. Type a different field name into `Color Field` (e.g. `ring`, if the topic has
   one) and confirm the color changes to reflect THAT field instead - since Field
   Min/Max won't have auto-updated (documented tradeoff above), it may look
   over/under-saturated until you also adjust Field Min/Max by hand to that field's
   real range; confirm doing so fixes it. Clear `Color Field` back to empty and
   confirm it returns to the real/baked "Color" attribute.
6. Type a nonexistent field name (e.g. `not_a_real_field`) into `Color Field` and
   confirm the cloud shows one flat color (not an error/crash) - this is the
   documented "missing Named Attribute reads as 0.0" behavior.
7. Open a Geometry Nodes editor on the object and select the "ROSBag PointCloud
   Crossfade" group to see/edit the Color Ramp node's 5 stops directly (shared
   across every point-cloud topic).
8. If step 3's dissolve or step 5's field switch looks wrong, the most likely
   culprits are documented inline in `point_cloud_nodes.py`'s `_build_node_group()`:
   the `GeometryNodeSwitch`/`Delete Geometry` selection wiring for the dissolve, and
   the `FunctionNodeStringLength` -> `GeometryNodeSwitch` wiring (its "String" input
   socket name is the part least certain without a live Blender session) for the
   field switch.

**Grid map heightfield:**

1. Import a bag with a `GridMap` topic, keeping it checked as "Grid Map" (if the
   topic isn't auto-detected - e.g. the bag reports a slightly different type
   string - use the "Import As" dropdown to force it).
2. Select the imported object; it should be a Mesh (not a Point Cloud) laid out as
   a heightfield grid, parented to the TF empty for the map's `header.frame_id`, and
   offset within that frame by `info.pose` rather than sitting at the frame's origin.
3. In the Modifier Properties tab you should see a "ROSBag GridMap Coloring"
   modifier with three inputs: Use Elevation Color, Elevation Min, Elevation Max.
4. Scrub the timeline - the heightfield should update to each new scan's shape.
   Confirm any NaN/unmapped region in the source data appears as an actual gap in
   the mesh, not a flat `z = 0` patch.
5. **Most important check given the unverified assumption above:** compare the
   heightfield's overall shape against the same bag played in RViz (or against
   known terrain features in the recording) - if it looks shifted, torn, or mirrored
   along one axis compared to RViz, the circular-buffer unwrap direction or
   axis assignment in `grid_map_decode._unwrap_layer` is the first thing to check
   (its docstring says exactly what to try flipping).
6. Toggle "Use Elevation Color" and confirm the color switches between a packed
   color layer (if the topic has one) and the blue-to-red rainbow Color Ramp over
   height (open a Geometry Nodes editor on the object, select the "ROSBag GridMap
   Coloring" group, and inspect the Color Ramp node's 5 stops if you want to
   restyle it - shared across every grid map topic, like the point cloud's ramp).
7. For a topic with no packed color layer, confirm it defaults to "Use Elevation
   Color" checked; for one with real packed color, confirm it defaults unchecked.

**Image sequence hold/sync (the pink-flash/desync fix):**

1. Import a bag with an `Image` or `CompressedImage` topic, keeping it checked as
   "Image Sequence". Check the `<topic>_frames` output folder next to the bag: the
   extracted files should be named `frame_000001.ext`, `frame_000002.ext`, ...
   consecutively (NOT the sparse/skippy numbers you'd get from each message's own
   Blender frame - that naming was the bug).
2. Select the imported Image Empty; its `image_user.frame_start` should be 1 and
   `frame_duration` should equal the number of extracted files (reported in the
   "N image frames (animated)" info message after import).
3. Scrub the timeline slowly across where a message's frame falls. The image should
   change to the new frame's picture with NO pink/magenta flash at any point,
   including frames between two messages (where it should just keep showing the
   previous message's picture, not go blank/pink) and frames before the very first
   message or after the very last one (should hold the first/last picture,
   respectively - never show nothing).
4. `image_user.frame_offset` (visible in the Empty's Object Data / Image Properties
   panel) should visibly change value as you scrub - it's being set every frame from
   Python (see `import_images.py`), not a driver, so (like Blend Factor above) it
   won't show driver purple/keyframe coloring - that's expected, not a sign it isn't
   working.
5. **Cross-modality sync check** (the second reported bug): import a bag with BOTH
   an image topic and a point cloud (or grid map) topic that publish at around the
   same real-world rate. Pick a specific real time in the recording - e.g. "1 second
   in" - compute the corresponding Blender frame (`fps` seconds in), scrub to it,
   and confirm the image shown and the point cloud/grid map scan shown both
   correspond to messages recorded at roughly that same bag timestamp (check
   `rqt_bag`/`ros2 bag info` or similar against the original bag if precise
   confirmation is needed) - not two unrelated moments.
6. If step 1's files aren't consecutively numbered, or step 3 still shows pink, the
   most likely culprit is `import_images.py`'s `extract_image_sequence` (check
   `seq_index` really is 1-based and gapless) or `create_image_empty` (check
   `image_user.frame_start`/`frame_duration` match). If sync still looks off despite
   steps 1-4 passing, check `frame_bracket.image_user_frame_offset`'s formula against
   Blender's actual `ImageUser` behavior for your Blender version (this add-on
   targets 4.2 - the formula is documented inline, but not verified against a live
   `ImageUser`).
