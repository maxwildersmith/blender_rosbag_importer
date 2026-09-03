# blender_rosbag_importer

Blender add-on that imports ROS bags (ROS1 `.bag`, ROS2 `.db3`/`.mcap`, via
[`rosbags`](https://pypi.org/project/rosbags/)) as animated scenes:

- `/tf` and `/tf_static` become keyframed Empties, one per child frame, parented to
  each other to reproduce the TF tree.
- Point cloud topics (`sensor_msgs/PointCloud2`) become one native Blender Point
  Cloud object per message, parented to the Empty matching the message's
  `header.frame_id`, visible only at its own frame (so scrubbing the timeline shows
  the scan that was actually captured at that time while the parent Empty's motion
  moves it correctly).
- Image topics (`sensor_msgs/Image` or `CompressedImage`) are extracted to a numbered
  frame sequence on disk and loaded as a Blender image sequence on an Image Empty,
  best-effort parented to the TF frame matching the topic's name prefix.
- Playback is time-aligned to start at frame 0, anchored to the earliest message
  among the selected topics.

## Layout

The installable add-on is the **inner** `blender_rosbag_importer/` package - point
Blender at that folder (or zip it) when installing, not the repo root.

```
blender_rosbag_importer/          # repo root
├── blender_rosbag_importer/      # <- the actual Blender add-on package
│   ├── __init__.py                bl_info + register/unregister
│   ├── preferences.py             "Install Dependencies" button (see below)
│   ├── bag_reader.py              rosbags wrapper - no bpy import, unit-testable
│   ├── pointcloud2_decode.py      PointCloud2 -> numpy (no bpy import)
│   ├── image_decode.py            Image/CompressedImage -> numpy/bytes (no bpy import)
│   ├── tf_tree.py                 /tf + /tf_static -> per-frame transform time series
│   ├── time_alignment.py          bag timestamp -> Blender frame number
│   ├── import_point_clouds.py     bpy: TF empties + PointCloud objects
│   ├── import_images.py           bpy: frame extraction + Image Empty
│   ├── properties.py              RosbagTopicEntry PropertyGroup (per-topic checkbox row)
│   ├── ui_list.py                 UIList rendering the topic checklist
│   └── operators.py               IMPORT_OT_rosbag - the actual "File > Import > ROS Bag"
└── tests/
    └── test_pointcloud2_decode.py  plain pytest/python, no Blender required
```

The bag-parsing/TF/time-alignment/decode modules never import `bpy`, so they can be
run and tested with plain `python3` outside Blender. Only the `import_*`/`operators`/
`properties`/`ui_list`/`preferences` modules touch `bpy`.

## Installing

1. In Blender: Edit > Preferences > Add-ons > Install..., point it at this repo's
   inner `blender_rosbag_importer/` folder (or a zip of it), and enable it.
2. `rosbags` isn't bundled with Blender. Open the add-on's preferences panel and
   click **Install Dependencies** - this pip-installs `rosbags` into a `vendor/`
   folder *inside the add-on*, not into Blender's own (often read-only, e.g. under
   snap/flatpak) site-packages.
3. File > Import > ROS Bag (.bag/.db3/.mcap).

## Usage notes

- The topic checklist appears in the file browser's side panel once you've picked a
  bag file. Point cloud and image topics default to selected; everything else
  defaults to skipped. Each row's "Import As" dropdown can override the guessed kind.
- `Point Cloud Stride` / `Image Stride` import every Nth message - use this to keep
  large bags tractable. `Max Points/Scan` decimates each individual point cloud
  message (0 = no limit).
- A point cloud topic publishing under more than one `frame_id`, or a TF frame whose
  parent changes over time, logs a warning to Blender's console rather than failing
  the import.

## Known limitations (v0.1)

- Raw (uncompressed) `sensor_msgs/Image` support covers `rgb8`/`bgr8`/`mono8`/`mono16`
  only; other encodings are skipped with a warning.
- The point-cloud "one object per scan + visibility keyframes" approach can produce
  overlapping visibility keyframes if a topic publishes faster than the render FPS
  (two scans landing on the same or adjacent Blender frame) - raise the FPS or the
  stride if that happens.
- No `PointCloud2` topic existed in the bags used to test this (`~/misc/allegro/*.bag`)
  - that path is covered by `tests/test_pointcloud2_decode.py` plus a synthetic-message
  smoke test in Blender's own Python, not by an end-to-end real-bag run. TF-empty
  creation and image-sequence import *were* both run end-to-end against a real bag.
