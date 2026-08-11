---
description: Lite3 camera — RGB-D topic bridge for the external Orbbec Gemini 335 (snapshot via ROS image topics).
---
# Lite3 camera (`robonix/primitive/camera`)
Bridges the Orbbec Gemini 335's color + depth streams (published by the
`orbbec_camera` driver on `/camera/color/image_raw`, `/camera/depth/image_raw`,
`/camera/color/camera_info`) into robonix contracts and provides JPEG snapshots.

The onboard monocular RTSP/ffmpeg snapshot path from the original package is
removed (ADR-0002): the Orbbec is the single perception head.
