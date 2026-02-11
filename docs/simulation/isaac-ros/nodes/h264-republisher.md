# image_transport republisher

Encodes raw camera images to H.264 using the GPU's hardware encoder (NVENC).

| | |
|---|---|
| **Entrypoint** | `60-h264-republisher.sh` |
| **Subscribes to** | `/front_stereo_camera/left/image_rect_color` (raw, ~2.7 MB/frame) |
| **Publishes** | `/front_stereo_camera/left/compressed_video` (H.264, ~50 KB/frame) |

Without this, streaming raw camera images over the network to Foxglove is too slow. The NVENC encoder runs on dedicated GPU silicon and doesn't affect simulation performance.

!!! note
    H.265 does NOT work with Foxglove (browser can't decode keyframes). Use H.264 only.
