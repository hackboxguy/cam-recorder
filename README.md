# cam-recorder

Camera live view and on-demand recording for embedded Linux boards, built on
GStreamer and V4L2. Shared source for two very different images:

| board | OS | build |
|---|---|---|
| NVIDIA Jetson Orin Nano | Yocto (OE4T), `jetson-edge-platform` | bitbake recipe, local git mirror + pinned `SRCREV` |
| Raspberry Pi 5 | Raspberry Pi OS Lite | `misc-tools` image hook, `board=pi5-gmsl-vd56g4` |

Both were brought up with a VD56G4 GMSL2 camera behind a MAX9296A / MAX96716A
deserializer, but nothing here is specific to that sensor — the apps consume
whatever `/dev/video0` offers.

## What's here

```
cam-recorder   live view on the local display + on-demand recording to USB,
               with a red REC dot shown on screen but never in the file
cam-keyd       reads the keyboard, drives the apps over a unix socket
cam-recctl     command-line client for that socket (scriptable over SSH)
camview        minimal live-view-only script, for quick camera checks
```

`cam-recorder` and `camview` both want exclusive `/dev/video0`, so their units
declare `Conflicts=` on each other and exactly one is enabled at boot
(`/etc/cam-display-app` names which).

## Keys

| key | action |
|---|---|
| `Enter` / `KP Enter` | start/stop recording |
| `-` | switch to `cam-recorder` |
| `+` | switch to `camview` |
| `/` and `*` | per-channel record, reserved for dual camera |

Both keypad and main-keyboard variants are handled — they are distinct keycodes,
and handling only one is indistinguishable from "the key does nothing".

## Control socket

Newline-delimited text on `/run/cam-recorder.sock`, deliberately shaped so a
richer input daemon can replace `cam-keyd` without this program changing.

```
REC START|STOP|TOGGLE [main|left|right]   -> OK RECORDING <path> | OK IDLE | ERR <why>
STATUS                                    -> STATE <ch> IDLE|RECORDING|NO_MEDIA [path] [secs]
```

`STATE` lines are also pushed asynchronously to every connected client. Note
`STATUS` is answered with a single `STATE` line and **no** `OK`/`ERR`
terminator — a client that waits for one will hang.

## Build

```sh
make
sudo make install                # or: make install DESTDIR=/staging
```

Cross-compiles by passing `CC`/`CFLAGS`/`LDFLAGS` in, which is what
OpenEmbedded already exports. `cam-keyd` and `cam-recctl` are plain POSIX and
build without GStreamer development packages.

Runtime needs GStreamer core plus `plugins-base`, `plugins-good` (for
`gdkpixbufoverlay`), `plugins-bad` (`kmssink`) and **`plugins-ugly` for
`x264enc`** — the last is easy to miss and its absence is silent, which costs an
afternoon on both platforms. Neither the Orin Nano nor the Pi 5 has a hardware
H.264 encoder, so the CPU encoder is not a fallback, it is the only option.

## Platform notes

The pipeline differs at the ends, not in the middle:

| | Jetson (tegra) | Pi 5 (generic) |
|---|---|---|
| flip / scale | `nvvidconv` (hardware) | `videoconvert ! videoscale` |
| display sink | `nvdrmvideosink`, `offset-x` | `kmssink`, `render-rectangle` |
| tee buffers | system memory (NVMM will not negotiate) | system memory |

Measured, and worth knowing before changing either pipeline:

- **`kmssink` on Pi 5 only negotiates at the display mode size.** 888x1080 fails
  `not-negotiated (-4)` with or without `render-rectangle`; scale to the full
  mode and position with `render-rectangle`.
- **Do not benchmark a `kmssink` pipeline by wall clock.** `force-modesetting`
  restores the console mode on shutdown, adding ~3.4 s — enough to make a true
  30 fps look like 22 fps. Use `fpsdisplaysink`'s rendered-frame accounting.
- Pi 5 sustains **30.03 fps display with 0 dropped frames while recording
  300 frames in 10.03 s**, including a CPU 180° flip and the dot overlay.
- The display queue must be **small and leaky**. Left at GStreamer defaults the
  live view ran 1–1.5 s behind on the Jetson — and that delay is invisible to a
  PTS-based probe, because `v4l2src` timestamps at dequeue.

## License

GPL-2.0-only. See `LICENSE`.
