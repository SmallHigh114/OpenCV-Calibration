# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

C++17, CMake 3.10+, depends on `libopencv-dev libyaml-cpp-dev libfmt-dev libeigen3-dev` and the bundled HIK SDK (`hikSDK/lib/{amd64,arm64}` selected by `CMAKE_SYSTEM_PROCESSOR`).

```bash
cmake -S . -B build
cmake --build build -j
```

There is no test suite and no lint target wired into CMake. `.clang-format` and `.clang-tidy` exist at the repo root — invoke them directly (`clang-format -i <file>`, `clang-tidy -p build <file>`). `compile_commands.json` is exported for clangd.

The four executables all live in `build/` and accept `--config-path=<yaml>` (default `config/calibration.yaml`):

- `calibrateCamera` — produces `camera_calibration.yaml` at repo root.
- `calibrateHandEye` — produces `handeye_calibration.yaml`. Use `-l -d <dir>` to skip live capture and recompute from a previously saved `image_*.jpg` + `pose_*.yaml` directory.
- `calculateError` — live RMSE display, no output files.
- `validateHandEye` — `--handeye-path=<yaml>` to override the result file under test.

## Architecture

Each executable is a thin `main` (`calibrateCamera.cpp`, `calibrateHandEye.cpp`, `calculateError.cpp`, `validateHandEye.cpp`) at the repo root. All shared logic lives under `src/` + `include/` and is built once into the `calibration_core` static library, which every executable links. When adding shared functionality, put it in `src/` so all four binaries get it for free; only put loop/CLI glue in the top-level `*.cpp` files.

**Device abstraction.** `qd::Device::Device` (`include/device.hpp`) is a pure virtual `read(img, timestamp)` interface. Three implementations: `Hik_Camera` (HIK SDK + background thread + `ThreadSafeQueue`), `UVC_Camera` (`cv::VideoCapture`), `Image_Reader` (offline directory; the only one that ever returns `is_exhausted() == true`). `qd::app::create_device(config_path)` (`src/device_factory.cpp`) is the single switch on the YAML `device:` field — add new sources here. The factory also returns a `wait_time` hint (0 for `IMG`, 1 for live cameras) used as the `WebViewer::waitKey` delay.

**UI is a Web viewer, not OpenCV HighGUI.** `qd::WebViewer` (`include/web_viewer.hpp`) runs an embedded HTTP/MJPEG server on port 8080 and exposes a `cv::imshow`/`cv::waitKey`/`cv::namedWindow`-shaped API. The user opens `http://localhost:8080` and must click the page so it has focus before keys (`s`/`a`/`c`/`r`/`ESC`) are captured. Do **not** add `cv::imshow` or `cv::waitKey` calls — keep using the viewer instance the executables already construct.

**Calibration core** is `qd::calibrate::Calibrate` in `include/calibrate.hpp` / `src/calibrate.cpp` — a single class that owns: chessboard/circle-grid corner finding, intrinsics solve (`calibrate_camera`), hand-eye solve (`calibrate_handeye`), reprojection-error overlay (`display_error`), and the hand-eye validation flow (`load_handeye_calibration`, `validate_handeye`, `reset_validation_stats`). It also persists results to YAML and statistics RPY ranges over collected samples. `Paramer` (same header) parses board geometry from YAML.

**Auto-collection (`include/auto_collector.hpp` / `src/auto_collector.cpp`)** is a direct port of ROS `image_pipeline/camera_calibration`'s `calibrator.py` (BSD-3, attribution preserved in headers). It deduplicates samples in a 4-D normalized parameter space (X/Y/Size/Skew), tracks per-axis coverage progress, and draws the ROS-style progress bars on the live image. The thresholds (`auto_collect_param_distance`, `auto_collect_param_ranges`, `auto_collect_goodenough_samples`, `auto_collect_max_chessboard_speed`) match ROS defaults; `auto_collect_sharpness_threshold` is a project-specific Laplacian-variance gate that is **not** in the ROS original. Toggle at runtime with `a`.

**Serial / IMU.** `Serial_driver` (`include/serial_driver.hpp`) reads quaternion data from a UART (`UartTransporter`) on a daemon thread and serves time-aligned poses via linear interpolation against the camera frame timestamp. Used by `calibrateHandEye` and `validateHandEye`; not used by camera-only flows. Default port `/dev/rm_usb0` (configurable under `Serial:` in YAML).

## Configuration

`config/calibration.yaml` is the single source of runtime configuration for all executables. Important coupling to be aware of:

- The `camera_matrix` / `distort_coeffs` arrays in this file are inputs to hand-eye calibration and reprojection error — after running `calibrateCamera`, the workflow expects you to copy values from the generated `camera_calibration.yaml` back into `config/calibration.yaml` before running hand-eye flows.
- `device:` selects which `*_Camera` implementation gets constructed; the corresponding sub-section (`HIK:` / `UVC:` / `IMG:`) is read by that implementation's constructor.

## Platform setup (HIK)

HIK USB cameras need a udev rule for non-root access — see README §1. Without it, `Hik_Camera` will fail to enumerate the device.
