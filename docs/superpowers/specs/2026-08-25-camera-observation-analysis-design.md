# Camera Observation Analysis Design

## Goal

Reduce the remaining responsibilities of `youbot_web.c` by extracting the pure
RGB-frame analysis used for real-camera obstacle detection. Preserve current
runtime behavior exactly and leave Docker work out of scope.

## Scope

Create `controller_camera.c/.h` and a standalone
`controller_camera_test.c`. The module analyzes an RGB image through a small
pixel-reader callback and returns measurements derived from the image. It must
not call Webots, read lidar data, update maps, write files, or access controller
globals.

The following remain in `youbot_web.c`:

- camera initialization and `wb_camera_get_image`;
- the adapter that reads RGB channels from the Webots image;
- visible-frustum map updates;
- lidar-based range refinement;
- angle calculation from the returned normalized offset and camera FOV;
- obstacle/free-space map updates;
- virtual camera rendering, image export, scheduling, and telemetry.

## Public interface

The header defines:

- `ControllerCameraPixel`, containing integer red, green, and blue channels;
- `ControllerCameraPixelReader`, a callback receiving context, x, and y;
- `ControllerCameraAnalysisConfig`, containing width, height, sampling step,
  crop fractions, and minimum obstacle score;
- `ControllerCameraObservation`, containing visibility, sample and hit counts,
  score, normalized center offset, image fallback range, and hit bounds;
- a default-config initializer matching the constants currently embedded in
  `update_camera_obstacle_hint`;
- one analysis function that initializes the result and returns a neutral
  observation for invalid input.

The callback avoids any dependency on Webots' BGRA memory layout. The Webots
adapter remains responsible for using `wb_camera_image_get_red/green/blue`, so
the extraction does not reinterpret or copy the frame.

## Preserved algorithm

The module preserves these values and formulas without tuning:

- crop: x `[14%, 86%)`, y `[18%, 82%)`;
- sampling step: 4 pixels;
- warm-pixel thresholds: red above 105, red dominance over green and blue,
  saturation above 44, and the existing green/red and blue/red ratios;
- vertical weight from 0.65 with the existing 0.70 gradient;
- score as hits divided by samples;
- visibility threshold `0.025`;
- normalized horizontal centroid clamped to `[-1, 1]`;
- fallback range formula and clamp `[0.42, 2.25]`;
- hit bounds used by the caller for map confidence.

No persistence, filtering, smoothing, or new recognition behavior is added.

## Data flow

`update_camera_obstacle_hint` resets the existing globals, obtains the Webots
image, and updates the visible frustum as before. It then passes image dimensions
and a Webots pixel-reader adapter to `controller_camera_analyze`. If the returned
observation is visible, the controller copies score, offset, and hit count,
computes the obstacle angle, refines the range with lidar, and performs the same
map updates with the same confidence formula.

## Invalid input and errors

Null result pointers are rejected without writing. Missing readers, non-positive
dimensions, invalid crop ranges, or non-positive sample steps produce a fully
initialized neutral observation. There is no allocation and no I/O, so analysis
has no partial external side effects.

## Testing

Development follows red-green-refactor. Standalone C tests cover:

- default configuration values;
- invalid input producing a neutral observation;
- an image without matching pixels remaining invisible;
- a sufficiently large warm obstacle becoming visible;
- left/right normalized offsets and hit bounds;
- the current score and fallback-range formulas at representative coordinates;
- a warm region below the score threshold remaining invisible while retaining
  its measured score and hit count.

After integration, run all standalone C tests with
`-Wall -Wextra -Werror`, the full Webots controller build and link with the same
flags, all JavaScript tests, ESLint, the Vite production build, and the bridge
smoke test.

## Success criteria

- Camera detection calculations and thresholds remain behaviorally identical.
- `controller_camera` compiles and tests without the Webots SDK.
- `youbot_web.c` contains orchestration and Webots adaptation but not the RGB
  scan/classification loop.
- Every existing verification command remains green.
- Docker files and behavior are unchanged.
