# ASTRA Photo Booth: current status and Jetson test plan

## What is in the pushed GUI

The pushed baseline (`e016d9c`, branch `shrihan/photo-booth-feedback`) has Live,
Photos, and Split views, annotated detections, and one-second photo capture. That
baseline stores photos on the Jetson.

## What is in the local follow-up draft

The current local branch is `shrihan/photo-booth-feedback`. Its changes are not
committed or pushed. They add:

- **Controller** view with a hold joystick, keyboard movement, hold zoom and focus,
  absolute zoom, autofocus, point autofocus, photo, center, and recording toggle.
- Recent annotated photos retained in each browser tab's memory, capped at 60 by
  default. No photo archive is written by the viewer to the Jetson.
- A Camera SD gallery that requests photo listings and images from the ZR10 only
  when needed, plus multi-select ZIP downloads (up to five photos per batch).
- A capped 5 FPS, 960-by-540 maximum raw preview with a recent detection overlay.
  The overlay comes from the latest detector result and can lag the moving image;
  saved browser captures use the matching image and detection result.
- A ROS-side movement watchdog and browser stop-on-release, disconnect, hidden tab,
  and leaving Controller safeguards.

Recent browser photos are local to that browser tab and disappear when it closes.
Camera SD photos remain on the camera. This draft has not yet been tested on the
Jetson or with a real ZR10. The controls cover only functions currently represented
in our ROS code; they do not expose every camera setting. The camera does not report
command acknowledgements or current recording state through this UI.

## Jetson test plan

1. Check out or copy the local follow-up branch to the Jetson. Keep the camera,
   detector, and viewer from running more than once.
2. Build the updated `zr10_ros2` package in the ASTRA `ros2_ws` and source its
   `install/setup.bash`. Keep the existing model and detector environment.
3. First start the camera/detection launch with its default `dry_run:=true`. Confirm
   frames, detections, and browser-only photo capture work. Open the viewer from a
   second device using the Jetson's reachable network address.
4. Browse Camera SD, select a few photos, and download them. Check that Jetson free
   space does not drop from viewer photo storage and that recent previews disappear
   after closing that browser tab.
5. Test controls in dry-run mode first to confirm the UI locks/arms and the ROS
   bridge accepts commands without sending camera movement.
6. Only test real camera motion after the camera is secured and the operator
   explicitly chooses to enable live control (`dry_run:=false` plus viewer
   `--enable-controller`). Begin with a brief, low-speed gimbal movement and release
   it; verify the camera stops before trying zoom/focus or other actions. UDP sends
   are not camera acknowledgements, so confirm each action on the device.

## Known limits for the first test

- Camera SD browsing depends on the ZR10 media API being reachable at the configured
  camera address and port 82. We have not verified the camera's returned media URLs
  against this firmware.
- The new manual focus, absolute zoom, point autofocus, and media browsing code
  have not been exercised on hardware.
- Preview rate and resolution are conservative defaults, not measured power or
  smoothness guarantees. Detector load is unchanged.
- Browser-local recent photos are separate on each device; they are not shared
  between connected browsers.
- Advanced camera settings and any ZR10 controls absent from our ROS driver remain
  out of scope until their protocol and firmware behavior are verified.

No tests were run during this implementation pass. No changes have been committed
or pushed.
