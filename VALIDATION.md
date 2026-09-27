# Color-first validation

Checked on 28 September 2026 (Asia/Bangkok). Source:
`CPE-Exploratory3-main(1).zip`, with the user's `image.png` and `image1.png`.

## Automated checks

| Check | Result |
|---|---|
| Original root-project suite, before changes | 58 tests; `test_long_thin_cluster_rejected` failed |
| Updated root-project suite | 73 tests passed |
| Python compilation of changed modules and tests | Passed |
| CLI help for color preview, sampler, zone finder, arena calibration, autonomy and grip calibration | Passed |
| Audit of fresh `field/calib.json` | Correct 2100 x 1200 mm dimensions; six missing colors and no corners reported |
| Real-photo headless preview, evidence serialization and saved images | Passed |

The 15 new tests cover color separation in saturation, circular red hue, rejecting
conflicting or insufficient samples, keeping evaluation samples out of fitting,
sample compatibility, glare rejection, non-guessed zone labels, marker motion,
missing markers, revocation of stale targets, and the two supplied photos.

Test environment: Windows, Python 3.12.4, NumPy 2.5.3 and OpenCV 5.0.0
(headless test runtime). Field windows require the GUI OpenCV package from
`requirements.txt`; the temporary test runtime is not included in the ZIP.

Commands, from the extracted project directory:

```bash
python -m unittest discover -s tests -q
python color_preview.py --audit --config field/calib.json
python color_preview.py --image examples/image.png --background examples/image1.png --config calib.json --check-reference --headless --output evidence/provided-photos
```

The gesture subproject is copied from the archive and is not part of this 73-test
count. Its models and training data were not changed or retrained.

## What the supplied photos establish

With the historical corner mapping, six fixed scoring-circle centers differ by
25.1, 28.3, 28.5, 30.5, 48.2 and 48.9 rectified pixels. The updated detector reports
`reference_moved` and zero eligible targets. The original detector reported `ok`.
The images alone cannot identify whether the camera, field, individual circles,
or the supplied calibration changed.

The historical cyan/skyblue boxes overlap at H 78..101, S 71..255, V 98..255.
Crimson/orange overlap at H 2..15, S 105..255, V 122..255. These intersections
explain ambiguous pixels; they do not supply new stone-color ground truth.

Saved visual evidence:
[preview](evidence/provided-photos/20260927T184951Z-b44125/first-frame/preview.png),
[diagnostics](evidence/provided-photos/20260927T184951Z-b44125/last-frame-report.json).
Red arrows show marker displacement. White pixels in the right panel match more
than one color range. Blob counts include rejected foreground components, not
the number of stones. Millimeter distances in this historical-config report use
the old scale and must not be treated as measured physical displacements.

## Limits and next field checks

No live camera GUI, exposure controls, motor, servo or real robot was exercised.
There is no labeled held-out set of whole stones, so no stone classification
accuracy, detection recall, localization accuracy or field FPS is claimed.
No neural color detector has been trained.

Recalibrate the current empty field, collect labeled patches in multiple sessions,
then reserve a separate evaluation session. Test individual stones at measured
locations, shadows, glare, an empty field and robot occlusion. Verify marker-guard
thresholds and camera settings on the real setup. A passing marker check is not
proof that every aspect of calibration is correct.

Autonomous pickup remains a later validation stage. The separately reviewed
firmware/teleop protocol mismatch is not fixed here. Firmware and historical
`calib.json` are byte-for-byte unchanged from the source archive; the new measured
profile is separate. No firmware was flashed.
