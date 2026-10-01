# Project context

The current package contains overhead color perception, localization, robot control,
and the CHROMA gesture subproject. The current priority is color recognition.
Read [COLOR_FIRST.md](COLOR_FIRST.md) for the updated design and field workflow,
and [VALIDATION.md](VALIDATION.md) for test evidence. Use `field/calib.json` with
the measured 2100 x 1200 mm field. The older root `calib.json` is historical.

Gesture control lives in `CHROMA-Gesture-Control/` (two hands, one trained command set, since
2026-10-01): see [CHROMA-Gesture-Control/README.md](CHROMA-Gesture-Control/README.md).
