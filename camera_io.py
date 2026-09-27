"""Apply the same requested camera properties in calibration and operation."""
import cv2


def apply_camera_properties(cap, cfg):
    readings = {}
    for name, value in cfg.get('camera_properties', {}).items():
        prop = getattr(cv2, 'CAP_PROP_' + name, None)
        accepted = prop is not None and bool(cap.set(prop, value))
        actual = float(cap.get(prop)) if prop is not None else None
        readings[name] = {'requested': value, 'accepted': accepted, 'readback': actual}
        if not accepted:
            print(f'Warning: camera rejected {name}={value}')
    return readings  # Successful readback does not prove an optical setting.
