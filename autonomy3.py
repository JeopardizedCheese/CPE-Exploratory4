"""V3 of the autonomous run: V2 (autonomy2.py) + the grip check by the gripper camera
(HuskyLens on the ESP32). Same calib.json, same flags; the switches it sets are listed in
profiles.py. Refuses to start unless the firmware reports "gripcam":"ok".

    python autonomy3.py 172.20.10.2 --camera 1 --record            # real robot
    python autonomy3.py --dry-run --camera 1                       # camera/planner only
    python autonomy3.py --sim --show                               # simulator (planner part only)
    python autonomy3.py ... --set grip_check=false                 # = V2
"""
import autonomy

PROFILE = 'v3'

if __name__ == '__main__':
    autonomy.main(profile=PROFILE)
