"""V2 of the autonomous run: V1 (autonomy.py) + the pile fix + outermost pile stone + target
commitment. Same calib.json, same flags; the switches it sets are listed in profiles.py.

    python autonomy2.py 10.178.188.50 --camera 1 --record          # real robot
    python autonomy2.py --dry-run --camera 1                       # camera/planner only
    python autonomy2.py --sim --show                               # simulator (planner part only)
    python autonomy2.py ... --set vision.pile_outermost=false      # turn one V2 switch back
"""
import autonomy

PROFILE = 'v2'

if __name__ == '__main__':
    autonomy.main(profile=PROFILE)
