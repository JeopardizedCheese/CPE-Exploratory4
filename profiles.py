"""Three versions of the robot's behaviour that share one calib.json.

    python autonomy.py  ...     V1: the behaviour we field-tested (pile = one "?" blob)
    python autonomy2.py ...     V2: V1 + the pile fix + outermost pile stone + target commitment
    python autonomy3.py ...     V3: V2 + the grip check by the gripper camera (HuskyLens)

All read the same calib.json and accept the same flags. A profile only sets the switches
below, in memory, before --set is applied (so --set still wins); calib.json is never written.
The run folder's config.json records the profile and every switch.

V1 sets every switch explicitly to its legacy value (= the code defaults), so V1 stays V1
even if one of these keys is ever added to calib.json.

V2 switches (each can be turned back individually with --set, e.g. --set vision.pile_outermost=false):
  vision.pile_edge_pixels "nearest"  a stone's pale highlight / blurred rim (pixels that vote for no
                                     colour) belongs to the nearest stone in the pile, so edge stones
                                     of a pile get a free corridor (PILE_FIX_PLAN.md). Aim point =
                                     core + rim centroid.
  vision.pile_outermost   true       a pile with no free edge stone still gives one target: the stone
                                     farthest from the pile's centre, approached toward the centre.
                                     Only the pile itself may lie in its corridor (walls, the robot
                                     and other objects still block it).
  vision.pile_regions     true       every stone-like single-colour region of a pile is also reported
                                     as a coloured (not pickable) observation, beside the pile's "?"
                                     blob, so the planner can follow a locked stone inside a pile.
  autonomy.commit_target  true       a locked stone stays locked while it is still seen at its spot,
                                     even when vision stops offering it as a target; a neighbour of
                                     another colour no longer breaks the lock.
  autonomy.skip_alone_s   6          after a failed attempt a stone is skipped for skip_s (25 s); when
                                     it is the only stone on offer, retry it after this many seconds
                                     instead of parking.
  autonomy.carry_wall_clamp true     while carrying, a detour around another zone that would run along
                                     a wall passes on the zone's other side (else is pulled in to the
                                     wall margin). The zone centre itself is never moved.
vision.pile_outermost and vision.pile_regions need pile_edge_pixels "nearest" (ignored with "legacy").

V3 = V2 + one switch (turn it off with --set grip_check=false; V1 + grip check = autonomy.py
--set grip_check=true):
  autonomy.grip_check     true       once the jaws have closed, the gripper camera classifies them:
                                     Empty -> open, back off, skip the spot; Single/Multiple/Unsure
                                     -> carry as V2 (pick check included). V3 refuses to start
                                     unless the firmware reports "gripcam":"ok". The trained IDs per
                                     verdict are autonomy.grip_check_ids (calib.json or --set).
"""

PROFILES = {
    'v1': {
        'vision': {'pile_edge_pixels': 'legacy', 'pile_outermost': False, 'pile_regions': False},
        'autonomy': {'commit_target': False, 'skip_alone_s': None, 'carry_wall_clamp': False, 'grip_check': False},
    },
    'v2': {
        'vision': {'pile_edge_pixels': 'nearest', 'pile_outermost': True, 'pile_regions': True},
        'autonomy': {'commit_target': True, 'skip_alone_s': 6, 'carry_wall_clamp': True, 'grip_check': False},
    },
    'v3': {
        'vision': {'pile_edge_pixels': 'nearest', 'pile_outermost': True, 'pile_regions': True},
        'autonomy': {'commit_target': True, 'skip_alone_s': 6, 'carry_wall_clamp': True, 'grip_check': True},
    },
}

TITLES = {
    'v1': 'V1 (legacy: field-tested behaviour)',
    'v2': 'V2 (pile fix + outermost pile stone + target commitment)',
    'v3': 'V3 (V2 + grip check by the gripper camera)',
}


def _is_bool(v):
    return isinstance(v, bool)


def _is_positive(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and v > 0


# vision keys that --set accepts by name (vision.<key>=VALUE), with their check
VISION_SWITCHES = {
    'pile_edge_pixels': (lambda v: v in ('legacy', 'nearest'), '"legacy" or "nearest"'),
    'own_reach_mm': (_is_positive, 'a positive number of mm'),
    'pile_outermost': (_is_bool, 'true or false'),
    'pile_regions': (_is_bool, 'true or false'),
}


def apply_profile(cfg, name):
    """Set the profile's switches in cfg (in memory). Returns cfg."""
    if name not in PROFILES:
        raise ValueError(f'unknown profile {name!r} (one of {", ".join(PROFILES)})')
    for section, values in PROFILES[name].items():
        cfg.setdefault(section, {}).update(values)
    cfg['profile'] = name
    return cfg
