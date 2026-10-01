# gemBot autonomy

The robot plays the autonomous round: it finds stones on the arena, carries each one to the zone of its colour, and earns points for correct placements. This glossary fixes the words used in code, logs and docs.

## Stones and vision

**Stone**:
The physical object on the arena floor that the robot collects. (หิน)
_Avoid_: gem, rock, block

**Observation**:
One vision detection of something stone-like in one frame, with a position and a Colour (possibly Unknown colour). (สิ่งที่กล้องเห็น)
_Avoid_: detection, blob

**Target**:
An Observation vision judges pickable: known Colour, and clear space to approach it. (หินที่หยิบได้)
_Avoid_: candidate, pick

**Jaw stone**:
A Target already lying between the open jaws; it is gripped on the spot, even if a different Stone was locked. (หินที่อยู่ในปากคีบแล้ว)
_Avoid_: held stone, stone in grip

## Colours and zones

**Colour**:
One of the six stone classes, numbered 1–6; each has exactly one Placement zone. (สีของหิน 6 สี)
_Avoid_: class, label

**Unknown colour**:
Colour 0 — vision saw something but is not sure which colour it is; never picked up, and never treated as a colour change. (ไม่แน่ใจว่าสีอะไร)
_Avoid_: none, no-colour, rainbow

**Placement zone**:
The marked floor area that accepts stones of one Colour. (โซนวางหินของแต่ละสี)
_Avoid_: scoring zone, target zone, goal

**Reference check**:
The per-frame comparison of the six zone circles against the empty-field background, to catch a camera that has moved. A circle under the robot's footprint is a covered circle and is not evidence either way. (ตรวจว่ากล้องขยับหรือไม่)
_Avoid_: guard, calibration check

**Camera moved**:
The Reference check's verdict that the camera itself has shifted — enough visible circles displaced beyond tolerance for a sustained time, not one noisy frame. The robot stops. (กล้องขยับจริง)
_Avoid_: reference moved (as a one-frame event)

## Scoring

**Placement**:
Any Stone left fully inside a Placement zone after the robot has let go and backed off, as judged by the TA; +5 in the matching zone, −1 in any other — including Stones pushed there by accident. The TA removes the Stone by hand right after. (การวางที่กรรมการนับคะแนน)
_Avoid_: drop, delivery

**Misplacement**:
A Placement in a zone that does not match the stone's Colour (−1).
_Avoid_: wrong drop

**Release**:
The robot opening its gripper to let go of a stone — the planner's belief that a Placement happened, not the Placement itself. (หุ่นเปิดปากคีบ)
_Avoid_: place, placed

## Grip check

**Gripper camera**:
The camera on the robot that looks down into the closed jaws; it judges only what the jaws hold, never where the robot is. (กล้องที่ปากคีบ)
_Avoid_: HuskyLens, hand cam, second camera

**Overhead camera**:
The fixed camera above the field; the only source of the robot pose and of Observations. (กล้องด้านบน)
_Avoid_: main camera, webcam

**Grip check**:
The Gripper camera's verdict on the closed jaws, taken once per grip while the robot stands still: Empty, Single, Multiple, or Unsure. A stone touching the tips of the closed jaws counts as held: the jaws push it along. (ตรวจว่าคีบได้กี่ก้อน)
_Avoid_: grab check, grip confirmation

**Unsure grip**:
A Grip check with no reliable verdict (no answer, no majority, unknown class); the robot carries on as if there were no Gripper camera. (ไม่แน่ใจว่าคีบได้)
_Avoid_: failed check, error

**Pick check**:
The Overhead camera's check, once the robot has moved off the pick spot, that the locked stone did not stay behind; a failure is a Missed pick. (ตรวจว่าหินยังอยู่ที่เดิมไหม)
_Avoid_: grab check, grab missed

## Points on the robot

**Tag centre**:
The centre of the robot's AprilTag; the robot pose is measured here. (จุดกลางแท็ก)
_Avoid_: robot position, centre

**Axle point**:
The middle of the wheel axle — the point that stays put when the robot turns in place. (จุดกึ่งกลางเพลาล้อ)
_Avoid_: robot centre, pivot

**Grip point**:
The point between the gripper jaws where a held stone sits. (จุดกลางปากคีบ)
_Avoid_: gripper, nose, front

**Stage point**:
Where the Axle point must stand so the Grip point is `stage_mm` short of the stone on its approach line. "Reached the stage" always means the Axle point is there. (จุดเตรียมคีบ)
_Avoid_: staging position, pre-grip point
