Shared Growth — Milestone 1: Robot Drawing Base

Goal
Get the myCobot 280 JN reliably drawing simple shapes on paper. No camera tracking or experiment logic yet.

Setup

myCobot 280 JN 2023
available normal/flex gripper
pen
fixed A4/A5 drawing surface
preferably fixed overhead RGB camera already mounted for later

Tasks

Connect to the robot via Python / pymycobot
Define safe home + working pose
Grip and hold the pen reliably
Define pen_up / pen_down
Calibrate a small drawing area
Draw simple primitives:
point
straight line
curve
Y-branch
Repeat the same movement several times and check wobble / positioning error
Implement safe stop + recovery

Useful software interfaces

home()
get_pose()

grab_pen()
release_pen()

pen_up()
pen_down()

move_to(x, y, z)
draw_line(start, end)
draw_curve(points)
draw_branch(origin, direction, length)

Acceptance criteria

Pen stays securely mounted
Robot can approach and leave paper safely
Same 5–10 cm line can be repeated consistently
Simple Y-branch can be drawn
No collision with table / paper / gripper
Drawing workspace and safe Z-height are documented

Out of scope for M1

OpenCV stroke recognition
human–robot interaction logic
growth algorithm
delay / jitter conditions
adaptive behavior
EDA / physiology (?)
