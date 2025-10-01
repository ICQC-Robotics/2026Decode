Localization

Localization is the process of determining where your robot is on the field (its x and y position and heading) at any given time.

Why it matters: Correct localization helps your robot follow precise paths, avoid obstacles, and score consistently. Without it, your robot might drift or miss targets.

Key Constants
1. TRACK_WIDTH

This is the distance between the left and right wheels of your robot.

It is used to calculate turning arcs and changes in robot heading.

If it’s too small, turns become too sharp; if it’s too large, turns are too wide.

2. WHEEL_RADIUS

This is the radius of your drive wheels.

It determines how far the robot moves for each wheel rotation.

An incorrect value means the robot moves either too little or too far with each motor rotation.

3. GEAR_RATIO

This is the ratio between the motor output and wheel rotation.

For example, a 2:1 gear ratio means the motor spins twice for one wheel rotation.

An incorrect ratio leads to errors in distance and speed calculations.

Drive Feedforward vs Follower PID

Drive Feedforward predicts how much motor power is needed for a specific velocity or acceleration.

It’s like estimating the power required.

Follower PID corrects errors between the actual and desired positions or speeds.

It works like steering adjustments to keep the robot on its path.

Combining Feedforward and PID results in smooth, accurate motion.