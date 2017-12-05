# DeltaArm
Delta Arm prototype for the DPEA. Contains inverse kinematics for guiding effector to a position in (x,y,z) space.

## Introduction to Delta Robots

A Delta Robot has a series of motors mounted at fixed points, which attach to series of joints convering on an end effector. 

![Image of Delta Robot](https://industrial.omron.us/en/media/Delta_robot_XL_tcm849-100817.jpg)

For more information see [wikipedia](https://en.wikipedia.org/wiki/Delta_robot) or [The Astley Lectures](https://www.youtube.com/watch?v=dQw4w9WgXcQ)

## Kinematics

To control a delta robot, we need to convert a point in **(x,y,z)** space into a series of joint angles that will drive the end effector to that spot. This is called an _inverse kinematics_ problem.

Let's consider one arm of the robot. Since it is attached to the end effector on a ball joint, the lower segment of the arm can theoretically trace out a sphere around the end effector. In contrast, the upper joint is a fixed joint, meaning it only rotates in one plane. This means that the upper segment can only trace out a circle around its motor shaft. Since both segments of the arm are rigid bodies, the the circle and sphere traced out by the upper and lower arm segments must intersect in order for the end effector to move to a certain point.

![Image of Joint](https://i.stack.imgur.com/E1h9z.png)

So basically, all we have to do is draw a sphere around the point we want to go to, draw a cricle around the motor shaft of our joint, and find where they intersect. After we know the intersection point, we can use some simple trig to find the joint angle. EZ.

Now time for math. 




