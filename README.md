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

Now time for math. Let's say our end effector is at an arbitrary point **(x0,y0,z0)**. We'll do the computations for a single arm first. Let's say the motor shaft is at point **(x1,y1,z1)**, the upper segment is length **rf**, the lower segment is length **re**, and the end effector is a equilateral triangle of side length **e**.

We know we have a sphere with center near **(x0,y0,z0)** (offset a bit by the length of the effector) and a circle with center **(x1,y1,z1)**, and we have to find the intersection of those two shapes. To make it easier on ourselves, lets define the **yz** plane to be the plane of the fixed joint of the arm we're considering, with the **z** axis at the top of the arm and the **y** axis at the center of the robot. In this new coordinate system, the motor shaft is at point **(0, -f/(2sqrt(3)),0)**. 

The equation of our upper circle in the **yz** plane is **(y + f/(2sqrt(3)))^2 + z^2 = re^2**. The **yz** plane also intersects the sphere (see above image) to form a circle.  If we can find the equation of that circle in the **yz** plane, we have a much simpler problem of finding the intersection of two circles. First, we have to collapse the 3D point **(x0, y0, z0)** into the **yz** plane. We'll start by rotating the point so that the **y-axis** is aligned with the arm under consideration. If this arm makes an angle **theta** with our cartesian coordinate axis, we can perform this transformation using a rotation matrix:

**x = x0\*cos(theta) + y0\*sin(theta)**

**y = -x0\*sin(theta) + y0\*cos(theta)**

These **(x,y)** values become our new **x0 and y0**, and **z0** stays the same, since it's oe the axis of rotation. Since we know that our lower circle lies in the **yz** plane, its center must be at the point **(y0,z0)** using our new rotated vales. Great! Now all we need to fully define the lower circle is it's radius.

The lower joint is offset from the **yz** plane by a distance of **x0**, which follows naturally from our definition of the **yz** plane. With a little bit of geometric imagination, we can draw a right triangle with a hyportenuse of **rf** and leg lengths of **x0** and the radius of the lower circle. Thus, we can calculate the radius of this cricle as **sqrt(rf^2 - x0^2)**.

Now we have two coplanar circles with equations

**(y + f/(2sqrt(3)))^2 + z^2 = re^2**

and

**(y - y0 + e/(2sqrt(3))^2 + (z - z0)^2 = rf^2 - x0^2**

2 equations, 2 unkowns. Now just solve for **x** and **y**!




