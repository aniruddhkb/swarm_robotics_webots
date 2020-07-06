# swarm_robotics_webots
![Image](https://github.com/aniruddhkb/swarm_robotics_webots/raw/master/Screenshot.png)

This is a repo containing my work in implementing common swarm robotics algorithms using the Webots simulator.

This has been tested on Webots R2020b Nightly Build 26/6/2020. Hopefully, this should work on your system as well.

To use this, simply clone this repo to a folder of your choice and open one of the world files in a repo. A good starting 
point would be /worlds/swarm_basic_flocking.wbt and /controllers/swarm_basic_flocking/swarm_basic_flocking.py (WARNING--
YET TO BE COMMENTED AND PENDING REFACTORING).

Please feel free to contribute through issues and pull requests.

I won't be making a Wiki yet, since there are so few files involved. This may change, and I believe in over-documentation as 
opposed to under-documentation. Still, it may be some time before I get to that stage. For the time being, below is an explanation of the main robot models and controllers.

Thanks for dropping by!

# ChuhaBasic and ChuhaLidarCamera

ChuhaBasic is a simple differential-drive robot. It comes with two wheels (driven by motors with encoders) and an IMU.

The robot is roughly 6 cm in diameter and the wheels are 1.5 cm in diameter.

The motors can rotate at 60 radians/sec (or a little less than 600 RPM). The encoders give the angular position of the 
wheels.
The IMU gives the roll-pitch-yaw for the robot (though I think the IMU is misaligned by 90 degrees -- use only yaw for now).

ChuhaLidarCamera has the same, plus a LIDAR, and two cameras. There are three displays in total -- two for the cameras and 
one which is programmable. This third display can be used to visualize the LIDAR data.

Both of these can be found in the worlds/chuha_definitions.wbt world. They can be used using the relevant PROTO files.

# The worlds:
chuha_definitions : Has the two robots. Can play around a bit.

lidar_visualization: To understand how to use the LIDAR and the display.

odometry_basic_teleop: Basic odometry and dead-heading implementation.

basic_SLAM_simple: SLAM is a misnomer here. The map is 
generated (in a very basic way), but the data is not used 
in any way.

swarm_basic_flocking: My first swarm algorithm. Flocking 
without any care for collisions.
