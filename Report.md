[//]: # (Image References)

[image1]: ./results/misc1.png
[image2]: ./results/misc2.png
[image3]: ./results/misc3.png
[image4]: ./results/image-1.png
[image5]: ./results/codecogseqn-3.gif
[image6]: ./results/codecogseqn-2.gif
[image7]: ./results/pasted_image_at_2017_07_02_10_21_pm.png
[image8]: ./results/imageedit_2_8171765159.jpg
[image9]: ./results/eq1.png

[image10]: ./results/inverse_kinematics.launch.JPG
[image11]: ./results/target_description.launch.JPG
[image12]: ./results/init_v.JPG
[image13]: ./results/Init_Variables.JPG
[image14]: ./results/sll_step.jpg


---

# Robotic arm - Pick & Place

* Make sure you are using robo-nd VM or have Ubuntu+ROS installed locally.

---


### One time Gazebo setup step:
* Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```
* To run projects from this repository you need version 7.7.0+
* If your gazebo version is not 7.7.0+, perform the update as follows:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

* Once again check if the correct version was installed:
```sh
$ gazebo --version
```
### For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly
* The project has a demo mode which shows the proper beahavior of the arm.

* For demo mode make sure the **demo** flag is set to _"true"_ in `inverse_kinematics.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch

* In addition, you can also control the spawn location of the target object in the shelf. 
* To do this, modify the **spawn_location** argument in `target_description.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.

![alt text][image11]

* You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

* If you are running in demo mode, this is all you need. 
* To run your own Inverse Kinematics code change the **demo** flag described above to _"false"_. 

![alt text][image10]

* And run your code (once the project has successfully loaded) by:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```
* Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:

	- Robot
	
	- Shelf
	
	- Blue cylindrical target in one of the shelves
	
	- Dropbox right next to the robot
	

* If any of these items are missing, report as an issue.

* Once all these items are confirmed, open rviz window, hit Next button to continue with the simulation.

---


# Robotic arm - Pick & Place : Reports

## Inverse Kinematics analysis 

# Kinematic Analysis
### Building the Transform matrices 
* The first step was to create a DH parameters table, the DH parameters table will help us in building the matrices to calculate the individual transforms between the links. 
* The DH parameters table is shown below, the diagram used to calculate the DH paramters is shown below the table:

Joint | alpha | a | d | theta
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.75 | 0
2 | -pi/2 | 0.35 | 0 | q2-pi/2
3 | 0 | 1.25 | 0 | 0
4 | -pi/2 | -0.054 | 1.50 | 0
5 | pi/2 | 0 | 0 | 0
6 | -pi/2 | 0 | 0 | 0
Gripper | 0 | 0 | 0.303| 0

![alt text][image8]

* Using this table we were able to construct the individual transform matrices, which are shown below:

![alt text][image13]

* Then, using them we can calculate the total transform between the base link and the end-effector which is denoted by the variable T0_7 and is shown below.

```python
T0_7 = ((((((T0_1 * T1_2) * T2_3) * T3_4) * T4_5) * T5_6) * T6_7)
```

* Which can be simplified to be written in terms of the gripper orientation (pitch, roll and yaw) and its position (px, py, pz)

**T0_7=** Matrix([  
[cos(pitch)\*cos(yaw)                               , -sin(yaw)\*cos(pitch                                , sin(pitch)            , px],  
[sin(pitch)\*sin(roll)\*cos(yaw)+sin(yaw)\*cos(roll), -sin(pitch)\*sin(roll)\*sin(yaw)+cos(roll)\*cos(yaw), -sin(roll)\*cos(pitch), py],  
[-sin(pitch)\*cos(roll)\*cos(yaw)+sin(roll)\*sin(yaw), sin(pitch)\*sin(yaw)\*cos(roll)+sin(roll)\*cos(yaw), cos(pitch)\*cos(roll) , pz],  
[0                                                   , 0                                                  , 0                     , 1 ])  

* Note that it is a simple multiplication of the matrices going from each link from the base to the gripper (the end-effector)

* Also note that this matrices are calculated in a functions called `Init_Variables()` they are then called on the function `IK_server` which is called whenever the program runs. 
* This is done to increase performance as the program was building the transformation matrices every run of the main loop.

## Calculating the joint angles
* An inverse kinematic problem can be divided into two, 
	- the first part is determining the postion of the end-effector this is done by simply calculating the angles for every joint **before** the wrist, 
	- and the orientation problem which is solved by calculating the angles for every joint **after** the wrist
* The wrist was determined to be in link 3 as joints 4, 5 and 6 are what give the enf-effector its orientation.

### Inverse Position Kinematic Problem
* As mentioned before to solve for the position we would need to find the angles for joints 1, 2 and 3. 
* Luckily the end-effector position and orientation are known. 
* Given these parameters, teh calculation of the wrist center and its x,y and z coordinates becomes trivial, and all that must be done is perform the calculations shown below

```python
px = req.poses[x].position.x
py = req.poses[x].position.y
pz = req.poses[x].position.z

(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
    [req.poses[x].orientation.x, req.poses[x].orientation.y,
        req.poses[x].orientation.z, req.poses[x].orientation.w])

# Given the orientation for the end -affector we can calculate its final rotation    
Rrpy = (R_x * R_y * R_z).evalf(subs={rollsym:roll,pitchsym:pitch,yawsym:yaw})

# With the orientation matrix and the coordinates point of the end-effector we calculate its wrist center 
wx = (px - (d6 + d7) * Rrpy[0,0]).subs(s)
wy = (py - (d6 + d7) * Rrpy[1,0]).subs(s)
wz = (pz - (d6 + d7) * Rrpy[2,0]).subs(s)
```

* Where `Rrpy` is calculated by performing a rotation matrix along each axis by the given roll, pitch and yawn angles, and `d6` is the length between the end-effector and link 3. 
* Now that we know the wrist coordinates we can start calculating the angles. 
* To calculate the angles geometry was heavily used, to better understand how the angles were positioned the diagram below was used

![alt text][image7]

* Thanks to Guangwei Wang (gwwang in Slack) for this image.

* The first angle to be calculated is theta 1 which as shown in the figure below is nothing more than the arctan of the y and x coordinates of the wrist.

* Joint angles 2 and 3 are far trickier. 
* To calculate the angle of joint 3 we have to acount for the extra angle caused by the second joint which creates and extra angle. 
* However, we can easily calculate the extra angle and the new x and z coordinates of the wrist using the equations below

```python
xtraangle = atan2(wz-1.94645, wx)
wx = wx-0.054*sin(extraangle)
wz = wz+0.054*cos(extraangle)
wxdist = sqrt(wy*wy+wx*wx)
```

* The last line calculates the distance between the wrist center and the new origin. 
* Then using cosine law we find D and with D calculate the third joint angle.

```python
D=(wxdist*wxdist + wzdist*wzdist - l1*l1-l2*l2)/(2*l1*l2)
theta3 = atan2(-sqrt(1-D*D),D
```

* With the third joint angle we can then calculate the second joint angle given by

```python
S1=((l1+l2*cos(theta3))*wzdist-l2*sin(theta3)*wxdist) / (wxdist*wxdist + wzdist*wzdist)
C1=((l1+l2*cos(theta3))*wxdist+l2*sin(theta3)*wzdist) / (wxdist*wxdist + wzdist*wzdist)
theta2=atan2(S1,C1)
```

### Inverse Orientation Kinematics
* To solve for the orientation of the end-effector we needto calculate the rotation between link 0 and link 6. 
* This rotation is nothing more than the rotation from the base link to link 3 multiplied with the Rrpy that we calculated earlier.

* After we obtain this rotation we can apply the following formulas.

![alt text][image4]

![alt text][image5]

![alt text][image6]

* However in the IK_server.py we use the code below from the tf library which calculate these formulas for us.

```python
alpha, beta, gamma = tf.transformations.euler_from_matrix(np.array(R3_6).astype(np.float64), "ryzy")
```

### Robotic arm - Pick & Place : Demo
![alt text][image14]
* Here's a [link to my video result](./results/RoboND-Kinematics-Project-Result(10x_encoding).avi)
