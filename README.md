### Tutorial 2

Branch: Tutorial 2


For using keyboard to move around: rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/pioneer/cmd_vel\\

For starting the robot: roslaunch rto_bringup_sim robot.launch\\

Starting my node: rosrun my_package_tut2 dontHitWallsNode.py\\





## 1 Does your controller work out of the box? Can you hit walls?
The controller does not work out of box because I have to adjust the topic for the subscriber and the publisher


## 2 Does your controller work out of the box? Can you hit walls?
My first solution workes for both, differential and omnidirectional drive as I use a min and maxDistance around the robot in every direction.

## 3 Make sure your controller works now for both robots by testing it in the simulated world.
It works if it doesn't get any new Input from the keyboard (No idea how to solve this)

