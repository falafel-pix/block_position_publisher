the combination required to run can be changed.
Geomtric_controller.py has trajectory_tracking enabled and will run by default.
If you want to control using gamapad, geomtric_gamepad should be run, along with sender file in power to send button commands to WSL.
The sender.py should be running in powershell, even though here it is shown in the packge itself.
visuo_front should be running if you want front camera feed which has YOLO nano.
 visuo_inertial using feature tracking to estimate velocity and hence position.
   Both of the above are already executables and not in launch file, they need to be run separately as per requirement.
   Running to many image processing nodes causes RTF(real time factor) of gazebo to go down and hence the controller effectiveness decreases
