# robotics_report2
## Ros flow
### Prerequisites:
1. roscore
2. roslaunch ur_gazebo ur5e_bringup.launch
3. rosrun ur5e_control ur5e_controller
4. cd ~/Downloads && ./play_data.sh
5. rosrun robot_vision_lectures crop_visualize_3D
6. rosrun ur5e_control task_space_traj
7. roslaunch ur5e_control frame_publisher.launch
8. rosrun rviz rviz (to see frames and sphere)
9. rosrun rqt_gui rqt_gui (to trigger initialization and movement booleans)
10. Add message publishers that publish to /toggleInit and /toggleMove (make sure both stay False for now)

### Running movement plan
First run
1. rosrun robotics_report2 detect_ball.py
2. rosrun robotics_report2 sphere_fit.py
3. rosrun robotics_report2 cam_to_bot.py
4. rosrun robotics_report2 pickup_motion.py

Then
1. publish True to the /toggleInit topic
2. rosrun robotics_report2 brun_init.py
3. wait for good parameters in cam_to_bot and sphere_fit
4. stop brun_init
5. publich False to /toggleInit to stop initialization 

Finally
1. publish True to /toggleMove topic
