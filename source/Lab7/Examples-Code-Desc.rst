.. Description of examples code
   10/11/24
   Abhishekh Reddy

Description of the examples code
================================

TODO: Work in progress.

Here's the same snippet of code from the source file with the comments removed.

.. code-block:: c++

   void UR3eMoveInterface::examplesMoveIt()
   {
      waitForMoveGroupInterface();
      RCLCPP_INFO(this->get_logger(), "Running examples...");

      // Example 1

      RCLCPP_INFO(this->get_logger(), "Example 1");

      std::vector<double> target_joint_positions {M_PI_2, -M_PI_2, M_PI_2, -M_PI, -M_PI_2, 0};
      move_group_interface_->setJointValueTarget(target_joint_positions);
      moveit::planning_interface::MoveGroupInterface::Plan motion_plan_joints;

      bool joint_space_plan_success {planToJointSpaceGoal(target_joint_positions, motion_plan_joints)};

      if (joint_space_plan_success)
         move_group_interface_->move();

      // Example 2

      RCLCPP_INFO(this->get_logger(), "Example 2");

      std::vector<geometry_msgs::msg::Pose> waypoints;
      auto current_pose = move_group_interface_->getCurrentPose().pose;
      waypoints.push_back(current_pose);

      double triangle_side {0.25};

      auto pose1 = current_pose;
      pose1.position.z += std::sqrt(3) * 0.25 * triangle_side;
      waypoints.push_back(pose1);

      auto pose2 = pose1;
      pose2.position.z -= std::sqrt(3) * 0.5 * triangle_side;
      pose2.position.y += 0.5 * triangle_side;
      waypoints.push_back(pose2);

      auto pose3 = pose2;
      pose3.position.y -= triangle_side;
      waypoints.push_back(pose3);

      auto pose4 = pose3;
      pose4.position.z += std::sqrt(3) * 0.5 * triangle_side;
      pose4.position.y += 0.5 * triangle_side;
      waypoints.push_back(pose4);

      auto pose5 = pose4;
      pose5.position.z -= std::sqrt(3) * 0.25 * triangle_side;
      waypoints.push_back(pose5);

      moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
      bool cartesian_plan_success {planCartesianPath(waypoints, cartesian_trajectory)};

      auto track_request {std::make_shared<lab7::srv::TrackRequest::Request>()};

      track_request->tf_root_frame_name = move_group_interface_->getPlanningFrame();
      track_request->tf_tip_frame_name = move_group_interface_->getEndEffectorLink();

      track_request->plot_title = "Example: A Triangle in the vertical plane";
      track_request->plot_axis_x = lab7::srv::TrackRequest::Request::Y_AXIS;
      track_request->plot_axis_y = lab7::srv::TrackRequest::Request::Z_AXIS;

      track_request->status = lab7::srv::TrackRequest::Request::START;

      bool track_request_success {sendEEFTrackRequest(track_request)};

      if (cartesian_plan_success && track_request_success)
         move_group_interface_->execute(cartesian_trajectory);

      track_request->status = lab7::srv::TrackRequest::Request::STOP;
      sendEEFTrackRequest(track_request);

      // Example 3
      RCLCPP_INFO(this->get_logger(), "Example 3");

      geometry_msgs::msg::Pose target_pose;
      target_pose = move_group_interface_->getCurrentPose().pose;
      target_pose.position.z += 0.05;
      target_pose.position.y += 0.1;

      /**
         * Step 2 Continuation - Setting the target orientation relative to current orientation.
         *
         * Steps to rotate the end-effector by a certain value:
         * - Create an empty TF2 Quaternion message
         * - Set the rotation as desired using roll-pitch-yaw angles
         * - Apply the rotation by pre-multiplying with the current orientation quaternion
         * - Convert the TF2 quaternion message to geometry message with the new
         *   orientation encoded into the target pose.
         *
         * In short, TF2 quaternions are used as an intermediate datatype to do
         * rotation math since geometry quaternions do not have that functionality.
         */
      tf2::Quaternion desired_rotation_tf;
      desired_rotation_tf.setRPY(0, -M_PI_2, 0);

      tf2::Quaternion current_orientation_tf;
      tf2::convert(target_pose.orientation, current_orientation_tf);
      auto target_orientation_tf = desired_rotation_tf * current_orientation_tf;

      tf2::convert(target_orientation_tf, target_pose.orientation);

      // Step 3
      move_group_interface_->setPoseTarget(target_pose);

      // Step 4
      moveit::planning_interface::MoveGroupInterface::Plan motion_plan_pose;

      // Step 5
      bool pose_plan_success {planToPoseGoal(target_pose, motion_plan_pose)};

      // Step 6
      if (pose_plan_success)
         move_group_interface_->move();

      /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
         * EXAMPLE 4 - Moving the robot to a named, predefined position.
         *
         * All the predefined positions are defined in lab.srdf from the lab_moveit_config package in
         * joint space.
         */
      moveit::planning_interface::MoveGroupInterface::Plan motion_plan_named_target;
      bool named_target_plan_success {planToNamedTarget("up", motion_plan_named_target)};

      if (named_target_plan_success)
         move_group_interface_->move();
   }
