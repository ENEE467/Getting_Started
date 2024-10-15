.. Description of examples code
   10/11/24
   Abhishekh Reddy

Description of the examples code
================================

Here's the same snippet of code from the source file with the comments removed.

.. code-block:: C++

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
         move_group_interface_->execute(motion_plan_joints);

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

      tf2::Quaternion current_orientation_tf;
      tf2::convert(target_pose.orientation, current_orientation_tf);

      tf2::Quaternion desired_rotation_tf;
      desired_rotation_tf.setRPY(0, -M_PI_2, 0);

      auto target_orientation_tf = desired_rotation_tf * current_orientation_tf;
      tf2::convert(target_orientation_tf, target_pose.orientation);

      move_group_interface_->setPoseTarget(target_pose);
      move_group_interface_->move();

      // Example 4

      move_group_interface_->setNamedTarget("up");
      move_group_interface_->move();
   }

Each example follows these three steps at a high-level to move the robotic arm.

- Set the target (Either in joint-space or Cartesian space)

- Create a motion plan to the target state

- Execute the motion plan

Beginning part of the code
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: C++

   waitForMoveGroupInterface();

The first part of the code waits for the Move Group Interface node to initialize in the constructor
before calling any of its methods. It is an empty while loop conditioned to exit when
the Move Group Interface node initializes.

Example 1
^^^^^^^^^

The first example demonstrates moving the arm by setting a joint-space goal.

Set the target
--------------

.. code-block:: C++

   std::vector<double> target_joint_positions {M_PI_2, -M_PI_2, M_PI_2, -M_PI, -M_PI_2, 0};
   move_group_interface_->setJointValueTarget(target_joint_positions);

Here, a vector of doubles is used to store the desired joint positions in radians as a list. The
size of this vector corresponds to the number of joints in the robotic arm.

The ``setJointValueTarget`` method from the Move Group Interface takes this vector as the target
joint configuration, setting the goal for the robot's motion planning.

.. tip::

   The math library ``cmath`` provides constants for certain angles in radians like
   :math:`\frac{\pi}{2}` (``M_PI_2``) and :math:`\pi` (``M_PI``).

Create a motion plan to the target state
----------------------------------------

.. code-block:: C++

   moveit::planning_interface::MoveGroupInterface::Plan motion_plan_joints;
   bool joint_space_plan_success {planToJointSpaceGoal(target_joint_positions, motion_plan_joints)};

The motion plan is generated by the wrapper function ``planToJointSpaceGoal``, which internally calls
the ``plan()`` method of the Move Group Interface. The generated plan is stored in an external
``MoveGroupInterface::Plan`` variable. However, we wouldn't use the plan stored in this variable for
this example.

Wrapper functions, such as ``planToJointSpaceGoal``, are useful for abstracting the configuration,
condition checks, and error handling involved in generating a plan using the Move Group Interface.

Execute the motion plan
-----------------------

.. code-block:: C++

   if (joint_space_plan_success)
      move_group_interface_->move();

Finally, the ``move()`` method is called to execute the motion plan. MoveIt! automatically stores
the generated plan internally when the ``plan()`` method is called, so after the motion plan is
successfully generated, the arm can be moved using this stored plan when ``move()`` is called.
