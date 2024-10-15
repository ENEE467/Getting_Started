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

   The included math library ``cmath`` provides constants for certain angles in radians like
   :math:`\frac{\pi}{2}` (``M_PI_2``) and :math:`\pi` (``M_PI``).

Create a motion plan to the target state
----------------------------------------

.. code-block:: C++

   moveit::planning_interface::MoveGroupInterface::Plan motion_plan_joints;
   bool joint_space_plan_success {planToJointSpaceGoal(target_joint_positions, motion_plan_joints)};

The motion plan is generated by the wrapper function ``planToJointSpaceGoal``, which internally calls
the ``plan()`` method of the Move Group Interface. The generated plan is stored in an external
``MoveGroupInterface::Plan`` variable.

Wrapper functions, such as ``planToJointSpaceGoal``, abstract the configuration, condition checks,
and error handling involved in generating a plan using the Move Group Interface.

Execute the motion plan
-----------------------

.. code-block:: C++

   if (joint_space_plan_success)
      move_group_interface_->execute(motion_plan_joints);

Finally, the ``execute()`` method is called to execute the motion plan which is previously generated
and stored in a variable.

Example 2
^^^^^^^^^

This example demonstrates drawing an equilateral triangle with the arm using a list of waypoints for
the end-effector to follow in Cartesian space.

Set the target
--------------

.. code-block:: C++

   std::vector<geometry_msgs::msg::Pose> waypoints;

   auto current_pose = move_group_interface_->getCurrentPose().pose;
   waypoints.push_back(current_pose);

A vector of ``Pose`` messages is used as a list to store the target waypoints.

Then the current pose of the end-effector is added as an initial waypoint. This position will also
be the center point of the shape, which depends on the arm's position following its previous motion
(Example 1).

Then the remaning waypoints relative to the first waypoint were added. For a triangle those
waypoints relative to the first waypoint could be:

A point directly above the center by half the height of the triangle
:math:`\frac{1}{2} \times \frac{\sqrt{3}}{2} \times a` , where :math:`a` is the side length in meters
which is hard coded to ``0.25``.

.. code-block:: C++

   double triangle_side {0.25};

   auto pose1 = current_pose;
   pose1.position.z += std::sqrt(3) * 0.25 * triangle_side;
   waypoints.push_back(pose1);

A point below the previous point by the triangle height :math:`\frac{\sqrt{3}}{2} \times a` and
towards the right by half the side length :math:`\frac{a}{2}`.

.. code-block:: C++

  auto pose2 = pose1;
  pose2.position.z -= std::sqrt(3) * 0.5 * triangle_side;
  pose2.position.y += 0.5 * triangle_side;
  waypoints.push_back(pose2);

A point towards the left from the previous point by side length :math:`a`.

.. code-block:: C++

  auto pose3 = pose2;
  pose3.position.y -= triangle_side;
  waypoints.push_back(pose3);

A point above by the triangle height :math:`\frac{\sqrt{3}}{2} \times a` and towards the right by
half the side length :math:`\frac{a}{2}`. (Same as the second point, closing the triangle loop)

.. code-block:: C++

  auto pose4 = pose3;
  pose4.position.z += std::sqrt(3) * 0.5 * triangle_side;
  pose4.position.y += 0.5 * triangle_side;
  waypoints.push_back(pose4);

Finally, a point just below the previous point by half the triangle height
:math:`\frac{1}{2} \times \frac{\sqrt{3}}{2}`, returning back to the center/initial point.

.. code-block:: C++

  auto pose5 = pose4;
  pose5.position.z -= std::sqrt(3) * 0.25 * triangle_side;
  waypoints.push_back(pose5);

.. note::

   As per the ROS convention `REP103 <REP103 Link_>`_ the directions of the coordinate axes are:

   - Positive Z-Axis for up direction
   - Positive X-Axis for forward direction
   - Positive Y-Axis for left direction (or right when seen from the front in this lab exercise)

Create a motion plan to the target state
----------------------------------------

.. code-block:: C++

  moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
  bool cartesian_plan_success {planCartesianPath(waypoints, cartesian_trajectory)};

Unlike the previous example, Cartesian plan is stored in a different object type.
``planCartesianPath`` provides abstraction from calling the ``computeCartesianPath`` method in the
Move Group Interface and time parameterization of the generated motion plan as a post-processing
step.

Execute the motion plan
-----------------------

.. code-block:: C++

  if (cartesian_plan_success && track_request_success)
    move_group_interface_->execute(cartesian_trajectory);

Executes Cartesian motion plan to make the end-effector follow the triangular path.

Example 3
^^^^^^^^^

This example demonstrates moving the robotic arm by setting a Cartesian space goal for the
end-effector.

Set the target
--------------

Unlike the previous example, only a single ``Pose`` message is used to set the target instead of a
list. This target pose is initialized with the current end-effector pose to begin with.

.. code-block:: C++

   geometry_msgs::msg::Pose target_pose;
   target_pose = move_group_interface_->getCurrentPose().pose;

Linear displacements along the Z and Y axes are applied relative to the current position.

.. code-block:: C++

   target_pose.position.z += 0.05;
   target_pose.position.y += 0.1;

Angular displacements are applied similarly, relative to the current orientation. However, since
``Pose`` messages store orientation using quaternions, it is not intuitive to set rotation values
directly.

Instead, angular displacements can be defined using roll-pitch-yaw values and then converted to
quaternions. While quaternions from ``geometry_msgs`` do not support this conversion, ``TF2``
quaternions do, and they also provide functionality for converting between these two types.

So first, the current orientation of the end-effector is converted from ``geometry_msgs`` quaternion
to ``TF2`` quaternion.

.. code-block:: C++

   tf2::Quaternion current_orientation_tf;
   tf2::convert(target_pose.orientation, current_orientation_tf);

Then, a new ``TF2`` quarternion message is defined with the desired rotations in roll-pitch-yaw.

.. code-block:: C++

   tf2::Quaternion desired_rotation_tf;
   desired_rotation_tf.setRPY(0, -M_PI_2, 0);

This new rotation is then applied relative to current orientation by pre-multiplying it with the
current orientation quarternion.

.. code-block:: C++

   auto target_orientation_tf = desired_rotation_tf * current_orientation_tf;
   tf2::convert(target_orientation_tf, target_pose.orientation);

The resultant ``TF2`` quarternion is then converted back to ``geometry_msgs`` quaternion to set the
target pose.

.. code-block:: C++

   move_group_interface_->setPoseTarget(target_pose);

Create a motion plan to the target state
----------------------------------------

This will be automatically handled in the next step.

Execute the motion plan
-----------------------

.. code-block:: C++

   move_group_interface_->move();

.. tip::

   The ``move()`` method combines both the planning ``plan()`` and execution ``execute()`` steps
   into a single function call. However, using the ``plan()`` and ``execute()`` methods separately
   gives more flexibility in many cases.

   This does not work for planning and executing Cartesian paths from Example 2.

Example 4
^^^^^^^^^

This example demonstrates moving the robotic arm to a predefined position using names.

Set the target
--------------

.. code-block:: C++

   move_group_interface_->setNamedTarget("up");

The available options for named targets you can set is based on the MoveIt! configuration for the
robot. More specifically, based on the definitions in the *Semantic Robot Description Format (.srdf)*
file.

The listed definitions in `Lab's SRDF file <Lab SRDF Link_>`_ are shown in the snippet below.

.. code-block:: xml

   <!--GROUP STATES - Purpose - Define a named state for a particular group, in terms of joint values. -->
   <!--This is useful to define states like 'folded arms'-->

   <group_state name="home" group="ur_manipulator">
      <joint name="elbow_joint" value="0" />
      <joint name="shoulder_lift_joint" value="-1.5707" />
      <joint name="shoulder_pan_joint" value="0" />
      <joint name="wrist_1_joint" value="0" />
      <joint name="wrist_2_joint" value="0" />
      <joint name="wrist_3_joint" value="0" />
   </group_state>

   <group_state name="up" group="ur_manipulator">
      <joint name="elbow_joint" value="0" />
      <joint name="shoulder_lift_joint" value="-1.5707" />
      <joint name="shoulder_pan_joint" value="0" />
      <joint name="wrist_1_joint" value="-1.5707" />
      <joint name="wrist_2_joint" value="0" />
      <joint name="wrist_3_joint" value="0" />
   </group_state>

   <group_state name="test_configuration" group="ur_manipulator">
      <joint name="elbow_joint" value="1.4" />
      <joint name="shoulder_lift_joint" value="-1.62" />
      <joint name="shoulder_pan_joint" value="1.54" />
      <joint name="wrist_1_joint" value="-1.2" />
      <joint name="wrist_2_joint" value="-1.6" />
      <joint name="wrist_3_joint" value="-0.11" />
   </group_state>

Create a motion plan to the target state
----------------------------------------

This will be automatically handled in the next step.

Execute the motion plan
-----------------------

.. code-block:: C++

   move_group_interface_->move();
