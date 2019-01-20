#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <gripper_stiffness_controller/gripper_control.h>


class GripperStiffnessController
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle priv_nh_;

	ros::Subscriber sub_CurrentState; //pos. vel. eff.
	ros::Publisher pub_CurrentGoalPose;
	ros::Publisher pub_CurrentGoalEffort;
	ros::ServiceServer service;

	float M_admittance, K_admittance, B_admittance, K_impedance, B_impedance;
	trajectory_msgs::JointTrajectory goal;

	void GetCurrentState(const sensor_msgs::JointState::ConstPtr& msg_curr_state);
	bool ControlGripper(gripper_stiffness_controller::gripper_control::Request &req, gripper_stiffness_controller::gripper_control::Response &res);
	void SetGoal(float pos_left, float pos_right, float vel_left, float vel_right, float acc_left, float acc_right, float eff_left, float eff_right, ros::Duration dt);

public:
	float pos_gripper_left,pos_gripper_right,eff_gripper_left,eff_gripper_right,vel_gripper_left,vel_gripper_right;

	GripperStiffnessController(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
	{
		sub_CurrentState = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 10, &GripperStiffnessController::GetCurrentState, this);
		pub_CurrentGoalPose = nh_.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command",10);
		pub_CurrentGoalEffort = nh_.advertise<trajectory_msgs::JointTrajectory>("/gripper_effort_controller/command",10);
    service = nh_.advertiseService("gripper_control", &GripperStiffnessController::ControlGripper, this);

<<<<<<< HEAD
    priv_nh_.getParam("/gain/M_admittance", M_admittance);
    priv_nh_.getParam("/gain/K_admittance", K_admittance);
    priv_nh_.getParam("/gain/B_admittance", B_admittance);
    priv_nh_.getParam("/gain/K_impedance", K_impedance);
    priv_nh_.getParam("/gain/B_impedance", B_impedance);
    //ROS_INFO_STREAM(M_admittance);
=======
		priv_nh_.getParam("M_admittance", M_admittance);
    priv_nh_.getParam("K_admittance", K_admittance);
		priv_nh_.getParam("B_admittance", B_admittance);
		priv_nh_.getParam("K_impedance", K_impedance);
		priv_nh_.getParam("B_impedance", B_impedance);
		priv_nh_.getParam("K_admittance", K_admittance);
		priv_nh_.getParam("K_admittance", K_admittance);
>>>>>>> parent of 89671ce... demo

    goal.joint_names.push_back("gripper_left_finger_joint");
		goal.joint_names.push_back("gripper_right_finger_joint");
		goal.points.resize(1);
		goal.points[0].positions.resize(2);
		goal.points[0].velocities.resize(2);
		goal.points[0].accelerations.resize(2);
		goal.points[0].effort.resize(2);

		ROS_INFO("Gripper Controller initialization done!");
	}

	~GripperStiffnessController()
	{

	}
};

void GripperStiffnessController::GetCurrentState(const sensor_msgs::JointState::ConstPtr& msg_curr_state)
{
	pos_gripper_left = (msg_curr_state->position)[7];
	pos_gripper_right = (msg_curr_state->position)[8];
	vel_gripper_left = (msg_curr_state->velocity)[7];
	vel_gripper_right = (msg_curr_state->velocity)[8];
	eff_gripper_left = (msg_curr_state->effort)[7];
	eff_gripper_right = (msg_curr_state->effort)[8];
<<<<<<< HEAD
  ROS_INFO_STREAM("eff_gripper_left="<<eff_gripper_left);
=======
>>>>>>> parent of 89671ce... demo
}

void GripperStiffnessController::SetGoal(float pos_left, float pos_right, float vel_left, float vel_right, float acc_left, float acc_right, float eff_left, float eff_right, ros::Duration dt)
{
	goal.points[0].positions[0] = pos_left;
	goal.points[0].positions[1] = pos_right;
	goal.points[0].velocities[0] = vel_left;
	goal.points[0].velocities[1] = vel_right;
	goal.points[0].accelerations[0] = acc_left;
	goal.points[0].accelerations[1] = acc_right;
	goal.points[0].effort[0] = eff_left;
	goal.points[0].effort[1] = eff_right;
	goal.points[0].time_from_start = dt;
	goal.header.stamp = ros::Time::now();
}


bool GripperStiffnessController::ControlGripper(gripper_stiffness_controller::gripper_control::Request &req, gripper_stiffness_controller::gripper_control::Response &res)
{
	ros::Duration dt;
	if(req.command == "grip")
	{
<<<<<<< HEAD
    dt = ros::Duration(0.02);
    //SetGoal(float(0.02), float(0.02), float(0), float(0), float(0), float(0), float(0), float(0), dt);
    //pub_CurrentGoalPose.publish(goal);
    //dt.sleep();
    SetGoal(float(0.04), float(0.04), float(0), float(0), float(0), float(0), float(-0.2), float(-0.2), dt);
    pub_CurrentGoalPose.publish(goal);
    dt.sleep();
    res.reply = true;

=======
		dt = ros::Duration(2.0);
    //SetGoal(float(0), float(0), float(0), float(0), float(0), float(0), float(0), float(0), dt);
    //pub_CurrentGoalPose.publish(goal);
		SetGoal(float(0.02), float(0.02), float(0), float(0), float(0), float(0), float(0), float(0), dt);
		pub_CurrentGoalPose.publish(goal);
		res.reply = true;
>>>>>>> parent of 89671ce... demo
///////////////////////////////////////////////////////////////////////////////
		/*float pos_left, pos_right, vel_left, vel_right, acc_left, acc_right, eff_left, eff_right;
		float pre_pos_left = 0;
    float pre_pos_right = 0;
<<<<<<< HEAD
    ROS_INFO("Start admittance control!!!!!!!!!!!!");

    vel_left = 0;
    vel_right = 0;

=======
>>>>>>> parent of 89671ce... demo

    while(ros::ok())
		{
///////////////////////////////Admittance Control///////////////////////////////
      float t = 0.02;
			pos_left = pos_gripper_left;//open 0.04; closed 0;
			pos_right = pos_gripper_right;
      //vel_left = vel_gripper_left;
      //vel_right = vel_gripper_right;
      acc_left = 1/M_admittance*(-eff_gripper_left-K_admittance*pos_gripper_left-B_admittance*vel_left);
      acc_right = 1/M_admittance*(-eff_gripper_right-K_admittance*pos_gripper_right-B_admittance*vel_right);
      vel_left = vel_left + acc_left * t;
      vel_right = vel_right + acc_right * t;
      pos_left = pos_left + vel_left * t;
      pos_right = pos_right + vel_right * t;
      eff_left = 0;
			eff_right = 0;
      dt = ros::Duration(t);
      SetGoal(pos_left, pos_right, vel_left, vel_right, acc_left, acc_right, eff_left, eff_right, dt);
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////Impedance Control////////////////////////////////
			pos_left = pos_gripper_left;//open 0.04; closed 0;
			pos_right = pos_gripper_right;
			vel_left = vel_gripper_left;
			vel_right = vel_gripper_right;
			acc_left = 0;
			acc_right = 0;
			eff_left = B_impedance * vel_gripper_left + K_impedance * pos_gripper_left - eff_gripper_left;
			eff_right = B_impedance * vel_gripper_right + K_impedance * pos_gripper_right - eff_gripper_right;
      SetGoal(pos_left, pos_right, vel_left, vel_right, acc_left, acc_right, eff_left, eff_right, dt);
////////////////////////////////////////////////////////////////////////////////

			pub_CurrentGoalPose.publish(goal);
      if((pos_left - pre_pos_left)^2 + (pos_right - pre_pos_right)^2 <= 0.00001)
			{
				res.reply = true;
				break;
			}
			else
			{
				pre_pos_left = pos_left;
        pre_pos_right = pos_right;
<<<<<<< HEAD
      }*/
      //pub_CurrentGoalPose.publish(goal);
      dt.sleep();
    }
=======
			}
    }*/
>>>>>>> parent of 89671ce... demo
	}
	else if(req.command == "release")
	{
    dt = ros::Duration(1.0);
		SetGoal(0.04, 0.04, float(0), float(0), float(0), float(0), float(0), float(0), dt);
		pub_CurrentGoalPose.publish(goal);
		res.reply = true;
	}
	else
	{
		ROS_INFO("command should be \"grip\" or \"release\"!");
		res.reply = false;
	}
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "gripper_controller");

	ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2); // Use 2 threads
  spinner.start();
	GripperStiffnessController node(nh);
  //ros::spin();
  ros::waitForShutdown();
	return 0;
}
