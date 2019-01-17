#include <cmath>
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
		pub_CurrentGoalPose = nh_.advertise<trajectory_msgs::JointTrajectory>("/parallel_gripper_controller/command",10);
		service = nh_.advertiseService("gripper_control", &GripperStiffnessController::ControlGripper, this);

		priv_nh_.getParam("M_admittance", M_admittance);
    priv_nh_.getParam("K_admittance", K_admittance);
		priv_nh_.getParam("B_admittance", B_admittance);
		priv_nh_.getParam("K_impedance", K_impedance);
		priv_nh_.getParam("B_impedance", B_impedance);
		priv_nh_.getParam("K_admittance", K_admittance);
		priv_nh_.getParam("K_admittance", K_admittance);

		goal.joint_names[0] = "";
		goal.joint_names[1] = "";
		SetGoal(0, 0, 0, 0, 0, 0, 0, 0, 0);
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
	if(req.command == "grip")
	{
		SetGoal(0, 0, 0, 0, 0, 0, 0, 0, 3);
		pub_CurrentGoalPose.publish(goal);

///////////////////////////////////////////////////////////////////////////////
		float pos_left, pos_right, vel_left, vel_right, acc_left, acc_right, eff_left, eff_right;
		float pre_pos_left = 0;
		float pre_pose_right = 0;

		while(ros::ok())
		{
///////////////////////////////Admittance Control///////////////////////////////
			pos_left = pos_gripper_left;//open 0.04; closed 0;
			pos_right = pos_gripper_right;
			vel_left = vel_gripper_left;
			vel_right = vel_gripper_right;
			acc_left = 1/M_admittance*(-eff_gripper_left-K_admittance*pos_gripper_left-B_admittance*vel_gripper_left);
			acc_right = 1/M_admittance*(-eff_gripper_right-K_admittance*pos_gripper_right-B_admittance*vel_gripper_right);
			eff_left = 0;
			eff_right = 0;
			SetGoal(pos_left, pos_right, vel_left, vel_right, acc_left, acc_right, eff_left, eff_right, 0.1);
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
			SetGoal(pos_left, pos_right, vel_left, vel_right, acc_left, acc_right, eff_left, eff_right, 0.1);
////////////////////////////////////////////////////////////////////////////////

			pub_CurrentGoalPose.publish(goal);
			if(fabs(pos_left - pre_pos_left) + fabs(pose_right - pre_pose_right) <= 0.005)
			{
				res.reply = true;
				break;
			}
			else
			{
				pre_pos_left = pos_left;
				pre_pose_right = pose_right;
			}
		}
	}
	else if(req.command == "release")
	{
		SetGoal(0.04, 0.04, 0, 0, 0, 0, 0, 0, 1);
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
    GripperStiffnessController node(nh);
		ros::spin();
    return 0;
}