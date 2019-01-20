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

	float M_admittance, K_admittance, B_admittance, M_impedance, K_impedance, B_impedance, M_current, K_current, B_current;
	float pos_left, pos_right, vel_left, vel_right, acc_left, acc_right, eff_left, eff_right;
	trajectory_msgs::JointTrajectory goal;

	void GetCurrentState(const sensor_msgs::JointState::ConstPtr& msg_curr_state);
	bool ControlGripper(gripper_stiffness_controller::gripper_control::Request &req, gripper_stiffness_controller::gripper_control::Response &res);
	void SetGoal(float pos_left, float pos_right, float vel_left, float vel_right, ros::Duration dt);

public:
	float t;
	float pos_target_left, pos_target_right;
	float pos_gripper_left, pos_gripper_right, eff_gripper_left, eff_gripper_right, vel_gripper_left, vel_gripper_right;

	void ControlProcess();

	GripperStiffnessController(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
	{
		sub_CurrentState = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 10, &GripperStiffnessController::GetCurrentState, this);
		pub_CurrentGoalPose = nh_.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command",10);
    service = nh_.advertiseService("gripper_control", &GripperStiffnessController::ControlGripper, this);

    priv_nh_.getParam("/gain/M_admittance", M_admittance);
    priv_nh_.getParam("/gain/K_admittance", K_admittance);
    priv_nh_.getParam("/gain/B_admittance", B_admittance);
		priv_nh_.getParam("/gain/M_impedance", M_impedance);
    priv_nh_.getParam("/gain/K_impedance", K_impedance);
    priv_nh_.getParam("/gain/B_impedance", B_impedance);

		M_current = M_admittance;
		K_current = K_admittance;
		B_current = B_admittance;

		vel_left = 0;
		vel_right = 0;

    goal.joint_names.push_back("gripper_left_finger_joint");
		goal.joint_names.push_back("gripper_right_finger_joint");
		goal.points.resize(1);
		goal.points[0].positions.resize(2);
		goal.points[0].velocities.resize(2);

		t = 0.02;

		pos_target_left = 0.04;
		pos_target_right = 0.04;

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
	if(eff_gripper_left < -0.3 || eff_gripper_right < -0.3)
	{
		ROS_INFO("Convert to Impedance Control");
		M_current = M_impedance;
		K_current = K_impedance;
		B_current = B_impedance;
	}
}

void GripperStiffnessController::SetGoal(float pos_left, float pos_right, float vel_left, float vel_right, ros::Duration dt)
{
	goal.points[0].positions[0] = pos_left;
	goal.points[0].positions[1] = pos_right;
	goal.points[0].velocities[0] = vel_left;
	goal.points[0].velocities[1] = vel_right;
	goal.points[0].time_from_start = dt;
	goal.header.stamp = ros::Time::now();
}


bool GripperStiffnessController::ControlGripper(gripper_stiffness_controller::gripper_control::Request &req, gripper_stiffness_controller::gripper_control::Response &res)
{
	ros::Duration dt;
	if(req.command == "grip")
	{
		pos_target_left = 0.0;
		pos_target_right = 0.0;
		M_current = M_admittance;
		K_current = K_admittance;
		B_current = B_admittance;
		res.reply = true;
	}
	else if(req.command == "release")
	{
		pos_target_left = 0.04;
		pos_target_right = 0.04;
		M_current = M_admittance;
		K_current = K_admittance;
		B_current = B_admittance;
		res.reply = true;
	}
	else
	{
		ROS_INFO("command should be \"grip\" or \"release\"!");
		res.reply = false;
	}
}

void GripperStiffnessController::ControlProcess()
{
	pos_left = pos_gripper_left;//open 0.04; closed 0;
	pos_right = pos_gripper_right;
	eff_left = -eff_gripper_left-K_current*(pos_gripper_left-pos_target_left)-B_current*vel_left;
	eff_right = -eff_gripper_right-K_current*(pos_gripper_right-pos_target_right)-B_current*vel_right;
	acc_left = 1/M_current*(eff_left);
	acc_right = 1/M_current*(eff_right);
	vel_left = vel_left + acc_left * t;
	vel_right = vel_right + acc_right * t;
	pos_left = pos_left + vel_left * t;
	pos_right = pos_right + vel_right * t;
	if((eff_left + eff_gripper_left) * (eff_left + eff_gripper_left) + (eff_right + eff_gripper_right)* (eff_right + eff_gripper_right) > 0.001)
	{
		SetGoal(pos_left, pos_right, vel_left, vel_right, ros::Duration(t));
		pub_CurrentGoalPose.publish(goal);
	}
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "gripper_controller");

	ros::NodeHandle nh;
	ros::Rate r(50);
	GripperStiffnessController node(nh);
	while (ros::ok())
	{
		node.ControlProcess();
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
