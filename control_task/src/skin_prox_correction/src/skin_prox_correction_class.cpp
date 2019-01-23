#include <skin_prox_correction_class.h>

SkinProxCorrection::SkinProxCorrection(ros::NodeHandle nh) : nh_(nh)
{
  //sub_goal_position_from_perception = nh_.subscribe<geometry_msgs::PoseStamped>("/*add_topic*/", 10, &SkinProxCorrection::Get_position_from_percertion, this);
  pub_goal_position_correction = nh_.advertise<geometry_msgs::PoseStamped>("/control/goal_pose", 10);
  patch_1 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch1", 10, &SkinProxCorrection::Get_prox_pathch_1, this);
  patch_2 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch2", 10, &SkinProxCorrection::Get_prox_pathch_2, this);
  patch_3 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch3", 10, &SkinProxCorrection::Get_prox_pathch_3, this);
  patch_4 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch4", 10, &SkinProxCorrection::Get_prox_pathch_4, this);
  patch_5 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch5", 10, &SkinProxCorrection::Get_prox_pathch_5, this);
  patch_6 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch6", 10, &SkinProxCorrection::Get_prox_pathch_6, this);
  patch_7 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch7", 10, &SkinProxCorrection::Get_prox_pathch_7, this);
  patch_8 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch8", 10, &SkinProxCorrection::Get_prox_pathch_8, this);
  patch_9 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch9", 10, &SkinProxCorrection::Get_prox_pathch_9, this);
  patch_10 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch10", 10, &SkinProxCorrection::Get_prox_pathch_10, this);

  for(int i=0;i<10;i++)
  {
    cellId_patch[i] = 0;
    prox_patch[i] = 0;
  }

  weight_x = 5;
  weight_y = 5;
  weight_z = 5;

  threshold_1 = 0.1;
  threshold_2 = 0.1;
  delta_x = 0;
  max_delta_x = 1;
  direction = 1;

  current_position.x = 0;
  current_position.y = 0;
  current_position.z = 0;
  goal_position_from_perception.x = 0;
  goal_position_from_perception.y = 0;
  goal_position_from_perception.z = 0;
}

/*void SkinProxCorrection::Get_position_from_percertion(const geometry_msgs::PoseStamped::ConstPtr &position_from_perception) //how to judge the postion which was changed ???
{
  goal_position_from_perception.x = position_from_perception->pose.position.x;
  goal_position_from_perception.y = position_from_perception->pose.position.y;
  goal_position_from_perception.z = position_from_perception->pose.position.z;
}*/

void SkinProxCorrection::Get_prox_pathch_1(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of left finger
{
  prox_patch[0] = data_patch->data.data()->prox[0];
  Movement_correction();
}

void SkinProxCorrection::Get_prox_pathch_2(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  prox_patch[1] = data_patch->data.data()->prox[0];
  Movement_correction();
}

void SkinProxCorrection::Get_prox_pathch_3(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  for(int i=0,i<data_patch->data.size(),i++)
  {
    if(data_patch->data[i].data()->prox[0] > prox_patch[2])
    {
      prox_patch[2] = data_patch->data[i].data()->prox[0];
      cellId_patch[2] = data_patch->data[i].data()->cellId;
      Movement_correction();
    }
    else if(cellId_patch[2] == data_patch->data[i].data()->cellId)
    {
      prox_patch[2] = data_patch->data[i].data()->prox[0];
      Movement_correction();
    }
  }
}

void SkinProxCorrection::Get_prox_pathch_4(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  for(int i=0,i<data_patch->data.size(),i++)
  {
    if(data_patch->data[i].data()->prox[0] > prox_patch[3])
    {
      prox_patch[3] = data_patch->data[i].data()->prox[0];
      cellId_patch[3] = data_patch->data[i].data()->cellId;
      Movement_correction();
    }
    else if(cellId_patch[3] == data_patch->data[i].data()->cellId)
    {
      prox_patch[3] = data_patch->data[i].data()->prox[0];
      Movement_correction();
    }
  }
}

void SkinProxCorrection::Get_prox_pathch_5(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  for(int i=0,i<data_patch->data.size(),i++)
  {
    if(data_patch->data[i].data()->prox[0] > prox_patch[4])
    {
      prox_patch[4] = data_patch->data[i].data()->prox[0];
      cellId_patch[4] = data_patch->data[i].data()->cellId;
      Movement_correction();
    }
    else if(cellId_patch[4] == data_patch->data[i].data()->cellId)
    {
      prox_patch[4] = data_patch->data[i].data()->prox[0];
      Movement_correction();
    }
  }
}

void SkinProxCorrection::Get_prox_pathch_6(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  for(int i=0,i<data_patch->data.size(),i++)
  {
    if(data_patch->data[i].data()->prox[0] > prox_patch[5])
    {
      prox_patch[5] = data_patch->data[i].data()->prox[0];
      cellId_patch[5] = data_patch->data[i].data()->cellId;
      Movement_correction();
    }
    else if(cellId_patch[5] == data_patch->data[i].data()->cellId)
    {
      prox_patch[5] = data_patch->data[i].data()->prox[0];
      Movement_correction();
    }
  }
}

void SkinProxCorrection::Get_prox_pathch_7(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  for(int i=0,i<data_patch->data.size(),i++)
  {
    if(data_patch->data[i].data()->prox[0] > prox_patch[6])
    {
      prox_patch[6] = data_patch->data[i].data()->prox[0];
      cellId_patch[6] = data_patch->data[i].data()->cellId;
      Movement_correction();
    }
    else if(cellId_patch[6] == data_patch->data[i].data()->cellId)
    {
      prox_patch[6] = data_patch->data[i].data()->prox[0];
      Movement_correction();
    }
  }
}

void SkinProxCorrection::Get_prox_pathch_8(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  for(int i=0,i<data_patch->data.size(),i++)
  {
    if(data_patch->data[i].data()->prox[0] > prox_patch[7])
    {
      prox_patch[7] = data_patch->data[i].data()->prox[0];
      cellId_patch[7] = data_patch->data[i].data()->cellId;
      Movement_correction();
    }
    else if(cellId_patch[7] == data_patch->data[i].data()->cellId)
    {
      prox_patch[7] = data_patch->data[i].data()->prox[0];
      Movement_correction();
    }
  }
}

void SkinProxCorrection::Get_prox_pathch_9(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  for(int i=0,i<data_patch->data.size(),i++)
  {
    if(data_patch->data[i].data()->prox[0] > prox_patch[8])
    {
      prox_patch[8] = data_patch->data[i].data()->prox[0];
      cellId_patch[8] = data_patch->data[i].data()->cellId;
      Movement_correction();
    }
    else if(cellId_patch[8] == data_patch->data[i].data()->cellId)
    {
      prox_patch[8] = data_patch->data[i].data()->prox[0];
      Movement_correction();
    }
  }
}

void SkinProxCorrection::Get_prox_pathch_10(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  for(int i=0,i<data_patch->data.size(),i++)
  {
    if(data_patch->data[i].data()->prox[0] > prox_patch[9])
    {
      prox_patch[9] = data_patch->data[i].data()->prox[0];
      cellId_patch[9] = data_patch->data[i].data()->cellId;
      Movement_correction();
    }
    else if(cellId_patch[9] == data_patch->data[i].data()->cellId)
    {
      prox_patch[9] = data_patch->data[i].data()->prox[0];
      Movement_correction();
    }
  }
}

void SkinProxCorrection::Movement_correction()
{
  prox_xL = (prox_patch[2]>prox_patch[5])?prox_patch[2]:prox_patch[5];
  prox_xL = (prox_patch[9]>prox_xL)?prox_patch[9]:prox_xL;
  prox_xR = (prox_patch[3]>prox_patch[4])?prox_patch[3]:prox_patch[4];
  prox_xR = (prox_patch[7]>prox_xR)?prox_patch[7]:prox_xR;
  prox_yL = prox_patch[8];
  prox_yR = prox_patch[6];
  prox_zL = (prox_patch[0]>prox_patch[1])?prox_patch[0]:prox_patch[1];
  prox_zR = 0;
  delta_ax = weight_x * (prox_xL * prox_xL - prox_xR * prox_xR);
  delta_ay = weight_y * (prox_yL * prox_yL - prox_yR * prox_yR);
  delta_az = weight_z * (prox_zL * prox_zL - prox_zR * prox_zR);
}

void SkinProxCorrection::Position_correction()
{
  if( prox_patch[0] < threshold_1 && prox_patch[1] < threshold_2)
  {
    ROS_INFO_STREAM("OK, I can grip !!!!!!!!!!");
    //ros::service::call
    return true;
  }
  else
  {
    if(prox_patch[0] > threshold_1 && prox_patch[1] < threshold_2)
    {
      direction = -1;
      ROS_INFO_STREAM("Move left !!!!!!!!!!");
    }
    if(prox_patch[0] < threshold_1 && prox_patch[1] > threshold_2)
    {
      direction = 1;
      ROS_INFO_STREAM("Move right !!!!!!!!!!");
    }

    delta_x = delta_x + 0.1;

    goal_position_correction.pose.position.x = goal_position_from_perception.pose.position.x - delta_x * direction;
    goal_position_correction.pose.position.y = goal_position_from_perception.pose.position.y;
    goal_position_correction.pose.position.z = goal_position_from_perception.pose.position.z;
    pub_goal_position_correction.publish(goal_position_correction);

    if(delta_x >= max_delta_x)
    {
      delta_x = 0;
      direction = -direction;
    }
    return false;
  }
}
