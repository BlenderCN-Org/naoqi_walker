#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <footstep_planner/FootstepPlanner.h>
#include <humanoid_nav_msgs/ClipFootstep.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("naoqi_odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Publisher step_cmd = n.advertise<humanoid_nav_msgs::StepTarget>("footstep",50);
  
  std::vector<humanoid_nav_msgs::StepTarget> footsteps;
  geometry_msgs::Pose2D startP;startP.x=0.5;startP.y=0.3;startP.theta=0;
  geometry_msgs::Pose2D goalP;  goalP.x=2.5;goalP.y=0.3;   goalP.theta=0;
  ros::ServiceClient client=n.serviceClient<humanoid_nav_msgs::PlanFootsteps>("plan_footsteps");
  humanoid_nav_msgs::PlanFootsteps srv;
  srv.request.start=startP;
  srv.request.goal =goalP;
  if (client.call(srv))
  {
     ROS_INFO("result: %d, cost:%f", srv.response.result,srv.response.costs);
     if(srv.response.result==0) ROS_ERROR("plannning fail!");
     footsteps=srv.response.footsteps;
     ROS_INFO("step size: %d", footsteps.size());
  }
  else
  {
     ROS_ERROR("Failed to plan");
     //return 1;
  }
  
  
  int count=0;
  int step_count=0;
  double px;
  
  double x = -0.5;
  double y = -0.3;
  double th = 0.0;

  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();


  
  
  humanoid_nav_msgs::StepTarget current_step;

    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    //x += delta_x;
    //y += delta_y;
    //th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "base_link";
    odom_trans.child_frame_id = "map";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = -0.32;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "base_link";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = -0.32;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "map";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

  ros::Rate r(10);
  while(n.ok()){

    ros::spinOnce();// check for incoming messages
    current_time = ros::Time::now();


  //startP.x+=0.01;
  count++;
  
  if(step_count==0)step_count++;
  if(count==80){count=1;step_count++;}

  //if(step_count==footsteps.size())
  // { goalP.x+=0.1;if(goalP.x==4)goalP.x=0.02;}

/*
  if (client.call(srv))
  {
     ROS_INFO("result: %d, cost:%f", srv.response.result,srv.response.costs);
     if(srv.response.result==0) ROS_ERROR("plannning fail!");
     footsteps=srv.response.footsteps;
     ROS_INFO("step size: %d", footsteps.size());
  }
  else
  {
     ROS_ERROR("Failed to plan");
     //return 1;
  }
   
*/
  if (step_count<=footsteps.size())
  { 
    if(count==1){
       current_step=footsteps[step_count-1];
       ROS_INFO("leg(right=0,left=1): %d, pose(%f,%f,%f)", current_step.leg, current_step.pose.x,current_step.pose.y,current_step.pose.theta);
       step_cmd.publish(current_step);
     }
  }
  else
  {
   step_count=0;
  }
   

 if(step_count>1){
  if(footsteps.size()>0)
  {
    if(count==1){px=footsteps[step_count-2].pose.x;}
    else if (count>5&&count<15) px=px+(current_step.pose.x-footsteps[step_count-2].pose.x)/10;
    else ;
    

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "base_link";
    odom_trans.child_frame_id = "map";

    odom_trans.transform.translation.x = -px;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = -0.32;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "base_link";

    //set the position
    odom.pose.pose.position.x = -px;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = -0.32;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "map";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

   }}

    last_time = current_time;
    r.sleep();
  }
}
