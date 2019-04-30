#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <footstep_planner/FootstepPlanner.h>
#include <humanoid_nav_msgs/ClipFootstep.h>
#include <iostream>


std::vector<humanoid_nav_msgs::StepTarget> footsteps;
geometry_msgs::PoseWithCovarianceStamped  start_pose;
nav_msgs::Odometry odom_data;
geometry_msgs::PoseStamped goal_pose;
int startflag=0;int goalflag=0; bool initflag=0;
void footstepsCallback(const visualization_msgs::MarkerArray& msg)
{
   ROS_INFO("Start to callback footsteps!");
   footsteps.clear();
   ROS_INFO("x=%f",msg.markers.begin()->pose.position.x);
   for(int i=0;i<msg.markers.size();i++)
   {       
           ROS_INFO("count: %d",i);
	   humanoid_nav_msgs::StepTarget msgR;
	   msgR.pose.x=msg.markers[i].pose.position.x;
	   msgR.pose.y=msg.markers[i].pose.position.y;
	   msgR.pose.theta=tf::getYaw(msg.markers[i].pose.orientation);
	   if (msg.markers[i].color.g==1.0)
		  msgR.leg=0; //right=0.left=1

	   else
		 msgR.leg=1;
	   footsteps.push_back(msgR);
	   ROS_INFO("Success to callback footsteps!");
	}  
}
void startCallback(geometry_msgs::PoseWithCovarianceStamped msg)
{
	ROS_INFO("get the Start pose!");
    	start_pose=msg;
	startflag=1;
	
}

void goalCallback(geometry_msgs::PoseStamped msg)
{
	ROS_INFO("get the goal pose!");
    	goal_pose=msg;
	goalflag=1;
	
}

void odomCallback(nav_msgs::Odometry msg)
{
	//ROS_INFO("get the odom data!");
    	odom_data=msg;
	//goalflag=1;
	
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_publisher");

	ros::NodeHandle n;
	ros::Publisher step_cmd = n.advertise<humanoid_nav_msgs::StepTarget>("footstep",50);
	//ros::Subscriber sub = n.subscribe("footstep_planner/footsteps_array", 1000, footstepsCallback);
	ros::Subscriber odom_sub = n.subscribe("odom",1000,odomCallback);

	ros::Subscriber start_sub = n.subscribe("initialpose",1000,startCallback);

	ros::Subscriber goal_sub = n.subscribe("goal",1000,goalCallback);

	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom_1", 50);
	tf::TransformBroadcaster odom_broadcaster;
	
	  geometry_msgs::Pose2D startP; startP.x=0.5;startP.y=0.3;startP.theta=0;
	  geometry_msgs::Pose2D goalP;  goalP.x=2.5;goalP.y=0.3;   goalP.theta=0;


	  ros::ServiceClient client=n.serviceClient<humanoid_nav_msgs::PlanFootsteps>("plan_footsteps");
  	double vx = 0.1;
	double vy = -0.1;
	double vth = 0.1;
 
	humanoid_nav_msgs::StepTarget current_step;

	ROS_INFO("nao_odom initialized!");
	ros::Rate r(1);
       ros::Time current_time;
	int step_count=0;
	while(n.ok())
	{
		ros::spinOnce();// check for incoming messages
		current_time=ros::Time::now();
		  if (footsteps.size() && step_count<footsteps.size())
		  { 
			step_count+=1;
		       current_step=footsteps[step_count-1];
		       ROS_INFO("leg(right=0,left=1): %d, pose(%f,%f,%f)", current_step.leg, current_step.pose.x,current_step.pose.y,current_step.pose.theta);
		       //step_cmd.publish(current_step);
			//if(step_count==footsteps.size()) step_count=0;

		  }
		
		if (startflag==1 && goalflag==1)
		{
                   ROS_INFO("start(%f,%f); goal(%f,%f)",start_pose.pose.pose.position.x,start_pose.pose.pose.position.y,goal_pose.pose.position.x,goal_pose.pose.position.y);
                   step_count=0;
		   //since all odometry is 6DOF we'll need a quaternion created from yaw
		    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);

		    //first, we'll publish the transform over tf
		    geometry_msgs::TransformStamped odom_trans;
		    odom_trans.header.stamp = current_time;
		    odom_trans.header.frame_id = "base_footprint";
		    odom_trans.child_frame_id = "map";

		    odom_trans.transform.translation.x = -start_pose.pose.pose.position.x;//x;
		    odom_trans.transform.translation.y = -start_pose.pose.pose.position.y;//y;
		    odom_trans.transform.translation.z = -0.32;
		    odom_trans.transform.rotation = odom_quat;

		    //send the transform
		    odom_broadcaster.sendTransform(odom_trans);

		    //next, we'll publish the odometry message over ROS
		    nav_msgs::Odometry odom;
		    odom.header.stamp = current_time;
		    odom.header.frame_id = "base_footprint";

		    //set the position
		    odom.pose.pose.position.x = -start_pose.pose.pose.position.x;
		    odom.pose.pose.position.y = -start_pose.pose.pose.position.y;
		    odom.pose.pose.position.z = -0.32;
		    odom.pose.pose.orientation = odom_quat;

		    //set the velocity
		    odom.child_frame_id = "map";
		    odom.twist.twist.linear.x = vx;
		    odom.twist.twist.linear.y = vy;
		    odom.twist.twist.angular.z = vth;

		    //publish the message
		    odom_pub.publish(odom);


			startP.x=start_pose.pose.pose.position.x;//0.5;
			startP.y=start_pose.pose.pose.position.y;//0.3;
			startP.theta=tf::getYaw(start_pose.pose.pose.orientation);
			goalP.x=goal_pose.pose.position.x;//2.5;
			goalP.y=goal_pose.pose.position.y;//0.3;   
			goalP.theta=tf::getYaw(goal_pose.pose.orientation);
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
	  
			//startflag=0;goalflag=0; initflag=1;
		}
		/*if (initflag==1)
		{
                   
 			//since all odometry is 6DOF we'll need a quaternion created from yaw
		    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);

		    //first, we'll publish the transform over tf
		    geometry_msgs::TransformStamped odom_trans;
		    odom_trans.header.stamp = current_time;
		    odom_trans.header.frame_id = "base_link";
		    odom_trans.child_frame_id = "map";

		    odom_trans.transform.translation.x = -start_pose.pose.pose.position.x-odom_data.pose.pose.position.x;//x;
		    odom_trans.transform.translation.y = -start_pose.pose.pose.position.y;//y;
		    odom_trans.transform.translation.z = -0.32;
		    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(-tf::getYaw(start_pose.pose.pose.orientation)-tf::getYaw(odom_data.pose.pose.orientation));

		    //send the transform
		    odom_broadcaster.sendTransform(odom_trans);

		    //next, we'll publish the odometry message over ROS
		    nav_msgs::Odometry odom;
		    odom.header.stamp = current_time;
		    odom.header.frame_id = "base_link";


		   odom.pose.pose.position.x = -start_pose.pose.pose.position.x-odom_data.pose.pose.position.x;//x;
		    odom.pose.pose.position.y = -start_pose.pose.pose.position.y;//y;
		    odom.pose.pose.position.z = -0.32;
		    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(-tf::getYaw(start_pose.pose.pose.orientation)-tf::getYaw(odom_data.pose.pose.orientation));

		    //set the velocity
		    odom.child_frame_id = "map";
		    odom.twist.twist.linear.x = vx;
		    odom.twist.twist.linear.y = vy;
		    odom.twist.twist.angular.z = vth;

		    //publish the message
		    odom_pub.publish(odom);

		}*/

		r.sleep();
	}
}

