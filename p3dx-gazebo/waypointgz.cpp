//Program to make Pioneer P3-DX mobile robot navigate desired waypoints with a proportional controller

#include<ros/ros.h>
#include<geometry_msgs/Twist.h>	// For geometry_msgs::Twist
#include<nav_msgs/Odometry.h>	// For nav_msgs::Odometry
#include<tf/tf.h>	// For quaternion to yaw conversion
#include<iomanip>	// For std::setprecision and std::fix
#include<math.h>	// For sqrt() and pow() functions
#include<ros/time.h>	// For calculating the time
#include<ros/duration.h>	// For calculating the time difference
#include<iostream>	// For I/O operations	

const float kp_l = 0.25;	// Controller gain for linear velocity
const float kp_w = 0.5;	// Controller gain for angular velocity 

#define PI 3.14159

float targetx, targety, nextx, nexty, currentx, currenty, currentyaw;
float m, anglem, xdiff, ydiff, linvel, angvel, del_theta, del_lin;
double roll, pitch, yaw;

int n, i;
float waypt[100][2];

ros::Publisher pub;	//	Declare publisher object globally
ros::Subscriber sub; //	Declare subscriber object globally

geometry_msgs::Twist msgv;
nav_msgs::Odometry msgp;

void publishVelocity()
{
	xdiff = nextx - currentx;
	ydiff = nexty - currenty;		

	if ( (xdiff == 0) && (ydiff == 0))	// atan2 is undefined
		anglem = 0;
	else
		anglem = atan2 (ydiff, xdiff);	// Desired heading
	
	// Display received message
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "RECEIVED	pos = (" << currentx << "," << currenty 
		<< " ang = " << currentyaw <<")\n");

	del_lin = fabs( sqrt( xdiff * xdiff + ydiff * ydiff) );	// Error in Euclidean distance
	del_theta = anglem - currentyaw ;	// Error in heading
		
	if (del_theta > PI)
		del_theta = del_theta - 2*PI;
	else if (del_theta < -PI)
		del_theta = del_theta + 2*PI;

	// Proportional controller for robot navigation
	linvel = kp_l * del_lin * fabs( cos (del_theta) );
	angvel = kp_w * del_theta ;
	
	if ( i == n - 1)
	{
		linvel = kp_l * del_lin * fabs( cos (del_theta) );	
		if ( linvel >= 0.5)
			linvel = 0.5;
	}
	else
	{
		linvel = 0.1 * fabs (cos(del_theta) );
	}
		
	
	if ( del_lin < 0.01)
	{
		std::cout << "\n\n\n ***************** Waypoint "<< i+1 <<" reached. **************** \n\n\n";
		i++;
		nextx = waypt[i][0];
		nexty = waypt[i][1];

		if ( i == n)
		{
			linvel = 0;
			angvel = 0;
		}
	}
		
	msgv.linear.x = linvel;
	msgv.angular.z = angvel;
	
	// Publish the message
	pub.publish (msgv);

	// Send a message to rosout with the details
	ROS_INFO_STREAM ("SENT:	" << "linear = " << msgv.linear.x << "; " << "angular=" << msgv.angular.z );
}

void poseMessageReceived ( const nav_msgs::Odometry &msgp)
{
	currentx = msgp.pose.pose.position.x;
	currenty = msgp.pose.pose.position.y;	
	
	// Convert quaternion to yaw Euler angle
	tf::Quaternion q( msgp.pose.pose.orientation.x, msgp.pose.pose.orientation.y, msgp.pose.pose.orientation.z, msgp.pose.pose.orientation.w );
	tf::Matrix3x3 mat(q);
	mat.getRPY (roll, pitch, yaw);
	currentyaw = float( yaw);
	
	// Call publisher function
	publishVelocity();
}

int main(int argc, char **argv)
{	
	//Initialize the ROS system	
	ros::init(argc, argv, "Waypoint");

	// Become a node.
	ros::NodeHandle nh;

	std::cout << "Enter no. of waypoints: ";
	std::cin >> n;
	for (i = 0; i < n; i ++ )
	{
		std::cout << "Enter waypoint " << i+1 << " co-ordinates: ";
		std::cin >> waypt[i][0] >> waypt[i][1];
	}
	
	// Initialize the publisher object.	 
	pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	// Initialize the subscriber object.
	sub = nh.subscribe("odom", 1000, &poseMessageReceived);
	
	// First waypoint target initialization
	i = 0;	
	nextx = waypt[i][0];
	nexty = waypt[i][1];
	targetx = waypt[i][0];
	targety = waypt[i][1];
	
	// Waypoint navigation loop
	while ((i < n) && ros::ok())
		ros::spinOnce();

	std::cout << "\n\nNavigation complete.\n";
}
