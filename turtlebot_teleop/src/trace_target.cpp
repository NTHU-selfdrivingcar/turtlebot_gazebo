#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include "stdio.h"
#include "math.h"

#define d2r 0.01745f
#define r2d 57.2958f

#define dbias 0.00f

double degree;
double x;
double y;

double targetX=2;
double targetY=2;
double targetd=90;
double targetq[4] = {0,0,0,0};

double v_cmd_x;
double v_cmd_y;
double w_cmd;
double target_alpha;
double target_beta;
bool flag=true;

double d,alpha,beta;
//0.01, 0.05, 0.005
double k[3] = {0.1, 0.3 ,0.2}; //kv kalpha kbeta

void loop_goal(const geometry_msgs::PoseStamped goalpose){
    targetX = goalpose.pose.position.x;
	targetY = goalpose.pose.position.y;
	targetq[0] = goalpose.pose.orientation.x;
	targetq[1] = goalpose.pose.orientation.y;
	targetq[2] = goalpose.pose.orientation.z;
	targetq[3] = goalpose.pose.orientation.w;

	targetd = atan2(2*(targetq[3]*targetq[2]+targetq[0]*targetq[1]),1-2*(targetq[2]*targetq[2]+targetq[1]*targetq[1]));

	targetd = targetd*r2d;
	ROS_INFO("tarx = %f, tary = %f, tardeg = %lf\n",targetX,targetY,targetd);
	flag=true;
}
void loop_pose(const geometry_msgs::Twist pose){
    x = pose.linear.x;
	y = pose.linear.y;
    degree = pose.angular.z * r2d;
    while(degree>360) degree -= 360;
    while(degree<0) degree += 360;
	if (degree > 180)
		degree -=360;
    //printf("theta = %f\n",pose->theta);
	ROS_INFO("x = %f, y = %f, deg = %lf\n",x,y,degree);
}



int main(int argc, char ** argv){
	ros::init(argc,argv,"turtle_publish");
    ros::NodeHandle n;
    ros::Publisher publish = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    ros::Subscriber pose = n.subscribe("robot_pose", 1000, loop_pose);
    ros::Subscriber target = n.subscribe("move_base_simple/goal", 1000, loop_goal);
    int rate = 10;
    ros::Rate loop_rate(rate);

    geometry_msgs::Twist msg;
    while(ros::ok()){
     	d = sqrt(pow(targetX-x,2)+pow(targetY-y,2));
		target_alpha = atan2(targetY-y,targetX-x)*r2d;
		target_beta = targetd;
		
		
		double degree_360 = degree;
		if(target_alpha<0) target_alpha+=360;
		if(target_beta<0) target_beta+=360;
		if(degree<0)degree_360 +=360;

		alpha = target_alpha-degree_360;
		beta = target_beta - degree_360;

		if(alpha<-180)alpha=360+alpha;
		if(alpha>180)alpha=alpha-360;

		if(beta<-180)beta=360+beta;
		if(beta>180)beta=beta-360;
		
		v_cmd_x = k[0]*d;
		v_cmd_y = 0;
		if (d < dbias||flag==false)
		{
			flag = false;
			v_cmd_x = k[0]*d*0.0;
			w_cmd = k[2]*beta*0.0;//if reach position increase k beta
		}
		else
			w_cmd = k[1]*alpha + k[2]*beta;
        
		ROS_INFO("cmd v = %lf, w = %lf, tx = %lf, ty = %lf, td = %lf\n",v_cmd_x,w_cmd,targetX,targetY,targetd);

		msg.linear.x = v_cmd_x;
		msg.linear.y = v_cmd_y;
		msg.angular.z = w_cmd*d2r;
		loop_rate.sleep();
        publish.publish(msg);
		ros::spinOnce();
    }              
                   
    return 0;
}                  
                   
