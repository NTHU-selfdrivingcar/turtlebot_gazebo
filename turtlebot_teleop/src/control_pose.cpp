#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include "turtlesim/Pose.h"
#include "stdio.h"
#include "math.h"
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/LinearMath/Transform.h"


class CAR{

    ros::NodeHandle nh;
    geometry_msgs::Twist car_command;
    ros::Subscriber pose;
    ros::Subscriber nav_goal;
    ros::Publisher publish;
    public :
        double x;
        double y;
        double deg;
        double x_error;
        double y_error;
        double deg_error;
        double target_x;
        double target_y;
        double target_deg;
        double error_distance;
        double deg_from_point_to_target;
        bool arrive_pose;
        bool arrive_deg;
        bool arrival;
        double alpha;
        double beta;
        double k_alpha;
        double k_beta;
        double k_rho;
        double distance_margin;
        double degree_margin;
        short orientation;
        int speed_constant;

        void set_degree_to_defined_domain(double * degree_to_set){
            if(*degree_to_set > 3.1415926)
                *degree_to_set -= 2 * 3.1415926;
            else if(*degree_to_set < -3.1415926)
                *degree_to_set += 2 * 3.1415926;
        }

        CAR(){
            publish = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);  
            pose = nh.subscribe("robot_pose", 1000, &CAR::look_pose,this);
            nav_goal = nh.subscribe("move_base_simple/goal", 1000, &CAR::set_goal,this);
            x = 0;
            y = 0;
            deg = 0;
            x_error = 0;
            y_error = 0;
            deg_error = 0;
            target_x = 0;
            target_y = 0;
            speed_constant = 5;
            k_alpha = 0.08 * speed_constant;
            k_beta = -0.015 * speed_constant;
            k_rho = 0.03 * speed_constant;
            target_deg = 0;
            error_distance = 0;
            arrive_pose = 0;
            arrive_deg = 0;
            distance_margin = 0.2;
            degree_margin = 0.1;
            arrival = 0;
        }
        void setTarget(){
            nh.getParam("target_x",target_x);
            nh.getParam("target_y",target_y);
            nh.getParam("target_deg",target_deg);
        }

        void check_Error(){
            x_error = target_x - x;
            y_error = target_y - y;
            error_distance = sqrt(pow(x_error,2) + pow(y_error,2));

            deg_from_point_to_target = atan2(y_error , x_error);
            alpha = deg_from_point_to_target - deg;
            set_degree_to_defined_domain(&alpha);
            beta = target_deg - deg; 
            set_degree_to_defined_domain(&beta);
            if(fabs(error_distance) < distance_margin)
                arrive_pose = 1;
            else
                arrive_pose = 0;
            if(fabs(beta) < degree_margin )
                arrive_deg = 1;
            else
                arrive_deg = 0;
            if(arrive_pose && arrive_deg)
                arrival = 1;
            else    arrival = 0;
            //printf("dis = %f\n",error_distance);
            //printf("alpha = %f\n",alpha);
            //printf("beta = %f\n",beta);
            //printf("arrive_dis = %d\n",arrive_pose);
            //printf("arrive_deg = %d\n",arrive_deg);
        }

        void set_car_cmd(){
            if(!arrival){
                car_command.linear.x = k_rho * error_distance;
                car_command.linear.y = 0;
                car_command.linear.z = 0;
                car_command.angular.x = 0;
                car_command.angular.y = 0;
                car_command.angular.z = k_alpha * alpha + k_beta * beta;
            }
            else{
                car_command.linear.x = 0;
                car_command.linear.y = 0;
                car_command.linear.z = 0;
                car_command.angular.x = 0;
                car_command.angular.y = 0;
                car_command.angular.z = 0;
            
            }
            publish.publish(car_command);
        }

        void look_pose(const geometry_msgs::Twist::ConstPtr& pose){
            x = pose->linear.x;
            y = pose->linear.y;
            deg = pose->angular.z;
            set_degree_to_defined_domain(&deg); 
            //setTarget();
            check_Error();
            set_car_cmd();
        }
        float quaternion_to_theta(float x, float y, float z,float w){
              tf2::Quaternion q(x,y,z,w);
              tf2::Matrix3x3 m(q);
              double roll,pitch,yaw;
              m.getRPY(roll,pitch,yaw);
	      set_degree_to_defined_domain(&yaw);
              return yaw;
        }
        void set_goal(const geometry_msgs::PoseStamped::ConstPtr& goal){
            target_x = goal->pose.position.x;
            target_y = goal->pose.position.y;
            printf("x = %f\n",goal->pose.position.x);
            printf("y = %f\n",goal->pose.position.y);
            printf("theta = %f\n",quaternion_to_theta(goal->pose.orientation.x,goal->pose.orientation.y,goal->pose.orientation.z,goal->pose.orientation.w));
            target_deg = quaternion_to_theta(goal->pose.orientation.x,goal->pose.orientation.y,goal->pose.orientation.z,goal->pose.orientation.w);
        }
};          
int main(int argc, char ** argv){
	ros::init(argc,argv,"pose_control");
    CAR car;
    ros::spin();
    return 0;
}                  
                   
