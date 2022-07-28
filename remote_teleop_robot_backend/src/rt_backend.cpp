#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <remote_teleop/TurnInPlaceAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

/*-----------------------------------------------------------------------------------*/
#define THRESHOLD 0.08


class RemoteTeleop {

protected:
  
  // Define node handle
  ros::NodeHandle nh_;
  
  // create the turn in place action server
  actionlib::SimpleActionServer<remote_teleop_robot_backend::TurnInPlaceAction> turn_in_place_as_;
  
  // create the turn in place cmd_vel publisher
  ros::Publisher turn_in_place_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1)
  
  // create the odometry subscriber
  ros::Subscriber odom_sub_ = nh_.subscribe("/odom", 1, odom_callback);
  
  // TODO: I DON'T ACTUALLY KNOW WHAT THIS IS FOR
  std::string action_name_;
  
  // create messages that are used to publish result
  remote_teleop_robot_backend::TurnInPlaceResult turn_in_place_result_;
  
  // other variables
  float angle_;
  bool turn_left_;
  float lin_vel_;
  float ang_vel_;
  tfScalar roll_;
  tfScalar pitch_;
  tfScalar yaw_;
  
public:

  TurnInPlaceAction() :
    turn_in_place_as_(nh_, name, boost:bind(&TurnInPlaceAction::turn_in_place_callback, this, _1), false), action_name_(name) {
    
    turn_in_place_as_.start();
  }
  
  ~TurnInPlaceAction(void) {}
  
  void turn_in_place_callback(const remote_teleop_robot_backend::TurnInPlaceGoal::ConstPtr &goal) {
    // TODO: implement functionality to gray out the rviz plugin buttons
    // while a turn is being executed so no more than 1 command can be
    // sent at a time
    
    // get the inputs from Rviz and store them in variables
    angle_ = goal.degrees;
    turn_left_ = goal.turn_left;
    
    //TODO: get the lin/angular velocity updates here
    // TODO: might put this somewhere else
    lin_vel_ = 0.0;
    ang_vel_ = 0.5;
    
    // convert from degrees to radians
    angle_ = angle_ * M_PI / 180;
    
    turn_in_place();
    
    turn_in_place_result_.success = true;
    turn_in_place_as_.setSucceeded(turn_in_place_result_);
  }
  
  /* Function: odom_callback
     Purpose:  extract the incoming odometry data and update the
               values accordingly
  */
  void odom_callback(const geometry_msgs::Odometry::ConstPtr &msg) {
    // extract the yaw, pitch, roll values and store them
    q = tf.transformations.euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    tf::Matrix3x3 mat(q);
    mat.getEulerYPR(&yaw_, &pitch_, &roll_);
  }
  
  
  /*Function: turn_in_place
    Purpose:  send cmd_vel messages to the drivers to rotate the
              robot to the desired angle
  */
  void turn_in_place() {
    geometry_msgs::Twist command;
    
    if(turn_left_ == false) {
      // TURNING RIGHT
      float goal_yaw = yaw_ - angle_;
      if(goal_yaw < -M_PI) {
        goal_yaw += 2*M_PI;
      }
    } else {
      // TURNING LEFT
      float goal_yaw = yaw_ + angle_;
      if(goal_yaw > M_PI) {
        goal_yaw -= 2*M_PI;
      }
    }
    
    while(abs(goal_yaw - yaw_) > THRESHOLD) {
      command.angular.z = ang_vel_ * (goal_yaw - yaw_);
      
      if(turn_left_ == true && command.angular.z < 0.0) {
        command.angular.z *= -1;
      } else if(turn_left_ == false && command.angular.z > 0.0) {
        command.angular.z *= -1;
      }
      turn_in_place_pub_.publish(command)
    }
    
    command.angular.z = 0.0;
    turn_in_place_pub_.publish(command);
  }
  
};

/*-----------------------------------------------------------------------------------*/

int main(int argc, char** argv) {
  ros::init(argc, argv, "remote_teleop");
  
  ros::NodeHandle n;
  
  // TODO: NOT SURE WHAT THIS DOES
  TurnInPlaceAction turn_in_place_action();
  
  ros::spin();
  
  return 0;
}
