
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "FollowControll.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "custom_object_follow");
    ROS_INFO("Started my object_follow Node..Try");

    FollowControll * followControll = new FollowControll();

    followControll->follow();

    delete followControll;
}


FollowControll::FollowControll(): set_reference_target_width_height(false), flight_mode(TRACKING), controller_rate(100)
{
    
    _sub_tracked_object = _n.subscribe("tld_tracked_object", 1000, &FollowControll::tracked_object_cb, this);
    _model_subscriber = _n.subscribe("/gazebo/model_states", 10, &FollowControll::ModelStatecallback, this);

    _pub_cmd_vel = _n.advertise<geometry_msgs::Twist>("cmd_vel",1);
  	_pub_tracking_error = _n.advertise<geometry_msgs::Point>("tracking_error",1);
    ROS_INFO("Publishing from thesis");

    _pid = new Controller();
    fovx = 70;
    fovy = 38;
    controller_rate = 100;
  	_tracker_rect_reference.set_point.x = _tracker_rect_reference.set_point.y = _tracker_rect_reference.set_point.yaw = 1/2.0;
    
}


void FollowControll::tracked_object_cb(const tld_msgs::BoundingBox &msg)
{
  if( (msg.width> 1 || msg.height > 1) && (msg.confidence*100)>_min_fly_confidance)
    {
      _found_tracked_object = true;

      _tracked_rect.set(msg.x,msg.y,msg.width,msg.height,0);
      _tracked_rect.apply_pitch_degrees(navdata_telemetry.rotY,fovy);

      // _tracker_rect_reference.set(FRONTCAM_RESOLUTION_WIDTH/2-msg.width/2,FRONTCAM_RESOLUTION_HEIGHT/2-msg.height/2,msg.width,msg.height,1/2.0);
      _tracker_rect_reference.set(FRONTCAM_RESOLUTION_WIDTH/2,FRONTCAM_RESOLUTION_HEIGHT/2,48,78,1/2.0);
      _tracker_rect_reference.set_point.z = (48*78);
        

      float max_x_error = 2.0/fovx;
      float x_error = _tracker_rect_reference.set_point.x-_tracked_rect.set_point.x;
      if(fabs(x_error)>max_x_error){
          _tracked_rect.set_point.yaw =(x_error>0)?_tracked_rect.set_point.x+max_x_error:_tracked_rect.set_point.x-max_x_error;
        }
      else
         _tracked_rect.set_point.yaw =0.5;

      geometry_msgs::Point tracking_error;
      tracking_error.x = _tracked_rect.set_point.x - _tracker_rect_reference.set_point.x;
      tracking_error.y = _tracked_rect.set_point.y - _tracker_rect_reference.set_point.y;
      tracking_error.z = _tracked_rect.set_point.z - _tracker_rect_reference.set_point.z;
      _pub_tracking_error.publish(tracking_error);

      // =============================================================================================


    }
  else
    {
      _found_tracked_object = false;
    }

}

void FollowControll::follow()
{
  ros::Rate loop_rate(controller_rate);
  double time_ms = ((double)(1.0/((double)controller_rate)))*1000.0;

  while (ros::ok())
    {
      if((!_tracker_rect_reference.set_point.is_zero() && flight_mode == TRACKING))
        {
        // if( (msg.width> 1 || msg.height > 1) && (msg.confidence*100)>_min_fly_confidance)
        //   {
            // _found_tracked_object = true;
            ROS_INFO("TRACKING IS ON BABY, FLIGHT MODE: %d",flight_mode);
            set_reference_target_width_height = true;
            if(_found_tracked_object)
              {
                _error_pid = _pid->get_pid(_tracker_rect_reference.set_point,_tracked_rect.set_point,time_ms);
                publish_command();

              }else{
                // _error_pid = 0;
                // _pid->reset_I();
                // publish_command();
                geometry_msgs::Twist cmd;
                cmd.linear.x = 0.0;
                cmd.linear.y = 0.0;
                cmd.linear.z = 0.0;
                cmd.angular.x = 0.0;
                cmd.angular.y = 0.0;
                cmd.angular.z = 0.0;
                _pub_cmd_vel.publish(cmd);
              }
          
          }
      else
        {
          ROS_INFO("TRACKING IS OFF, FLIGHT MODE: %d",flight_mode);
          geometry_msgs::Twist cmd;
          cmd.linear.x = 0.0;
          cmd.linear.y = 0.0;
          cmd.linear.z = 0.0;
          cmd.angular.x = 0.0;
          cmd.angular.y = 0.0;
          cmd.angular.z = 0.0;
          _pub_cmd_vel.publish(cmd);
          cv::waitKey(1000);
        }

      ros::spinOnce();
      loop_rate.sleep();
    }
}

void FollowControll::publish_command()
{
  // use gazebo model states for actual positions
  geometry_msgs::Twist cmd;

  cmd.linear.x = -_error_pid.z;
  // cmd.linear.z = _error_pid.y;

  if (_error_pid.x == 0){
    cmd.angular.z = 0;
  }
  else{
    cmd.angular.z = _error_pid.x;//cmd.linear.y>0?cmd.linear.y/10:-cmd.linear.y/10;
  }
  
  // cmd.angular.x = cmd.angular.y = (flight_mode == TRACKING)?1:0;
  _pub_cmd_vel.publish(cmd);
}

void FollowControll::recon_callback()
{
  // double kp=1, ki=1, kd=1, pLim =1, iLim =1, lpf=1; //for all x,y,range,yaw
  Const_PID x{1.3,0,0.05,50,0};
  Const_PID y{0.5,0,0.0,50,0};
  Const_PID range{0.05,0,0,50,0};
  Const_PID yaw{0.0,0,0.0,50,0};

  _pid->set_k_pid(x,y,range,yaw,_pid->set_d_lpf_alpha(20,(double)1.0/controller_rate));
  _min_fly_confidance = 10;
}

void FollowControll::ModelStatecallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{

  // double x = (msg->pose[1].position.x);
}