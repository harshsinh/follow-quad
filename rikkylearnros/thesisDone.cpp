
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

    // auto f = boost::bind(&FollowControll::dyn_recon_callback,this, _1, _2);

    FollowControll * followControll = new FollowControll();

    followControll->follow();

    delete followControll;
}

 FollowControll::FollowControll(): set_reference_target_width_height(false), flight_mode(TRACKING), controller_rate(100)
{
    
    _sub_tracked_object = _n.subscribe("/boundingbox", 1000, &FollowControll::tracked_object_cb, this);
    _model_subscriber = _n.subscribe("/gazebo/model_states", 10, &FollowControll::ModelStatecallback, this);

    _pub_cmd_vel = _n.advertise<geometry_msgs::Twist>("cmd_vel",1);
  	_pub_tracking_error = _n.advertise<geometry_msgs::Point>("tracking_error",1);
    ROS_INFO("Publishing from thesis");

    // f = boost::bind(&FollowControll::dyn_recon_callback,this, _1, _2);
    // srv.setCallback(f);


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
      // _tracker_rect_reference.set_point.z = (48*78);
        

      float max_x_error = 2.0/fovx;
      float x_error = _tracker_rect_reference.set_point.x-_tracked_rect.set_point.x;
      
      // if(fabs(x_error)>max_x_error){
          _tracked_rect.set_point.yaw =(x_error>0)?_tracked_rect.set_point.x+max_x_error:_tracked_rect.set_point.x-max_x_error;
        // }
      // else
        //  _tracked_rect.set_point.yaw =0.5;

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

  recon_callback();

  while (ros::ok())
    {
      if((!_tracker_rect_reference.set_point.is_zero() && flight_mode == TRACKING))
        {
        // if( (msg.width> 1 || msg.height > 1) && (msg.confidence*100)>_min_fly_confidance)
        //   {
            // _found_tracked_object = true;
            ROS_INFO("TRACKING IS ON, FLIGHT MODE: %d",flight_mode);
            set_reference_target_width_height = true;
            
            if(!_found_tracked_object)
              ROS_WARN("Object to track NOT found!!");
            
            if(_found_tracked_object)
              {
                _error_pid = _pid->get_pid(_tracker_rect_reference.set_point,_tracked_rect.set_point,time_ms);
                // ROS_INFO("pid vals : %s, %s, %s", _pid->cX.cP, _pid->cX.cI, _pid->cX.cD);
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

  cmd.linear.x = _error_pid.z;
  // cmd.linear.y = _error_pid.x;
  cmd.linear.z = _error_pid.y;

  // if (_error_pid.x == 0){
  //   cmd.angular.z = 0;
  // }
  // else{
    cmd.angular.z = _error_pid.x;//cmd.linear.y>0?cmd.linear.y/10:-cmd.linear.y/10;
  // }
  
  // cmd.angular.x = cmd.angular.y = (flight_mode == TRACKING)?1:0;
  cmd.angular.x = cmd.angular.y = 0;
  _pub_cmd_vel.publish(cmd);
}

void FollowControll::recon_callback()
{
  float kpz=2.0,kpx=2.0,kpy=2.0,kpyaw=0.0;
  bool temp = _n.getParam("kpz", kpz);
  temp = _n.getParam("kpx", kpx);
  temp = _n.getParam("kpy", kpy);
  temp = _n.getParam("kpyaw", kpyaw);
  // double kp=1, ki=1, kd=1, pLim =1, iLim =1, lpf=1; //for all x,y,range,yaw
  Const_PID x{kpx,0,0.0,50,0};
  Const_PID y{kpy,0,0.0,50,0};
  Const_PID range{kpz,0,0,50,0};
  Const_PID yaw{kpyaw,0,0.0,50,0};

  _pid->set_k_pid(x,y,range,yaw,_pid->set_d_lpf_alpha(20,(double)1.0/controller_rate));
  _min_fly_confidance = 10;
}

void FollowControll::ModelStatecallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{

  // double x = (msg->pose[1].position.x);
}

// void FollowControll::dyn_recon_callback(rikkylearnros::pidParamConfig &config, uint32_t level)
// {
//   ROS_INFO("Reconfigure request XY: %f, %f, %f",
//            config.groups.pid_x.kPx,config.groups.pid_x.kDx,config.groups.pid_x.pMaxX);
//   ROS_INFO("Reconfigure request Range: %f, %f, %f",
//            config.groups.pid_range.kPrange,config.groups.pid_range.kDrange,config.groups.pid_range.pMaxRange);
//   ROS_INFO("Reconfigure request Yaw: %f, %f, %f",
//            config.groups.pid_yaw.kPyaw,config.groups.pid_yaw.kDyaw,config.groups.pid_yaw.pMaxYaw);
//   ROS_INFO("LPF: %f",
//            config.LPF);

//   Const_PID x{config.groups.pid_x.kPx,0,config.groups.pid_x.kDx,config.groups.pid_x.pMaxX,0};
//   Const_PID y{config.groups.pid_y.kPy,0,config.groups.pid_y.kDy,config.groups.pid_y.pMaxY,0};
//   Const_PID range{ config.groups.pid_range.kPrange,0,config.groups.pid_range.kDrange,config.groups.pid_range.pMaxRange,0};
//   Const_PID yaw{config.groups.pid_yaw.kPyaw,0,config.groups.pid_yaw.kDyaw,config.groups.pid_yaw.pMaxYaw,0};

//   _pid->set_k_pid(x,y,range,yaw,_pid->set_d_lpf_alpha(config.LPF,(double)1.0/controller_rate));

//   _min_fly_confidance = config.min_confidance;

// }
