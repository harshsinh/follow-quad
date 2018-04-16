#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "tld_msgs/BoundingBox.h"
    
int main(int argc, char **argv) 
{ 

      ros::init(argc, argv, "my_pub");
      ros::NodeHandle nh;

    //  image_transport::ImageTransport it(nh);

    //  //publishing the images
    //  image_transport::Publisher pub1 = it.advertise("/camera/rgb/image_raw", 1);
    //  cv::Mat image1 = cv::imread("/home/adminr/Pictures/ww.jpg", CV_LOAD_IMAGE_COLOR);

    //  image_transport::Publisher pub2 = it.advertise("/stereo/right/image_raw", 1);
    //  cv::Mat image2 = cv::imread("/home/rtz/Desktop/images/000000_10b.png", CV_LOAD_IMAGE_COLOR);

    // A publisher for the movement data 
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10); 

    ros::Publisher pub2 = nh.advertise<tld_msgs::BoundingBox>("tld_tracked_object", 10);

    // Drive forward at a given speed.  The robot points up the x-axis. 
    // The default constructor will set all commands to 0 
    geometry_msgs::Twist msg;
    msg.linear.x = 0.5;
    msg.linear.y = 0.2;
    msg.linear.z = 5;

    tld_msgs::BoundingBox detection;
    detection.x = 2;
    detection.y = 2;
    detection.width = 240;
    detection.height = 320;
    detection.confidence = 55.0;
    ROS_INFO_STREAM(detection);
    //  cv::waitKey(30);

     //changing the images to sensor messages
    //  sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image1).toImageMsg();
    //  sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image2).toImageMsg();

     //cv::imshow( "Display window", image);                   // Show our image inside it.
     ROS_INFO_STREAM("Working I guess");
     ros::Rate rate(10);

     while (ros::ok()) {
      //sending the messages to publishers
       pub.publish(msg);
       pub2.publish(detection);
      //  ros::spinOnce();
       rate.sleep();
     }
}

