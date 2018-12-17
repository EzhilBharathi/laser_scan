/**
    Laserscan Publisher
    laser_pub.cpp

    Purpose: Publishes laser scan messages and adds a dynamic transform between robot body and map frame
    @author Ezhil Bharathi
    @version 1.0 12/16/18 
*/
#include "laserscanpublisher.h"
   
   //Function initializing the messages laserscan and vel_cmd for it to be published
void  LaserScanPublisher::initializePublisher(unsigned int total_readings,double scan_frequency)
{  
    //initializing laser_scan_pub publisher
    laser_scan_pub=n_.advertise<sensor_msgs::LaserScan>("/scan",100);

    //constructing the laser_scan message
    ros::Time time_now=ros::Time::now();

    //initializing header of laser_scan message
    laser_scan.header.stamp=time_now;
    laser_scan.header.frame_id="laser";

    //initializing angle components of laser_scan message
    laser_scan.angle_min=-3.14;
    laser_scan.angle_max=3.14;
    laser_scan.angle_increment=6.28/total_readings;

    //initializing time component of laser_scan message
    laser_scan.time_increment=(1/scan_frequency)/total_readings;
    laser_scan.scan_time=0;
    
    //initializing range and intensity components of laser_scan message
    laser_scan.range_min=0.1;
    laser_scan.range_max=100.0;
    laser_scan.ranges.resize(total_readings);
    laser_scan.intensities.resize(total_readings);
    for(unsigned int i=0;i<total_readings;i++)
    {
        laser_scan.ranges[i]=2;
        laser_scan.intensities[i]=6;
        
    }

    //initializing vel_pub publisher
    vel_pub=n_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",10);

    //constructing the vel_cmd message
    vel_cmd.linear.x=0.3;
    vel_cmd.angular.z=0.3;
    
    

}
 //Function initializing the dynamic transform to be broadcasted
void LaserScanPublisher::initializeBroadcaster(ros::Time prev_time)
{   
    ros::Time current_time=ros::Time::now();
    double dt = (current_time-prev_time).toSec();
    double linear_v=vel_cmd.linear.x;
    double angular_v=vel_cmd.angular.z;
    double x_dt=(linear_v *cos(theta) - linear_v *sin(theta))*dt;
    double y_dt=(linear_v*sin(theta)+linear_v*cos(theta))*dt;
    double theta_dt=angular_v*dt;
    x +=x_dt;
    y +=y_dt;
    theta +=theta_dt;

    //geometry_msgs::Quaternion theta_quat = tf::createQuaternionMsgFromYaw(theta);
    tfm.setOrigin(tf::Vector3(x,y,0));
    tfm.setRotation(tf::Quaternion(theta,0,0));
        
}

//Constructor for LaserScanPublisher class
LaserScanPublisher::LaserScanPublisher(ros::NodeHandle* nodehandle):n_(*nodehandle)
{   
    total_readings=100;
    scan_frequency=2.0;
    x=0;
    y=0;
    theta=0;
    initializePublisher(total_readings,scan_frequency);
    
}

//Funtion publishing the laser_scan and vel_cmd messages using the publishers laser_scan_pub and vel_pub
void LaserScanPublisher::publishMessage()
{
    
    laser_scan_pub.publish(laser_scan);
    vel_pub.publish(vel_cmd);
    
}

//Funtion broadcasting the transform between base_link and map frames
void LaserScanPublisher::sendDynamicTransform()
{
    
    tb.sendTransform(tf::StampedTransform(tfm,ros::Time::now(),"base_link","map"));

}

int main(int argc,char** argv)
{//ros-initial settings
    ros::init(argc,argv,"laser_scan_publisher");
    ros::NodeHandle nh;
    //Instantiating an object to LaserScanPublisher class and passing a pointer to nodehandle for its constructor
    LaserScanPublisher laserpub(&nh);
    while(ros::ok())
    {//untill ros is live/ok dynamic transform between base_link and map is broadcasted while publishing laserscans
        laserpub.prev_time=ros::Time::now();
        laserpub.publishMessage();
        laserpub.initializeBroadcaster(laserpub.prev_time);
        laserpub.sendDynamicTransform();
    }
    return 0;
}
