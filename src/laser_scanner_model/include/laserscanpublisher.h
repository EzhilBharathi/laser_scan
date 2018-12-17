/**
    Laserscan Publisher
    laserscanpublisher.h

    Purpose: Publishes laser scan messages and adds a dynamic transform between robot body and map frame
    @author Ezhil Bharathi
    @version 1.0 12/16/18 
*/
#ifndef  LASER_SCAN_PUBLISHER_H_
#define  LASER_SCAN_PUBLISHER_H_

//including the ros header
#include <ros/ros.h>
//including the ros packages and messages used in the program
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h> 
#include <nav_msgs/Odometry.h>

//Defining a class LaserScanPublisher and declaring its constructor,member functions and member variables.
class LaserScanPublisher{
        private:

            ros::NodeHandle n_;
            // Dectlaring the member variables            
            unsigned int total_readings;
            double scan_frequency;           
            

            //Declaring the messages laser_scan and vel_cmd
            sensor_msgs::LaserScan laser_scan;
            geometry_msgs::Twist vel_cmd;

            //Declaring the TransformBroadcaster and Transform
            tf::TransformBroadcaster tb;
            tf::Transform tfm;

            //Declaring the publishers
            ros::Publisher laser_scan_pub;
            ros::Publisher vel_pub; 

            
            //Declaring member function initializePublisher 
            void initializePublisher(unsigned int total_readings,double scan_frequency); 
             
            
        public:
            
            //Constructor
            LaserScanPublisher(ros::NodeHandle* nodehandle); 
            double x;
            double y;
            double theta;
            ros::Time prev_time;
            
                       
            //member functions publishMessage
            void publishMessage();  

            //member functions related to dynamic transform broadcasting
            void initializeBroadcaster(ros::Time prev_time);             
            void sendDynamicTransform();   
          
        
};

#endif
