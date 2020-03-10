#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <iostream>

#include <ros/ros.h>                
#include <boost/asio.hpp>                  
#include <boost/bind.hpp>
#include <math.h>
           
#include <stdlib.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h> 
#include <string>
using namespace std;
using namespace boost::asio;    
io_service iosev;

//boost::asio::serial_port
serial_port sp1(iosev, "/dev/ttyUSB1");
#define _SS_MAX_RX_BUFF 4
class MMW_lidar
{
private:
  ros::NodeHandle nh;
  ros::Subscriber sub_odom;
  ros::Publisher pcl_pub_, pcl_p, pcl_x, pcl_y, pcl_mark;
  std_msgs::Header point_cloud_header_;
  double yaw;
  ros::Time Time;
    
public:
    MMW_lidar()
    {
        sub_odom = nh.subscribe("/gps/odom", 1, &MMW_lidar::odomCallback, this);
        pcl_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/MMW_lidar_points", 10);
        pcl_mark = nh.advertise<sensor_msgs::PointCloud2>("/mark_line", 10);

        sp1.set_option(serial_port::baud_rate(115200));
        sp1.set_option(serial_port::flow_control());
        sp1.set_option(serial_port::parity());
        sp1.set_option(serial_port::stop_bits());
        sp1.set_option(serial_port::character_size(8));
        Time= ros::Time::now();
    }
    void odomCallback(const nav_msgs::Odometry &msg_odom);
    void SplitString(const string& s, const string& c, vector<string>& v);
};

void MMW_lidar::SplitString(const string& s, const string& c, vector<string>& v)
{
    string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
   while(string::npos != pos2)
    {
       v.push_back(s.substr(pos1, pos2-pos1));
       pos1 = pos2 + c.size();
       pos2 = s.find(c, pos1);
   }
   if(pos1 != s.length())
        v.push_back(s.substr(pos1));
}

void MMW_lidar::odomCallback(const nav_msgs::Odometry &msg_odom)
{
    std::string write_buf_data;
    //计算当前车速（单位：m/s）
    float velocity=sqrt(pow(msg_odom.twist.twist.linear.x,2)+pow(msg_odom.twist.twist.linear.y,2));
    //计算偏航角速度（单位：deg/s）
    float yaw_rate=(msg_odom.twist.twist.angular.z)*180/M_PI;
        
          //将车速和偏航角速度扩大10倍后取整并转换为字符串类型
    write_buf_data="#"+std::to_string(round(velocity*10))+","+std::to_string(round(yaw_rate*10)); 

    std_msgs::String mssg;
    std::stringstream sss;
    sss <<write_buf_data;
    mssg.data = sss.str()+"\r\n";
        
          //boost::asio::buffer(,)
    write(sp1, buffer(mssg.data.c_str(),mssg.data.size()));

    boost::asio::streambuf buf;
    read_until(sp1, buf, '$');
    int buf_size=buf.size();
    vector<string> vec;
    boost::asio::streambuf::const_buffers_type cbt=buf.data();
    std::string buf_data(boost::asio::buffers_begin(cbt),boost::asio::buffers_end(cbt));
//    ROS_INFO_STREAM(buf_data);
    SplitString(buf_data, ";", vec); 
    vector<string> vec1;
    for(vector<string>::size_type i = 0; i != (vec.size()-1); ++i)
    {
        SplitString(vec[i], ",", vec1); 
    }
    std::vector<double> distance;
    std::vector<double> angle;
    for (int i=0; i<vec1.size(); ++i)
    {
      istringstream iss(vec1[i]);
      float num;
      iss >> num;
      if(i%2==0)
          distance.push_back(num);
      else
          angle.push_back(num);
    }
//    ROS_INFO("1=%d",angle.size());
    pcl::PointCloud<pcl::PointXYZI>::Ptr MMW_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI  point;
    for (int i=0; i<distance.size(); i++)
    {
       //if((distance[i]>0)&&(distance[i]<30))
      // {
          float distance_temp=((float)distance[i]/10);
          float angle_temp=((float)-1*angle[i]/10)*M_PI/180;
          point.x=distance_temp*cos(angle_temp);
          point.y=distance_temp*sin(angle_temp);
          MMW_points->points.push_back(point);
     //  }
    }
    sensor_msgs::PointCloud2 MMW_points_temp;
    pcl::toROSMsg(*MMW_points, MMW_points_temp);
    MMW_points_temp.header =point_cloud_header_ ;
    MMW_points_temp.header.frame_id = "/velodyne";
    pcl_pub_.publish(MMW_points_temp);

    std::vector<double> vec_radius;
    pcl::PointXYZI  mark_line_points;
    pcl::PointCloud<pcl::PointXYZI>::Ptr mark_line(new pcl::PointCloud<pcl::PointXYZI>);
    vec_radius={0.5, 1.0, 1.5, 2.0, 2.5, 3.0};
    int range=M_PI*100;
    for (int j=0; j<vec_radius.size(); j++)
    {
        for(int i=0; i<range; i++)
        {
           float theta=(-1*M_PI)/2+0.01*i;
           mark_line_points.x=vec_radius[j]*cos(theta);
           mark_line_points.y=vec_radius[j]*sin(theta);
           mark_line->points.push_back(mark_line_points);
        }
    }
    sensor_msgs::PointCloud2 mark_line_temp;
    pcl::toROSMsg(*mark_line, mark_line_temp);
    mark_line_temp.header =point_cloud_header_ ;
    mark_line_temp.header.frame_id = "/velodyne";
    pcl_mark.publish(mark_line_temp);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MMW_lidar");
  MMW_lidar start_detec;
    
        ros::spin();
  return 0;
}
