#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

class My_Filter {
     public:
        My_Filter();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
     private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
        // tf::TransformBroadcaster br_;
        // tf::Transform transform_;
};

My_Filter::My_Filter(){
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &My_Filter::scanCallback, this);
        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/points_raw", 100, false);
        tfListener_.setExtrapolationLimit(ros::Duration(0.1));
        //tfListener_.sendTransform(transform_, ros::Time::now(), "velodyne", "laser")
}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 points_raw;
    projector_.transformLaserScanToPointCloud("laser", *scan, points_raw, tfListener_);
    point_cloud_publisher_.publish(points_raw);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_filter");
    My_Filter filter;
    ros::spin();
    return 0;
}
