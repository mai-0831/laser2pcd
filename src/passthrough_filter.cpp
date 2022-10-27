#include <ros/ros.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

static std::string str_point_topic = "/points_scan";

class PassthroughFilter
{
    private:
        ros::NodeHandle nh;
        ros::Publisher cloud_publiser;
        ros::Subscriber cloud_sub;

    public:
        PassthroughFilter();
        void PointCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};

PassthroughFilter::PassthroughFilter()
{
    cloud_sub = nh.subscribe(str_point_topic, 1, &PassthroughFilter::PointCallback, this);
    cloud_publiser = nh.advertise<sensor_msgs::PointCloud2>("points_filter", 1);
}

void PassthroughFilter::PointCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ> input;
    pcl::fromROSMsg(*cloud_msg, input);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(input));

    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-5.5, 0.0);
    pass_x.setInputCloud(cloud);
    pass_x.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-1.5, 2.0);
    pass_y.setInputCloud(cloud);
    pass_y.filter(*cloud);

    sensor_msgs::PointCloud2 cloud_pub;
    pcl::toROSMsg(*cloud, cloud_pub);

    cloud_publiser.publish(cloud_pub);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "passthrough_filter");
    PassthroughFilter filter;
    ros::spin();
    return 0;
}