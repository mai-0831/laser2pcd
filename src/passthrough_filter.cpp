#include <ros/ros.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

class PassthroughFilter
{
    private:
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Publisher cloud_publiser;
        ros::Subscriber cloud_sub;

        bool voxel_grid_bool = false;
        double leafsize = 0.1;
        std::string str_points_topic = "/points_no_ground";
        double min_x = 0.0;
        double max_x = 5.5;
        double min_y = -2.4;
        double max_y = 1.5;
        double min_z = 0.0;
        double max_z = 5.5;

    public:
        PassthroughFilter();
        void PointCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};

PassthroughFilter::PassthroughFilter()
    :private_nh("~")
{
    private_nh.getParam("voxel_grid_bool", voxel_grid_bool);
    private_nh.getParam("leafsize", leafsize);   
    private_nh.getParam("str_points_topic", str_points_topic);
    private_nh.getParam("min_x", min_x);
    private_nh.getParam("max_x", max_x);
    private_nh.getParam("min_y", min_y);
    private_nh.getParam("max_y", max_y);
    private_nh.getParam("min_z", min_z);
    private_nh.getParam("max_z", max_z);

    ROS_INFO("[passthrough_filter] Subsrcibe topic: %s", str_points_topic.c_str());

    cloud_sub = nh.subscribe(str_points_topic, 1, &PassthroughFilter::PointCallback, this);
    cloud_publiser = nh.advertise<sensor_msgs::PointCloud2>("points_pass_filter", 1);
}

void PassthroughFilter::PointCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ> input;
    pcl::fromROSMsg(*cloud_msg, input);
    std::cout << "\nRaw data size: " << input.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(input));

    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(min_x, max_x);//points_scanにやるときは-5.5, 0.0
    pass_x.setInputCloud(cloud);
    pass_x.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(min_y, max_y);//points_scanにやるときは-1.5, 2.0
    pass_y.setInputCloud(cloud);
    pass_y.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(min_z, max_z);//points_scanにやるときはなし
    pass_z.setInputCloud(cloud);
    pass_z.filter(*cloud);

    std::cout << "Passthrough filtering data size: " << cloud->size() << std::endl;

    if(voxel_grid_bool == true){
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud (cloud);
        vg.setLeafSize (leafsize, leafsize, leafsize);
        vg.filter (*cloud);
        std::cout << "Voxel grid filering data size: " << cloud->size() << std::endl;
    }

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