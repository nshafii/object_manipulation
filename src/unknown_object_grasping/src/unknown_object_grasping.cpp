//ROS includes
#include <ros/ros.h>

//tf includes
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

//PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

//Arm driver includes
#include "INESC_Robotis_Driver/SetToolPose.h"
#include "INESC_Robotis_Driver/HomeArm.h"

void CalcAverageDistToBase(const pcl::PointCloud<pcl::PointXYZ>& cloud,
		Eigen::Vector3d& average) {
	double average_x = 0;
	double average_y = 0;
	double average_z = 0;
	for (unsigned i = 0; i < cloud.points.size(); i++) {
		// Some points may have NaN values for position.
		pcl::PointXYZ point = cloud.points.at(i);

		if (isnan(point.x) || isnan(point.y) || isnan(point.z)) {
			continue;
		}
		average_x += point.x;
		average_y += point.y;
		average_z += point.z;
	}
	average.x() = average_x / cloud.points.size();
	average.y() = average_y / cloud.points.size();
	average.z() = average_z / cloud.points.size();

}

void SortDistToBase(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& objects_clustered_points,
		std::vector<int>& sortedIndx) {

	if(objects_clustered_points.size() == 0){
		ROS_ERROR("Sorting Error: there is no object on the table");
		return;
	}


	std::vector<double> dist;

	for (int i = 0; i < objects_clustered_points.size(); ++i) {
		Eigen::Vector3d temp_pose;
		CalcAverageDistToBase(*objects_clustered_points.at(i),temp_pose);
		dist.push_back(temp_pose.norm());
	}

	std::cout << "here size of dist :"<< dist.size()<< std::endl;

	for (int i = 0; i < dist.size(); ++i) {
		double min_dist=10000;
		int min_dist_iter = 0;
		for (int j = i; j < dist.size(); ++j){

			if (dist.at(j) < min_dist) {
				min_dist_iter =  j;
				min_dist = dist.at(j) ;
			}
		}

		sortedIndx.push_back(min_dist_iter);

	}


	std::cout << "here2 size of dist :"<< sortedIndx.size()<< std::endl;

}
int main(int argc, char *argv[]) {
	ros::init(argc, argv, "unknown_object_grasping");

	//global variables
	ros::NodeHandle nh_;
	tf::TransformListener tf_;

	ros::ServiceClient client1 = nh_.serviceClient<INESC_Robotis_Driver::SetToolPose>("/GoToPose_service/setToolPose");

	ros::ServiceClient client = nh_.serviceClient<INESC_Robotis_Driver::HomeArm>("/GoToPose_service/homeArm");
	INESC_Robotis_Driver::HomeArm srv;
	if (client.call(srv))
	   {
	     ROS_INFO("calling home arm");
	   }
	else
	   {
	     ROS_ERROR("home arm");
	     return 1;
	   }


	std::string topic3 = nh_.resolveName("object_cloud");

	unsigned int queue_size;
	ros::Publisher pub2 = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >(topic3,
			queue_size);

	ros::Rate loop_rate(10);

	std::string topic = nh_.resolveName("/table_top_cloud");

	nh_.setParam("/preprocessing/x_filter_min", 0.53);
	nh_.setParam("/preprocessing/x_filter_max", 0.88);

	while (ros::ok()) {

		ros::Time start_time = ros::Time::now();
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr table_top_cloud_prt =
				ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ> >(
						topic, nh_, ros::Duration(10.0));

		if (!table_top_cloud_prt or table_top_cloud_prt->points.size()<100) {
			ROS_ERROR(
					"unknown_object_grasping: no table top cloud prt has been received, check the preproicessing node");
			client.call(srv);
		}
		else{
			// Creating the KdTree object for the search method of the extraction
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
					new pcl::search::KdTree<pcl::PointXYZ>);
			tree->setInputCloud(table_top_cloud_prt);

			std::vector<pcl::PointIndices> cluster_indices;
			pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
			ec.setClusterTolerance(0.02); // 2cm
			ec.setMinClusterSize(50);
			ec.setMaxClusterSize(25000);
			ec.setSearchMethod(tree);
			ec.setInputCloud(table_top_cloud_prt);
			ec.extract(cluster_indices);

			std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> objects_clustered_points;

			for (std::vector<pcl::PointIndices>::const_iterator it =
					cluster_indices.begin(); it != cluster_indices.end(); ++it) {
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
						new pcl::PointCloud<pcl::PointXYZ>);

				for (std::vector<int>::const_iterator pit = it->indices.begin();
						pit != it->indices.end(); ++pit)
					cloud_cluster->points.push_back(
							table_top_cloud_prt->points[*pit]); //*

				cloud_cluster->width = cloud_cluster->points.size();
				cloud_cluster->height = 1;
				cloud_cluster->is_dense = true;
				cloud_cluster->header.frame_id = "world";
				//cloud_cluster->header.stamp = ros::Time::now().toNSec();
				std::cout << "PointCloud representing the Cluster: "
						<< cloud_cluster->points.size() << " data points."
						<< std::endl;

				objects_clustered_points.push_back(cloud_cluster);

			}

			std::vector<int> sortedIndex;
			SortDistToBase(objects_clustered_points, sortedIndex);


			pcl::PointCloud<pcl::PointXYZ>::Ptr object_clustered_point(
					new pcl::PointCloud<pcl::PointXYZ>);


			object_clustered_point = objects_clustered_points.at(sortedIndex.front());

			pub2.publish(object_clustered_point);

	//		grasping the nearest


			Eigen::Vector3d center;
			CalcAverageDistToBase(*object_clustered_point, center);

			center.x() = 0.5;//center.x() - 0.15;

			std::cout <<center ;

			objects_clustered_points.clear();

			INESC_Robotis_Driver::SetToolPose srv2;

			srv2.request.pos_x = center.x();
			srv2.request.pos_y = center.y();
			srv2.request.pos_z = center.z();
			srv2.request.Yaw_angle_z = 3.14;
			srv2.request.Pitch_angle_y = 1.57;
			srv2.request.Roll_angle_x = 0;
			srv2.request.linear_velocity = 0.07;

			if (client1.call(srv2))
			   {
				 ROS_INFO("calling set toll Pose");
			   }
			else
			   {
				 ROS_ERROR("calling set toll Pose");
				 return 1;
			   }
		}
		ros::spinOnce();

		loop_rate.sleep();

	}

	return 0;
}

