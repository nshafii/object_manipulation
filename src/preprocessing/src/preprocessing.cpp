//ROS includes
#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

//PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>


tf::Transform getPlaneTransform(pcl::ModelCoefficients coeffs,
		double up_direction) {
	ROS_ASSERT(coeffs.values.size() > 3);
	double a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d =
			coeffs.values[3];

	tf::Vector3 position(-a * d, -b * d, -c * d);
	tf::Vector3 z(a, b, c);

	z[0] = z[1] = 0;
	z[2] = up_direction;

	tf::Vector3 x(1, 0, 0);
	if (fabs(z.dot(x)) > 1.0 - 1.0e-4)
		x = tf::Vector3(0, 1, 0);
	tf::Vector3 y = z.cross(x).normalized();
	x = y.cross(z).normalized();

	tf::Matrix3x3 rotation;
	rotation[0] = x; 	// x
	rotation[1] = y; 	// y
	rotation[2] = z; 	// z
	rotation = rotation.transpose();
	tf::Quaternion orientation;
	rotation.getRotation(orientation);
	return tf::Transform(orientation, position);
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "preprocessing");

	//global variables
	ros::NodeHandle nh_;
	std::string arm_base_frame_;
	tf::TransformListener tf_;
	double z_filter_min_, z_filter_max_, y_filter_min_, y_filter_max_,
			x_filter_min_, x_filter_max_;
	double voxel_size_;
	double table_z_filter_min_;
	double table_z_filter_max_;
//	nh_.setParam("/preprocessing/arm_base_frame", "world");
//	nh_.setParam("/preprocessing/z_filter_min", -0.1);
//	nh_.setParam("/preprocessing/z_filter_max", 0.2);
//	nh_.setParam("/preprocessing/y_filter_min", 0);
//	nh_.setParam("/preprocessing/y_filter_max", 0.40);
//	nh_.setParam("/preprocessing/x_filter_min", 0.15);
//	nh_.setParam("/preprocessing/x_filter_max", 0.75);
//	nh_.setParam("/preprocessing/voxel_size", 0.005);
//	nh_.setParam("/preprocessing/table_z_filter_min", 0.01);

//	nh_.setParam("/preprocessing/z_filter_min", -0.2);
//	nh_.setParam("/preprocessing/z_filter_max", 0.2);
//	nh_.setParam("/preprocessing/y_filter_min", -0.2);
//	nh_.setParam("/preprocessing/y_filter_max", 0.40);
//	nh_.setParam("/preprocessing/x_filter_min", 0.25);
//	nh_.setParam("/preprocessing/x_filter_max", 0.7);
//	nh_.setParam("/preprocessing/voxel_size", 0.005);
//	nh_.setParam("/preprocessing/table_z_filter_min", 0.01);

	std::string topic2 = nh_.resolveName("table_top_cloud");
	uint32_t queue_size = 1;
	ros::Publisher pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >(topic2,
			queue_size);

	ros::Rate loop_rate(20);

	std::string topic = nh_.resolveName("/camera/depth/points");

	ros::Duration(3.0).sleep();

	while (ros::ok()) {
		nh_.getParam("/preprocessing/arm_base_frame", arm_base_frame_);
		nh_.getParam("/preprocessing/z_filter_min", z_filter_min_);
		nh_.getParam("/preprocessing/z_filter_max", z_filter_max_);
		nh_.getParam("/preprocessing/y_filter_min", y_filter_min_);
		nh_.getParam("/preprocessing/y_filter_max", y_filter_max_);
		nh_.getParam("/preprocessing/x_filter_min", x_filter_min_);
		nh_.getParam("/preprocessing/x_filter_max", x_filter_max_);
		nh_.getParam("/preprocessing/voxel_size", voxel_size_);
		nh_.getParam("/preprocessing/table_z_filter_min", table_z_filter_min_);
		nh_.getParam("/preprocessing/table_z_filter_max", table_z_filter_max_);

		ros::Time start_time = ros::Time::now();
		sensor_msgs::PointCloud2::ConstPtr kinect_sensor_cloud =
				ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh_,
						ros::Duration(3.0));

		if (!kinect_sensor_cloud) {
			ROS_ERROR("Preprocessing: no point_cloud2 has been received");
			return true;
		}

		ROS_INFO_STREAM(
				"Point cloud received after " << ros::Time::now() - start_time << " seconds; processing");

		if (!arm_base_frame_.empty()) {

			ROS_INFO("wait for transform");
			tf_.waitForTransform("world", "camera_depth_optical_frame",
					ros::Time(0), ros::Duration(4));
			ROS_INFO("transform is received");

			//transform the clouad
			sensor_msgs::PointCloud2 transforemed_sensor_cloud;
			tf::StampedTransform transformStamp;
			tf_.lookupTransform(arm_base_frame_,
					kinect_sensor_cloud->header.frame_id.c_str(), ros::Time(0),
					transformStamp);
			tf::Transform transform;
			transform.setBasis(transformStamp.getBasis());
			transform.setOrigin(transformStamp.getOrigin());
			pcl_ros::transformPointCloud(arm_base_frame_, transform,
					*kinect_sensor_cloud, transforemed_sensor_cloud);

			// step 1: remove NAN points
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl_ptr(
					new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud_pcl_ptr(
					new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg(transforemed_sensor_cloud, *cloud_pcl_ptr);
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*cloud_pcl_ptr, *ref_cloud_pcl_ptr,
					indices);
			ROS_INFO("Step 1 done, NAN Points: %i points", indices.size());

			// step 2: down sampling
			pcl::PassThrough<pcl::PointXYZ> pass_;

			pass_.setInputCloud(cloud_pcl_ptr);
			pass_.setFilterFieldName("z");
			pass_.setFilterLimits(z_filter_min_, z_filter_max_);
			pcl::PointCloud<pcl::PointXYZ>::Ptr z_cloud_filtered_ptr(
					new pcl::PointCloud<pcl::PointXYZ>);
			pass_.filter(*z_cloud_filtered_ptr);

			pass_.setInputCloud(z_cloud_filtered_ptr);
			pass_.setFilterFieldName("y");
			pass_.setFilterLimits(y_filter_min_, y_filter_max_);
			pcl::PointCloud<pcl::PointXYZ>::Ptr y_cloud_filtered_ptr(
					new pcl::PointCloud<pcl::PointXYZ>);
			pass_.filter(*y_cloud_filtered_ptr);

			pass_.setInputCloud(y_cloud_filtered_ptr);
			pass_.setFilterFieldName("x");
			pass_.setFilterLimits(x_filter_min_, x_filter_max_);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr(
					new pcl::PointCloud<pcl::PointXYZ>);
			pass_.filter(*cloud_filtered_ptr);

			pcl::VoxelGrid<pcl::PointXYZ> grid_;
			grid_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
			grid_.setDownsampleAllData(true);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled_ptr(
					new pcl::PointCloud<pcl::PointXYZ>);
			grid_.setInputCloud(cloud_filtered_ptr);
			grid_.filter(*cloud_downsampled_ptr);

			ROS_INFO("Step 2 done");


			// Step 3 : Detect the plane
			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n3d_;
			pcl::search::KdTree<pcl::PointXYZ>::Ptr normals_tree_;

			n3d_.setKSearch(10);
			n3d_.setSearchMethod(normals_tree_);
			pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr(
					new pcl::PointCloud<pcl::Normal>);
			n3d_.setInputCloud(cloud_downsampled_ptr);
			n3d_.compute(*cloud_normals_ptr);

			pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_;
			pcl::ProjectInliers<pcl::PointXYZ> proj_;

			seg_.setDistanceThreshold(0.05);
			seg_.setMaxIterations(10000);
			seg_.setNormalDistanceWeight(0.1);
			seg_.setOptimizeCoefficients(true);
			seg_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
			seg_.setMethodType(pcl::SAC_RANSAC);
			seg_.setProbability(0.99);

			proj_.setModelType(pcl::SACMODEL_PLANE);

			pcl::PointIndices::Ptr table_inliers_ptr(new pcl::PointIndices);
			pcl::ModelCoefficients::Ptr table_coefficients_ptr(
					new pcl::ModelCoefficients);
			seg_.setInputCloud(cloud_downsampled_ptr);
			seg_.setInputNormals(cloud_normals_ptr);
			seg_.segment(*table_inliers_ptr, *table_coefficients_ptr);

			if (table_coefficients_ptr->values.size() <= 3) {
				ROS_INFO("Failed to detect table in scan");
				return 0;
			}

			unsigned int inlier_threshold_ = 300;
			if (table_inliers_ptr->indices.size() < inlier_threshold_) {
				ROS_ERROR(
						"Plane detection has %d inliers, below min threshold of %d",
						(int )table_inliers_ptr->indices.size(),
						inlier_threshold_);
				ROS_ERROR("there is no flat table");
				return 0;
			}

			ROS_INFO("Table found found with %d inliers: [%f %f %f %f].",
					(int )table_inliers_ptr->indices.size(),
					table_coefficients_ptr->values[0],
					table_coefficients_ptr->values[1],
					table_coefficients_ptr->values[2],
					table_coefficients_ptr->values[3]);
			ROS_INFO("Step 3 done");

			// Step 4 : Extract the points of the table and create the model of the table
			pcl::PointCloud<pcl::PointXYZ>::Ptr table_projected_ptr(
					new pcl::PointCloud<pcl::PointXYZ>);
			proj_.setInputCloud(cloud_downsampled_ptr);
			proj_.setIndices(table_inliers_ptr);
			proj_.setModelCoefficients(table_coefficients_ptr);
			proj_.filter(*table_projected_ptr);

			tf::Transform table_plane_trans;
			table_plane_trans = getPlaneTransform(*table_coefficients_ptr, -1);



			tf::Vector3 flat_table_pos;
			double avg_x, avg_y, avg_z;
			avg_x = avg_y = avg_z = 0;
			for (size_t i = 0; i < table_projected_ptr->points.size(); i++) {
				avg_x += table_projected_ptr->points[i].x;
				avg_y += table_projected_ptr->points[i].y;
				avg_z += table_projected_ptr->points[i].z;
			}
			avg_x /= table_projected_ptr->points.size();
			avg_y /= table_projected_ptr->points.size();
			avg_z /= table_projected_ptr->points.size();

			nh_.setParam("/preprocessing/table_z", avg_z);

			flat_table_pos[0] = avg_x;
			flat_table_pos[1] = avg_y;
			flat_table_pos[2] = avg_z;

			table_plane_trans.setOrigin(flat_table_pos);

			ROS_INFO("Step 4 done");

			// step 5: Extract the table top objects do the table top segmentation
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_objects(
					new pcl::PointCloud<pcl::PointXYZ>);

			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud(cloud_downsampled_ptr);
			extract.setIndices(table_inliers_ptr);
			extract.setNegative(true);
			extract.filter(*cloud_objects);

			pcl::PointCloud<pcl::PointXYZ>::Ptr table_hull_ptr(
					new pcl::PointCloud<pcl::PointXYZ>);

			pcl::ConvexHull<pcl::PointXYZ> hull_;
			hull_.setInputCloud(table_projected_ptr);
			hull_.reconstruct(*table_hull_ptr);
			pcl::PointIndices cloud_object_indices;
			pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism_;

			prism_.setInputCloud(cloud_objects);
			prism_.setInputPlanarHull(table_hull_ptr);



			std::cout << "table_z_filter_max_: " << table_z_filter_max_ <<std:: endl;

			std::cout << "table_z_filter_min_: " << table_z_filter_min_ <<std:: endl;

			prism_.setHeightLimits(table_z_filter_min_, table_z_filter_max_);
			prism_.segment(cloud_object_indices);

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_objects_ptr(
					new pcl::PointCloud<pcl::PointXYZ>);
			pcl::ExtractIndices<pcl::PointXYZ> extract_object_indices;
			extract_object_indices.setInputCloud(cloud_objects);

			extract_object_indices.setIndices(
					boost::make_shared<const pcl::PointIndices>(
							cloud_object_indices));
			extract_object_indices.filter(*cloud_objects_ptr);

			ROS_INFO("Step 5 done");

			std::cout << "PointCloud representing the objects component: "
					<< cloud_objects_ptr->points.size() << " data points."
					<< std::endl;

			pub.publish(*cloud_objects_ptr);
			ROS_INFO("published the objects point cloud");


		} else {
			ROS_ERROR("The point cloud from kinect has not been recieved");
		}

		ros::spinOnce();

		loop_rate.sleep();

	}

	return 0;
}

