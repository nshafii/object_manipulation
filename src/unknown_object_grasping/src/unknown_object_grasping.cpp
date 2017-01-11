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
#include <pcl/common/pca.h>

//Arm driver includes
#include "INESC_Robotis_Driver/SetToolPose.h"
#include "INESC_Robotis_Driver/HomeArm.h"

// visualization
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>

#include <unknown_object_grasping/GraspingNearestObject.h>


ros::ServiceClient setToolPosSrv;
Eigen::Vector3d graspPose;
Eigen::Affine3d objectPose;
double orientation;

void ObjectCentericFrameRelatedToTable(
		const pcl::PointCloud<pcl::PointXYZ>& target_pc,
		pcl::PointCloud<pcl::PointXYZ>& transformed_cloud,
		Eigen::Affine3d& pose)
		{

      // calculate the dimension of bounding box in Arm frame

      double minX, minY, minZ;

      double maxX, maxY, maxZ;

      minX= minY= minZ=100000;

      maxX= maxY= maxZ=-100000;

      pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>);

      pcl::PointCloud<pcl::PointXYZ>::Ptr modelTemp(new pcl::PointCloud<pcl::PointXYZ>);

      pcl::PointCloud<pcl::PointXYZ>::Ptr transforemedCloudTemp(new pcl::PointCloud<pcl::PointXYZ>);

      for (unsigned int i=0; i < target_pc.points.size(); i++)

      {

              double X = target_pc.points.at(i).x;

              double Y = target_pc.points.at(i).y;

              double Z = target_pc.points.at(i).z;

              Eigen::Vector3d PointInArmFrameWithOutTranslition;
              PointInArmFrameWithOutTranslition << X, Y, Z;

              pcl::PointXYZ pointInArmFrame;

              pointInArmFrame.x = PointInArmFrameWithOutTranslition.x();

              pointInArmFrame.y = PointInArmFrameWithOutTranslition.y();

              pointInArmFrame.z = PointInArmFrameWithOutTranslition.z();

              model->points.push_back(pointInArmFrame);

              if(pointInArmFrame.x < minX) minX = pointInArmFrame.x;

              if(pointInArmFrame.y < minY) minY = pointInArmFrame.y;

              if(pointInArmFrame.z < minZ) minZ = pointInArmFrame.z;

              if(pointInArmFrame.x > maxX) maxX = pointInArmFrame.x;

              if(pointInArmFrame.y > maxY) maxY = pointInArmFrame.y;

              if(pointInArmFrame.z > maxZ) maxZ = pointInArmFrame.z;

      }

      /// transformation------------------------

      /// computePose

      Eigen::Vector3d centroid;

      centroid << (minX+maxX) / 2.0, (minY+maxY) / 2.0, (minZ+maxZ) / 2.0;

      pcl::copyPointCloud(*model, *modelTemp);

      /// project on ground

      for(int i = 0; i < model->size(); ++i)

              model->points[i].z = 0;

      /// calc the PCA

      Eigen::Vector3d eval;

      Eigen::Matrix3f evec;

      pcl::PCA<pcl::PointXYZ> pca2D;

      pca2D.setInputCloud(model);

      evec = pca2D.getEigenVectors();

      /// calc the Orientation

      orientation = std::atan2(evec(1,0), evec(0,0) );

      ROS_INFO("direct yaw %f", orientation * 180 / M_PI);

      // put the cloud to the calclated object frame

      pose = Eigen::Translation3d(centroid) * Eigen::AngleAxisd(orientation, Eigen::Vector3d::UnitZ());

      pcl::transformPointCloud(*modelTemp, *transforemedCloudTemp, pose.inverse().cast<float>());

      // check the sign ambiguty

      float countMinus = 0;

      float countPositive = 0;

      for (unsigned int i=0; i < transforemedCloudTemp->points.size(); i++)

      {

              if(transforemedCloudTemp->points[i].x <= 0 )

                      countMinus = countMinus + transforemedCloudTemp->points[i].x;

              else

                      countPositive = countPositive + transforemedCloudTemp->points[i].x;

      }

      if(countMinus < countPositive){

              pcl::copyPointCloud(*transforemedCloudTemp, transformed_cloud);

      }

      else

      {

              ROS_INFO("sign ambiguity has been solved");

              Eigen::Affine3d poseAmb = Eigen::Translation3d(centroid) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());

              pcl::transformPointCloud(*transforemedCloudTemp, transformed_cloud, poseAmb.inverse());

              pose = Eigen::Translation3d(centroid) * Eigen::AngleAxisd(orientation + M_PI, Eigen::Vector3d::UnitZ()) ;
              // here do nothing
              //pcl::copyPointCloud(*transforemedCloudTemp, transformed_cloud);

      }

}



void graspStrategyAndPointDetection(
		const pcl::PointCloud<pcl::PointXYZ>& target_pc)

		{
	double LENGTH_PROPER = 0.05; //this is under discussion and unknown

	int orientationCorrection = 1; // this is for sign anbiguty

	bool sideStrategy, topStrategy, flatStrategy;

	std::vector<geometry_msgs::Point> objectPointCloud;

	geometry_msgs::Point leftPoint;

	geometry_msgs::Point rightPoint;


	// calculate the dimension of bounding box

	double minX, minY, minZ;

	double maxX, maxY, maxZ;

	minX = minY = minZ = 100000;

	maxX = maxY = maxZ = -100000;

	objectPointCloud.clear();

	for (unsigned int i = 0; i < target_pc.points.size(); i++)

	{

		double X = target_pc.points.at(i).x;

		double Y = target_pc.points.at(i).y;

		double Z = target_pc.points.at(i).z;

		geometry_msgs::Point point;

		point.x = X;

		point.y = Y;

		point.z = Z;

		if (X < minX)
			minX = X;

		if (Y < minY)
			minY = Y;

		if (Z < minZ)
			minZ = Z;

		if (X > maxX)
			maxX = X;

		if (Y > maxY)
			maxY = Y;

		if (Z > maxZ)
			maxZ = Z;

		objectPointCloud.push_back(point);

	}

	// calculate the dimension of bounding box

	double xDimention = fabs(maxX - minX);

	double yDimention = fabs(maxY - minY);

	double zDimention = fabs(maxZ - minZ);

	ROS_INFO("xDimention: %f", xDimention);

	ROS_INFO("yDimention: %f", yDimention);

	ROS_INFO("zDimention: %f", zDimention);

	int devisionSize = 5;

	// Analyze the face of the object related to view Point grasp horizontally

	if (zDimention + 0.03 > yDimention && zDimention + 0.04 > xDimention)

	{

		ROS_INFO("Z is the principle axis");

		sideStrategy = true;

		topStrategy = false;

		flatStrategy = false;

		double zInterval = zDimention / devisionSize;

		double minArray[devisionSize];

		double maxArray[devisionSize];

		std::fill_n(minArray, devisionSize, 100000);

		std::fill_n(maxArray, devisionSize, -100000);

		double minXDimention = 1000;

		double ticknessTreshhold = 0.001;

		int foundIter = -1;

		for (int zIntervalIter = 2; zIntervalIter < 3; zIntervalIter++)

		{

			for (unsigned int i = 0; i < objectPointCloud.size(); i++)

			{

				double X = objectPointCloud.at(i).x;

				objectPointCloud.at(i).y = orientationCorrection * yDimention
						/ 2;

				double Y = objectPointCloud.at(i).y;

				double Z = objectPointCloud.at(i).z;

				geometry_msgs::Point point;

				point.x = X;

				point.y = Y;

				// project the point cloud on the plain

				point.z = Z;

				if (Z >= ((zIntervalIter * zInterval) + minZ)
						&& Z < (((zIntervalIter + 1) * zInterval) + minZ)) {

					if (X <= minArray[zIntervalIter])
						minArray[zIntervalIter] = X;

					if (X >= maxArray[zIntervalIter])
						maxArray[zIntervalIter] = X;

				}

			}

			double dimentionX = fabs(
					maxArray[zIntervalIter] - minArray[zIntervalIter]);

			ROS_INFO("dimentionX[%i]: %f", zIntervalIter, dimentionX);

			double diffDimentionX = fabs(dimentionX - LENGTH_PROPER);

			if (diffDimentionX <= minXDimention
					&& dimentionX > ticknessTreshhold)

					{

				minXDimention = diffDimentionX;

				foundIter = zIntervalIter;

			}

			ROS_INFO("minXDimention, Sector founds [%i]: %f ", foundIter,
					minXDimention);

			ROS_INFO("size Point Cloud= %i .size transferred= %i",
					target_pc.points.size(), objectPointCloud.size());

			// ToDo Should be fixed

			leftPoint.z = ((foundIter + 0.5) * zInterval) + minZ;

			rightPoint.z = ((foundIter + 0.5) * zInterval) + minZ;

			leftPoint.y = orientationCorrection * yDimention / 2;

			rightPoint.y = orientationCorrection * yDimention / 2;

			leftPoint.x = fabs(maxArray[foundIter] - minArray[foundIter]) / 2;

			rightPoint.x = -1 * fabs(maxArray[foundIter] - minArray[foundIter])
					/ 2;

		}

	}

	// Analyze the face of the object related to view Point grasp vertical

	else if (xDimention > (zDimention + 0.02)
			&& yDimention > (zDimention + 0.02))

			{

		ROS_INFO(" flat object , x and y are the principale axis");

		flatStrategy = true;

		sideStrategy = false;

		topStrategy = false;

		double xInterval = xDimention / devisionSize;

		//std::map< int, pair<double,double> > viewMap;

		//std::vector < std::vector<double> > viewMap;

		double minArray[devisionSize];

		double maxArray[devisionSize];

		std::fill_n(minArray, devisionSize, 100000);

		std::fill_n(maxArray, devisionSize, -100000);

		double minZDimention = 1000;

		double ticknessTreshhold = 0.0015;

		int foundIter = -1;

		for (int xIntervalIter = 2; xIntervalIter < 3; xIntervalIter++)

		{

			for (unsigned int i = 0; i < objectPointCloud.size(); i++)

			{

				double X = objectPointCloud.at(i).x;

				objectPointCloud.at(i).y = orientationCorrection * yDimention
						/ 2;

				double Y = objectPointCloud.at(i).y;

				double Z = objectPointCloud.at(i).z;

				if (X >= ((xIntervalIter * xInterval) + minX) &&

				X < (((xIntervalIter + 1) * xInterval) + minX)) {

					if (Z <= minArray[xIntervalIter])
						minArray[xIntervalIter] = Z;

					if (Z >= maxArray[xIntervalIter])
						maxArray[xIntervalIter] = Z;

				}

			}

			double dimentionZ = fabs(
					maxArray[xIntervalIter] - minArray[xIntervalIter]);

			ROS_INFO("dimentionZ[%i]: %f", xIntervalIter, dimentionZ);

			double diffDimentionZ = fabs(dimentionZ - LENGTH_PROPER);

			if (diffDimentionZ <= minZDimention
					&& dimentionZ > ticknessTreshhold)

					{

				minZDimention = diffDimentionZ;

				foundIter = xIntervalIter;

			}

		}

		ROS_INFO("minZDimention, Sector founds [%i]: %f ", foundIter,
				minZDimention);

		ROS_INFO("size Point Cloud= %i .size transferred= %i",
				target_pc.points.size(), objectPointCloud.size());

		// ToDo Should be fixed

		leftPoint.x = ((foundIter + 0.5) * xInterval) + minX;

		rightPoint.x = ((foundIter + 0.5) * xInterval) + minX;

		leftPoint.y = orientationCorrection * yDimention / 2;

		rightPoint.y = orientationCorrection * yDimention / 2;

		leftPoint.z = fabs(maxArray[foundIter] - minArray[foundIter]) / 2;
		;

		rightPoint.z = -1 * fabs(maxArray[foundIter] - minArray[foundIter]) / 2;

	}

	else

	{

		ROS_INFO(" x is the principale axis");

		topStrategy = true;

		sideStrategy = false;

		flatStrategy = false;

		double xInterval = xDimention / devisionSize;

		//std::map< int, pair<double,double> > viewMap;

		//std::vector < std::vector<double> > viewMap;

		double minArray[devisionSize];

		double maxArray[devisionSize];

		std::fill_n(minArray, devisionSize, 100000);

		std::fill_n(maxArray, devisionSize, -100000);

		double minYDimention = 1000;

		double ticknessTreshhold = 0.0015;

		int foundIter = -1;

		for (int xIntervalIter = 2; xIntervalIter < 3; xIntervalIter++)

		{

			for (unsigned int i = 0; i < objectPointCloud.size(); i++)

			{

				double X = objectPointCloud.at(i).x;

				double Y = objectPointCloud.at(i).y;

				objectPointCloud.at(i).z = zDimention / 2;

				double Z = objectPointCloud.at(i).z;

				if (X >= ((xIntervalIter * xInterval) + minX) &&

				X < (((xIntervalIter + 1) * xInterval) + minX)) {

					if (Y <= minArray[xIntervalIter])
						minArray[xIntervalIter] = Y;

					if (Y >= maxArray[xIntervalIter])
						maxArray[xIntervalIter] = Y;

				}

			}

			double dimentionY = fabs(
					maxArray[xIntervalIter] - minArray[xIntervalIter]);

			ROS_INFO("dimentionY[%i]: %f", xIntervalIter, dimentionY);

			double diffDimentionY = fabs(dimentionY - LENGTH_PROPER);

			if (diffDimentionY <= minYDimention
					&& dimentionY > ticknessTreshhold)

					{

				minYDimention = diffDimentionY;

				foundIter = xIntervalIter;

			}

		}

		ROS_INFO("minYDimention, Sector founds [%i]: %f ", foundIter,
				minYDimention);

		ROS_INFO("size Point Cloud= %i .size transferred= %i",
				target_pc.points.size(), objectPointCloud.size());

		// ToDo Should be fixed

		leftPoint.x = ((foundIter + 0.5) * xInterval) + minX;

		rightPoint.x = ((foundIter + 0.5) * xInterval) + minX;

		leftPoint.y = fabs(maxArray[foundIter] - minArray[foundIter]) / 2;

		rightPoint.y = -1 * fabs(maxArray[foundIter] - minArray[foundIter]) / 2;

		leftPoint.z = zDimention / 2;

		rightPoint.z = zDimention / 2;

	}

	return;

}

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

void SortDistToBase(
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& objects_clustered_points,
		std::vector<int>& sortedIndx) {

	if (objects_clustered_points.size() == 0) {
		ROS_ERROR("Sorting Error: there is no object on the table");
		return;
	}

	std::vector<double> dist;

	for (int i = 0; i < objects_clustered_points.size(); ++i) {
		Eigen::Vector3d temp_pose;
		CalcAverageDistToBase(*objects_clustered_points.at(i), temp_pose);
		dist.push_back(temp_pose.norm());
	}

	std::cout << "here size of dist :" << dist.size() << std::endl;

	for (int i = 0; i < dist.size(); ++i) {
		double min_dist = 10000;
		int min_dist_iter = 0;
		for (int j = i; j < dist.size(); ++j) {

			if (dist.at(j) < min_dist) {
				min_dist_iter = j;
				min_dist = dist.at(j);
			}
		}

		sortedIndx.push_back(min_dist_iter);

	}

	std::cout << "here2 size of dist :" << sortedIndx.size() << std::endl;

}

geometry_msgs::Point eigenVectorToPointMsg(Eigen::Vector3d vec){
	geometry_msgs::Point out;
	out.x = vec.x();
	out.y = vec.y();
	out.z = vec.z();
	return out;
}

visualization_msgs::MarkerArray visualizeObjectAxis(Eigen::Affine3d objectAffine){

	Eigen::Vector3d x1, x2, y1, y2, z1, z2;
	visualization_msgs::MarkerArray array;
	visualization_msgs::Marker line_x;
	line_x.header.frame_id = "/world";
	line_x.header.stamp = ros::Time::now();
	line_x.ns = "object_axis_x";
	line_x.pose.orientation.w = 1.0;
	line_x.id = 1;
	line_x.action = visualization_msgs::Marker::ADD;
	line_x.type = visualization_msgs::Marker::LINE_LIST;
	line_x.scale.x = 0.01;
	line_x.color.r = 1.0;
	line_x.color.a = 1.0;

	visualization_msgs::Marker line_y;
	line_y.header.frame_id = "/world";
	line_y.header.stamp = ros::Time::now();
	line_y.ns = "object_axis_y";
	line_y.pose.orientation.w = 1.0;
	line_y.id = 2;
	line_y.action = visualization_msgs::Marker::ADD;
	line_y.type = visualization_msgs::Marker::LINE_LIST;
	line_y.scale.x = 0.01;
	line_y.color.g = 1.0;
	line_y.color.a = 1.0;

	visualization_msgs::Marker line_z;
	line_z.header.frame_id = "/world";
	line_z.header.stamp = ros::Time::now();
	line_z.ns = "object_axis_z";
	line_z.pose.orientation.w = 1.0;
	line_z.id = 3;
	line_z.action = visualization_msgs::Marker::ADD;
	line_z.type = visualization_msgs::Marker::LINE_LIST;
	line_z.scale.x = 0.01;
	line_z.color.b = 1.0;
	line_z.color.a = 1.0;



	double length = 0.1;
	x1 << length,0,0;
	y1 << 0,length,0;
	z1 << 0,0,length;

	Eigen::Vector3d centroid = objectAffine.translation();

	x2 = objectAffine.linear() * x1 + centroid;
	y2 = objectAffine.linear() * y1 + centroid;
	z2 = objectAffine.linear() * z1 + centroid;


	line_x.points.push_back(eigenVectorToPointMsg(centroid));

	line_x.points.push_back(eigenVectorToPointMsg(x2));

	line_y.points.push_back(eigenVectorToPointMsg(centroid));

	line_y.points.push_back(eigenVectorToPointMsg(y2));

	line_z.points.push_back(eigenVectorToPointMsg(centroid));

	line_z.points.push_back(eigenVectorToPointMsg(z2));

	array.markers.push_back(line_x);
	array.markers.push_back(line_y);
	array.markers.push_back(line_z);

	return array;

}

bool graspNearestObject(unknown_object_grasping::GraspingNearestObject::Request &req,
unknown_object_grasping::GraspingNearestObject::Response &res)
{

	Eigen::Vector3d objectPosition = objectPose.translation();
	//CalcAverageDistToBase(*object_clustered_point, center);
	Eigen::Vector3d behind;
	behind<<-0.1, 0, 0;

	graspPose = objectPosition + behind;
	//center.x() = 0.5; //center.x() - 0.15;

	//std::cout << center;

	//objects_clustered_points.clear();

	INESC_Robotis_Driver::SetToolPose srv2;

	srv2.request.pos_x = graspPose.x();
	srv2.request.pos_y = graspPose.y();
	srv2.request.pos_z = graspPose.z();
	srv2.request.Yaw_angle_z = 3.14;
	srv2.request.Pitch_angle_y = 1.57;
	srv2.request.Roll_angle_x = orientation;
	srv2.request.linear_velocity = 0.1;

	ROS_INFO ( "Go to Grasp Pose ( %f , %f , %f )", graspPose.x(), graspPose.y(), graspPose.z() );

	if (setToolPosSrv.call(srv2)) {

		ROS_INFO("calling set toll Pose");
	} else {
		ROS_ERROR("calling set toll Pose");
		return 1;
	}

}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "unknown_object_grasping");

	//global variables
	ros::NodeHandle nh_;
	ros::Publisher marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("visualization_array", 100);
	tf::TransformListener tf_;

	setToolPosSrv = nh_.serviceClient<
			INESC_Robotis_Driver::SetToolPose>("/GoToPose_service/setToolPose");

	ros::ServiceClient client =
			nh_.serviceClient<INESC_Robotis_Driver::HomeArm>(
					"/GoToPose_service/homeArm");
	INESC_Robotis_Driver::HomeArm srv;
	if (client.call(srv)) {
		ROS_INFO("calling home arm");
	} else {
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

	ros::ServiceServer service = nh_.advertiseService("graspNearestObject",graspNearestObject);

	while (ros::ok()) {

		ros::Time start_time = ros::Time::now();
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr table_top_cloud_prt =
				ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ> >(
						topic, nh_, ros::Duration(10.0));

		if (!table_top_cloud_prt or table_top_cloud_prt->points.size() < 100) {
			ROS_ERROR(
					"unknown_object_grasping: no table top cloud prt has been received, check the preproicessing node");
			client.call(srv);
		} else {
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
					cluster_indices.begin(); it != cluster_indices.end();
					++it) {
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

			object_clustered_point = objects_clustered_points.at(
					sortedIndex.front());

			pub2.publish(object_clustered_point);



			pcl::PointCloud<pcl::PointXYZ> object_frame_points;
			ObjectCentericFrameRelatedToTable(*object_clustered_point, object_frame_points, objectPose);
			visualization_msgs::MarkerArray axis = visualizeObjectAxis(objectPose);
			marker_pub.publish(axis);
			//atan2(objectPose.inverse());
			//std::cout<< "yaw calc: "<< (objectPose.inverse()) << std::endl;
			/*
//			grasping the nearest
			Eigen::Vector3d center;
			CalcAverageDistToBase(*object_clustered_point, center);

			center.x() = 0.5; //center.x() - 0.15;

			std::cout << center;

			objects_clustered_points.clear();

			INESC_Robotis_Driver::SetToolPose srv2;

			srv2.request.pos_x = center.x();
			srv2.request.pos_y = center.y();
			srv2.request.pos_z = center.z();
			srv2.request.Yaw_angle_z = 3.14;
			srv2.request.Pitch_angle_y = 1.57;
			srv2.request.Roll_angle_x = 0;
			srv2.request.linear_velocity = 0.1;

			if (client1.call(srv2)) {
				ROS_INFO("calling set toll Pose");
			} else {
				ROS_ERROR("calling set toll Pose");
				return 1;
			}
			*/



		}
		ros::spinOnce();

		loop_rate.sleep();

	}

	return 0;
}

