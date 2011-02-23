#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/PoseArray.h>

#include "octomap/octomap.h"
#include "octomap_ros/conversions.h"
#include <octomap_ros/OctomapBinary.h>
#include "pcl_to_octree/octree/OcTreePCL.h"
#include "pcl_to_octree/octree/OcTreeNodePCL.h"
#include "pcl_to_octree/octree/OcTreeServerPCL.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl_ros/publisher.h>
#include <pcl_ros/pcl_nodelet.h>

#include <tf/transform_listener.h>

#include <visualization_msgs/MarkerArray.h>

#include <vector>

#include <nodelet/nodelet.h>
#include <math.h>

#include <iostream>
#include <fstream>
#include <sstream>

/**
@file

@brief  next best view.

@par Advertises
 - \b /nbv_octree topic

@par Subscribes
 - \b /cloud_pcd topic

@par Parameters
- \b input_cloud_topic
- \b output_octree_topic
 */
typedef struct
{
	double x,y;
}point_2d;

namespace autonomous_mapping
{

void write_pgm (std::string filename, std::vector<std::vector<int> > cm)
{
	std::ofstream myfile;
	myfile.open (filename.c_str());
	myfile << "P2" << std::endl;
	myfile << (int)cm[0].size() << " " << (int)cm.size() << std::endl;
	int max = 0;
    for (std::vector<std::vector<int> >::iterator it = cm.begin (); it != cm.end (); it++)
        for (std::vector<int>::iterator jt = it->begin(); jt != it->end (); jt++)
        	if (max < (*jt))
        		max = (*jt);
	myfile << max << std::endl;

    for (std::vector<std::vector<int> >::iterator it = cm.begin (); it != cm.end (); it++)
    {
        for (std::vector<int>::iterator jt = it->begin(); jt != it->end (); jt++)
        	myfile << (*jt) << " ";
        myfile << std::endl;
    }
    myfile.close();
}

class NextBestView:public pcl_ros::PCLNodelet
{

public:
	void onInit ();
protected:
	using pcl_ros::PCLNodelet::pnh_;
	//parameters
	std::string input_cloud_topic_;
	std::string output_octree_topic_;
	std::string output_pose_topic_;
	std::string laser_frame_;

	double octree_res_, octree_maxrange_;
	int level_, free_label_, occupied_label_, unknown_label_;
	bool check_centroids_;
	bool visualize_octree_;

	double normal_search_radius_;
	int min_pts_per_cluster_;
	double eps_angle_;
	double tolerance_;
	double boundary_angle_threshold_;

	//objects needed
	tf::TransformListener tf_listener_;

	//datasets
	octomap::OcTreePCL* octree_;
	octomap::ScanGraph* octomap_graph_;

	octomap::KeyRay ray;

	sensor_msgs::PointCloud2 cloud_in_;
	geometry_msgs::PoseArray nbv_pose_array_;

	ros::Subscriber cloud_sub_;
	ros::Publisher octree_pub_;
	pcl_ros::Publisher<pcl::PointNormal> border_cloud_pub_;
	pcl_ros::Publisher<pcl::PointXYZ> cluster_cloud_pub_;
	pcl_ros::Publisher<pcl::PointXYZ> cluster_cloud2_pub_;
	pcl_ros::Publisher<pcl::PointXYZ> cluster_cloud3_pub_;
	ros::Publisher pose_pub_;

	// Publishes the octree in MarkerArray format so that it can be visualized in rviz
	ros::Publisher octree_marker_array_publisher_;
	/* The following publisher, even though not required, is used because otherwise rviz
	 * cannot visualize the MarkerArray format without advertising the Marker format*/
	ros::Publisher octree_marker_publisher_;
	// Marker array to visualize the octree. It displays the occupied cells of the octree
	visualization_msgs::MarkerArray octree_marker_array_msg_;
	static bool compareClusters(pcl::PointIndices c1, pcl::PointIndices c2) { return (c1.indices.size() < c2.indices.size()); }

	void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg);
	void createOctree (pcl::PointCloud<pcl::PointXYZ>& pointcloud2_pcl, octomap::pose6d laser_pose);
	void visualizeOctree(const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg, geometry_msgs::Point viewpoint);
	void castRayAndLabel(pcl::PointCloud<pcl::PointXYZ>& cloud, octomap::pose6d origin);
	void findBorderPoints(pcl::PointCloud<pcl::PointXYZ>& border_cloud, std::string frame_id);
	void computeBoundaryPoints(pcl::PointCloud<pcl::PointXYZ>& border_cloud, pcl::PointCloud<pcl::Normal>& border_normals, std::vector<pcl::PointIndices>& clusters, pcl::PointCloud<pcl::PointXYZ>* cluster_clouds);

	void extractClusters (const pcl::PointCloud<pcl::PointXYZ> &cloud,
			const pcl::PointCloud<pcl::Normal> &normals,
			float tolerance,
			const boost::shared_ptr<pcl::KdTree<pcl::PointXYZ> > &tree,
			std::vector<pcl::PointIndices> &clusters, double eps_angle,
			unsigned int min_pts_per_cluster = 1,
			unsigned int max_pts_per_cluster = std::numeric_limits<int>::max ());

	//void cumputeKernel (double fi,double d_min,double d_max,double step_size,double degree_size);

public:
	NextBestView();

	~NextBestView();
};


NextBestView::NextBestView () {

}

NextBestView::~NextBestView()
{
	ROS_INFO("Shutting down NextBestView!");

	octree_marker_array_publisher_.shutdown();
	octree_marker_publisher_.shutdown();
}
void NextBestView::onInit()
{
	pcl_ros::PCLNodelet::onInit ();
	pnh_->param("input_cloud_topic", input_cloud_topic_, std::string("/cloud_pcd"));
	pnh_->param("output_octree_topic", output_octree_topic_, std::string("/nbv_octree"));
	pnh_->param("output_pose_topic", output_pose_topic_, std::string("/nbv_pose"));
	pnh_->param("laser_frame", laser_frame_, std::string("/laser_tilt_link"));
	pnh_->param("octree_resolution", octree_res_, 0.1);

	pnh_->param("octree_maxrange", octree_maxrange_, -1.0);
	pnh_->param("level", level_, 0);
	pnh_->param("check_centroids", check_centroids_, false);
	pnh_->param("free_label", free_label_, 0);
	pnh_->param("occupied_label", occupied_label_, 1);
	pnh_->param("unknown_label", unknown_label_, -1);
	pnh_->param("visualize_octree", visualize_octree_, true);

	pnh_->param("normal_search_radius", normal_search_radius_, 0.6);
	pnh_->param("min_pts_per_cluster", min_pts_per_cluster_, 10);
	pnh_->param("eps_angle", eps_angle_, 0.25);
	pnh_->param("tolerance", tolerance_, 0.3);
	pnh_->param("boundary_angle_threshold", boundary_angle_threshold_, 2.5);

	cloud_sub_ = pnh_->subscribe (input_cloud_topic_, 1, &NextBestView::cloud_cb, this);
	octree_pub_ = pnh_->advertise<octomap_ros::OctomapBinary> (output_octree_topic_, 1);
	border_cloud_pub_ = pcl_ros::Publisher<pcl::PointNormal> (*pnh_, "border_cloud", 1);
	cluster_cloud_pub_ = pcl_ros::Publisher<pcl::PointXYZ> (*pnh_, "cluster_cloud", 1);
	cluster_cloud2_pub_ = pcl_ros::Publisher<pcl::PointXYZ> (*pnh_, "cluster_cloud2", 1);
	cluster_cloud3_pub_ = pcl_ros::Publisher<pcl::PointXYZ> (*pnh_, "cluster_cloud3", 1);
	pose_pub_ = pnh_->advertise<geometry_msgs::PoseArray> (output_pose_topic_, 1);

	octree_marker_array_publisher_ = pnh_->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
	octree_marker_publisher_ = pnh_->advertise<visualization_msgs::Marker>("visualization_marker", 100);

	octree_ = NULL;
}
/**
 * \brief cloud callback and the core filtering function at the same time
 * \param pointcloud2_msg input point cloud to be processed
 */
void NextBestView::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg) {

	pcl::PointCloud<pcl::PointXYZ> pointcloud2_pcl;
	octomap::point3d octomap_point3d;

	ROS_INFO("Received point cloud");

	//get the latest parameters
	pnh_->getParam("normal_search_radius", normal_search_radius_);
	pnh_->getParam("min_pts_per_cluster", min_pts_per_cluster_);
	pnh_->getParam("eps_angle", eps_angle_);
	pnh_->getParam("tolerance", tolerance_);
	pnh_->getParam("boundary_angle_threshold", boundary_angle_threshold_);

	//get the viewpoint (position of laser) from tf
	tf::StampedTransform transform;
	try {
		ros::Time acquisition_time = pointcloud2_msg->header.stamp;
		ros::Duration timeout(1.0 / 30);
		tf_listener_.waitForTransform(pointcloud2_msg->header.frame_id, laser_frame_, acquisition_time, timeout);
		tf_listener_.lookupTransform(pointcloud2_msg->header.frame_id, laser_frame_, acquisition_time, transform);
	}
	catch (tf::TransformException& ex) {
		ROS_WARN("[next_best_view] TF exception:\n%s", ex.what());
	}
	tf::Point pt = transform.getOrigin();
	octomap::pose6d laser_pose (pt.x(), pt.y(), pt.z(),0,0,0);
	ROS_INFO("viewpoint [%f %f %f]", pt.x(), pt.y(), pt.z());


	//Converting PointCloud2 msg format to pcl pointcloud format in order to read the 3d data
	pcl::fromROSMsg(*pointcloud2_msg, pointcloud2_pcl);

	// create or update the octree
	if (octree_ == NULL) {
		octomap_graph_ = new octomap::ScanGraph();
		octree_ = new octomap::OcTreePCL(octree_res_);
	}
	createOctree(pointcloud2_pcl, laser_pose);

	/*
	 * assign new points to Leaf Nodes  and cast rays from laser pos to point
	 */
	castRayAndLabel(pointcloud2_pcl, laser_pose);

	/*
	 * find unknown voxels with free neighbors and add them to a pointcloud
	 */
	pcl::PointCloud<pcl::PointXYZ> border_cloud;
	findBorderPoints(border_cloud, pointcloud2_msg->header.frame_id);

	// Create the filtering objects
	pcl::PassThrough<pcl::PointXYZ> pass;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::ExtractIndices<pcl::Normal> nextract;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

	//creating datasets
	pcl::PointCloud<pcl::Normal> border_normals;

	//filter out ceiling
	pass.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (border_cloud));
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0, 2.2);
	pass.filter(border_cloud);

	// tree object used for search
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree =
			boost::shared_ptr <pcl::KdTreeFLANN<pcl::PointXYZ> > (new pcl::KdTreeFLANN<pcl::PointXYZ> ());

	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (border_cloud));
	ne.setRadiusSearch(normal_search_radius_);
	//ne.setKSearch(0);
	ne.compute(border_normals);

	//filter again to remove spurious NaNs
	pcl::PointIndices nan_indices;
	for (unsigned int i = 0; i < border_normals.points.size(); i++) {
		if (isnan(border_normals.points[i].normal[0]))
			nan_indices.indices.push_back(i);
	}
	ROS_INFO("%d NaNs found", (int)nan_indices.indices.size());
	//in pointcloud
	extract.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (border_cloud));
	extract.setIndices(boost::make_shared<pcl::PointIndices> (nan_indices));
	extract.setNegative(true);
	extract.filter(border_cloud);
	ROS_INFO("%d points in border cloud after filtering and NaN removal", (int)border_cloud.points.size());
	//and in the normals
	nextract.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::Normal> > (border_normals));
	nextract.setIndices(boost::make_shared<pcl::PointIndices> (nan_indices));
	nextract.setNegative(true);
	nextract.filter(border_normals);
	ROS_INFO("%d points in normals cloud after NaN removal", (int)border_normals.points.size());

	// set params: nr_dirs=8, d_min=1.0, d_max=4.0, grid_cell_size=0.2
	// create costmaps
	//    min,max x,y from input --> grid map size
	//    create vec<vec<vec<int> > > > .... 0
	// create vis. kernel from sensor model
	//    see notes
	//
	double pi=3.141;
	int nr_dirs=8;
	double d_min=1.0;
	double d_max=4.0;
	double grid_cell_size=0.1;
	geometry_msgs::Point min,max;
	min.x=(double)border_cloud.points[0].x;
	min.y=(double)border_cloud.points[0].y;
	max.x=min.x;
	max.y=min.y;
	for (int i=1;i<border_cloud.points.size();i++)
	{
		if (min.x > border_cloud.points[i].x) min.x = border_cloud.points[i].x;
		if (min.y > border_cloud.points[i].y) min.y = border_cloud.points[i].y;
		if (max.x < border_cloud.points[i].x) max.x = border_cloud.points[i].x;
		if (max.y < border_cloud.points[i].y) max.y = border_cloud.points[i].y;
	}
	double x_dim=(int)abs((max.x-min.x)/grid_cell_size)+1;
	double y_dim=(int)abs((max.y-min.y)/grid_cell_size)+1;
	ROS_INFO("X,Y min, max: %f, %f, %f, %f", min.x, min.y, max.x, max.y);
	ROS_INFO("X,Y DIMENSIONS OF THE COSTMAP: %f, %f",x_dim,y_dim);
	std::vector<std::vector<std::vector<int> > > costmap;
	costmap.resize(nr_dirs);
	for (int i = 0; i < nr_dirs; ++i) {
		costmap[i].resize(x_dim);

		for (int j = 0; j < x_dim; ++j)
			costmap[i][j].resize(y_dim);
	}
	std::vector<point_2d> vis_ker;
	double step_size=0.1;
	double degree_size=10.0*pi/180.0;
	double fi=pi;
	int ker_counter=0;
	point_2d ker_pt;
	for(double d=d_min;d<d_max;d+=step_size)
	{
		for (double fi2 = -pi/2; fi2 < pi/2; fi2 += degree_size)
		{
			ker_pt.x=d*cos(fi2);
			ker_pt.y=d*sin(fi2);
			vis_ker.push_back(ker_pt);
			ker_counter++;
		}
	}
	for (int dir=0;dir<nr_dirs;dir++)
	{
		double theta=dir*2*pi/nr_dirs;
		for (int i=0;i<border_cloud.points.size();i++)
		{
			for (int j=0;j<ker_counter;j++)
			{
				double x=(double)border_cloud.points[i].x+sin(theta)*vis_ker[j].x-cos(theta)*vis_ker[j].y;
				double y=(double)border_cloud.points[i].y+cos(theta)*vis_ker[j].x+sin(theta)*vis_ker[j].y;
				int id_x=(int)(x_dim*(x-min.x)/(max.x-min.x));
				int id_y=(int)(y_dim*(y-min.y)/(max.y-min.y));
				if (id_x >= 0 && id_x < x_dim &&
					id_y >= 0 && id_y < y_dim)
					costmap[dir][id_x][id_y]++;
			}
		}
	}
	int max_reward = 0;
	int best_i=0, best_j=0, best_k=0;
	for (int i = 0; i < nr_dirs; ++i)
	{
		std::stringstream ss;
		ss << "costmap_" << border_cloud.header.stamp << "_" << i;
		write_pgm (ss.str(), costmap[i]);
		for (int j = 0; j < x_dim; ++j)
			for (int k = 0; k < y_dim; k++)
			{
				int reward = costmap[i][j][k];
				if (reward > max_reward)
				{
					max_reward = reward;
					best_i = i;
					best_j = j;
					best_k = k;
				}
			}
	}
	nbv_pose_array_.header.frame_id = border_cloud.header.frame_id;
	nbv_pose_array_.header.stamp = ros::Time::now();
	nbv_pose_array_.poses.resize(0);
	geometry_msgs::Pose p;
	p.position.x = min.x + (best_j + 0.5) * grid_cell_size;
	p.position.y = min.y + (best_k + 0.5) * grid_cell_size;
	p.position.z = 0;
	btVector3 axis(0, 0, 1);
	btQuaternion quat (axis, best_i*2.0*pi/nr_dirs);
	ROS_INFO("BEST I,J,K::: %i %i %i (reward %i)", best_i, best_j, best_k, max_reward);
	ROS_INFO("BEST robot pose::: x,y = [%f , %f], theta = %f", p.position.x, p.position.y, best_i*2.0*pi/nr_dirs);
	geometry_msgs::Quaternion quat_msg;
	tf::quaternionTFToMsg(quat, quat_msg);
	p.orientation = quat_msg;
	nbv_pose_array_.poses.push_back(p);

	//computeKernel(180.0,d_min,d_max,0.1,10*pi/180);
	//ROS_INFO("THE NUMBER OF ELEMENTS IN THE VECTOR POINT %d",ker_counter);
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree2 =
			boost::shared_ptr <pcl::KdTreeFLANN<pcl::PointXYZ> > (new pcl::KdTreeFLANN<pcl::PointXYZ> ());

	tree2->setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (border_cloud));
	// Decompose a region of space into clusters based on the euclidean distance between points, and the normal
	std::vector<pcl::PointIndices> clusters;
	pcl::extractEuclideanClusters <pcl::PointXYZ, pcl::Normal> (border_cloud, border_normals, tolerance_, tree2, clusters, eps_angle_, min_pts_per_cluster_);
	//
	pcl::PointCloud<pcl::PointXYZ> cluster_clouds[3];
	//
	//  /*
	//   * compute boundary points from clusters and put them into pose array
	//   */
	//  computeBoundaryPoints(border_cloud, border_normals, clusters, cluster_clouds);
	///////////////////////////////////

	//publish poses
	pose_pub_.publish(nbv_pose_array_);

	//publish border cloud for visualization
	pcl::PointCloud<pcl::PointNormal> border_pn_cloud;
	if (border_cloud.points.size() > 0) {
		pcl::concatenateFields(border_cloud, border_normals, border_pn_cloud);
	} else {
		border_pn_cloud.header.frame_id = border_cloud.header.frame_id;
		border_pn_cloud.header.stamp = ros::Time::now();
	}
	border_cloud_pub_.publish(border_pn_cloud);

	//publish the clusters for visualization
	for (unsigned int i = 0; i < 3; i++) {
		if (cluster_clouds[i].points.size() == 0) {
			cluster_clouds[i].header.frame_id = border_cloud.header.frame_id;
			cluster_clouds[i].header.stamp = ros::Time::now();
		}
	}
	cluster_cloud_pub_.publish(cluster_clouds[0]);
	cluster_cloud2_pub_.publish(cluster_clouds[1]);
	cluster_cloud3_pub_.publish(cluster_clouds[2]);

	// publish binary octree
	if (0) {
		octomap_ros::OctomapBinary octree_msg;
		octomap_server::octomapMapToMsg(*octree_, octree_msg);
		octree_pub_.publish(octree_msg);
	}

	ROS_INFO("All computed and published");

	//**********************************************************************************
	//Visualization
	//**********************************************************************************
	if (visualize_octree_)
	{
		geometry_msgs::Point viewpoint;
		viewpoint.x = pt.x();
		viewpoint.y = pt.y();
		viewpoint.z = pt.z();
		visualizeOctree(pointcloud2_msg, viewpoint);
	}
}
/*void NextBestView::computeKernel(double fi,double d_min,double d_max,double step_size,double,degree_size)
{
	 std::vector<point_2d> vis_ker;
	    int ker_counter=0;
	    point_2d ker_pt;
	    for(double d=d_min;d<d_max;d+=step_size)
	    {
	    	for (double fi2=-90*pi/180;fi2<90*pi/180;fi2+=degree_size)
	    	{
	    	  ker_pt.x=d*sin(fi2);
	    	  ker_pt.y=d*cos(fi2);
	    	  vis_ker.push_back(ker_pt);
	    	  ker_counter++;
	    	}
	    }
}*/
void NextBestView::computeBoundaryPoints(pcl::PointCloud<pcl::PointXYZ>& border_cloud, pcl::PointCloud<pcl::Normal>& border_normals, std::vector<pcl::PointIndices>& clusters, pcl::PointCloud<pcl::PointXYZ>* cluster_clouds) {
	//clear old poses
	nbv_pose_array_.poses.clear();

	if (clusters.size() > 0) {
		ROS_INFO ("%d clusters found.", (int)clusters.size());
		// sort the clusters according to number of points they contain
		std::sort(clusters.begin(), clusters.end(), compareClusters);

		pcl::ExtractIndices<pcl::PointXYZ> extract;
		pcl::ExtractIndices<pcl::Normal> nextract;

		for (unsigned int nc = 0; nc < clusters.size(); nc++) {
			//extract maximum of 3 biggest clusters
			if (nc == 3)
				break;

			//extract a cluster
			pcl::PointCloud<pcl::PointXYZ> cluster_cloud;
			extract.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (border_cloud));
			extract.setIndices(boost::make_shared<pcl::PointIndices> (clusters.back()));
			extract.setNegative(false);
			extract.filter(cluster_cloud);
			ROS_INFO ("PointCloud representing the cluster %d: %d data points.", nc, cluster_cloud.width * cluster_cloud.height);

			//extract normals of cluster
			pcl::PointCloud<pcl::Normal> cluster_normals;
			nextract.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::Normal> > (border_normals));
			nextract.setIndices(boost::make_shared<pcl::PointIndices> (clusters.back()));
			nextract.setNegative(false);
			nextract.filter(cluster_normals);

			// find boundary points of cluster
			pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree3 = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
			pcl::PointCloud<pcl::Boundary> boundary_cloud;
			pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> be;
			be.setSearchMethod(tree3);
			be.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cluster_cloud));
			be.setInputNormals(boost::make_shared<pcl::PointCloud<pcl::Normal> > (cluster_normals));
			be.setRadiusSearch(.5);
			be.angle_threshold_ = boundary_angle_threshold_;
			be.compute(boundary_cloud);

			geometry_msgs::Pose nbv_pose;
			unsigned int nbp = 0;
			/*for (unsigned int i = 0; i < boundary_cloud.points.size(); ++i) {
				if (boundary_cloud.points[i].boundary_point) {
					nbv_pose.position.x = cluster_cloud.points[i].x;
					nbv_pose.position.y = cluster_cloud.points[i].y;
					nbv_pose.position.z = cluster_cloud.points[i].z;
					btVector3 axis(0, -cluster_normals.points[i].normal[2], cluster_normals.points[i].normal[1]);
					btQuaternion quat(axis, axis.length());
					geometry_msgs::Quaternion quat_msg;
					tf::quaternionTFToMsg(quat, quat_msg);
					nbv_pose.orientation = quat_msg;
					nbv_pose_array_.poses.push_back(nbv_pose);
					nbp++;
				}
			}*/
			geometry_msgs::Point minim,maxim,geometric_center;
			bool first_boundary = true;
			minim.x=(double)cluster_cloud.points[0].x;
			minim.y=(double)cluster_cloud.points[0].y;
			minim.z=(double)cluster_cloud.points[0].z;
			maxim.x=minim.x;
			maxim.y=minim.y;
			maxim.z=minim.z;
			for (unsigned int i=1; i<boundary_cloud.points.size();++i)
			{
				if (boundary_cloud.points[i].boundary_point)
				{
					if (first_boundary)
					{
						minim.x=(double)cluster_cloud.points[i].x;
						minim.y=(double)cluster_cloud.points[i].y;
						minim.z=(double)cluster_cloud.points[i].z;
						maxim.x=minim.x;
						maxim.y=minim.y;
						maxim.z=minim.z;
						first_boundary = false;
					}
					else
					{
						if (cluster_cloud.points[i].x<minim.x)
							minim.x=(double)cluster_cloud.points[i].x;
						if (cluster_cloud.points[i].y<minim.y)
							minim.y=(double)cluster_cloud.points[i].y;
						if (cluster_cloud.points[i].z<minim.z)
							minim.z=(double)cluster_cloud.points[i].z;
						if (cluster_cloud.points[i].x>maxim.x)
							maxim.x=(double)cluster_cloud.points[i].x;
						if (cluster_cloud.points[i].y>maxim.y)
							maxim.y=(double)cluster_cloud.points[i].y;
						if (cluster_cloud.points[i].z>maxim.z)
							maxim.z=(double)cluster_cloud.points[i].z;
					}
				}
			}
			geometric_center.x=minim.x+(maxim.x-minim.x)/2.0;
			geometric_center.y=minim.y+(maxim.y-minim.y)/2.0;
			geometric_center.z=minim.z+(maxim.z-minim.z)/2.0;
			ROS_INFO("the coordinates of the geometric center are %f, %f, %f",geometric_center.x,geometric_center.y,geometric_center.z);
			pcl::PointCloud<pcl::PointXYZ> cluster_cloud_2,cluster_cloud_final;
			cluster_cloud_2.header = boundary_cloud.header;
			cluster_cloud_2.points.resize(boundary_cloud.points.size());
			cluster_cloud_final.header = boundary_cloud.header;
			cluster_cloud_final.points.resize(boundary_cloud.points.size());
			double fi,dist;
			double dx, dy;
			double factor = 0.1;

			for (unsigned i=0; i< boundary_cloud.points.size(); ++i)
			{

				if (boundary_cloud.points[i].boundary_point)
				{
					fi=atan2(((double)cluster_cloud.points[i].x-geometric_center.x),
							((double)cluster_cloud.points[i].y-geometric_center.y));
					dist=abs((((double)cluster_cloud.points[i].x-geometric_center.x) / sin(fi)));
					dx = (double)cluster_cloud.points[i].x-geometric_center.x;
					dy = (double)cluster_cloud.points[i].y-geometric_center.y;
					//todo? normalize(dx,dy) to unit length, then factor is in meters..
							cluster_cloud_2.points[i].x = cluster_cloud.points[i].x + factor * dx; //(geometric_center.x+factor*dist*sin(fi));
							cluster_cloud_2.points[i].y = cluster_cloud.points[i].y + factor * dy; //(geometric_center.y+factor*dist*cos(fi));
				}
			}
			//ROS_INFO("TEST MESAGE %f",cluster_cloud_2.points[3].x);
			int pt_counter = 0;
			geometry_msgs::Point d,d_norm;
			for (unsigned int i=0; i<boundary_cloud.points.size();++i)
			{
				if (boundary_cloud.points[i].boundary_point)
				{
					octomap::OcTreeNodePCL * free_node = octree_->search(cluster_cloud_2.points[i].x,cluster_cloud_2.points[i].y,cluster_cloud_2.points[i].y);
					if ((free_node != NULL) && (free_node->getLabel() != occupied_label_))
					{

						nbv_pose.position.x = cluster_cloud_2.points[i].x;
						nbv_pose.position.y = cluster_cloud_2.points[i].y;
						nbv_pose.position.z = cluster_cloud_2.points[i].z;
						cluster_cloud_final.points[pt_counter].x=cluster_cloud_2.points[i].x;
						cluster_cloud_final.points[pt_counter].y=cluster_cloud_2.points[i].y;
						cluster_cloud_final.points[pt_counter].z=cluster_cloud_2.points[i].z;
						pt_counter++;
						//btVector3 axis(0, -cluster_normals.points[i].normal[2], cluster_normals.points[i].normal[1]);
						d.x=cluster_cloud_2.points[i].x-geometric_center.x;
						d.y=cluster_cloud_2.points[i].y-geometric_center.y;
						d.z=cluster_cloud_2.points[i].z-geometric_center.z;
						double norm=sqrt(pow(d.x,2)+pow(d.y,2)+pow(d.z,2));
						d_norm.x=d.x/norm;
						d_norm.y=d.y/norm;
						d_norm.z=d.z/norm;
						btVector3 axis(0, 0, 1);
						btQuaternion quat (axis, acos(d_norm.x));
						//ROS_INFO("ANGLE IS::: %f",cluster_cloud_2.points[i].z-geometric_center.z);
						geometry_msgs::Quaternion quat_msg;
						tf::quaternionTFToMsg(quat, quat_msg);
						nbv_pose.orientation = quat_msg;
						nbv_pose_array_.poses.push_back(nbv_pose);
						nbp++;
					}
				}
			}
			ROS_INFO ("%d boundary points in cluster %d.", nbp, nc);
			cluster_cloud_final.points.resize (pt_counter);
			//save this cluster pointcloud for visualization
			cluster_clouds[nc] = cluster_cloud_final;
			//pop the just used cluster from indices
			clusters.pop_back();
		}
		nbv_pose_array_.header.frame_id = border_cloud.header.frame_id;
		nbv_pose_array_.header.stamp = ros::Time::now();
	}
	else {
		ROS_INFO ("No clusters found!");
	}
}

void NextBestView::castRayAndLabel(pcl::PointCloud<pcl::PointXYZ>& cloud, octomap::pose6d origin) {
	octomap::point3d octomap_point3d;

	BOOST_FOREACH (const pcl::PointXYZ& pcl_pt, cloud.points) {
		octomap_point3d(0) = pcl_pt.x;
		octomap_point3d(1) = pcl_pt.y;
		octomap_point3d(2) = pcl_pt.z;
		octomap::OcTreeNodePCL * octree_end_node = octree_->search(octomap_point3d);
		if (octree_end_node != NULL) {
			// Get the nodes along the ray and label them as free
			if (octree_->computeRayKeys(origin.trans(), octomap_point3d, ray)) {
				for(octomap::KeyRay::iterator it=ray.begin(); it != ray.end(); it++) {
					octomap::OcTreeNodePCL * free_node = octree_->search(*it);
					if (free_node != NULL) {
						if (free_node->getLabel() != occupied_label_)
						{
							free_node->setLabel(free_label_);

						}
					}

					else
						ROS_DEBUG("node in ray not found!");
				}
			}
			else {
				ROS_DEBUG("could not compute ray from [%f %f %f] to [%f %f %f]", origin.x(), origin.y(), origin.z(), pcl_pt.x, pcl_pt.y, pcl_pt.z);
			}
			octree_end_node->set3DPointInliers(0);
			octree_end_node->setLabel(occupied_label_);
		}
		else {
			ROS_DEBUG("ERROR: node at [%f %f %f] not found", pcl_pt.x, pcl_pt.y, pcl_pt.z);
		}
	}




}

void NextBestView::findBorderPoints(pcl::PointCloud<pcl::PointXYZ>& border_cloud, std::string frame_id) {
	border_cloud.header.frame_id = frame_id;
	border_cloud.header.stamp = ros::Time::now();
	std::list<octomap::OcTreeVolume> leaves;
	octree_->getLeafNodes(leaves);
	BOOST_FOREACH(octomap::OcTreeVolume vol, leaves) {
		octomap::point3d centroid;
		centroid(0) = vol.first.x(),  centroid(1) = vol.first.y(),  centroid(2) = vol.first.z();
		octomap::OcTreeNodePCL *octree_node = octree_->search(centroid);

		// if free voxel -> check for unknown neighbors
		if (octree_node != NULL && octree_node->getLabel() == free_label_) {
			for (int i=0; i<3; i++) {
				octomap::point3d neighbor_centroid = centroid;
				for (int j=-1; j<2; j+=2) {
					neighbor_centroid(i) += j * octree_res_;
					octomap::OcTreeNodePCL *neighbor = octree_->search(neighbor_centroid);
					if (neighbor != NULL && neighbor->getLabel() == unknown_label_) {
						// add to list of border voxels
						pcl::PointXYZ border_pt (centroid.x(), centroid.y(), centroid.z());
						border_cloud.points.push_back(border_pt);
						break;
					}
				}
			}
		}
	}
	ROS_INFO("%d points in border cloud", (int)border_cloud.points.size());
}

/**
 * creating an octree from pcl data
 */
void NextBestView::createOctree (pcl::PointCloud<pcl::PointXYZ>& pointcloud2_pcl, octomap::pose6d laser_pose) {

	octomap::point3d octomap_3d_point;
	octomap::Pointcloud octomap_pointcloud;

	//Reading from pcl point cloud and saving it into octomap point cloud
	BOOST_FOREACH (const pcl::PointXYZ& pt, pointcloud2_pcl.points) {
		octomap_3d_point(0) = pt.x;
		octomap_3d_point(1) = pt.y;
		octomap_3d_point(2) = pt.z;
		octomap_pointcloud.push_back(octomap_3d_point);
	}

	// Converting from octomap point cloud to octomap graph
	octomap_pointcloud.transform(laser_pose.inv());
	octomap::ScanNode* scan_node = octomap_graph_->addNode(&octomap_pointcloud, laser_pose);

	ROS_INFO("Number of points in scene graph: %d", octomap_graph_->getNumPoints());

	// Converting from octomap graph to octomap tree (octree)
	octree_->insertScan(*scan_node, octree_maxrange_, false);

	octree_->expand();

	//
	// create nodes that are unknown
	//
	//octomap::point3d min, max;
	double min[3], max[3];
	octree_->getMetricMin(min[0], min[1], min[2]);
	octree_->getMetricMax(max[0], max[1], max[2]);
	//ROS_DEBUG("octree min bounds [%f %f %f]", min(0), min(1), min(2));
	//ROS_DEBUG("octree max bounds [%f %f %f]", max(0), max(1), max(2));

	double x,y,z;
	for (x = min[0]+octree_res_/2; x < max[0]-octree_res_/2; x+=octree_res_) {
		for (y = min[1]+octree_res_/2; y < max[1]-octree_res_/2; y+=octree_res_) {
			for (z = min[2]+octree_res_/2; z < max[2]-octree_res_/2; z+=octree_res_) {
				octomap::point3d centroid (x, y, z);
				if (z > max[2])
					ROS_INFO("ahrg node at [%f %f %f]", centroid(0), centroid(1), centroid(2));
				octomap::OcTreeNodePCL *octree_node = octree_->search(centroid);
				if (octree_node != NULL) {
					octree_node->setCentroid(centroid);
				} else {
					//ROS_INFO("creating node at [%f %f %f]", centroid(0), centroid(1), centroid(2));
					//corresponding node doesn't exist yet -> create it
					octomap::OcTreeNodePCL *new_node = octree_->updateNode(centroid, false);
					new_node->setCentroid(centroid);
					new_node->setLabel(unknown_label_);
				}
			}
		}
	}


	if (check_centroids_) {
		//int cnt = 0;
		// get all existing leaves
		std::list<octomap::OcTreeVolume> leaves;
		//octree_->getLeafNodes(leaves);

		//find Leaf Nodes' centroids, assign controid coordinates to Leaf Node
		BOOST_FOREACH(octomap::OcTreeVolume vol, leaves) {
			//ROS_DEBUG("Leaf Node %d : x = %f y = %f z = %f side length = %f ", cnt++, it1->first.x(), it1->first.y(), it1->first.z(), it1->second);
			octomap::point3d centroid;
			centroid(0) = vol.first.x(), centroid(1) = vol.first.y(), centroid(2) = vol.first.z();
			octomap::OcTreeNodePCL *octree_node = octree_->search(centroid);
			if (octree_node != NULL) {
				octomap::point3d test_centroid;
				test_centroid = octree_node->getCentroid();
				if (centroid.distance(test_centroid) > octree_res_/4)
					ROS_INFO("node at [%f %f %f] has a wrong centroid: [%f %f %f]", centroid(0), centroid(1), centroid(2), test_centroid(0), test_centroid(1), test_centroid(2));
			}
			else {
				ROS_INFO("node at [%f %f %f] not found", centroid(0), centroid(1), centroid(2));
			}
		}
	}

}


void NextBestView::visualizeOctree(const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg, geometry_msgs::Point viewpoint) {
	// each array stores all cubes of a different size, one for each depth level:
	octree_marker_array_msg_.markers.resize(4);
	double lowestRes = octree_->getResolution();
	//ROS_INFO_STREAM("lowest resolution: " << lowestRes);

	std::list<octomap::OcTreeVolume> all_cells;
	//getting the cells at level 0
	octree_->getLeafNodes(all_cells, 0);
	BOOST_FOREACH(octomap::OcTreeVolume vol, all_cells)
	{
		geometry_msgs::Point cube_center;
		cube_center.x = vol.first.x();
		cube_center.y = vol.first.y();
		cube_center.z = vol.first.z();
		octomap::point3d octo_point (cube_center.x, cube_center.y, cube_center.z);
		octomap::OcTreeNodePCL * node = octree_->search(octo_point);
		if (node != NULL) {
			if (occupied_label_ == node->getLabel())
				octree_marker_array_msg_.markers[0].points.push_back(cube_center);
			else if (free_label_ == node->getLabel())
				octree_marker_array_msg_.markers[1].points.push_back(cube_center);
			else if (unknown_label_ == node->getLabel())
				octree_marker_array_msg_.markers[2].points.push_back(cube_center);
		}
	}

	octree_marker_array_msg_.markers[3].points.push_back(viewpoint);

	// occupied cells
	octree_marker_array_msg_.markers[0].ns = "Occupied cells";
	octree_marker_array_msg_.markers[0].color.r = 1.0f;
	octree_marker_array_msg_.markers[0].color.g = 0.0f;
	octree_marker_array_msg_.markers[0].color.b = 0.0f;
	octree_marker_array_msg_.markers[0].color.a = 0.5f;

	// free cells
	octree_marker_array_msg_.markers[1].ns ="Free cells";
	octree_marker_array_msg_.markers[1].color.r = 0.0f;
	octree_marker_array_msg_.markers[1].color.g = 1.0f;
	octree_marker_array_msg_.markers[1].color.b = 0.0f;
	octree_marker_array_msg_.markers[1].color.a = 0.5f;

	// unknown cells
	octree_marker_array_msg_.markers[2].ns = "Unknown cells";
	octree_marker_array_msg_.markers[2].color.r = 0.0f;
	octree_marker_array_msg_.markers[2].color.g = 0.0f;
	octree_marker_array_msg_.markers[2].color.b = 1.0f;
	octree_marker_array_msg_.markers[2].color.a = 0.05f;

	// viewpoint
	octree_marker_array_msg_.markers[3].ns = "viewpoint";
	octree_marker_array_msg_.markers[3].color.r = 1.0f;
	octree_marker_array_msg_.markers[3].color.g = 1.0f;
	octree_marker_array_msg_.markers[3].color.b = 0.0f;
	octree_marker_array_msg_.markers[3].color.a = 0.8f;

	for (unsigned i = 0; i < octree_marker_array_msg_.markers.size(); ++i)
	{
		octree_marker_array_msg_.markers[i].header.frame_id = pointcloud2_msg->header.frame_id;
		octree_marker_array_msg_.markers[i].header.stamp = ros::Time::now();
		octree_marker_array_msg_.markers[i].id = i;
		octree_marker_array_msg_.markers[i].lifetime = ros::Duration::Duration();
		octree_marker_array_msg_.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
		octree_marker_array_msg_.markers[i].scale.x = lowestRes;
		octree_marker_array_msg_.markers[i].scale.y = lowestRes;
		octree_marker_array_msg_.markers[i].scale.z = lowestRes;

		if (octree_marker_array_msg_.markers[i].points.size() > 0)
			octree_marker_array_msg_.markers[i].action = visualization_msgs::Marker::ADD;
		else
			octree_marker_array_msg_.markers[i].action = visualization_msgs::Marker::DELETE;
	}

	octree_marker_array_publisher_.publish(octree_marker_array_msg_);

	for (unsigned int i = 0; i < octree_marker_array_msg_.markers.size(); i++)
	{
		if (!octree_marker_array_msg_.markers[i].points.empty())
		{
			octree_marker_array_msg_.markers[i].points.clear();
		}
	}
	octree_marker_array_msg_.markers.clear();
}
}

PLUGINLIB_DECLARE_CLASS(autonomous_mapping, NextBestView, autonomous_mapping::NextBestView, nodelet::Nodelet);
