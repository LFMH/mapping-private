#include <sys/time.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/io/pcd_io.h>
#include "pcl/filters/voxel_grid.h"
#include "pcl/features/rsd.h"
#include <algorithm>
#include <pcl_cloud_algos/pcl_cloud_algos_point_types.h>
#include <pcl/features/normal_3d.h>
#include <terminal_tools/parse.h>
#include <pcl/surface/mls.h>

typedef pcl::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;
using namespace terminal_tools;

//* time
double t1,t2;
double my_clock()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + (double)tv.tv_usec*1e-6;
}

#define NR_CLASS 5 
#define NOISE 0 
#define PLANE 1 
#define CYLINDER 2
#define SPHERE 3  
#define EDGE 4 
#define EMPTY 5 

#define NR_DIV 7 // number of normal angle divisions

double min_radius_plane_;
double min_radius_edge_;
double min_radius_noise_, max_radius_noise_;
double max_min_radius_diff_;

int get_type (float min_radius, float max_radius)
{    
  //               0.066
  if (min_radius > min_radius_plane_) 
    return PLANE; 
  //                     0.030                               0.050
  else if ((min_radius < min_radius_noise_) && (max_radius < max_radius_noise_))
    return NOISE; 
  //                                 0.02
  else if (max_radius - min_radius < max_min_radius_diff_) 
    return SPHERE;
  //                    0.030
  else if (min_radius < min_radius_edge_) 
    return EDGE;
  else
    return CYLINDER;
}

int main( int argc, char** argv )
{
  if( argc < 2 )
  {
    ROS_ERROR ("Need one parameter! Syntax is: %s {input_pointcloud_filename.pcd}\n", argv[0]);
    return(-1);
  }
  min_radius_plane_ = 0.066;
  min_radius_noise_ = 0.030, max_radius_noise_ = 0.050;
  max_min_radius_diff_ = 0.02;
  min_radius_edge_ = 0.030;
  bool save_to_disk = true;
  
  double rsd_radius_search = 0.01;
  parse_argument (argc, argv, "-rsd", rsd_radius_search);
  
  double normals_radius_search = 0.02;
  parse_argument (argc, argv, "-normals", normals_radius_search);
  
  // 1cm voxel size by default
  double downsample_leaf = 0.01;                          
  parse_argument (argc, argv, "-leaf", downsample_leaf);
  ROS_INFO("rsd %f, normals %f, leaf %f", rsd_radius_search, normals_radius_search, downsample_leaf);

  // read input cloud
  pcl::PointCloud<pcl::PointXYZ> input_cloud;
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  reader.read ( argv[1], input_cloud);
  //pcl::io::savePCDFile ("hogehoge.pcd",input_cloud,true);
  
  //Normal estimation
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n3d; 
  pcl::PointCloud<pcl::Normal> cloud_normals;
  n3d.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (input_cloud));
  //n3d.setKSearch (10);
  n3d.setRadiusSearch (normals_radius_search);
  KdTreePtr normals_tree;
  normals_tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
  n3d.setSearchMethod (normals_tree);
  t1 = my_clock();
  n3d.compute (cloud_normals);
  ROS_INFO("Normal compute done in %f seconds.", my_clock()-t1);

    // mls (for testing)
//   pcl::PointCloud<pcl::PointNormal> cloud_normals;
//   pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
//   mls.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (input_cloud));
// //  mls.setIndices (boost::make_shared <vector<int> > (indices));
//   mls.setPolynomialFit (false);
//   KdTreePtr normals_tree;
//   normals_tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
//   mls.setSearchMethod (normals_tree);
//   mls.setSearchRadius (normals_radius_search);
//   mls.reconstruct (cloud_normals);



  pcl::PointCloud<pcl::PointNormal> cloud_xyz_normals;
  pcl::concatenateFields (input_cloud, cloud_normals, cloud_xyz_normals);
  if (save_to_disk)
    writer.write("normals.pcd", cloud_xyz_normals, true);

  // create the voxel grid
  pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud = boost::make_shared<const pcl::PointCloud<pcl::PointNormal> > (cloud_xyz_normals);
  pcl::PointCloud<pcl::PointNormal> cloud_downsampled;
  pcl::VoxelGrid<pcl::PointNormal> grid;
  grid.setLeafSize (downsample_leaf, downsample_leaf, downsample_leaf);
  grid.setInputCloud (cloud);
  grid.setSaveLeafLayout(true);
  grid.filter (cloud_downsampled);
  pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud_downsampled_ptr;
  cloud_downsampled_ptr.reset (new pcl::PointCloud<pcl::PointNormal> (cloud_downsampled));
  

  // Compute RSD
  pcl::RSDEstimation <pcl::PointNormal, pcl::PointNormal, pcl::PrincipalRadiiRSD> rsd;
  rsd.setInputCloud(cloud_downsampled_ptr);
  rsd.setSearchSurface(cloud);
  rsd.setInputNormals(cloud);
  ROS_INFO("radius search: %f", std::max(rsd_radius_search, downsample_leaf/2 * sqrt(3)));
  rsd.setRadiusSearch(std::max(rsd_radius_search, downsample_leaf/2 * sqrt(3)));
  pcl::KdTree<pcl::PointNormal>::Ptr tree2 = boost::make_shared<pcl::KdTreeFLANN<pcl::PointNormal> > ();
  tree2->setInputCloud (cloud);
  rsd.setSearchMethod(tree2);
  pcl::PointCloud<pcl::PrincipalRadiiRSD> radii;
  t1 = my_clock();
  rsd.compute(radii);
  ROS_INFO("RSD compute done in %f seconds.", my_clock()-t1);

  pcl::PointCloud<pcl::PointNormalRADII> cloud_downsampled_radii;
  pcl::concatenateFields (cloud_downsampled, radii, cloud_downsampled_radii);

  if (save_to_disk)
    writer.write("radii.pcd", cloud_downsampled_radii, false);
  
  t1 = my_clock();
  // Get rmin/rmax for adjacent 27 voxel
  Eigen::MatrixXi relative_coordinates (3, 13);

  Eigen::MatrixXi transition_matrix =  Eigen::MatrixXi::Zero(6, 6);

  int idx = 0;
  
  // 0 - 8
  for( int i=-1; i<2; i++ )
  {
    for( int j=-1; j<2; j++ )
    {
      relative_coordinates( 0, idx ) = i;
      relative_coordinates( 1, idx ) = j;
      relative_coordinates( 2, idx ) = -1;
      idx++;
    }
  }
  // 9 - 11
  for( int i=-1; i<2; i++ )
  {
    relative_coordinates( 0, idx ) = i;
    relative_coordinates( 1, idx ) = -1;
    relative_coordinates( 2, idx ) = 0;
    idx++;
  }
  // 12
  relative_coordinates( 0, idx ) = -1;
  relative_coordinates( 1, idx ) = 0;
  relative_coordinates( 2, idx ) = 0;

  Eigen::MatrixXi relative_coordinates_all (3, 26);
  relative_coordinates_all.block<3, 13>(0, 0) = relative_coordinates;
  relative_coordinates_all.block<3, 13>(0, 13) = -relative_coordinates;
  
  // Get transition matrix
  std::vector<int> types (radii.points.size());
 
  for (size_t idx = 0; idx < radii.points.size (); ++idx)
    types[idx] = get_type(radii.points[idx].r_min, radii.points[idx].r_max);
  
  // TODO voxelization does not re-normalize the normals!
  for (size_t idx = 0; idx < cloud_downsampled_ptr->points.size (); ++idx)
    cloud_downsampled.points[idx].getNormalVector3fMap ().normalize ();

  // TODO use consts and iterators
  std::vector<Eigen::MatrixXi> transition_matrix_list (NR_DIV, Eigen::MatrixXi::Zero (NR_CLASS, NR_CLASS));
  Eigen::VectorXi transitions_to_empty = Eigen::VectorXi::Zero (NR_CLASS);
//  double unit_angle = (M_PI/2) / NR_DIV;
  for (size_t idx = 0; idx < cloud_downsampled_ptr->points.size (); ++idx)
  {
    int source_type = types[idx];
    std::vector<int> neighbors = grid.getNeighborCentroidIndices (cloud_downsampled_ptr->points[idx], relative_coordinates_all);
    Eigen::Vector3f source_normal = cloud_downsampled.points[idx].getNormalVector3fMap ();
    for (unsigned id_n = 0; id_n < neighbors.size(); id_n++)
    {
      // TODO merge into PlusGRSD computation code
      int neighbor_type;
      if (neighbors[id_n] == -1)
        neighbor_type = EMPTY;
      else
        neighbor_type = types[neighbors[id_n]];

      // TODO remove
      transition_matrix(source_type, neighbor_type)++;

      // count transitions: TODO optimize Asako style :)  but what about empty?
      if (neighbors[id_n] == -1)
        transitions_to_empty(source_type)++;
      else
      {
        // compute angle between average normals of voxels
        assert (source_type != EMPTY);
//        double sine = source_normal.cross (cloud_downsampled.points[neighbors[id_n]].getNormalVector3fMap ()).norm ();
//        int angle_bin = std::min (NR_DIV-1, (int) floor (sine * NR_DIV));
//        transition_matrix_list[angle_bin](source_type, neighbor_type)++;
        transition_matrix_list[std::min (NR_DIV-1, (int) floor (sqrt (source_normal.cross (cloud_downsampled.points[neighbors[id_n]].getNormalVector3fMap ()).norm ()) * NR_DIV))](source_type, neighbor_type)++;
//        double unsigned_cosine = fabs(source_normal.dot (cloud_downsampled.points[neighbors[id_n]].getNormalVector3fMap ()));
//        if (unsigned_cosine > 1) unsigned_cosine = 1;
//        int angle_bin = std::min (NR_DIV-1, (int) floor (acos (unsigned_cosine) / unit_angle));
        //std::cerr << "angle difference = " << acos (unsigned_cosine) << "(" << unsigned_cosine << ") => index: " << angle_bin << std::endl;
      }
    }
  }
  std::cerr << "List of transition matrices by normal angle:" << std::endl;
  std::copy (transition_matrix_list.begin (), transition_matrix_list.end (), std::ostream_iterator<Eigen::MatrixXi> (std::cerr, "\n---\n"));
  std::cerr << "Transitions to empty: " << transitions_to_empty.transpose () << std::endl;

  int nrf = 0;
  pcl::PointCloud<pcl::PlusGRSDSignature110> cloud_grsd;
  cloud_grsd.points.resize(1);
  
  //std::cerr << "Number of transitions by normal angle:";
  for (std::vector<Eigen::MatrixXi>::const_iterator it = transition_matrix_list.begin ();
      it != transition_matrix_list.end (); ++it)
  {
    //std::cerr << " " << (*it).sum ();
    for (int i=0; i<NR_CLASS; i++)
      for (int j=i; j<NR_CLASS; j++)
        cloud_grsd.points[0].histogram[nrf++] = (*it)(i, j);
  }
  for (int it = 0; it < transitions_to_empty.rows (); ++it)
    cloud_grsd.points[0].histogram[nrf++] = transitions_to_empty[it];
  //std::cerr << std::endl;
  std::cerr << "PlusGRSDS: " << cloud_grsd.points[0] << std::endl;

  if (save_to_disk)
    writer.write("grsd.pcd", cloud_grsd, false);
  std::cerr << "Complete (old) transition matrix" << std::endl << transition_matrix << std::endl;
  ROS_INFO("GRSD compute done in %f seconds.", my_clock()-t1);
  return(0);
}
