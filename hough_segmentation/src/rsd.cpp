#include <algorithm>
#include "terminal_tools/parse.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/features/rsd.h"
#include "pcl/features/feature.h"
#include "pcl/features/normal_3d.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_cloud_algos/pcl_cloud_algos_point_types.h"

using namespace terminal_tools;

// Needed for normal estimation
typedef pcl::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;
typedef pcl::KdTree<pcl::PointNormal>::Ptr KdTreePtr2;

// Needed for time profiling
double t1,t2;
double my_clock()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + (double)tv.tv_usec*1e-6;
}

int main( int argc, char** argv )
{
  // Parameter parsing
  if( argc < 2 )
  {
    std::cerr << "Need one parameter! Syntax is: " << argv [0] << " {input_point_cloud_filename.pcd}\n" << std::endl;
    return(-1);
  }

  // Get position of dot in path of file 
  std::string file = argv [1];
  size_t dot = file.find (".");

  // Create names for saving pcd files
  std::string for_normals = argv [1];
  for_normals.insert (dot, "-normals");

  std::string for_radii = argv [1];
  for_radii.insert (dot, "-radii");

  // Radius search, shall be approximately the same as downsample_leaf (voxel size)
  double rsd_radius_search = 0.001;
  parse_argument (argc, argv, "-rsd", rsd_radius_search);  
  // We use radius search-based normal estimation
  double normals_radius_search = 0.001;
  parse_argument (argc, argv, "-normals", normals_radius_search);
  // Voxel size
  double downsample_leaf = 0.001;
  parse_argument (argc, argv, "-leaf", downsample_leaf);
  std::cerr << "rsd: " << rsd_radius_search << " normals: " << normals_radius_search <<  " leaf: " << downsample_leaf << std::endl;

  // Read input cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  reader.read ( argv[1], *input_cloud);

  // Estimate normals
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n3d; 
  n3d.setInputCloud (input_cloud);
  n3d.setRadiusSearch (normals_radius_search);
  KdTreePtr normals_tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
  n3d.setSearchMethod (normals_tree);
  t1 = my_clock();
  n3d.compute (*cloud_normals);
  std::cerr << "Normal compute done in %f seconds." <<  my_clock()-t1 << std::endl;

  // Concatenate XYZ + Normal fields
  pcl::PointCloud<pcl::PointNormal> cloud_xyz_normals;
  pcl::concatenateFields (*input_cloud, *cloud_normals, cloud_xyz_normals);
  writer.write(for_normals, cloud_xyz_normals, true);

  // Create the voxel grid
  pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud (new pcl::PointCloud<pcl::PointNormal> (cloud_xyz_normals));
  pcl::PointCloud<pcl::PointNormal> cloud_downsampled;
  pcl::VoxelGrid<pcl::PointNormal> grid;
  grid.setLeafSize (downsample_leaf, downsample_leaf, downsample_leaf);
  grid.setInputCloud (cloud);
  grid.setSaveLeafLayout(true);
  grid.filter (cloud_downsampled);
  pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud_downsampled_ptr;
  cloud_downsampled_ptr.reset (new pcl::PointCloud<pcl::PointNormal> (cloud_downsampled));

  // Compute RSD, the result is saved as PrincipalRadiiRSD point type
  pcl::RSDEstimation <pcl::PointNormal, pcl::PointNormal, pcl::PrincipalRadiiRSD> rsd;
  rsd.setInputCloud(cloud_downsampled_ptr);
  rsd.setSearchSurface(cloud);
  rsd.setInputNormals(cloud);
  // Refine the RSD radius search to max(rsd_radius_search, downsample_leaf/2 * sqrt(3))
  // This value must make sure that all points inside one voxel are taken into account
  std::cerr << "radius search: " << std::max(rsd_radius_search, downsample_leaf/2 * sqrt(3)) << std::endl;
  rsd.setRadiusSearch(std::max(rsd_radius_search, downsample_leaf/2 * sqrt(3)));

  KdTreePtr2 rsd_tree (new pcl::KdTreeFLANN<pcl::PointNormal> ());
  rsd_tree->setInputCloud (cloud);
  rsd.setSearchMethod(rsd_tree);
  pcl::PointCloud<pcl::PrincipalRadiiRSD> radii;
  t1 = my_clock();
  rsd.compute(radii);
  std::cerr << "RSD compute done in " << my_clock()-t1 << "seconds." << std::endl;

  // Concatenate radii values with the fields of the downsampled cloud and save as pcd
  pcl::PointCloud<pcl::PointNormalRADII> cloud_downsampled_radii;
  pcl::concatenateFields (cloud_downsampled, radii, cloud_downsampled_radii);
  writer.write(for_radii, cloud_downsampled_radii, false);

  return(0);
}
