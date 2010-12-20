#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <color_chlac/color_chlac.h>

using namespace pcl;

template <typename T>
bool readPoints( const char *name, T& cloud_object_cluster ){
  if (pcl::io::loadPCDFile (name, cloud_object_cluster) == -1){
    ROS_ERROR ("Couldn't read file %s",name);
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int)(cloud_object_cluster.width * cloud_object_cluster.height), name, pcl::getFieldsList (cloud_object_cluster).c_str ());
  return(1);
}

//--------------------------------------------------------------------------------------------
// function1
bool writeVFHsignature( const char *name, pcl::PointCloud<pcl::PointXYZ> cloud_object_cluster ){
  // compute normals
  pcl::PointCloud<pcl::Normal>::Ptr normals = boost::make_shared<pcl::PointCloud<pcl::Normal> >();
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n3d;
  n3d.setKSearch (30); // have to use the same parameters as during training
  pcl::KdTree<pcl::PointXYZ>::Ptr tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
  n3d.setSearchMethod (tree);
  n3d.setInputCloud(cloud_object_cluster.makeShared ());
  n3d.compute(*normals);
  
  // compute vfh
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setSearchMethod (tree);
  vfh.setInputCloud(cloud_object_cluster.makeShared ());
  vfh.setInputNormals(normals);
  pcl::PointCloud<pcl::VFHSignature308> vfh_signature;
  vfh.compute(vfh_signature);

  // save vfh
  pcl::io::savePCDFile (name, vfh_signature);

  return(1);
}

//--------------------------------------------------------------------------------------------
// function2
bool writeColorCHLAC(const char *name, pcl::PointCloud<pcl::PointXYZRGB> cloud_object_cluster ){
  // ---[ Create the voxel grid
  pcl::PointCloud<PointXYZRGB> cloud_downsampled;
  pcl::VoxelGrid<PointXYZRGB> grid_;
  float voxel_size;
  FILE *fp = fopen( "voxel_size.txt", "r" );
  fscanf( fp, "%f\n", &voxel_size );
  fclose(fp);
  grid_.setLeafSize (voxel_size, voxel_size, voxel_size);

  grid_.setInputCloud (boost::make_shared<const pcl::PointCloud<PointXYZRGB> > (cloud_object_cluster));
  grid_.setSaveLeafLayout(true);
  grid_.filter (cloud_downsampled);
  pcl::PointCloud<PointXYZRGB>::ConstPtr cloud_downsampled_;
  cloud_downsampled_.reset (new pcl::PointCloud<PointXYZRGB> (cloud_downsampled));

  // color threshold
  int thR, thG, thB;
  fp = fopen( "color_threshold.txt", "r" );
  fscanf( fp, "%d %d %d\n", &thR, &thG, &thB );
  fclose(fp);

  // ---[ Compute ColorCHLAC
  pcl::PointCloud<ColorCHLACSignature981> colorCHLAC_signature;
  pcl::ColorCHLACEstimation<PointXYZRGB> colorCHLAC_;
  KdTree<PointXYZRGB>::Ptr normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointXYZRGB> > ();
  colorCHLAC_.setRadiusSearch (1.8);
  colorCHLAC_.setSearchMethod (normals_tree_);
  colorCHLAC_.setColorThreshold( thR, thG, thB );
  colorCHLAC_.setVoxelFilter (grid_);
  colorCHLAC_.setInputCloud (cloud_downsampled_);
  colorCHLAC_.compute( colorCHLAC_signature );

  // save colorCHLAC
  fp = fopen( name, "w" );
  fprintf(fp,"# .PCD v.? - Point Cloud Data file format\n");
  fprintf(fp,"FIELDS colorCHLAC\n");
  fprintf(fp,"SIZE 4\n");
  fprintf(fp,"TYPE F\n");
  fprintf(fp,"COUNT %d\n",DIM_COLOR_BIN_1_3+DIM_COLOR_1_3);
  fprintf(fp,"WIDTH 1\n");
  fprintf(fp,"HEIGHT 1\n");
  fprintf(fp,"POINTS 1\n");
  fprintf(fp,"DATA ascii\n");
  for( int t=0; t<DIM_COLOR_1_3_ALL; t++)
    fprintf(fp,"%f ",colorCHLAC_signature.points[0].histogram[ t ]);
  fprintf(fp,"\n");
  fclose(fp);

  return 1;
}

//--------------------------------------------------------------------------------------------
int main( int argc, char** argv ){
  if( argc != 3 ){
    ROS_ERROR ("Need two parameters! Syntax is: %s {input_pointcloud_filename.pcd} {feature_initial(v or c)} [option]\n", argv[0]);
    return(-1);
  }
  char filename[ 300 ];
  int length;
  if( argv[2][0] == 'v' ){
    //* write - VFH -
    pcl::PointCloud<PointXYZ> cloud_object_cluster;
    readPoints( argv[1], cloud_object_cluster );
    length = strlen( argv[1] );
    argv[1][ length-4 ] = '\0';
    sprintf(filename,"%s_vfh.pcd",argv[1]);
    writeVFHsignature( filename, cloud_object_cluster );
  }
  else if( argv[2][0] == 'c' ){
    //* write - ColorCHLAC -
    pcl::PointCloud<PointXYZRGB> cloud_object_cluster2;
    readPoints( argv[1], cloud_object_cluster2 );
    length = strlen( argv[1] );
    argv[1][ length-4 ] = '\0';
    sprintf(filename,"%s_colorCHLAC.pcd",argv[1]);
    writeColorCHLAC( filename, cloud_object_cluster2 );
  }
  else{
    ROS_ERROR ("Unknown feature type.\n");
    return(-1);
  }

//   pcl::io::savePCDFileASCII ("data/test_vfh.pcd", vfh_signature);
//   ROS_INFO ("Saved %d vfh descriptors to test_vfh.pcd.", (int)vfh_signature.points.size());

//   ros::NodeHandle nh;
//   ros::ServiceServer service = nh.advertiseService("vfh_extraction", vfhExtraction);
//   ros::spin();

  return(0);
}
