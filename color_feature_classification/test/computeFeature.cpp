#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <color_chlac/ColorCHLAC.hpp>
#include <color_chlac/ColorVoxel.hpp>

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
  pcl::KdTree<pcl::PointXYZ>::Ptr tree = boost::make_shared<pcl::KdTreeANN<pcl::PointXYZ> > ();
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
  pcl::io::savePCDFileASCII (name, vfh_signature);

  return(1);
}

//--------------------------------------------------------------------------------------------
// function2
bool writeColorCHLAC(const char *name, pcl::PointCloud<pcl::PointXYZRGB> cloud_object_cluster ){
  // compute voxel
  ColorVoxel voxel;
  voxel.setVoxelSize( 0.05 );
  voxel.points2voxel( cloud_object_cluster, SIMPLE_REVERSE );
  ColorVoxel voxel_bin;
  cout << endl;
  voxel_bin = voxel;
  for( int i=0; i<180; i++ ) printf("%d ",voxel.Vr()[i]);
  cout << endl;
  for( int i=0; i<180; i++ ) printf("%d ",voxel_bin.Vr()[i]);
  cout << endl;
  voxel_bin.binarize( 127, 127, 127 );
  for( int i=0; i<180; i++ ) printf("%d ",voxel.Vr()[i]);
  cout << endl;
  for( int i=0; i<180; i++ ) printf("%d ",voxel_bin.Vr()[i]);
  cout << endl;

  // compute colorCHLAC
  std::vector<float> colorCHLAC;
  ColorCHLAC::extractColorCHLAC( colorCHLAC, voxel );
  colorCHLAC.resize( DIM_COLOR_1_3+DIM_COLOR_BIN_1_3 );
  std::vector<float> tmp;
  ColorCHLAC::extractColorCHLAC_bin( tmp, voxel_bin );
  for(int t=0;t<DIM_COLOR_BIN_1_3;t++)
    colorCHLAC[ t+DIM_COLOR_1_3 ] = tmp[ t ];

  // save colorCHLAC
  FILE *fp = fopen( name, "w" );
  fprintf(fp,"# .PCD v.? - Point Cloud Data file format\n");
  fprintf(fp,"FIELDS colorCHLAC\n");
  fprintf(fp,"SIZE 4\n");
  fprintf(fp,"TYPE F\n");
  fprintf(fp,"COUNT %d\n",DIM_COLOR_BIN_1_3+DIM_COLOR_1_3);
  fprintf(fp,"WIDTH 1\n");
  fprintf(fp,"HEIGHT 1\n");
  fprintf(fp,"POINTS 1\n");
  fprintf(fp,"DATA ascii\n");
  for(int t=0;t<DIM_COLOR_BIN_1_3+DIM_COLOR_1_3;t++)
    fprintf(fp,"%f ",colorCHLAC[ t ]);
  fprintf(fp,"\n");
  fclose(fp);

  return 1;
}

//--------------------------------------------------------------------------------------------
int main( int argc, char** argv ){
  if( argc != 3 ){
    ROS_ERROR ("Need two parameters! Syntax is: %s {input_pointcloud_filename.pcd} {feature_initial(v or c)}\n", argv[0]);
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
