#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <float.h>
#include <c3_hlac/c3_hlac_tools.h>
#include <terminal_tools/parse.h>
#include <terminal_tools/print.h>

using namespace pcl;
using namespace std;
using namespace terminal_tools;

int totalNum = 0; // The number of voxels
int **h;          // histograms of R, G and B
float voxel_size;

void
readVoxelFromPoints ( int argc, char **argv, const std::string &extension )
{  
  char line[ 100 ];
  string tmpname;
  int dim, sample_num;
  std::vector<float> feature;

  for (int i = 1; i < argc; i++){
    string fname = string (argv[i]);
    // Needs to have the right size
    if (fname.size () <= extension.size ())
      continue;
    transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0){

      pcl::PointCloud<PointXYZRGB> input_cloud;
      if (pcl::io::loadPCDFile (argv[i], input_cloud) == -1){
	ROS_ERROR ("Couldn't read file %s",argv[i]);
	return ;
      }
      pcl::VoxelGrid<PointXYZRGB> grid;
      pcl::PointCloud<PointXYZRGB> cloud_downsampled;
      getVoxelGrid( grid, input_cloud, cloud_downsampled, voxel_size );
      const int p_num = (int)cloud_downsampled.points.size();
      for( int j=0; j<p_num; j++ ){
	const int color = *reinterpret_cast<const int*>(&(cloud_downsampled.points[j].rgb));
	const int r = (0xff0000 & color) >> 16;
	const int g = (0x00ff00 & color) >> 8;
	const int b =  0x0000ff & color;
	totalNum++;
	h[ 0 ][ r ] ++;
	h[ 1 ][ g ] ++;
	h[ 2 ][ b ] ++;
      }
    }
  }
}

int main(int argc, char** argv)
{
  if( argc < 4 ){
    ROS_ERROR ("Need at least two parameters! Syntax is: %s {input_pointcloud_filename.pcd} {config_txt_path} {output(color_threshold).txt}\n", argv[0]);
    return(-1);
  }
  int *threshold   = new int[ 3 ];     // threshold for RGB binarize (= results)
  double *totalAve = new double[ 3 ];  // total average of RGB values
  double **eachAve = new double*[ 3 ]; // each average of RGB values from 0 to j
  int **eachNum    = new int*[ 3 ];    // the number of occupied voxles with RGB values from 0 to j
  h = new int*[ 3 ];                   // histograms of RGB

  //* read the length of voxel side
  char voxel_size_file[1024];
  sprintf(voxel_size_file, "%s/voxel_size.txt", argv[argc-2]);
  FILE *fp = fopen( voxel_size_file, "r" );
  fscanf( fp, "%f\n", &voxel_size );
  fclose(fp);

  //* initialize
  for( int i = 0; i < 3; i++ ){
    threshold[ i ]    = 0;
    totalAve[ i ]     = 0;
    eachAve[ i ]      = new double[ 256 ];
    eachNum[ i ]      = new int[ 256 ];
    h[ i ]            = new int[ 256 ];
    for( int j = 0; j < 256; j++ )
      h[ i ][ j ]       = 0;
  }

  //* read points, voxelize, and make histograms of RGB
  std::string extension (".pcd");
  transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
  readVoxelFromPoints( argc, argv, extension );

  //* total average of RGB values
  const double scale = 1 / (double)totalNum;
  for( int i = 0; i < 3; i++ ){
    for( int j = 0; j < 256; j++ )
      totalAve[ i ] += j * h[ i ][ j ];
    totalAve[ i ] *= scale;
  }

  //* each average of RGB values from 0 to j
  for( int i = 0; i < 3; i++ ){
    eachAve[ i ][ 0 ] = 0;
    eachNum[ i ][ 0 ] = h[ i ][ 0 ];
    int tmp_eachAve = 0;
    for( int j = 1; j < 256; j++ ){
      eachNum[ i ][ j ] = eachNum[ i ][ j-1 ] + h[ i ][ j ];
      tmp_eachAve += j * h[ i ][ j ];
      if( eachNum[ i ][ j ] == 0 )
	eachAve[ i ][ j ] = 0;
      else
	eachAve[ i ][ j ] = (double)tmp_eachAve / eachNum[ i ][ j ];
    }
  }

  //* determine threshold of RGB
  //*   by looking for the case when between-class variance becomes the largest
  for( int i = 0; i < 3; i++ ){
    double max_var = 0;
    for( int j = 1; j < 256; j++ ){
      if( eachNum[ i ][ j ] != 0 ){
	if( eachNum[ i ][ j ] == totalNum ) break;
	const double ave_sub = eachAve[ i ][ j ] - totalAve[ i ];
	const double var = ave_sub * ave_sub * ( eachNum[ i ][ j ] / (double)( totalNum - eachNum[ i ][ j ] ) );
	if( var > max_var ){
	  max_var = var;
	  threshold[ i ] = j;
	}
      }
    }
  }
  
  printf("totalAverage: %f %f %f\n",  totalAve[0], totalAve[1], totalAve[2]);
  printf("threshold: %d %d %d\n",threshold[0],threshold[1],threshold[2]);

  //* output results into file
  fp = fopen( argv[ argc-1 ],"w" );
  fprintf(fp,"%d %d %d\n",threshold[0],threshold[1],threshold[2]);
  fclose(fp);

  delete[] h[ 0 ];
  delete[] h[ 1 ];
  delete[] h[ 2 ];
  delete[] h;

  return 0;
}
