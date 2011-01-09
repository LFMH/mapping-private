#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <float.h>
#include <color_chlac/grsd_colorCHLAC_tools.h>
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
      readPoints( argv[i], input_cloud );
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
  if( argc < 3 ){
    ROS_ERROR ("Need at least two parameters! Syntax is: %s {input_pointcloud_filename.pcd} {output(color_threshold).txt}\n", argv[0]);
    return(-1);
  }

  int *threshold   = new int[ 3 ];     // RGB値の2値化の閾値
  double *totalAve = new double[ 3 ];  // RGB値の全体の平均値
  double **eachAve = new double*[ 3 ]; // RGB値の0からj番めの値までの平均値
  int **eachNum    = new int*[ 3 ];    // RGB値の0からj番めの値までのボクセル個数
  h = new int*[ 3 ];

  //* voxel size (downsample_leaf)
  FILE *fp = fopen( "voxel_size.txt", "r" );
  fscanf( fp, "%f\n", &voxel_size );
  fclose(fp);

  //* 初期化
  for( int i = 0; i < 3; i++ ){
    threshold[ i ]    = 0;
    totalAve[ i ]     = 0;
    eachAve[ i ]      = new double[ 256 ];
    eachNum[ i ]      = new int[ 256 ];
    h[ i ]            = new int[ 256 ];
    for( int j = 0; j < 256; j++ )
      h[ i ][ j ]       = 0;
  }

  //* 読み込み、ヒストグラムの作成
  std::string extension (".pcd");
  transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
  readVoxelFromPoints( argc, argv, extension );

  //* 全体の平均の計算
  const double scale = 1 / (double)totalNum;
  for( int i = 0; i < 3; i++ ){
    for( int j = 0; j < 256; j++ )
      totalAve[ i ] += j * h[ i ][ j ];
    totalAve[ i ] *= scale;
  }

  //* 0からj番めの値までの値までの平均の計算
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

  //* 閾値の決定
  //* クラス間分散が最大になるときを探す
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

  //* 結果をファイルに出力
  fp = fopen( argv[ argc-1 ],"w" );
  fprintf(fp,"%d %d %d\n",threshold[0],threshold[1],threshold[2]);
  fclose(fp);

  delete[] h[ 0 ];
  delete[] h[ 1 ];
  delete[] h[ 2 ];
  delete[] h;

  return 0;
}
