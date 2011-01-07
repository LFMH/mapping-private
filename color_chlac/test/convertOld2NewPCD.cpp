#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

using namespace pcl;
using namespace std;

const float setPointRGB( const int r, const int g, const int b ){
  //cout << r << " " << g << " " << b << endl;
  int rgb = r << 16 | g << 8 | b;
  return *reinterpret_cast<float*>(&rgb);
}

int main( int argc, char** argv ){
  if( argc != 3 ){
    ROS_ERROR ("Need two parameters! Syntax is: %s {input_pointcloud_filename.pcd} {output_pointcloud_filename.pcd}\n", argv[0]);
    return(-1);
  }
  char line[ 300 ];
  int num;
  float x, y, z;
  int r, g, b, rgb;

  FILE *fpR = fopen( argv[1], "r" );
  //FILE *fpW = fopen( argv[2], "wb" );
  while( fgets( line, sizeof(line), fpR ) != NULL )
    if( line[0] != '#' ) break;
  fgets( line, sizeof(line), fpR );
  //cout << line;
  sscanf( line, "POINTS %d\n", &num );
  fgets( line, sizeof(line), fpR );
  //cout << line;

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.height   = 1;
  cloud.width    = num;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  // fprintf( fpW, "# .PCD v.6 - Point Cloud Data file format\n" );
  // fprintf( fpW, "FIELDS x y z rgb\n" );
  // fprintf( fpW, "SIZE 4 4 4 4\n" );
  // fprintf( fpW, "TYPE F F F F\n" );
  // fprintf( fpW, "COUNT 1 1 1 1\n" );
  // fprintf( fpW, "WIDTH %d\n", num );
  // fprintf( fpW, "HEIGHT 1\n" );
  // fprintf( fpW, "POINTS %d\n", num );
  // fprintf( fpW, "DATA binary\n" );

  for( int i = 0; i < num; i++ ){
    fscanf( fpR, "%f %f %f %d %d %d%\n", &x, &y, &z, &r, &g, &b );
    //cout << x << " " << y << " " << z << " " << r << " " << g << " " << b << endl;
    //fprintf( fpW, "%f %f %f %f%\n", x, y, z, setPointRGB( r, g, b ) );
    //rgb = setPointRGB( r, g, b );
    cloud.points[i].x = x;
    cloud.points[i].y = y;
    cloud.points[i].z = z;
    cloud.points[i].rgb = setPointRGB( r, g, b );
    // fwrite( &x, sizeof(float), 1, fpW );
    // fwrite( &y, sizeof(float), 1, fpW );
    // fwrite( &z, sizeof(float), 1, fpW );
    // fwrite( &rgb, sizeof(float), 1, fpW );
  }
  fclose(fpR);
  pcl::io::savePCDFile (argv[2], cloud, true);

  //fclose(fpW);

  return(0);
}

