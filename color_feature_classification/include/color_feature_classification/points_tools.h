#ifndef COLOR_FEATURE_CLASSIFICATION_POINTS_TOOLS_H_
#define COLOR_FEATURE_CLASSIFICATION_POINTS_TOOLS_H_

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

bool rotatePoints( const pcl::PointCloud<pcl::PointXYZRGB> &input_cloud, pcl::PointCloud<pcl::PointXYZRGB> &output_cloud, const double roll, const double pan, const double roll2 ){
  output_cloud = input_cloud;
  double R1[9];
  R1[0]=cos(roll);  R1[1]=-sin(roll);  R1[2]=0;
  R1[3]=sin(roll);  R1[4]=cos(roll);   R1[5]=0;
  R1[6]=0;          R1[7]=0;           R1[8]=1;
  double R2[9];
  R2[0]=cos(pan);   R2[1]=0;  R2[2]=sin(pan);
  R2[3]=0;          R2[4]=1;  R2[5]=0;
  R2[6]=-sin(pan);  R2[7]=0;  R2[8]=cos(pan);
  double R3[9];
  R3[0]=cos(roll2); R3[1]=-sin(roll2); R3[2]=0;
  R3[3]=sin(roll2); R3[4]=cos(roll2);  R3[5]=0;
  R3[6]=0;          R3[7]=0;           R3[8]=1;

  float x1, y1, z1, x2, y2, z2;
  const int v_num = input_cloud.points.size();
  for( int i=0; i<v_num; i++ ){
    x1 = R1[0] * input_cloud.points[ i ].x + R1[1] * input_cloud.points[ i ].y + R1[2] * input_cloud.points[ i ].z;
    y1 = R1[3] * input_cloud.points[ i ].x + R1[4] * input_cloud.points[ i ].y + R1[5] * input_cloud.points[ i ].z;
    z1 = R1[6] * input_cloud.points[ i ].x + R1[7] * input_cloud.points[ i ].y + R1[8] * input_cloud.points[ i ].z;
    
    x2 = R2[0]*x1 + R2[1]*y1 + R2[2]*z1;
    y2 = R2[3]*x1 + R2[4]*y1 + R2[5]*z1;
    z2 = R2[6]*x1 + R2[7]*y1 + R2[8]*z1;
	    
    output_cloud.points[ i ].x = R3[0]*x2 + R3[1]*y2 + R3[2]*z2;
    output_cloud.points[ i ].y = R3[3]*x2 + R3[4]*y2 + R3[5]*z2;
    output_cloud.points[ i ].z = R3[6]*x2 + R3[7]*y2 + R3[8]*z2;
  }

  return(1);
}

bool rotatePoints( const pcl::PointCloud<pcl::PointXYZRGBNormal> &input_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> &output_cloud, const double roll, const double pan, const double roll2 ){
  output_cloud = input_cloud;
  double R1[9];
  R1[0]=cos(roll);  R1[1]=-sin(roll);  R1[2]=0;
  R1[3]=sin(roll);  R1[4]=cos(roll);   R1[5]=0;
  R1[6]=0;          R1[7]=0;           R1[8]=1;
  double R2[9];
  R2[0]=cos(pan);   R2[1]=0;  R2[2]=sin(pan);
  R2[3]=0;          R2[4]=1;  R2[5]=0;
  R2[6]=-sin(pan);  R2[7]=0;  R2[8]=cos(pan);
  double R3[9];
  R3[0]=cos(roll2); R3[1]=-sin(roll2); R3[2]=0;
  R3[3]=sin(roll2); R3[4]=cos(roll2);  R3[5]=0;
  R3[6]=0;          R3[7]=0;           R3[8]=1;

  float x1, y1, z1, x2, y2, z2;
  float nx1, ny1, nz1, nx2, ny2, nz2;
  const int v_num = input_cloud.points.size();
  for( int i=0; i<v_num; i++ ){
    x1 = R1[0] * input_cloud.points[ i ].x + R1[1] * input_cloud.points[ i ].y + R1[2] * input_cloud.points[ i ].z;
    y1 = R1[3] * input_cloud.points[ i ].x + R1[4] * input_cloud.points[ i ].y + R1[5] * input_cloud.points[ i ].z;
    z1 = R1[6] * input_cloud.points[ i ].x + R1[7] * input_cloud.points[ i ].y + R1[8] * input_cloud.points[ i ].z;
    nx1 = R1[0] * input_cloud.points[ i ].normal_x + R1[1] * input_cloud.points[ i ].normal_y + R1[2] * input_cloud.points[ i ].normal_z;
    ny1 = R1[3] * input_cloud.points[ i ].normal_x + R1[4] * input_cloud.points[ i ].normal_y + R1[5] * input_cloud.points[ i ].normal_z;
    nz1 = R1[6] * input_cloud.points[ i ].normal_x + R1[7] * input_cloud.points[ i ].normal_y + R1[8] * input_cloud.points[ i ].normal_z;
    
    x2 = R2[0]*x1 + R2[1]*y1 + R2[2]*z1;
    y2 = R2[3]*x1 + R2[4]*y1 + R2[5]*z1;
    z2 = R2[6]*x1 + R2[7]*y1 + R2[8]*z1;
    nx2 = R2[0]*nx1 + R2[1]*ny1 + R2[2]*nz1;
    ny2 = R2[3]*nx1 + R2[4]*ny1 + R2[5]*nz1;
    nz2 = R2[6]*nx1 + R2[7]*ny1 + R2[8]*nz1;
	    
    output_cloud.points[ i ].x = R3[0]*x2 + R3[1]*y2 + R3[2]*z2;
    output_cloud.points[ i ].y = R3[3]*x2 + R3[4]*y2 + R3[5]*z2;
    output_cloud.points[ i ].z = R3[6]*x2 + R3[7]*y2 + R3[8]*z2;
    output_cloud.points[ i ].normal_x = R3[0]*nx2 + R3[1]*ny2 + R3[2]*nz2;
    output_cloud.points[ i ].normal_y = R3[3]*nx2 + R3[4]*ny2 + R3[5]*nz2;
    output_cloud.points[ i ].normal_z = R3[6]*nx2 + R3[7]*ny2 + R3[8]*nz2;
  }

  return(1);
}

bool rotatePoints( const pcl::PointCloud<pcl::PointNormal> &input_cloud, pcl::PointCloud<pcl::PointNormal> &output_cloud, const double roll, const double pan, const double roll2 ){
  output_cloud = input_cloud;
  double R1[9];
  R1[0]=cos(roll);  R1[1]=-sin(roll);  R1[2]=0;
  R1[3]=sin(roll);  R1[4]=cos(roll);   R1[5]=0;
  R1[6]=0;          R1[7]=0;           R1[8]=1;
  double R2[9];
  R2[0]=cos(pan);   R2[1]=0;  R2[2]=sin(pan);
  R2[3]=0;          R2[4]=1;  R2[5]=0;
  R2[6]=-sin(pan);  R2[7]=0;  R2[8]=cos(pan);
  double R3[9];
  R3[0]=cos(roll2); R3[1]=-sin(roll2); R3[2]=0;
  R3[3]=sin(roll2); R3[4]=cos(roll2);  R3[5]=0;
  R3[6]=0;          R3[7]=0;           R3[8]=1;

  float x1, y1, z1, x2, y2, z2;
  float nx1, ny1, nz1, nx2, ny2, nz2;
  const int v_num = input_cloud.points.size();
  for( int i=0; i<v_num; i++ ){
    x1 = R1[0] * input_cloud.points[ i ].x + R1[1] * input_cloud.points[ i ].y + R1[2] * input_cloud.points[ i ].z;
    y1 = R1[3] * input_cloud.points[ i ].x + R1[4] * input_cloud.points[ i ].y + R1[5] * input_cloud.points[ i ].z;
    z1 = R1[6] * input_cloud.points[ i ].x + R1[7] * input_cloud.points[ i ].y + R1[8] * input_cloud.points[ i ].z;
    nx1 = R1[0] * input_cloud.points[ i ].normal_x + R1[1] * input_cloud.points[ i ].normal_y + R1[2] * input_cloud.points[ i ].normal_z;
    ny1 = R1[3] * input_cloud.points[ i ].normal_x + R1[4] * input_cloud.points[ i ].normal_y + R1[5] * input_cloud.points[ i ].normal_z;
    nz1 = R1[6] * input_cloud.points[ i ].normal_x + R1[7] * input_cloud.points[ i ].normal_y + R1[8] * input_cloud.points[ i ].normal_z;
    
    x2 = R2[0]*x1 + R2[1]*y1 + R2[2]*z1;
    y2 = R2[3]*x1 + R2[4]*y1 + R2[5]*z1;
    z2 = R2[6]*x1 + R2[7]*y1 + R2[8]*z1;
    nx2 = R2[0]*nx1 + R2[1]*ny1 + R2[2]*nz1;
    ny2 = R2[3]*nx1 + R2[4]*ny1 + R2[5]*nz1;
    nz2 = R2[6]*nx1 + R2[7]*ny1 + R2[8]*nz1;
	    
    output_cloud.points[ i ].x = R3[0]*x2 + R3[1]*y2 + R3[2]*z2;
    output_cloud.points[ i ].y = R3[3]*x2 + R3[4]*y2 + R3[5]*z2;
    output_cloud.points[ i ].z = R3[6]*x2 + R3[7]*y2 + R3[8]*z2;
    output_cloud.points[ i ].normal_x = R3[0]*nx2 + R3[1]*ny2 + R3[2]*nz2;
    output_cloud.points[ i ].normal_y = R3[3]*nx2 + R3[4]*ny2 + R3[5]*nz2;
    output_cloud.points[ i ].normal_z = R3[6]*nx2 + R3[7]*ny2 + R3[8]*nz2;
  }

  return(1);
}

#endif
