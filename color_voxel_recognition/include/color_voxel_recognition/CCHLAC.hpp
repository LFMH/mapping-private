#ifndef MY_CCHLAC_HPP
#define MY_CCHLAC_HPP

#include <octave/config.h>
#include <octave/Matrix.h>
#include "Voxel.hpp"

/*****************************************************/
/* Color-CHLAC��ħ��Ȥ륯�饹                          */
/* �ޥ����� 3x3 �⡹1�� �Τߤ����                        */
/* �������Ȥ���RGB�ͤ�2�Ͳ����Ƥ�����Ȥ��Ƥʤ����Ȥǰۤʤ� */
/*****************************************************/

const int DIM_COLOR_1_3 = 495;     // RGB�ͤ�2�Ͳ����ʤ����� �ޥ�����3x3 �⡹1����Color-CHLAC��ħ�٥��ȥ�μ�����
const int DIM_COLOR_BIN_1_3 = 486; // RGB�ͤ�2�Ͳ��������  �ޥ�����3x3 �⡹1����Color-CHLAC��ħ�٥��ȥ�μ�����

//* ��ž�⡼��
//* 90�ٲ�ž�����뼴�ȸ����ˤ�ä�4�̤�
enum RotateMode{ R_MODE_1, R_MODE_2, R_MODE_3, R_MODE_4 };

class CCHLAC{
public:
  //* RGB�ͤ�2�Ͳ����ʤ�������ħ���
  static void extractColorCHLAC( ColumnVector &result, Voxel &voxel, int sx, int sy, int sz, int gx, int gy, int gz );
  
  //* RGB�ͤ�2�Ͳ����������ħ���
  static void extractColorCHLAC_bin( ColumnVector &result, Voxel &voxel, int sx, int sy, int sz, int gx, int gy, int gz );
  
  //* �ܥ������90�ٲ�ž�����Ȥ�����ħ�̤�׻�
  static void rotateFeature90( ColumnVector &output, const ColumnVector &input, RotateMode mode );

private:  
  static void extractColorCHLAC( ColumnVector &result, const unsigned char *red, const unsigned char *nred, const unsigned char *green, const unsigned char *ngreen, const unsigned char *blue, const unsigned char *nblue, int sx, int sy, int sz, int gx, int gy, int gz, int rx, int ry, int rz, int xsize, int ysize, int zsize );
  static void extractColorCHLAC_bin( ColumnVector &result, const unsigned char *red, const unsigned char *nred, const unsigned char *green, const unsigned char *ngreen, const unsigned char *blue, const unsigned char *nblue, int sx, int sy, int sz, int gx, int gy, int gz, int rx, int ry, int rz, int xsize, int ysize, int zsize );

  // no constructor
  CCHLAC();
};

#endif
