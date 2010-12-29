#ifndef MY_SEARCH_HPP
#define MY_SEARCH_HPP

#include "Voxel.hpp"

/***************************************************/
/* ʪ�θ��Ф򤹤륯�饹                                */
/* �Ķ����Τ��饹�饤�ǥ��󥰥ܥå��������ˤ��             */
/* �����о�ʪ�ΤȤ�����٤��⤤�ΰ���� rank_num�Ľ��Ϥ��� */
/***************************************************/

//* ���Х⡼��
//* �����о�ʪ�Τ�range1��range2��range3�Τ��줾���x��y��z�Τɤ�˳�����Ƥ뤫����6�̤�Ǿ��ʬ��
enum SearchMode{ S_MODE_1, S_MODE_2, S_MODE_3, S_MODE_4, S_MODE_5, S_MODE_6 };

class SearchObj{
public:
  double search_time; // ���Ф��פ������
  SearchObj();
  ~SearchObj();
  void setRange( int _range1, int _range2, int _range3 ); // �����о�ʪ�ΤΥ���������򥻥å�
  void setRank( int _rank_num );   // ��󥯲��̤ޤǤ��ΰ����Ϥ��뤫���򥻥å�
  void setThreshold( int _exist_voxel_num_threshold ); // �����ΰ����˴ޤޤ��ܥ��������������̤�����ä����Ǥ����뤫�����ͤ򥻥å�
  void readAxis( const char *filename, int dim, int dim_model, bool ascii, bool multiple_similarity ); // �����о�ʪ�Τ���ʬ���֤δ��켴���ɤ߹���
  void readData( const char *filenameF, const char *filenameN, int dim, bool ascii ); // scene�Ρ�ʬ���ΰ���Ρ���ħ�̤ȥܥ���������ɤ߹���
  void getRange( int &xrange, int &yrange, int &zrange, SearchMode mode ); // �����θ��Х⡼�ɤˤ����븡�Хܥå�����x,y,z�դ�Ĺ�������
  void search(); // ���Ƥθ��Х⡼�ɤǤ�ʪ�θ���
  void writeResult( const char *filename, int box_size ); // ��̤�ե�����˽���
  void cleanMax();

  // ����饤������ऱ
  void setSceneAxis( Matrix _axis );
  void setSceneAxis( Matrix _axis, ColumnVector var, int dim ); // whitening �ऱ
  void cleanData();
  void setData( Voxel& voxel, Voxel& voxel_bin, int dim, int box_size );
  const int XYnum() const { return xy_num; }
  const int Znum() const { return z_num; }
  const int maxX( int num ){ return max_x[ num ]; }
  const int maxY( int num ){ return max_y[ num ]; }
  const int maxZ( int num ){ return max_z[ num ]; }
  const double maxDot( int num ){ return max_dot[ num ]; }
  const int maxXrange( int num );
  const int maxYrange( int num );
  const int maxZrange( int num );

private:
  int range1; // �����о�ʪ�ΤΥܥå�������������1��
  int range2; // �����о�ʪ�ΤΥܥå�������������2��
  int range3; // �����о�ʪ�ΤΥܥå�������������3��
  int x_num;  // scene��x����Υܥå����ο�
  int y_num;  // scene��y����Υܥå����ο�
  int z_num;  // scene��z����Υܥå����ο�
  int xy_num; // x_num * _num
  int rank_num; // ��󥯲��̤ޤǽ��Ϥ�������
  int exist_voxel_num_threshold; // �����ΰ����˴ޤޤ��ܥ��������������̤�����ä����Ǥ����뤫������
  int *max_x; // �����ΰ�γ���������x��ɸ��rank_num�̤ޤ���¸��
  int *max_y; // �����ΰ�γ���������y��ɸ��rank_num�̤ޤ���¸��
  int *max_z; // �����ΰ�γ���������z��ɸ��rank_num�̤ޤ���¸��
  SearchMode *max_mode; // �����ΰ�򸡽Ф����Ȥ��θ��Х⡼��
  double *max_dot; // �����ΰ�ȸ����о�ʪ�ΤȤ�����١�rank_num�̤ޤ���¸��
  Matrix axis_q;   // �����о�ʪ�Τ���ʬ���֤δ��켴
  int *exist_voxel_num; // �ΰ���Υܥ�����θĿ�
  ColumnVector *nFeatures; // �ΰ����ħ��
  Matrix axis_p;  // ������ʬ���֤δ��켴

  int xRange( SearchMode mode ); // �����θ��Х⡼�ɤˤ����븡�Хܥå�����x�դ�Ĺ�����֤�
  int yRange( SearchMode mode ); // �����θ��Х⡼�ɤˤ����븡�Хܥå�����y�դ�Ĺ�����֤� 
  int zRange( SearchMode mode ); // �����θ��Х⡼�ɤˤ����븡�Хܥå�����z�դ�Ĺ�����֤�
  int checkOverlap( int x, int y, int z, SearchMode mode ); // ���˸��Ф����ΰ����뤫�ɤ���������å�
  void max_cpy( int src_num, int dest_num ); // ����ȯ�������ΰ�Υ�󥯤򤺤餹
  void max_assign( int dest_num, double dot, int x, int y, int z, SearchMode mode ); // ��ȯ�������ΰ�����ȯ�������ΰ���֤�������
  void search_part( SearchMode mode ); // �����θ��Х⡼�ɤǤ�ʪ�θ���
  template <typename T>
  T clipValue( T* ptr, const int x, const int y, const int z, const int xrange, const int yrange, const int zrange ); // ����ͤ���ʬ���줿���󤫤餽�ΰ��֤ˤ������ͤ���Ф�
};

#endif
