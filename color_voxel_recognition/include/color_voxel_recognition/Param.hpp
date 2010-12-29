#ifndef MY_PARAM_HPP
#define MY_PARAM_HPP

/********************************************/
/* �ѥ�᡼�����ɤ߹��ߡʤȰ����񤭹��ߡˤ򤹤륯�饹 */
/********************************************/

class Param{
public:

  //* 1�ܥ�������դ�Ĺ�� ���ɤ߹���
  static float readVoxelSize( const char* filename = "param/parameters.txt" );

  //* ���̤���������Color-CHLAC��ħ�٥��ȥ�μ��� ���ɤ߹���
  static int readDim( const char* filename = "param/parameters.txt" );

  //* �Ķ���ʬ���ΰ��ñ�̡�1�ܥå�����1�դ����ܥ�����Ǥ��뤫 ���ɤ߹����
  static int readBoxSize_scene( const char* filename = "param/parameters.txt" );

  //* �����о�ʪ�Τ�ʬ���ΰ��ñ�̡�1�ܥå�����1�դ����ܥ�����Ǥ��뤫�� ���ɤ߹���
  static int readBoxSize_model( const char* filename = "param/parameters.txt" );

  //* ���٤˺�������ܥ�������ξ�� ���ɤ߹���
  static int readMaxVoxelNum( const char* filename = "param/parameters.txt" );

  //* ʪ�Τγؽ��ˤ���������ΥХꥨ������� ���ɤ߹���
  static int readRotateNum( const char* filename = "param/parameters.txt" );

  //* �ܥ������RGB�ͤ�2�Ͳ������� ���ɤ߹���
  static void readColorThreshold( int &r, int &g, int &b, const char* filename = "param/color_threshold.txt" );

  //* ��ư��å��岽��ɬ�פʡ�SR���������͡פ��ɤ߹���
  static int readConfTh( const char* filename );

  //* ��ư��å��岽��ɬ�פʥѥ�᡼���������ɤ߹���
  static void readParamAuto( float& length_max_rate, float& length_th, float& distance_th, int& confidence_th, bool &relative_mode, const char* filename );

  //* ��ư��å��岽��ɬ�פʥѥ�᡼�������ν񤭹���
  static void writeParamAuto( float length_max_rate, float length_th, float distance_th, int confidence_th, bool relative_mode, const char* filename );

private:
  static bool readParam( const char* filename, const char *param_string, int &val );
  static bool readParam( const char* filename, const char *param_string, float &val );

  // no constructor
  Param();
};

#endif
