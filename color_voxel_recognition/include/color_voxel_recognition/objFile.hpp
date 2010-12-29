#ifndef MY_OBJ_HPP
#define MY_OBJ_HPP

#ifdef USE_OPENCV
#include <cv.h>
#endif

#include <float.h>

/*********************************************************************/
/* .obj �Υ�å���ǡ������ɤ߽񤭡������򤹤륯�饹                          */
/*  ��������å������"ĺ����ɸ���ֹ�/�ƥ��������ɸ���ֹ�"�ΤȤ��˸¤�          */
/*  �ޤ����ƥ�å����ĺ���ο���3�˸¤�                                     */
/*  ��1 OpenCV�������Ȥ��Ȥ��� #define USE_OPENCV ����                  */
/*  ��2 ��Ȥ��bmp�ե���������ꤷ�ƥƥ�������ޥåפ��������줿.obj���ɤ߹��߻��� */
/*      readMesh�ΤȤ�bmp_texmap_flg��true�ˤ��뤳��                     */
/*********************************************************************/

//* Ĺ����ñ�̡�m��mm����
enum UnitMode{ METER_MODE, MILLIMETER_MODE };

class Vertex{ // ĺ����ɸ
public:
  float x;
  float y;
  float z;
  Vertex(){}
  ~Vertex(){}
};

class VertexT{ // �ƥ��������ɸ
public:
  float x;
  float y;
  VertexT(){}
  ~VertexT(){}
};

//* �� ���ѥ�å������
class Face{ // ��å���
public:
  int v1;
  int v2;
  int v3;
  int vt1;
  int vt2;
  int vt3;
  Face(){}
  ~Face(){}
};

class Obj{ // ����
public:
  int v_num;   // ĺ���θĿ�
  int vt_num;  // �ƥ�������ޥåפˤ�����ĺ���θĿ�
  int f_num;   // ��å���θĿ�
  int width;   // ��Υ�����β���
  int height;  // ��Υ�����ν���
  float x_min; // ĺ����x��ɸ�κǾ���
  float x_max; // ĺ����x��ɸ�κ����� 
  float y_min; // ĺ����y��ɸ�κǾ���
  float y_max; // ĺ����y��ɸ�κ����� 
  float z_min; // ĺ����z��ɸ�κǾ���
  float z_max; // ĺ����z��ɸ�κ����� 
  Vertex *vertices;   // ĺ������
  VertexT *verticesT; // ĺ����ɸ����
  Face *faces;        // ��å��夿��
  unsigned char* confidences; // SR4000�ʤɤǡ���Υ����ο����١�ĺ�����
  int texture_id;    // �ƥ��������ֹ�

  //* ��å�����ɤ߹����.obj�ե�����Τߡ�
  void readMesh( const char *filename, bool bmp_texmap_flg = false );

  //* �������ɤ߹���ʥե����뤫���
  void readPoints( const char *filenameX, const char *filenameY, const char *filenameZ, const int _width, const int _height, const int _image_width, const int _image_height, const float resize_rate = 1, const float aspect_rate = 1, const int _shiftX = 0, const int _shiftY = 0, const int dis_1_pixel = 0, bool ascii = false );

  //* �������ɤ߹���ʥե����뤫���SR4000�ʤɤǡ������٤��ɤ߹���
  void readPoints( const char *filenameX, const char *filenameY, const char *filenameZ, const char *filenameC, const int _width, const int _height, const int _image_width, const int _image_height, const float resize_rate = 1, const float aspect_rate = 1, const int _shiftX = 0, const int _shiftY = 0, const int dis_1_pixel = 0, bool ascii = false );

#ifdef USE_OPENCV
  //* �������ɤ߹����OpenCV�����󤫤��
  void readPoints( const cv::Mat& matX, const cv::Mat& matY, const cv::Mat& matZ, const int _width, const int _height, const int _image_width, const int _image_height, const float resize_rate = 1, const float aspect_rate = 1, const int _shiftX = 0, const int _shiftY = 0, const int dis_1_pixel = 0 );

  //* �������ɤ߹����OpenCV�����󤫤��SR4000�ʤɤǡ������٤��ɤ߹���
  void readPoints( const cv::Mat& matX, const cv::Mat& matY, const cv::Mat& matZ, const cv::Mat& matC, const int _width, const int _height, const int _image_width, const int _image_height, const float resize_rate = 1, const float aspect_rate = 1, const int _shiftX = 0, const int _shiftY = 0, const int dis_1_pixel = 0 );

#endif
  //* ���������å������ Overlap�ΰ�����
  void getMesh( const char *filenameM, float length_max_rate, float length_th, float distance_th, unsigned char confidence_th = 0, bool ascii = false );

  //* ���������å������
  void getMesh( float length_max_rate, float length_th, float distance_th, unsigned char confidence_th = 0 );

  //* ��å����񤭽Ф���.obj�ե�����Τߡ�
  void writeMesh( const char *filename );
  
  //* mmñ�̤�mñ�̤��Ѵ�����  
  void mm2m();

  //* ��å�����ž������
  void rotateMesh( Obj& input, double roll, double pan, double roll2 );

  //* ��å�����ž/��ư������
  void transMesh( const char *filename, UnitMode mode );

  //* �ƥ��������ֹ�򥻥åȤ���
  void setTextureID( const int num ){ texture_id = num; }

  //* ����β���, x_min�ʤɤν����
  void deleteData(){ 
    if( vertices != NULL )  delete[] vertices;
    if( verticesT != NULL ) delete[] verticesT;
    if( faces != NULL )  delete[] faces;
    if( confidences != NULL )  delete[] confidences;
    vertices = NULL;
    verticesT = NULL;
    faces = NULL;
    confidences = NULL;
    v_num = 0;
    vt_num = 0;
    f_num = 0;
    x_min = FLT_MAX;
    x_max = -FLT_MAX;
    y_min = FLT_MAX;
    y_max = -FLT_MAX;
    z_min = FLT_MAX;
    z_max = -FLT_MAX;
  }

  Obj() : 
    v_num(0),
    vt_num(0),
    f_num(0),
    x_min(FLT_MAX),
    x_max(-FLT_MAX),
    y_min(FLT_MAX),
    y_max(-FLT_MAX),
    z_min(FLT_MAX),
    z_max(-FLT_MAX),
    vertices(NULL),
    verticesT(NULL),
    faces(NULL),
    confidences(NULL)
  { }
  ~Obj(){ 
    if( vertices != NULL )  delete[] vertices;
    if( verticesT != NULL ) delete[] verticesT;
    if( faces != NULL )  delete[] faces;
    if( confidences != NULL )  delete[] confidences;
  }
  Obj( const Obj &another ){ *this = another;  }
  const Obj& operator=(const Obj &another);

private:
  float sq_len( Vertex v1, Vertex v2 );
  bool checkConfidence( int idx, unsigned char confidence_th );
  void setMinMax( int vertex_num );
};

#endif
