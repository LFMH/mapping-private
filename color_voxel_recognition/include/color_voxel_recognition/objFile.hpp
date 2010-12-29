#ifndef MY_OBJ_HPP
#define MY_OBJ_HPP

#ifdef USE_OPENCV
#include <cv.h>
#endif

#include <float.h>

/*********************************************************************/
/* .obj のメッシュデータを読み書き、処理をするクラス                          */
/*  ただしメッシュ情報が"頂点座標値番号/テクスチャ座標値番号"のときに限る          */
/*  また、各メッシュの頂点の数は3に限る                                     */
/*  注1 OpenCVの配列を使うときは #define USE_OPENCV する                  */
/*  注2 もともとbmpファイルを想定してテクスチャマップが作成された.objの読み込み時は */
/*      readMeshのときbmp_texmap_flgをtrueにすること                     */
/*********************************************************************/

//* 長さの単位（mかmmか）
enum UnitMode{ METER_MODE, MILLIMETER_MODE };

class Vertex{ // 頂点座標
public:
  float x;
  float y;
  float z;
  Vertex(){}
  ~Vertex(){}
};

class VertexT{ // テクスチャ座標
public:
  float x;
  float y;
  VertexT(){}
  ~VertexT(){}
};

//* 注 三角メッシュ限定
class Face{ // メッシュ
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

class Obj{ // 本体
public:
  int v_num;   // 頂点の個数
  int vt_num;  // テクスチャマップにおける頂点の個数
  int f_num;   // メッシュの個数
  int width;   // 距離画像の横幅
  int height;  // 距離画像の縦幅
  float x_min; // 頂点のx座標の最小値
  float x_max; // 頂点のx座標の最大値 
  float y_min; // 頂点のy座標の最小値
  float y_max; // 頂点のy座標の最大値 
  float z_min; // 頂点のz座標の最小値
  float z_max; // 頂点のz座標の最大値 
  Vertex *vertices;   // 頂点たち
  VertexT *verticesT; // 頂点座標たち
  Face *faces;        // メッシュたち
  unsigned char* confidences; // SR4000などで、距離情報の信頼度（頂点毎）
  int texture_id;    // テクスチャ番号

  //* メッシュを読み込む（.objファイルのみ）
  void readMesh( const char *filename, bool bmp_texmap_flg = false );

  //* 点群を読み込む（ファイルから）
  void readPoints( const char *filenameX, const char *filenameY, const char *filenameZ, const int _width, const int _height, const int _image_width, const int _image_height, const float resize_rate = 1, const float aspect_rate = 1, const int _shiftX = 0, const int _shiftY = 0, const int dis_1_pixel = 0, bool ascii = false );

  //* 点群を読み込む（ファイルから）SR4000などで、信頼度も読み込む
  void readPoints( const char *filenameX, const char *filenameY, const char *filenameZ, const char *filenameC, const int _width, const int _height, const int _image_width, const int _image_height, const float resize_rate = 1, const float aspect_rate = 1, const int _shiftX = 0, const int _shiftY = 0, const int dis_1_pixel = 0, bool ascii = false );

#ifdef USE_OPENCV
  //* 点群を読み込む（OpenCVの配列から）
  void readPoints( const cv::Mat& matX, const cv::Mat& matY, const cv::Mat& matZ, const int _width, const int _height, const int _image_width, const int _image_height, const float resize_rate = 1, const float aspect_rate = 1, const int _shiftX = 0, const int _shiftY = 0, const int dis_1_pixel = 0 );

  //* 点群を読み込む（OpenCVの配列から）SR4000などで，信頼度も読み込む
  void readPoints( const cv::Mat& matX, const cv::Mat& matY, const cv::Mat& matZ, const cv::Mat& matC, const int _width, const int _height, const int _image_width, const int _image_height, const float resize_rate = 1, const float aspect_rate = 1, const int _shiftX = 0, const int _shiftY = 0, const int dis_1_pixel = 0 );

#endif
  //* 点群からメッシュ作成 Overlap領域を除く
  void getMesh( const char *filenameM, float length_max_rate, float length_th, float distance_th, unsigned char confidence_th = 0, bool ascii = false );

  //* 点群からメッシュ作成
  void getMesh( float length_max_rate, float length_th, float distance_th, unsigned char confidence_th = 0 );

  //* メッシュを書き出す（.objファイルのみ）
  void writeMesh( const char *filename );
  
  //* mm単位をm単位に変換する  
  void mm2m();

  //* メッシュを回転させる
  void rotateMesh( Obj& input, double roll, double pan, double roll2 );

  //* メッシュを回転/移動させる
  void transMesh( const char *filename, UnitMode mode );

  //* テクスチャ番号をセットする
  void setTextureID( const int num ){ texture_id = num; }

  //* メモリの解放, x_minなどの初期化
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
