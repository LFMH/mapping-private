#ifndef MY_SEARCH_HPP
#define MY_SEARCH_HPP

#include "Voxel.hpp"

/***************************************************/
/* 物体検出をするクラス                                */
/* 環境全体からスライディングボックス方式により             */
/* 検出対象物体との類似度が高い領域を上位 rank_num個出力する */
/***************************************************/

//* 検出モード
//* 検出対象物体のrange1、range2、range3のそれぞれをx、y、zのどれに割り当てるか。の6通りで場合分け
enum SearchMode{ S_MODE_1, S_MODE_2, S_MODE_3, S_MODE_4, S_MODE_5, S_MODE_6 };

class SearchObj{
public:
  double search_time; // 検出に要する時間
  SearchObj();
  ~SearchObj();
  void setRange( int _range1, int _range2, int _range3 ); // 検出対象物体のサイズ情報をセット
  void setRank( int _rank_num );   // ランク何位までの領域を出力するか、をセット
  void setThreshold( int _exist_voxel_num_threshold ); // 検出領域の中に含まれるボクセル数がいくつ未満だったら打ちきるかの閾値をセット
  void readAxis( const char *filename, int dim, int dim_model, bool ascii, bool multiple_similarity ); // 検出対象物体の部分空間の基底軸の読み込み
  void readData( const char *filenameF, const char *filenameN, int dim, bool ascii ); // sceneの（分割領域毎の）特徴量とボクセル数の読み込み
  void getRange( int &xrange, int &yrange, int &zrange, SearchMode mode ); // 引数の検出モードにおける検出ボックスのx,y,z辺の長さを取得
  void search(); // 全ての検出モードでの物体検出
  void writeResult( const char *filename, int box_size ); // 結果をファイルに出力
  void cleanMax();

  // オンライン処理むけ
  void setSceneAxis( Matrix _axis );
  void setSceneAxis( Matrix _axis, ColumnVector var, int dim ); // whitening むけ
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
  int range1; // 検出対象物体のボックスサイズ（辺1）
  int range2; // 検出対象物体のボックスサイズ（辺2）
  int range3; // 検出対象物体のボックスサイズ（辺3）
  int x_num;  // sceneのx軸上のボックスの数
  int y_num;  // sceneのy軸上のボックスの数
  int z_num;  // sceneのz軸上のボックスの数
  int xy_num; // x_num * _num
  int rank_num; // ランク何位まで出力したいか
  int exist_voxel_num_threshold; // 検出領域の中に含まれるボクセル数がいくつ未満だったら打ちきるかの閾値
  int *max_x; // 検出領域の開始地点のx座標（rank_num位まで保存）
  int *max_y; // 検出領域の開始地点のy座標（rank_num位まで保存）
  int *max_z; // 検出領域の開始地点のz座標（rank_num位まで保存）
  SearchMode *max_mode; // 検出領域を検出したときの検出モード
  double *max_dot; // 検出領域と検出対象物体との類似度（rank_num位まで保存）
  Matrix axis_q;   // 検出対象物体の部分空間の基底軸
  int *exist_voxel_num; // 領域内のボクセルの個数
  ColumnVector *nFeatures; // 領域の特徴量
  Matrix axis_p;  // 圧縮部分空間の基底軸

  int xRange( SearchMode mode ); // 引数の検出モードにおける検出ボックスのx辺の長さを返す
  int yRange( SearchMode mode ); // 引数の検出モードにおける検出ボックスのy辺の長さを返す 
  int zRange( SearchMode mode ); // 引数の検出モードにおける検出ボックスのz辺の長さを返す
  int checkOverlap( int x, int y, int z, SearchMode mode ); // 既に検出した領域と被るかどうかをチェック
  void max_cpy( int src_num, int dest_num ); // 既に発見した領域のランクをずらす
  void max_assign( int dest_num, double dot, int x, int y, int z, SearchMode mode ); // 今発見した領域を既に発見した領域と置き換える
  void search_part( SearchMode mode ); // 引数の検出モードでの物体検出
  template <typename T>
  T clipValue( T* ptr, const int x, const int y, const int z, const int xrange, const int yrange, const int zrange ); // 順に値が積分された配列からその位置における値を取り出す
};

#endif
