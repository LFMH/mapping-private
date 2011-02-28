#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>
#include <fstream>

#include <color_voxel_recognition/libPCA.hpp>
#include <color_voxel_recognition/Param.hpp>
#include <color_voxel_recognition/Search.hpp>

#include "./FILE_MODE"

/***************************************************************************************/
/* 環境全体からスライディングボックス方式により                                                 */
/* 検出対象物体との類似度が高い領域を上位 rank_num個出力する                                     */
/* 結果は./data_result/以下に出力される                                                     */
/*   パラメータ                                                                          */
/*   rank_num:   ランク上位何個の領域まで出力するか                                           */
/*   exist_voxel_num_threshold: 検出領域の中に含まれるボクセル数がいくつ未満だったら打ちきるかの閾値 */
/*   label:      検出物体の名前                                                           */
/*   dim_model:  検出対象物体の部分空間の基底の次元数                                         */
/*   range1:     検出対象物体の幅1 : 長さ/（ボクセルの一辺の長さ * 1ボックスに含まれるボクセル数）   */
/*   range2:     検出対象物体の幅2 : 長さ/（ボクセルの一辺の長さ * 1ボックスに含まれるボクセル数）   */
/*   range3:     検出対象物体の幅3 : 長さ/（ボクセルの一辺の長さ * 1ボックスに含まれるボクセル数）   */
/***************************************************************************************/

using namespace std;

int main(int argc, char** argv)
{

  if(argc!=8){
    cerr << "usage: " << argv[0] << " <rank_num> <exist_voxel_num_threshold> [label] <dim_model> <range1> <range2> <range3>" << endl;
    exit( EXIT_FAILURE );
  }
  char tmpname[100];

  //* 物体検出のインスタンス
  SearchObj search_obj;
  search_obj.setRange( atoi(argv[5]), atoi(argv[6]), atoi(argv[7]) );
  search_obj.setRank( atoi(argv[1]) );
  search_obj.setThreshold( atoi(argv[2]) );

  //* 分割領域の大きさの読み込み
  const int box_size = Param::readBoxSize_scene();

  //* 圧縮したCCHLAC特徴ベクトルの次元数の読み込み
  const int dim = Param::readDim();

  //* 検出対象物体の部分空間の基底の次元数
  const int dim_model = atoi(argv[4]);
  if( dim <= dim_model ){
    cerr << "ERR: dim_model should be less than dim(in dim.txt)" << endl; // 注 特徴量の次元数よりも多くなくてはいけません
    exit( EXIT_FAILURE );
  }

  //* 検出対象物体の部分空間の基底軸の読み込み
  sprintf(tmpname,"models/%s/pca_result",argv[3]);
  search_obj.readAxis( tmpname, dim, dim_model, ASCII_MODE_P, MULTIPLE_SIMILARITY );

  //* sceneの（分割領域毎の）特徴量とボクセル数の読み込み
  search_obj.readData( "scene/cchlac_small_add.dat", "scene/existNum_add.dat", dim, ASCII_MODE_F );

  //* 物体検出
  printf("start!\n");
  search_obj.search();
  printf("Total time = %10.6f\n",search_obj.search_time);// 経過時間の表示

  //* 検出結果をファイルに出力
  sprintf(tmpname,"data_result/%s.txt",argv[3]);
  search_obj.writeResult( tmpname, box_size );

  return 0;
}
