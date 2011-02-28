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
/* �Ķ����Τ��饹�饤�ǥ��󥰥ܥå��������ˤ��                                                 */
/* �����о�ʪ�ΤȤ�����٤��⤤�ΰ���� rank_num�Ľ��Ϥ���                                     */
/* ��̤�./data_result/�ʲ��˽��Ϥ����                                                     */
/*   �ѥ�᡼��                                                                          */
/*   rank_num:   ��󥯾�̲��Ĥ��ΰ�ޤǽ��Ϥ��뤫                                           */
/*   exist_voxel_num_threshold: �����ΰ����˴ޤޤ��ܥ��������������̤�����ä����Ǥ����뤫������ */
/*   label:      ����ʪ�Τ�̾��                                                           */
/*   dim_model:  �����о�ʪ�Τ���ʬ���֤δ���μ�����                                         */
/*   range1:     �����о�ʪ�Τ���1 : Ĺ��/�ʥܥ�����ΰ��դ�Ĺ�� * 1�ܥå����˴ޤޤ��ܥ��������   */
/*   range2:     �����о�ʪ�Τ���2 : Ĺ��/�ʥܥ�����ΰ��դ�Ĺ�� * 1�ܥå����˴ޤޤ��ܥ��������   */
/*   range3:     �����о�ʪ�Τ���3 : Ĺ��/�ʥܥ�����ΰ��դ�Ĺ�� * 1�ܥå����˴ޤޤ��ܥ��������   */
/***************************************************************************************/

using namespace std;

int main(int argc, char** argv)
{

  if(argc!=8){
    cerr << "usage: " << argv[0] << " <rank_num> <exist_voxel_num_threshold> [label] <dim_model> <range1> <range2> <range3>" << endl;
    exit( EXIT_FAILURE );
  }
  char tmpname[100];

  //* ʪ�θ��ФΥ��󥹥���
  SearchObj search_obj;
  search_obj.setRange( atoi(argv[5]), atoi(argv[6]), atoi(argv[7]) );
  search_obj.setRank( atoi(argv[1]) );
  search_obj.setThreshold( atoi(argv[2]) );

  //* ʬ���ΰ���礭�����ɤ߹���
  const int box_size = Param::readBoxSize_scene();

  //* ���̤���CCHLAC��ħ�٥��ȥ�μ��������ɤ߹���
  const int dim = Param::readDim();

  //* �����о�ʪ�Τ���ʬ���֤δ���μ�����
  const int dim_model = atoi(argv[4]);
  if( dim <= dim_model ){
    cerr << "ERR: dim_model should be less than dim(in dim.txt)" << endl; // �� ��ħ�̤μ���������¿���ʤ��ƤϤ����ޤ���
    exit( EXIT_FAILURE );
  }

  //* �����о�ʪ�Τ���ʬ���֤δ��켴���ɤ߹���
  sprintf(tmpname,"models/%s/pca_result",argv[3]);
  search_obj.readAxis( tmpname, dim, dim_model, ASCII_MODE_P, MULTIPLE_SIMILARITY );

  //* scene�Ρ�ʬ���ΰ���Ρ���ħ�̤ȥܥ���������ɤ߹���
  search_obj.readData( "scene/cchlac_small_add.dat", "scene/existNum_add.dat", dim, ASCII_MODE_F );

  //* ʪ�θ���
  printf("start!\n");
  search_obj.search();
  printf("Total time = %10.6f\n",search_obj.search_time);// �в���֤�ɽ��

  //* ���з�̤�ե�����˽���
  sprintf(tmpname,"data_result/%s.txt",argv[3]);
  search_obj.writeResult( tmpname, box_size );

  return 0;
}
