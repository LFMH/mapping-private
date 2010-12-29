#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <float.h>
#include "../param/FILE_MODE"

/******************************************************************************/
/* �ܥ�����ǡ������ɤ߹��ߡ�                                                      */
/* RGB���줾��ˤĤ������Ť����ͷ���ˡ�����ͤ���                                     */
/* param/color_threshold.txt �˷�̤����                                       */
/* �ʴĶ��������ħ��Ф����ˤ˹Ԥ����Ͼ�ά��ǽ���ǽ�˵�᤿�ͤ�����Ѥ��Ƥ褤��           */
/* �ʤ��뤤�ϡ�ñ���256��Ⱦʬ�� 127 127 127 �Ǥ�褤��                              */
/*  ��˥��ޥ�ɥ饤������˥ե����������ꤹ��ȡ�Voxel/�ե�����ʲ��Υե����뷲���оݤȤʤ� */
/******************************************************************************/

using namespace std;

int main(int argc, char** argv)
{
  if( ( argc != 1 )&&( argc != 2 ) ){
    cerr << "usage: " << argv[0] << endl;
    cerr << " or" << endl;
    cerr << "usage: " << argv[0] << " <registration_num>" << endl;
    exit( EXIT_FAILURE );
  }
  int file_num = 1;
  if( argc==2 ) file_num = atoi( argv[1] );

  int totalNum = 0;                    // �ܥ�����θĿ�
  int *threshold   = new int[ 3 ];     // RGB�ͤ�2�Ͳ�������
  double *totalAve = new double[ 3 ];  // RGB�ͤ����Τ�ʿ����
  double **eachAve = new double*[ 3 ]; // RGB�ͤ�0����j�֤���ͤޤǤ�ʿ����
  int **eachNum    = new int*[ 3 ];    // RGB�ͤ�0����j�֤���ͤޤǤΥܥ�����Ŀ�
  int **h          = new int*[ 3 ];    // RGB�ͤΥҥ��ȥ����

  //* �����
  for( int i = 0; i < 3; i++ ){
    threshold[ i ]    = 0;
    totalAve[ i ]     = 0;
    eachAve[ i ]      = new double[ 256 ];
    eachNum[ i ]      = new int[ 256 ];
    h[ i ]            = new int[ 256 ];
    for( int j = 0; j < 256; j++ )
      h[ i ][ j ]       = 0;
  }

  //* �ɤ߹��ߡ��ҥ��ȥ����κ���
  FILE *fp;
  char tmpname[ 100 ];
  if( ASCII_MODE_V ){
    for( int i=0; i<file_num; i++ ){
      if( argc==1 ) // ���ޥ�ɥ饤��������ʤ����ñ��Υե����뤫��
	fp = fopen( "scene/voxel_scene.dat","r" );
      else{         // ���ޥ�ɥ饤������������ʣ���Υե����뤫��
	sprintf( tmpname, "scene/Voxel/%03d.dat", i );
	fp = fopen( tmpname,"r" );
      }
      int tmp;
      fscanf(fp,"%d %d %d\n",&tmp,&tmp,&tmp);
      int x,y,z,r,g,b;
      while( fscanf(fp,"%d %d %d %d %d %d",&x,&y,&z,&r,&g,&b)!=EOF ){
	totalNum++;
	h[ 0 ][ r ] ++;
	h[ 1 ][ g ] ++;
	h[ 2 ][ b ] ++;
      }
      fclose( fp );
    }
  }
  else{
    for( int i=0; i<file_num; i++ ){
      if( argc==1 ) // ���ޥ�ɥ饤��������ʤ����ñ��Υե����뤫��
	fp = fopen( "scene/voxel_scene.dat","rb" );
      else{         // ���ޥ�ɥ饤������������ʣ���Υե����뤫��
	sprintf( tmpname, "scene/Voxel/%03d.dat", i );
	fp = fopen( tmpname,"rb" );
      }
      int tmp;
      fread(&tmp,sizeof(int),1,fp);
      fread(&tmp,sizeof(int),1,fp);
      fread(&tmp,sizeof(int),1,fp);
      fscanf(fp,"%d %d %d\n",&tmp,&tmp,&tmp);
      int x,y,z;
      unsigned char r,g,b;
      while( fread(&x,sizeof(int),1,fp) > 0 ){
	fread(&y,sizeof(int),1,fp);
	fread(&z,sizeof(int),1,fp);
	fread(&r,sizeof(unsigned char),1,fp);
	fread(&g,sizeof(unsigned char),1,fp);
	fread(&b,sizeof(unsigned char),1,fp);
	totalNum++;    
	h[ 0 ][ r ] ++;
	h[ 1 ][ g ] ++;
	h[ 2 ][ b ] ++;
      }
      fclose( fp );
    }
  }

  //* ���Τ�ʿ�Ѥη׻�
  const double scale = 1 / (double)totalNum;
  for( int i = 0; i < 3; i++ ){
    for( int j = 0; j < 256; j++ )
      totalAve[ i ] += j * h[ i ][ j ];
    totalAve[ i ] *= scale;
  }

  //* 0����j�֤���ͤޤǤ��ͤޤǤ�ʿ�Ѥη׻�
  for( int i = 0; i < 3; i++ ){
    eachAve[ i ][ 0 ] = 0;
    eachNum[ i ][ 0 ] = h[ i ][ 0 ];
    int tmp_eachAve = 0;
    for( int j = 1; j < 256; j++ ){
      eachNum[ i ][ j ] = eachNum[ i ][ j-1 ] + h[ i ][ j ];
      tmp_eachAve += j * h[ i ][ j ];
      if( eachNum[ i ][ j ] == 0 )
	eachAve[ i ][ j ] = 0;
      else
	eachAve[ i ][ j ] = (double)tmp_eachAve / eachNum[ i ][ j ];
    }
  }

  //* ���ͤη���
  //* ���饹��ʬ��������ˤʤ�Ȥ���õ��
  for( int i = 0; i < 3; i++ ){
    double max_var = 0;
    for( int j = 1; j < 256; j++ ){
      if( eachNum[ i ][ j ] != 0 ){
	if( eachNum[ i ][ j ] == totalNum ) break;
	const double ave_sub = eachAve[ i ][ j ] - totalAve[ i ];
	const double var = ave_sub * ave_sub * ( eachNum[ i ][ j ] / (double)( totalNum - eachNum[ i ][ j ] ) );
	if( var > max_var ){
	  max_var = var;
	  threshold[ i ] = j;
	}
      }
    }
  }
  
  printf("totalAverage: %f %f %f\n",  totalAve[0], totalAve[1], totalAve[2]);
  printf("threshold: %d %d %d\n",threshold[0],threshold[1],threshold[2]);

  //* ��̤�ե�����˽���
  fp = fopen( "param/color_threshold.txt","w" );
  fprintf(fp,"%d %d %d\n",threshold[0],threshold[1],threshold[2]);
  fclose(fp);

  delete[] h[ 0 ];
  delete[] h[ 1 ];
  delete[] h[ 2 ];
  delete[] h;

  return 0;
}
