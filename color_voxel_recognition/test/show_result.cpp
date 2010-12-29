#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>

/*********************************************************************/
/* �����ΰ�κ�ɸ���񤫤줿�ե�����ˤ���Ƹ����о�ʪ�Τ�¸���ΰ����Ӥ���         */
/* ȯ���ΰ褬��äƤ����餽���ΰ�Υ�󥯤򡢤��֤�ʤ��ä��� x �Ȥ��Ʒ�̤�ɽ������ */
/* �ե����ޥå�                                                         */
/*   <���Ф��줿�ΰ�ν��>: <���н��׻���> [�����о�ʪ�Τ�̾��]               */
/*********************************************************************/

using namespace std;

bool overlap( int sx, int dx, int sy, int dy, int sz, int dz, int _sx, int _dx, int _sy, int _dy, int _sz, int _dz ){
  int val1 = sx - _sx;
  if( val1 < 0 )    val1 = - val1 - dx;
  else              val1 -= _dx;
  int val2 = sy - _sy;
  if( val2 < 0 )    val2 = - val2 - dy;
  else              val2 -= _dy;
  int val3 = sz - _sz;
  if( val3 < 0 )    val3 = - val3 - dz;
  else              val3 -= _dz;

  if( ( val1 <= 0 ) && ( val2 <= 0 ) && ( val3 <= 0 ) )
    return true;
  
  return false;
}

int main( int argc, char* argv[])
{
  if( argc != 4 ){
    cerr << "usage: " << argv[0] << " [dir_name] [answer_file_name] <model_num>" << endl;
    exit( EXIT_FAILURE );
  }
  char line[100];
  char name[100];
  char tmpname[100];

  int model_num = atoi(argv[3]);
  FILE *fp_a = fopen( argv[2], "r" );

  int sx, dx, sy, dy, sz, dz;
  int _sx, _dx, _sy, _dy, _sz, _dz;
  double time;
  for( int m=0; m<model_num; m++ ){
    fscanf(fp_a,"%s\n",name);
    fscanf(fp_a,"%d %d %d %d %d %d\n",&sx,&dx,&sy,&dy,&sz,&dz);
    fscanf(fp_a,"\n");

    sprintf(tmpname,"%s/%s.txt",argv[1],name);
    FILE *fp_m = fopen( tmpname, "r" );

    bool break_flg = false;
    int rank = 0;
    while( fgets(line,sizeof(line),fp_m)!=NULL ){
      rank++;
      if( line[0]=='t' ){
	if( !break_flg )
	  printf(" x: ");
	sscanf(line+6,"%lf\n",&time);
	printf("%f %s\n",time,name);
	break;
      }

      if( !break_flg ){
	sscanf(line,"%d %d %d %d %d %d\n",&_sx,&_dx,&_sy,&_dy,&_sz,&_dz);
	if( overlap( sx, dx, sy, dy, sz, dz, _sx, _dx, _sy, _dy, _sz, _dz ) ){
	  printf("%2d: ",rank);
	  break_flg = true;
	}
      }
    }
    fclose(fp_m);
  }
  return 0;
}

  
