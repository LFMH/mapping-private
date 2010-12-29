#include <string.h>
#include <stdlib.h>
#include <sstream>
#include <fstream>

#include "color_voxel_recognition/ppmFile.hpp"

enum { FILETYPE_ASCII = 3, FILETYPE_BINARY = 6 };

void Ppm::read( const char *filename ){
  
  //* �����ե�����򳫤�
  ifstream fin( filename );
  if ( fin == NULL ){
    cerr << "ERR (in Ppm::read): cannot open file: " << filename << endl;
    exit( EXIT_FAILURE );
  }
  
  //* �����������ɤ߹���
  int fileType;
  skipComment( fin );
  if( buf[ 0 ] != 'P' ){
    cerr << "ERR (in Ppm::read): invalid ppm file" << endl;
    exit( EXIT_FAILURE );
  }
  else
    fileType = (int)( buf[ 1 ] - '0' );
  
  //* �����νĲ��������������
  skipComment( fin );
  sscanf( buf, "%d %d", &width, &height );
  const int size = width * height * 3;
  imageData = new unsigned char[ size ];  

  //* �����ǡ������ɤ߹���
  skipComment( fin );
  switch( fileType ){
  case FILETYPE_ASCII:
    {
      int buf_i;
      for( int i = 0; i < height; i++ ){
	for( int j = 0; j < width; j++ ){
	  for( int c = 0; c < 3; c++ ){
	    fin >> buf_i;
	    imageData[ ( j + i * width ) * 3 + c ] = (unsigned char) buf_i;
	  }
	}
      }
    }
    break;
  case FILETYPE_BINARY:
    {
      fin.read( (char *)imageData, size );
    }
    break;
  default:
    cerr << "ERR (in Ppm::read): invalid ppm file" << endl;
    exit( EXIT_FAILURE );
  }  
}

//**************//
//* private�ؿ� *//
//**************//

//* �����Ȥ��ɤ����Ф�
inline void Ppm::skipComment( ifstream &fin ){
  while( 1 ){
    fin.getline( buf, 128 );
    if( buf[ 0 ] != '#' ){
      const int len = strlen( buf );
      for( int i = 0; i < len; ++i )
	if( buf[ i ] != '\0' || buf[ i ] != ' ' )
	  return;
    }
  }
}
