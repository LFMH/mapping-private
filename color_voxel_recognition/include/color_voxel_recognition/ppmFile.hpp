#ifndef MY_PPM_HPP
#define MY_PPM_HPP

#include <iostream>

/************************************/
/* ���顼�ƥ������������.ppm�ˤ��ɤ߹���  */
/************************************/

using namespace std;

class Ppm{
public:
  Ppm() : imageData(NULL) {}
  ~Ppm(){ if( imageData != NULL ) delete [] imageData; }

  //* �����β�������
  const int Width() const { return width; }

  //* �����ν�������
  const int Height() const { return height; }

  //* �����ǡ�������
  const unsigned char* ImageData() const { return imageData; }

  //* �����ե������.ppm�ˤ��ɤ߹���
  void read( const char *filename );

private:
  int width;  // �����β���
  int height; // �����ν���
  unsigned char *imageData; // �����ǡ�����RGB��
  char buf[ 128 ];   // �ɤ߹����ѤΥХåե�

  //* �����Ȥ��ɤ����Ф�
  void skipComment( ifstream &fin );
};

#endif
