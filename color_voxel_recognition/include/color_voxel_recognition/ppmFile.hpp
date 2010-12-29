#ifndef MY_PPM_HPP
#define MY_PPM_HPP

#include <iostream>

/************************************/
/* カラーテクスチャ画像（.ppm）を読み込む  */
/************************************/

using namespace std;

class Ppm{
public:
  Ppm() : imageData(NULL) {}
  ~Ppm(){ if( imageData != NULL ) delete [] imageData; }

  //* 画像の横幅取得
  const int Width() const { return width; }

  //* 画像の縦幅取得
  const int Height() const { return height; }

  //* 画像データ取得
  const unsigned char* ImageData() const { return imageData; }

  //* 画像ファイル（.ppm）の読み込み
  void read( const char *filename );

private:
  int width;  // 画像の横幅
  int height; // 画像の縦幅
  unsigned char *imageData; // 画像データ（RGB）
  char buf[ 128 ];   // 読み込み用のバッファ

  //* コメントを読み飛ばす
  void skipComment( ifstream &fin );
};

#endif
