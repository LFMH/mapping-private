#include <stdio.h>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include "color_voxel_recognition/objFile.hpp"

using namespace std;

//************************************
//* .objファイルからメッシュデータを読み込む
//* 注 objファイルフォーマットについて： 
//*  (1) メッシュは "頂点座標値番号/テクスチャ座標値番号" のフォーマットに限る
//*      （頂点法線ベクトル番号がある場合はsscanfの読み込み方を変えてください）
//*  (2) 三角メッシュに限る
//*      （四角メッシュがある場合は三角メッシュに変換してください）
void Obj::readMesh( const char *filename, bool bmp_texmap_flg ){

  //* まずは頂点と面の数を数える v_num, vt_num, f_num
  FILE *fp=fopen(filename,"r");
  char line[200];
  while( fgets(line,sizeof(line),fp)!=NULL ){
    if( line[0]=='v' && line[1]==' ' )
      v_num++;
    else if( line[0]=='v' && line[1]=='t' && line[2]==' ' )
      vt_num++;
    else if( line[0]=='f' && line[1]==' ' )
      f_num++;    
  }
  fclose(fp);

  //* メモリ確保
  vertices = new Vertex[ v_num ];
  verticesT = new VertexT[ vt_num ];
  faces = new Face[ f_num ];

  //* 読み込み
  char tmpline[100];
  int now_v=0, now_vt=0, now_f=0;
  fp = fopen( filename, "r" );
  while( fgets(line,sizeof(line),fp)!=NULL ){
    if( line[0]=='v' && line[1]==' ' ){ // 頂点
      sscanf(line,"%s %f %f %f",tmpline, &(vertices[ now_v ].x), &(vertices[ now_v ].y), &(vertices[ now_v ].z) );
      now_v++;
    }
    else if( line[0]=='v' && line[1]=='t' && line[2]==' ' ){// テクスチャ頂点
      sscanf(line,"%s %f %f",tmpline, &(verticesT[ now_vt ].x), &(verticesT[ now_vt ].y) );
      if( bmp_texmap_flg )  // 注 上下を逆に
	verticesT[ now_vt ].y = 1 - verticesT[ now_vt ].y;
      now_vt++;
    }
    else if( line[0]=='f' && line[1]==' ' ){ // メッシュ
      sscanf(line,"%s %d/%d %d/%d %d/%d",tmpline, &(faces[ now_f ].v1), &(faces[ now_f ].vt1), &(faces[ now_f ].v2), &(faces[ now_f ].vt2), &(faces[ now_f ].v3), &(faces[ now_f ].vt3) );

      //* 1からはじまる番号を0からにする
      faces[ now_f ].v1 -= 1;
      faces[ now_f ].v2 -= 1;
      faces[ now_f ].v3 -= 1;
      faces[ now_f ].vt1 -= 1;
      faces[ now_f ].vt2 -= 1;
      faces[ now_f ].vt3 -= 1;

      //* 頂点座標の最小値、最大値を保存
      setMinMax( faces[ now_f ].v1 );
      setMinMax( faces[ now_f ].v2 );
      setMinMax( faces[ now_f ].v3 );

      now_f++;
    }
  }
  fclose(fp);
}


//*******************************
//* 点群データを読み込む（ファイルから）
void Obj::readPoints( const char *filenameX, const char *filenameY, const char *filenameZ, const int _width, const int _height, const int _image_width, const int _image_height, const float resize_rate, const float aspect_rate, const int _shiftX, const int _shiftY, const int dis_1_pixel, bool ascii ){
  readPoints( filenameX, filenameY, filenameZ, NULL, _width, _height, _image_width, _image_height, resize_rate, aspect_rate, _shiftX, _shiftY, dis_1_pixel, ascii );
}

//****************************************************
//* 点群を読み込む（ファイルから）SR4000などで、信頼度も読み込む
void Obj::readPoints( const char *filenameX, const char *filenameY, const char *filenameZ, const char *filenameC, const int _width, const int _height, const int _image_width, const int _image_height, const float resize_rate, const float aspect_rate, const int _shiftX, const int _shiftY, const int dis_1_pixel, bool ascii ){
  width  = _width;
  height = _height;
  v_num  = width * height;
  vt_num = v_num;
  vertices = new Vertex[ v_num ];
  verticesT = new VertexT[ vt_num ];
  if( filenameC != NULL )
    confidences = new unsigned char[ v_num ];
  faces = new Face[ ( width - 1 ) * ( height - 1 ) * 2 ]; // 注）最大の見積りでメモリ確保。f_numはまだ0

  const int image_width = _image_width * resize_rate * aspect_rate;
  const int image_height = _image_height * resize_rate;
  const float shiftX = ( 0.5 + _shiftX ) / image_width;
  const float shiftY = ( 0.5 + _shiftY ) / image_height;
  int shiftXadd;

  FILE *fpX, *fpY, *fpZ, *fpC=NULL;
  if( ascii ){
    fpX = fopen( filenameX, "r" );
    fpY = fopen( filenameY, "r" );
    fpZ = fopen( filenameZ, "r" );
    if( filenameC != NULL )
      fpC = fopen( filenameC, "r" );
  }
  else{
    fpX = fopen( filenameX, "rb" );
    fpY = fopen( filenameY, "rb" );
    fpZ = fopen( filenameZ, "rb" );
    if( filenameC != NULL )
      fpC = fopen( filenameC, "rb" );
  }

  int idx = 0;
  int tmpval;
  for(int j=0;j<height;j++){
    for(int i=0;i<width;i++){
      if( ascii ){
	fscanf( fpX, "%f", &( vertices[ idx ].x ) );
	fscanf( fpY, "%f", &( vertices[ idx ].y ) );
	fscanf( fpZ, "%f", &( vertices[ idx ].z ) );
	if( filenameC != NULL ){
	  fscanf( fpC, "%d", &tmpval );
	  confidences[ idx ] = tmpval;
	}
      }
      else{
	fread( &( vertices[ idx ].x ), sizeof(float), 1, fpX );
	fread( &( vertices[ idx ].y ), sizeof(float), 1, fpY );
	fread( &( vertices[ idx ].z ), sizeof(float), 1, fpZ );
	if( filenameC != NULL )
	  fread( confidences + idx, sizeof(unsigned char), 1, fpC );
      }

      if( vertices[ idx ].z == 0 )
	shiftXadd = 0;
      else{
	shiftXadd = (int)( dis_1_pixel / vertices[ idx ].z );
	while(1){
	  if( i + shiftXadd + shiftX < image_width ) break;
	  shiftXadd--;
	}
      }
      verticesT[ idx ].x = (float)( i + shiftXadd ) / image_width + shiftX;
      verticesT[ idx ].y = (float)j / image_height + shiftY;

      //* 頂点座標の最小値、最大値を保存
      //setMinMax( idx );

      idx++;
    }
  }
  fclose(fpX);
  fclose(fpY);
  fclose(fpZ);
  if( filenameC != NULL ) fclose(fpC);
}

#ifdef USE_OPENCV
//***********************************
//* 点群データを読み込む（OpenCVの配列から）
void Obj::readPoints( const cv::Mat& matX, const cv::Mat& matY, const cv::Mat& matZ, const int _width, const int _height, const int _image_width, const int _image_height, const float resize_rate, const float aspect_rate, const int _shiftX, const int _shiftY, const int dis_1_pixel ){
  width  = _width;
  height = _height;
  v_num  = width * height;
  vt_num = v_num;
  vertices = new Vertex[ v_num ];
  verticesT = new VertexT[ vt_num ];
  faces = new Face[ ( width - 1 ) * ( height - 1 ) * 2 ]; // 注）最大の見積りでメモリ確保。f_numはまだ0

  const int image_width = _image_width * resize_rate * aspect_rate;
  const int image_height = _image_height * resize_rate;
  const float shiftX = ( 0.5 + _shiftX ) / image_width;
  const float shiftY = ( 0.5 + _shiftY ) / image_height;
  int shiftXadd;

  int idx = 0;
  for(int j=0;j<height;j++){
    for(int i=0;i<width;i++){
      vertices[ idx ].x = matX.at<float>(j,i);
      vertices[ idx ].y = matY.at<float>(j,i);
      vertices[ idx ].z = matZ.at<float>(j,i);
      if( vertices[ idx ].z == 0 )
	shiftXadd = 0;
      else{
	shiftXadd = (int)( dis_1_pixel / vertices[ idx ].z );
	while(1){
	  if( i + shiftXadd + shiftX < image_width ) break;
	  shiftXadd--;
	}
      }
      verticesT[ idx ].x = (float)( i + shiftXadd ) / image_width + shiftX;
      verticesT[ idx ].y = (float)j / image_height + shiftY;

      //* 頂点座標の最小値、最大値を保存
      //setMinMax( idx );

      idx++;
    }
  }
}

//********************************************************
//* 点群を読み込む（OpenCVの配列から）SR4000などで，信頼度も読み込む
void Obj::readPoints( const cv::Mat& matX, const cv::Mat& matY, const cv::Mat& matZ, const cv::Mat& matC, const int _width, const int _height, const int _image_width, const int _image_height, const float resize_rate, const float aspect_rate, const int _shiftX, const int _shiftY, const int dis_1_pixel ){
  width  = _width;
  height = _height;
  v_num  = width * height;
  vt_num = v_num;
  vertices = new Vertex[ v_num ];
  verticesT = new VertexT[ vt_num ];
  confidences = new unsigned char[ v_num ];
  faces = new Face[ ( width - 1 ) * ( height - 1 ) * 2 ]; // 注）最大の見積りでメモリ確保。f_numはまだ0

  const int image_width = _image_width * resize_rate * aspect_rate;
  const int image_height = _image_height * resize_rate;
  const float shiftX = ( 0.5 + _shiftX ) / image_width;
  const float shiftY = ( 0.5 + _shiftY ) / image_height;
  int shiftXadd;

  int idx = 0;
  for(int j=0;j<height;j++){
    for(int i=0;i<width;i++){
      vertices[ idx ].x = matX.at<float>(j,i);
      vertices[ idx ].y = matY.at<float>(j,i);
      vertices[ idx ].z = matZ.at<float>(j,i);
      if( vertices[ idx ].z == 0 )
	shiftXadd = 0;
      else{
	shiftXadd = (int)( dis_1_pixel / vertices[ idx ].z );
	while(1){
	  if( i + shiftXadd + shiftX < image_width ) break;
	  shiftXadd--;
	}
      }
      verticesT[ idx ].x = (float)( i + shiftXadd ) / image_width + shiftX;
      verticesT[ idx ].y = (float)j / image_height + shiftY;
      confidences[ idx ] = matC.at<unsigned char>(j,i);

      //* 頂点座標の最小値、最大値を保存
      //setMinMax( idx );

      idx++;
    }
  }
}
#endif

//********************************
//* 点群からメッシュ作成 Overlapを除く
void Obj::getMesh( const char* filenameM, float length_max_rate, float length_th, float distance_th, unsigned char confidence_th, bool ascii ){
  const int width_1 = width - 1;
  const int height_1 = height - 1;
  length_th *= length_th; // 二乗してしまう

  unsigned char *delete_mask = new unsigned char[ width*height ];
  FILE *fp;
  int tmpval;  int idx = 0;
  if( ascii ){
    fp = fopen( filenameM, "r" );
    for(int j=0;j<height;j++){
      for(int i=0;i<width;i++){
	fscanf( fp, "%d", &tmpval );
	delete_mask[ idx++ ] = tmpval;
      }
    }
  }
  else{
    fp = fopen( filenameM, "rb" );
    for(int j=0;j<height;j++)
      for(int i=0;i<width;i++)
	fread( delete_mask + (idx++), sizeof(unsigned char), 1, fp );
  }

  idx = -1; // 注意
  float length1, length2, length_max, lengthR, lengthB;
  for( int j = 0; j < height_1; j++ ){
    idx++;
    for( int i = 0; i < width_1; i++ ){
      //if( checkConfidence( idx, confidence_th ) && ( delete_mask.at<unsigned char>(j,i) == 255 ) && ( delete_mask.at<unsigned char>(j,i+1) == 255 ) && ( delete_mask.at<unsigned char>(j+1,i) == 255 ) && ( delete_mask.at<unsigned char>(j+1,i+1) == 255 ) ){
      if( checkConfidence( idx, confidence_th ) && ( delete_mask[ idx ] == 255 ) && ( delete_mask[ idx+1 ] == 255 ) && ( delete_mask[ idx + width ] == 255 ) && ( delete_mask[ idx + width + 1 ] == 255 ) ){
	if( ( vertices[ idx ].z < distance_th ) && ( vertices[ idx + 1 ].z < distance_th ) && ( vertices[ idx + width ].z < distance_th ) && ( vertices[ idx + width + 1 ].z < distance_th ) ){
	  //* 右隣との距離
	  lengthR = sq_len( vertices[ idx ], vertices[ idx + 1 ] );
	  
	  //* 真下との距離
	  lengthB = sq_len( vertices[ idx ], vertices[ idx + width ] );

	  //* length_maxを決める
	  if( lengthR > lengthB ) length_max = lengthR * length_max_rate;
	  else                    length_max = lengthB * length_max_rate;

	  if( length_max < length_th ){
	    //* 左上から右下
	    length1 = sq_len( vertices[ idx ], vertices[ idx + 1 + width ] );
	
	    //* 右上から左下
	    length2 = sq_len( vertices[ idx + 1 ], vertices[ idx + width ] );

	    //if( (length1 > 0.000001) && (length2 > 0.000001) ){	
	    if( (length1 > FLT_MIN) && (length2 > FLT_MIN) ){	
	      //* 右上から左下か、左上から右下か。短い方を選ぶ
	      if( length1 < length_max ){
		if( length2 < length1 ){ // -> 右上から左下 を選ぶ
		  if( ( lengthR < length_max ) && ( lengthB < length_max ) ){
		    faces[ f_num ].v1 = idx;
		    faces[ f_num ].v2 = idx + width;
		    faces[ f_num ].v3 = idx + 1;
		    faces[ f_num ].vt1 = idx;
		    faces[ f_num ].vt2 = idx + width;
		    faces[ f_num ].vt3 = idx + 1;
		    f_num++;
		    setMinMax( idx );
		    setMinMax( idx + width );
		    setMinMax( idx + 1 );
		  }
		  if( ( sq_len( vertices[ idx + width + 1 ], vertices[ idx + 1 ] ) < length_max ) && ( sq_len( vertices[ idx + width + 1 ], vertices[ idx + width ] ) < length_max ) ){
		    faces[ f_num ].v1 = idx + width + 1;
		    faces[ f_num ].v2 = idx + 1;
		    faces[ f_num ].v3 = idx + width;
		    faces[ f_num ].vt1 = idx + width + 1;
		    faces[ f_num ].vt2 = idx + 1;
		    faces[ f_num ].vt3 = idx + width;
		    f_num++;
		    setMinMax( idx + width + 1 );
		    setMinMax( idx + 1 );
		    setMinMax( idx + width );
		  }
		}
		else{ // -> 左上から右下 を選ぶ
		  if( ( sq_len( vertices[ idx ], vertices[ idx + width + 1 ] ) < length_max ) && ( lengthB < length_max ) ){
		    faces[ f_num ].v1 = idx;
		    faces[ f_num ].v2 = idx + width;
		    faces[ f_num ].v3 = idx + width + 1;
		    faces[ f_num ].vt1 = idx;
		    faces[ f_num ].vt2 = idx + width;
		    faces[ f_num ].vt3 = idx + width + 1;
		    f_num++;
		    setMinMax( idx );
		    setMinMax( idx + width );
		    setMinMax( idx + width + 1 );
		  }
		  if( ( sq_len( vertices[ idx ], vertices[ idx + width + 1 ] ) < length_max ) && ( lengthR < length_max ) ){
		    faces[ f_num ].v1 = idx;
		    faces[ f_num ].v2 = idx + width + 1;
		    faces[ f_num ].v3 = idx + 1;
		    faces[ f_num ].vt1 = idx;
		    faces[ f_num ].vt2 = idx + width + 1;
		    faces[ f_num ].vt3 = idx + 1;
		    f_num++;
		    setMinMax( idx );
		    setMinMax( idx + width + 1 );
		    setMinMax( idx + 1 );
		  }
		}
	      }
	      else if( length2 < length_max ){ // -> 右上から左下 を選ぶ
		if( ( lengthR < length_max ) && ( lengthB < length_max ) ){
		  faces[ f_num ].v1 = idx;
		  faces[ f_num ].v2 = idx + width;
		  faces[ f_num ].v3 = idx + 1;
		  faces[ f_num ].vt1 = idx;
		  faces[ f_num ].vt2 = idx + width;
		  faces[ f_num ].vt3 = idx + 1;
		  f_num++;
		  setMinMax( idx );
		  setMinMax( idx + width );
		  setMinMax( idx + 1 );
		}
		if( ( sq_len( vertices[ idx + width + 1 ], vertices[ idx + 1 ] ) < length_max ) && ( sq_len( vertices[ idx + width + 1 ], vertices[ idx + width ] ) < length_max ) ){
		  faces[ f_num ].v1 = idx + width + 1;
		  faces[ f_num ].v2 = idx + 1;
		  faces[ f_num ].v3 = idx + width;
		  faces[ f_num ].vt1 = idx + width + 1;
		  faces[ f_num ].vt2 = idx + 1;
		  faces[ f_num ].vt3 = idx + width;
		  f_num++;
		  setMinMax( idx + width + 1 );
		  setMinMax( idx + 1 );
		  setMinMax( idx + width );
		}
	      }
	    }
	  }
	}
      }
      idx++;
    }
  }
  delete delete_mask;
}

//*******************
//* 点群からメッシュ作成
void Obj::getMesh( float length_max_rate, float length_th, float distance_th, unsigned char confidence_th ){
  const int width_1 = width - 1;
  const int height_1 = height - 1;
  length_th *= length_th; // 二乗してしまう

  int idx = -1; // 注意
  float length1, length2, length_max, lengthR, lengthB;
  for( int j = 0; j < height_1; j++ ){
    idx++;
    for( int i = 0; i < width_1; i++ ){
      if( checkConfidence( idx, confidence_th ) ){
	if( ( vertices[ idx ].z < distance_th ) && ( vertices[ idx + 1 ].z < distance_th ) && ( vertices[ idx + width ].z < distance_th ) && ( vertices[ idx + width + 1 ].z < distance_th ) ){
	  //* 右隣との距離
	  lengthR = sq_len( vertices[ idx ], vertices[ idx + 1 ] );

	  //* 真下との距離
	  lengthB = sq_len( vertices[ idx ], vertices[ idx + width ] );

	  //* length_maxを決める
	  if( lengthR > lengthB ) length_max = lengthR * length_max_rate;
	  else                    length_max = lengthB * length_max_rate;

	  if( length_max < length_th ){
	    //* 左上から右下
	    length1 = sq_len( vertices[ idx ], vertices[ idx + 1 + width ] );
	
	    //* 右上から左下
	    length2 = sq_len( vertices[ idx + 1 ], vertices[ idx + width ] );

	    //if( (length1 > 0.000001) && (length2 > 0.000001) ){	
	    if( (length1 > FLT_MIN) && (length2 > FLT_MIN) ){	
	      //* 右上から左下か、左上から右下か。短い方を選ぶ
	      if( length1 < length_max ){
		if( length2 < length1 ){ // -> 右上から左下 を選ぶ
		  if( ( lengthR < length_max ) && ( lengthB < length_max ) ){
		    faces[ f_num ].v1 = idx;
		    faces[ f_num ].v2 = idx + width;
		    faces[ f_num ].v3 = idx + 1;
		    faces[ f_num ].vt1 = idx;
		    faces[ f_num ].vt2 = idx + width;
		    faces[ f_num ].vt3 = idx + 1;
		    f_num++;
		    setMinMax( idx );
		    setMinMax( idx + width );
		    setMinMax( idx + 1 );
		  }
		  if( ( sq_len( vertices[ idx + width + 1 ], vertices[ idx + 1 ] ) < length_max ) && ( sq_len( vertices[ idx + width + 1 ], vertices[ idx + width ] ) < length_max ) ){
		    faces[ f_num ].v1 = idx + width + 1;
		    faces[ f_num ].v2 = idx + 1;
		    faces[ f_num ].v3 = idx + width;
		    faces[ f_num ].vt1 = idx + width + 1;
		    faces[ f_num ].vt2 = idx + 1;
		    faces[ f_num ].vt3 = idx + width;
		    f_num++;
		    setMinMax( idx + width + 1 );
		    setMinMax( idx + 1 );
		    setMinMax( idx + width );
		  }
		}
		else{ // -> 左上から右下 を選ぶ
		  if( ( sq_len( vertices[ idx ], vertices[ idx + width + 1 ] ) < length_max ) && ( lengthB < length_max ) ){
		    faces[ f_num ].v1 = idx;
		    faces[ f_num ].v2 = idx + width;
		    faces[ f_num ].v3 = idx + width + 1;
		    faces[ f_num ].vt1 = idx;
		    faces[ f_num ].vt2 = idx + width;
		    faces[ f_num ].vt3 = idx + width + 1;
		    f_num++;
		    setMinMax( idx );
		    setMinMax( idx + width );
		    setMinMax( idx + width + 1 );
		  }
		  if( ( sq_len( vertices[ idx ], vertices[ idx + width + 1 ] ) < length_max ) && ( lengthR < length_max ) ){
		    faces[ f_num ].v1 = idx;
		    faces[ f_num ].v2 = idx + width + 1;
		    faces[ f_num ].v3 = idx + 1;
		    faces[ f_num ].vt1 = idx;
		    faces[ f_num ].vt2 = idx + width + 1;
		    faces[ f_num ].vt3 = idx + 1;
		    f_num++;
		    setMinMax( idx );
		    setMinMax( idx + width + 1 );
		    setMinMax( idx + 1 );
		  }
		}
	      }
	      else if( length2 < length_max ){ // -> 右上から左下 を選ぶ
		if( ( lengthR < length_max ) && ( lengthB < length_max ) ){
		  faces[ f_num ].v1 = idx;
		  faces[ f_num ].v2 = idx + width;
		  faces[ f_num ].v3 = idx + 1;
		  faces[ f_num ].vt1 = idx;
		  faces[ f_num ].vt2 = idx + width;
		  faces[ f_num ].vt3 = idx + 1;
		  f_num++;
		  setMinMax( idx );
		  setMinMax( idx + width );
		  setMinMax( idx + 1 );
		}
		if( ( sq_len( vertices[ idx + width + 1 ], vertices[ idx + 1 ] ) < length_max ) && ( sq_len( vertices[ idx + width + 1 ], vertices[ idx + width ] ) < length_max ) ){
		  faces[ f_num ].v1 = idx + width + 1;
		  faces[ f_num ].v2 = idx + 1;
		  faces[ f_num ].v3 = idx + width;
		  faces[ f_num ].vt1 = idx + width + 1;
		  faces[ f_num ].vt2 = idx + 1;
		  faces[ f_num ].vt3 = idx + width;
		  f_num++;
		  setMinMax( idx + width + 1 );
		  setMinMax( idx + 1 );
		  setMinMax( idx + width );
		}
	      }
	    }
	  }
	}
      }
      idx++;
    }
  }
}


//************************************
//* メッシュデータを.objファイルに書き出す
void Obj::writeMesh( const char *filename ){
  // すべての番号に 1 をたす
  for( int i=0; i<f_num; i++ ){
    faces[ i ].v1++;
    faces[ i ].v2++;
    faces[ i ].v3++;
    faces[ i ].vt1++;
    faces[ i ].vt2++;
    faces[ i ].vt3++;
  }

  FILE *fp = fopen( filename,"w" );
  for( int i=0; i<v_num; i++ )
    fprintf(fp,"v %.8g %.8g %.8g\n",vertices[ i ].x, vertices[ i ].y, vertices[ i ].z );
  for( int i=0; i<vt_num; i++ )
    fprintf(fp,"vt %.8g %.8g\n",verticesT[ i ].x, verticesT[ i ].y );
  for( int i=0; i<f_num; i++ )
    fprintf(fp,"f %d/%d %d/%d %d/%d\n",faces[ i ].v1, faces[ i ].vt1, faces[ i ].v2, faces[ i ].vt2, faces[ i ].v3, faces[ i ].vt3 );
  fclose(fp);
}

//**********************
//* mm単位をm単位に変換する
void Obj::mm2m(){
  for( int v=0;v<v_num;v++ ){
    vertices[ v ].x /= 1000;
    vertices[ v ].y /= 1000;
    vertices[ v ].z /= 1000;
  }
  x_min /= 1000;
  y_min /= 1000;
  z_min /= 1000;
  x_max /= 1000;
  y_max /= 1000;
  z_max /= 1000;
}

//***************************
//* メッシュをコピーし、回転させる
void Obj::rotateMesh( Obj& input, double roll, double pan, double roll2 ){
  //* 個数情報のコピー
  v_num = input.v_num;
  f_num = input.f_num;
  vt_num = input.vt_num;

  //* メモリ確保
  vertices = new Vertex[ v_num ];
  verticesT = new VertexT[ vt_num ];
  faces = new Face[ f_num ];

  //* 回転行列
  double R1[9];
  R1[0]=cos(roll);
  R1[1]=-sin(roll);
  R1[2]=0;
  R1[3]=sin(roll);
  R1[4]=cos(roll);
  R1[5]=0;
  R1[6]=0;
  R1[7]=0;
  R1[8]=1;
  double R2[9];
  R2[0]=cos(pan);
  R2[1]=0;
  R2[2]=sin(pan);
  R2[3]=0;
  R2[4]=1;
  R2[5]=0;
  R2[6]=-sin(pan);
  R2[7]=0;
  R2[8]=cos(pan);
  double R3[9];
  R3[0]=cos(roll2);
  R3[1]=-sin(roll2);
  R3[2]=0;
  R3[3]=sin(roll2);
  R3[4]=cos(roll2);
  R3[5]=0;
  R3[6]=0;
  R3[7]=0;
  R3[8]=1;

  //* 頂点座標に回転をかける
  float x1, y1, z1, x2, y2, z2;
  for( int i=0; i<v_num; i++ ){
    x1 = R1[0] * input.vertices[ i ].x + R1[1] * input.vertices[ i ].y + R1[2] * input.vertices[ i ].z;
    y1 = R1[3] * input.vertices[ i ].x + R1[4] * input.vertices[ i ].y + R1[5] * input.vertices[ i ].z;
    z1 = R1[6] * input.vertices[ i ].x + R1[7] * input.vertices[ i ].y + R1[8] * input.vertices[ i ].z;
    
    x2 = R2[0]*x1 + R2[1]*y1 + R2[2]*z1;
    y2 = R2[3]*x1 + R2[4]*y1 + R2[5]*z1;
    z2 = R2[6]*x1 + R2[7]*y1 + R2[8]*z1;
	    
    vertices[ i ].x = R3[0]*x2 + R3[1]*y2 + R3[2]*z2;
    vertices[ i ].y = R3[3]*x2 + R3[4]*y2 + R3[5]*z2;
    vertices[ i ].z = R3[6]*x2 + R3[7]*y2 + R3[8]*z2;
  }

  //* テクスチャ頂点情報をコピー
  for( int i=0; i<vt_num; i++ ){
    verticesT[ i ].x = input.verticesT[ i ].x;
    verticesT[ i ].y = input.verticesT[ i ].y;
  }

  //* メッシュ情報をコピー
  for( int i=0; i<f_num; i++ ){
    faces[ i ].v1 = input.faces[ i ].v1;
    faces[ i ].v2 = input.faces[ i ].v2;
    faces[ i ].v3 = input.faces[ i ].v3;
    faces[ i ].vt1 = input.faces[ i ].vt1;
    faces[ i ].vt2 = input.faces[ i ].vt2;
    faces[ i ].vt3 = input.faces[ i ].vt3;

    //* 頂点座標の最小値、最大値を保存
    setMinMax( faces[ i ].v1 );
    setMinMax( faces[ i ].v2 );
    setMinMax( faces[ i ].v3 );
  }
}

//******************************************
//* メッシュを回転/移動させる
//* 読み込むカメラ座標の単位はmを仮定しているのに注意
void Obj::transMesh( const char *filename, UnitMode mode ){
  float x0,y0,z0;
  float R[9];
  FILE *fp=fopen(filename,"r");
  if( fp==NULL ){
    cerr << "Warning (in Obj::transMesh): Camera file \"" << filename << "\" not found. The mesh was not transformed." << endl;
    return;
  }

  fscanf(fp,"%f %f %f",&x0,&y0,&z0);
  for(int i=0;i<9;i++)
    fscanf(fp,"%f ",R+i);
  fclose(fp);

  if( mode == MILLIMETER_MODE ){
    x0 *= 1000; // 単位をmmにする
    y0 *= 1000; // 単位をmmにする
    z0 *= 1000; // 単位をmmにする
  }
    
  //* 頂点座標に回転/移動をかける
  float x,y,z;
  for( int i=0; i<v_num; i++ ){
    x = vertices[ i ].x;
    y = vertices[ i ].y;
    z = vertices[ i ].z;
    vertices[ i ].x = R[0] * x + R[1] * y + R[2] * z + x0;
    vertices[ i ].y = R[3] * x + R[4] * y + R[5] * z + y0;
    vertices[ i ].z = R[6] * x + R[7] * y + R[8] * z + z0;
  }
}
    
const Obj& Obj::operator=( const Obj &another ){
  if( v_num != another.v_num ){
    if( vertices != NULL ) delete[] vertices;
    if( confidences != NULL )  delete[] confidences;
    vertices = new Vertex[ another.v_num ];
    confidences = new unsigned char[ another.v_num ];
  }
  if( vt_num != another.vt_num ){
    if( verticesT != NULL ) delete[] verticesT;
    verticesT = new VertexT[ another.vt_num ];
  }
  if( f_num != another.f_num ){
    if( faces != NULL )  delete[] faces;
    faces = new Face[ another.f_num ];
  }

  v_num = another.v_num;
  vt_num = another.vt_num;
  f_num = another.f_num;
  width = another.width;
  height = another.height;
  x_min = another.x_min;
  x_max = another.x_max;
  y_min = another.y_min;
  y_max = another.y_max;
  z_min = another.z_min;
  z_max = another.z_max;
  texture_id = another.texture_id;

  for( int i=0; i<v_num; i++ ){
    vertices[ i ] = another.vertices[ i ];
    confidences[ i ] = another.confidences[ i ];
  }
  for( int i=0; i<vt_num; i++ )
    verticesT[ i ] = another.verticesT[ i ];
  for( int i=0; i<f_num; i++ )
    faces[ i ] = another.faces[ i ];

  return *this;
}

//**************//
//* private関数 *//
//**************//

inline float Obj::sq_len( Vertex v1, Vertex v2 ){
  const float tmpval1 = v1.x - v2.x;
  const float tmpval2 = v1.y - v2.y;
  const float tmpval3 = v1.z - v2.z;
  return tmpval1 * tmpval1 + tmpval2 * tmpval2 + tmpval3 * tmpval3;
}

inline bool Obj::checkConfidence( int idx, unsigned char confidence_th ){
  if( confidences == NULL ) return true;
  if( ( confidences[ idx ] > confidence_th ) && ( confidences[ idx + 1 ] > confidence_th ) && ( confidences[ idx + width ] > confidence_th ) && ( confidences[ idx + width + 1 ] > confidence_th ) )
    return true;
  return false;
}

inline void Obj::setMinMax( int vertex_num ){
  if( vertices[ vertex_num ].x < x_min )	x_min = vertices[ vertex_num ].x;
  if( vertices[ vertex_num ].x > x_max )	x_max = vertices[ vertex_num ].x;
  if( vertices[ vertex_num ].y < y_min )	y_min = vertices[ vertex_num ].y;
  if( vertices[ vertex_num ].y > y_max )	y_max = vertices[ vertex_num ].y;
  if( vertices[ vertex_num ].z < z_min )	z_min = vertices[ vertex_num ].z;
  if( vertices[ vertex_num ].z > z_max )	z_max = vertices[ vertex_num ].z;  
}
