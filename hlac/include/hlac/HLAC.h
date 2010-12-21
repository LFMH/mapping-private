#ifndef HLAC_HPP
#define HLAC_HPP 1

#include <cmath>
#include <iostream>
#include <cv.h>

// Activate normalization of HLAC by the number of pixels
//#define ENABLE_NORMALIZATION

/********************************************
 * 以下は, 通常使用しないと思われるHLAC特徴 *
 * 使用するときは, コメントアウトを外す     *
 ********************************************/
/* 高々3次の Binary HLAC（正方形サイズ：5x5）を使用可能にする */
//#define ENABLE_BIN_HLAC3_5
/* 高々3次の Gray-scale HLAC（正方形サイズ：5x5）を使用可能にする */
//#define ENABLE_GRAY_HLAC3_5


class HLAC
{
  /*************
   *  members  *
   *************/
public:
  /* constant numbers */
  enum {
    /* for binary image */
    //DIM_OF_BIN_HLAC0_3 = 1,
    DIM_OF_BIN_HLAC1_3 = 5,    //4
    DIM_OF_BIN_HLAC2_3 = 25,   //20
    DIM_OF_BIN_HLAC3_3 = 70,   //45
    //DIM_OF_BIN_HLAC0_5 = 1,
    DIM_OF_BIN_HLAC1_5 = 13,   //12
    DIM_OF_BIN_HLAC2_5 = 193,  //180
#ifdef ENABLE_BIN_HLAC3_5
    DIM_OF_BIN_HLAC3_5 = 1642, //1449
#endif
    
    /* for gray-scale image */
    //DIM_OF_GRAY_HLAC0_3 = 1,
    DIM_OF_GRAY_HLAC1_3 = 6,    //5
    DIM_OF_GRAY_HLAC2_3 = 35,   //29
    DIM_OF_GRAY_HLAC3_3 = 153,  //118
    //DIM_OF_GRAY_HLAC0_5 = 1,
    DIM_OF_GRAY_HLAC1_5 = 14,   //13
    DIM_OF_GRAY_HLAC2_5 = 219,  //205
#ifdef ENABLE_GRAY_HLAC3_5
    DIM_OF_GRAY_HLAC3_5 = 2245, //2026
#endif
    
    /* for color image */
    //DIM_OF_COLOR_HLAC0_3 = 3,
    DIM_OF_COLOR_HLAC1_3 = 45,  //42
    DIM_OF_COLOR_HLAC2_3 = 714, //669
    //DIM_OF_COLOR_HLAC0_5 = 3,
    DIM_OF_COLOR_HLAC1_5 = 117  //114
  };
  
private:
  
  /*************
   *  methods  *
   *************/
public:
  /* constructor */
  HLAC(){}
  /* destructor */
  ~HLAC(){}
  
  /*** HLAC ***/
  /* for binary image (at most order 2) */
  void extractBin( std::vector<float> &result, const cv::Mat &img,
		   int order = 2, int size = 3,
		   int rx = 1, int ry = 1,
		   int x = -1, int dx = -1,
		   int y = -1, int dy = -1 );
  /* for gray-scale image (at most order 2) */
  void extractGray( std::vector<float> &result, const cv::Mat &img,
		    int order = 2, int size = 3,
		    int rx = 1, int ry = 1,
		    int x = -1, int dx = -1,
		    int y = -1, int dy = -1 );
  /* for color image (at most order 1) */
  void extractColor( std::vector<float> &result, const cv::Mat &img,
		     int order = 1, int size = 3,
		     int rx = 1, int ry = 1,
		     int x = -1, int dx = -1,
		     int y = -1, int dy = -1 );
  
private:
  /* copy constructor */
  HLAC( const HLAC &hlac );
  HLAC &operator=( const HLAC &hlac );
  
  /*** HLAC ***/
  /* for binary image */
  void _extractBin1_3( std::vector<float> &result, const cv::Mat &img,
		       const int &rx, const int &ry,
		       const int &left, const int &right,
		       const int &up, const int &down );
  void _extractBin2_3( std::vector<float> &result, const cv::Mat &img,
		       const int &rx, const int &ry,
		       const int &left, const int &right,
		       const int &up, const int &down );
  void _extractBin3_3( std::vector<float> &result, const cv::Mat &img,
		       const int &rx, const int &ry,
		       const int &left, const int &right,
		       const int &up, const int &down );
  void _extractBin1_5( std::vector<float> &result, const cv::Mat &img,
		       const int &rx, const int &ry,
		       const int &left, const int &right,
		       const int &up, const int &down );
  void _extractBin2_5( std::vector<float> &result, const cv::Mat &img,
		       const int &rx, const int &ry,
		       const int &left, const int &right,
		       const int &up, const int &down );
#ifdef ENABLE_BIN_HLAC3_5
  void _extractBin3_5( std::vector<float> &result, const cv::Mat &img,
		       const int &rx, const int &ry,
		       const int &left, const int &right,
		       const int &up, const int &down );
#endif  

  /* for gray-scale image */
  void _extractGray1_3( std::vector<float> &result, const cv::Mat &img,
			const int &rx, const int &ry,
			const int &left, const int &right,
			const int &up, const int &down );
  void _extractGray2_3( std::vector<float> &result, const cv::Mat &img,
			const int &rx, const int &ry,
			const int &left, const int &right,
			const int &up, const int &down );
  void _extractGray3_3( std::vector<float> &result, const cv::Mat &img,
			const int &rx, const int &ry,
			const int &left, const int &right,
			const int &up, const int &down );
  void _extractGray1_5( std::vector<float> &result, const cv::Mat &img,
			const int &rx, const int &ry,
			const int &left, const int &right,
			const int &up, const int &down );
  void _extractGray2_5( std::vector<float> &result, const cv::Mat &img,
			const int &rx, const int &ry,
			const int &left, const int &right,
			const int &up, const int &down );
#ifdef ENABLE_GRAY_HLAC3_5
  void _extractGray3_5( std::vector<float> &result, const cv::Mat &img,
			const int &rx, const int &ry,
			const int &left, const int &right,
			const int &up, const int &down );
#endif
  
  /* for color image */
  void _extractColor1_3( std::vector<float> &result, const cv::Mat &img, 
			 const int &rx, const int &ry, 
			 const int &left, const int &right, 
			 const int &up, const int &down );
  void _extractColor2_3( std::vector<float> &result, const cv::Mat &img, 
			 const int &rx, const int &ry, 
			 const int &left, const int &right, 
			 const int &up, const int &down );
  void _extractColor1_5( std::vector<float> &result, const cv::Mat &img, 
			 const int &rx, const int &ry, 
			 const int &left, const int &right, 
			 const int &up, const int &down );
  
  /* 簡単な計算やエラー処理 */
  void checkDimensionBin( std::vector<float> &result, 
			  const int &order, const int &size );
  void checkDimensionGray( std::vector<float> &result, 
			   const int &order, const int &size );
  void checkDimensionColor( std::vector<float> &result, 
			    const int &order, const int &size );
  void decideWindow( int &left, int &right, int &up, int &down,
		     const int &x, const int &dx,
		     const int &y, const int &dy,
		     const cv::Mat &img, 
		     const int &size, 
		     const int &rx, const int &ry );
  
};


#include "HLAC.hpp"


#endif
