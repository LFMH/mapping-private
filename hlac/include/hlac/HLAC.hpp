/********************************************
 *  3 x 3 square の場合                     *
 *                                          *
 *  ( x, y ) : location of mask             *
 *                                          *
 *  ( 0, 0 ) corresponds to position vector *
 *                                          *
 *  ( -1, -1 ) | ( 0, -1 ) | ( 1, -1 )      *
 *  ----------------------------------      *
 *  ( -1,  0 ) | ( 0,  0 ) | ( 1,  0 )      *
 *  ----------------------------------      *
 *  ( -1,  1 ) | ( 0,  1 ) | ( 1,  1 )      *
 *                                          *
 ********************************************/


/*** HLAC特徴をとる関数 ********************************************
 *
 * result  : HLAC feature vector
 * ucImage : binary image
 * order   : order of HLAC
 * size    : square size
 * rx      : mask width of horizontal direction
 * ry      : mask width of vertical direction
 *
 * position vector in { ( X, Y )| x <= X < x + dx, y <= Y < y + dy }
 *
 *******************************************************************/
/*--------------------*
 *  for binary image  *
 *--------------------*/
 
void HLAC::extractBin( std::vector<float> &result, const cv::Mat &img, 
			  int order, int size, int rx, int ry, 
			  int x, int dx, int y, int dy )
{
  /*** check image type ***/
  if( img.channels() != 1 ){
    std::cerr << "HLAC:extractBin(): binary image is needed" 
	      << std::endl;
    exit(1);
  }
  
  /*** マスク幅をチェック ***/
  if( rx < 1 || ry < 1 ){
    std::cerr << "HLAC:extractBin(): Mask width must be positive" 
	      << std::endl;
    exit(1);
  }
  
  /*** 特徴ベクトルの次元をチェック ***/
  checkDimensionBin( result, order, size );
  
  /*** 特徴抽出を行う範囲を決める ***/
  int left, right, up, down;
  decideWindow( left, right, up, down, x, dx, y, dy, img, size, rx, ry );
  
  /*** 特徴抽出 ***/
  if( size == 3 ){
    switch( order ){
    case 1:
      _extractBin1_3( result, img, rx, ry, left, right, up, down );
      break;
    case 2:
      _extractBin2_3( result, img, rx, ry, left, right, up, down );
      break;
    case 3:
      _extractBin3_3( result, img, rx, ry, left, right, up, down );
      break;
    default:
      break;
    }
  }else if( size == 5 ){
    switch( order ){
    case 1:
      _extractBin1_5( result, img, rx, ry, left, right, up, down );
      break;
    case 2:
      _extractBin2_5( result, img, rx, ry, left, right, up, down );
      break;
#ifdef ENABLE_BIN_HLAC3_5
    case 3:
      _extractBin3_5( result, img, rx, ry, left, right, up, down );
      break;
#endif /* ENABLE_BIN_HLAC3_5 */
    default:
      break;
    }
  }
}

/*------------------------*
 *  for gray-scale image  *
 *------------------------*/
 
void HLAC::extractGray( std::vector<float> &result, const cv::Mat &img, 
			   int order, int size, int rx, int ry, 
			   int x, int dx, int y, int dy )
{
  /*** check image type ***/
  if( img.channels() != 1 ){
    std::cerr << "HLAC:extractGray(): gray-scale image is needed" 
	      << std::endl;
    exit(1);
  }
  
  /*** マスク幅をチェック ***/
  if( rx < 1 || ry < 1 ){
    std::cerr << "HLAC:extractGray(): Mask width must be positive" 
	      << std::endl;
    exit(1);
  }
  
  /*** 特徴ベクトルの次元をチェック ***/
  checkDimensionGray( result, order, size );
  
  /*** 特徴抽出を行う範囲を決める ***/
  int left, right, up, down;
  decideWindow( left, right, up, down, x, dx, y, dy, img, size, rx, ry );
  
  /*** 特徴抽出 ***/
  if( size == 3 ){
    switch( order ){
    case 1:
      _extractGray1_3( result, img, rx, ry, left, right, up, down );
      break;
    case 2:
      _extractGray2_3( result, img, rx, ry, left, right, up, down );
      break;
    case 3:
      _extractGray3_3( result, img, rx, ry, left, right, up, down );
      break;
    default:
      break;
    }
  }else if( size == 5 ){
    switch( order ){
    case 1:
      _extractGray1_5( result, img, rx, ry, left, right, up, down );
      break;
    case 2:
      _extractGray2_5( result, img, rx, ry, left, right, up, down );
      break;
#ifdef ENABLE_GRAY_HLAC3_5
    case 3:
      _extractGray3_5( result, img, rx, ry, left, right, up, down );
      break;
#endif /* ENABLE_GRAY_HLAC3_5 */
    default:
      break;
    }
  }
}

/*-------------------*
 *  for color image  *
 *-------------------*/
 
void HLAC::extractColor( std::vector<float> &result, const cv::Mat &img, 
			    int order, int size, int rx, int ry, 
			    int x, int dx, int y, int dy )
{
  /*** check image type ***/
  if( img.channels() != 3 ){
    std::cerr << "HLAC:extractCOLOR(): color image is needed" 
	      << std::endl;
    exit(1);
  }
  
  /*** マスク幅をチェック ***/
  if( rx < 1 || ry < 1 ){
    std::cerr << "HLAC:extractColor(): Mask width must be positive" 
	      << std::endl;
    exit(1);
  }
  
  /*** 特徴ベクトルの次元をチェック ***/
  checkDimensionColor( result, order, size );
  
  /*** 特徴抽出を行う範囲を決める ***/
  int left, right, up, down;
  decideWindow( left, right, up, down, x, dx, y, dy, img, size, rx, ry );
  
  /*** 特徴抽出 ***/
  if( size == 3 ){
    switch( order ){
    case 1:
      _extractColor1_3( result, img, rx, ry, left, right, up, down );
      break;
    case 2:
      _extractColor2_3( result, img, rx, ry, left, right, up, down );
      break;
    default:
      break;
    }
  }else if( size == 5 ){
    switch( order ){
    case 1:
      _extractColor1_5( result, img, rx, ry, left, right, up, down );
      break;
    default:
      break;
    }
  }
}


/*---------------------------------------------------------------------------
 * 内部関数
 *---------------------------------------------------------------------------*/
/*** 画像からの特徴抽出 ***/
/**************************************************
 * ここでは result のメモリ確保は成されており、
 * order 〜 down の変数は正しく変更されているとする
 **************************************************/
/*--------------------*
 *  for binary image  *
 *--------------------*/
/*** 高々1次の binary HLAC, square size = 3 **********************************/
 
void HLAC::_extractBin1_3( std::vector<float> &result, const cv::Mat &img,
			      const int &rx, const int &ry, 
			      const int &left, const int &right,
			      const int &up, const int &down )
{
  const int width_step = img.step;
  
  /*** initialize ***/
  double tmp[ DIM_OF_BIN_HLAC1_3 ];
  for(int i=0; i<DIM_OF_BIN_HLAC1_3; ++i) tmp[ i ] = 0.0;
  
  /*** feature extraction ***/
  for(int j=up; j<down; ++j){
    for(int i=left; i<right; ++i){
      if( img.data[ i + j * width_step ] ){
	tmp[ 0 ] += (double)img.data[   i        +   j        * width_step ];
	tmp[ 1 ] += (double)img.data[ ( i - rx ) + ( j - ry ) * width_step ];
	tmp[ 2 ] += (double)img.data[   i        + ( j - ry ) * width_step ];
	tmp[ 3 ] += (double)img.data[ ( i + rx ) + ( j - ry ) * width_step ];
	tmp[ 4 ] += (double)img.data[ ( i - rx ) +   j        * width_step ];
      }
    }
  }
  
  /*** normalize ***/
#ifdef ENABLE_NORMALIZATION
  double normalVal = (double)( ( right - left ) * ( down - up ) );
  for(int i=0; i<DIM_OF_BIN_HLAC1_3; ++i) result[ i ] = tmp[ i ] / normalVal;
#else /* !ENABLE_NORMALIZATION */
  for(int i=0; i<DIM_OF_BIN_HLAC1_3; ++i) result[ i ] = tmp[ i ];  
#endif /* ENABLE_NORMALIZATION */
}

/*** 高々2次の binary HLAC, square size = 3 **********************************/
 
void HLAC::_extractBin2_3( std::vector<float> &result, const cv::Mat &img,
			      const int &rx, const int &ry, 
			      const int &left, const int &right,
			      const int &up, const int &down )
{
  double a[ 9 ];
  const int width_step = img.step;
  
  /*** initialize ***/
  double tmp[ DIM_OF_BIN_HLAC2_3 ];
  for(int i=0; i<DIM_OF_BIN_HLAC2_3; ++i) tmp[ i ] = 0.0;
  
  /*** feature extraction ***/
  for(int j=up; j<down; ++j){
    for(int i=left; i<right; ++i){
      if( img.data[ i + j * width_step ] ){
	a[  0 ] = (double)img.data[ ( i - rx ) + ( j - ry ) * width_step ];
	a[  1 ] = (double)img.data[   i        + ( j - ry ) * width_step ];
	a[  2 ] = (double)img.data[ ( i + rx ) + ( j - ry ) * width_step ];
	a[  3 ] = (double)img.data[ ( i - rx ) +   j        * width_step ];
	a[  4 ] = (double)img.data[   i        +   j        * width_step ];
	a[  5 ] = (double)img.data[ ( i + rx ) +   j        * width_step ];
	a[  6 ] = (double)img.data[ ( i - rx ) + ( j + ry ) * width_step ];
	a[  7 ] = (double)img.data[   i        + ( j + ry ) * width_step ];
	a[  8 ] = (double)img.data[ ( i + rx ) + ( j + ry ) * width_step ];
	tmp[ 0 ] += a[ 4 ];
	tmp[ 1 ] += a[ 0 ];
	tmp[ 2 ] += a[ 1 ];
	tmp[ 3 ] += a[ 2 ];
	tmp[ 4 ] += a[ 3 ];
	tmp[ 5 ] += a[ 0 ] * a[ 1 ];
	tmp[ 6 ] += a[ 0 ] * a[ 2 ];
	tmp[ 7 ] += a[ 0 ] * a[ 3 ];
	tmp[ 8 ] += a[ 0 ] * a[ 5 ];
	tmp[ 9 ] += a[ 0 ] * a[ 6 ];
	tmp[ 10 ] += a[ 0 ] * a[ 7 ];
	tmp[ 11 ] += a[ 0 ] * a[ 8 ];
	tmp[ 12 ] += a[ 1 ] * a[ 2 ];
	tmp[ 13 ] += a[ 1 ] * a[ 3 ];
	tmp[ 14 ] += a[ 1 ] * a[ 6 ];
	tmp[ 15 ] += a[ 1 ] * a[ 7 ];
	tmp[ 16 ] += a[ 1 ] * a[ 8 ];
	tmp[ 17 ] += a[ 2 ] * a[ 3 ];
	tmp[ 18 ] += a[ 2 ] * a[ 6 ];
	tmp[ 19 ] += a[ 2 ] * a[ 7 ];
	tmp[ 20 ] += a[ 2 ] * a[ 8 ];
	tmp[ 21 ] += a[ 3 ] * a[ 5 ];
	tmp[ 22 ] += a[ 3 ] * a[ 8 ];
	tmp[ 23 ] += a[ 5 ] * a[ 6 ];
	tmp[ 24 ] += a[ 6 ] * a[ 8 ];
      }
    }
  }
  
  /*** normalize ***/
#ifdef ENABLE_NORMALIZATION
  double normalVal = (double)( ( right - left ) * ( down - up ) );
  for(int i=0; i<DIM_OF_BIN_HLAC2_3; ++i) result[ i ] = tmp[ i ] / normalVal;
#else /* !ENABLE_NORMALIZATION */
  for(int i=0; i<DIM_OF_BIN_HLAC2_3; ++i) result[ i ] = tmp[ i ];  
#endif /* ENABLE_NORMALIZATION */
}

/*** 高々3次の binary HLAC, square size = 3 **********************************/
 
void HLAC::_extractBin3_3( std::vector<float> &result, const cv::Mat &img,
			      const int &rx, const int &ry, 
			      const int &left, const int &right,
			      const int &up, const int &down )
{
  double a[ 9 ];
  const int width_step = img.step;
  
  /*** initialize ***/
  double tmp[ DIM_OF_BIN_HLAC3_3 ];
  for(int i=0; i<DIM_OF_BIN_HLAC3_3; ++i) tmp[ i ] = 0.0;
  
  /*** feature extraction ***/
  for(int j=up; j<down; ++j){
    for(int i=left; i<right; ++i){
      if( img.data[ i + j * width_step ] ){
	a[  0 ] = (double)img.data[ ( i - rx ) + ( j - ry ) * width_step ];
	a[  1 ] = (double)img.data[   i        + ( j - ry ) * width_step ];
	a[  2 ] = (double)img.data[ ( i + rx ) + ( j - ry ) * width_step ];
	a[  3 ] = (double)img.data[ ( i - rx ) +   j        * width_step ];
	a[  4 ] = (double)img.data[   i        +   j        * width_step ];
	a[  5 ] = (double)img.data[ ( i + rx ) +   j        * width_step ];
	a[  6 ] = (double)img.data[ ( i - rx ) + ( j + ry ) * width_step ];
	a[  7 ] = (double)img.data[   i        + ( j + ry ) * width_step ];
	a[  8 ] = (double)img.data[ ( i + rx ) + ( j + ry ) * width_step ];
	tmp[ 0 ] += a[ 4 ];
	tmp[ 1 ] += a[ 0 ];
	tmp[ 2 ] += a[ 1 ];
	tmp[ 3 ] += a[ 2 ];
	tmp[ 4 ] += a[ 3 ];
	tmp[ 5 ] += a[ 0 ] * a[ 1 ];
	tmp[ 6 ] += a[ 0 ] * a[ 2 ];
	tmp[ 7 ] += a[ 0 ] * a[ 3 ];
	tmp[ 8 ] += a[ 0 ] * a[ 5 ];
	tmp[ 9 ] += a[ 0 ] * a[ 6 ];
	tmp[ 10 ] += a[ 0 ] * a[ 7 ];
	tmp[ 11 ] += a[ 0 ] * a[ 8 ];
	tmp[ 12 ] += a[ 1 ] * a[ 2 ];
	tmp[ 13 ] += a[ 1 ] * a[ 3 ];
	tmp[ 14 ] += a[ 1 ] * a[ 6 ];
	tmp[ 15 ] += a[ 1 ] * a[ 7 ];
	tmp[ 16 ] += a[ 1 ] * a[ 8 ];
	tmp[ 17 ] += a[ 2 ] * a[ 3 ];
	tmp[ 18 ] += a[ 2 ] * a[ 6 ];
	tmp[ 19 ] += a[ 2 ] * a[ 7 ];
	tmp[ 20 ] += a[ 2 ] * a[ 8 ];
	tmp[ 21 ] += a[ 3 ] * a[ 5 ];
	tmp[ 22 ] += a[ 3 ] * a[ 8 ];
	tmp[ 23 ] += a[ 5 ] * a[ 6 ];
	tmp[ 24 ] += a[ 6 ] * a[ 8 ];
	tmp[ 25 ] += a[ 0 ] * a[ 1 ] * a[ 2 ];
	tmp[ 26 ] += a[ 0 ] * a[ 1 ] * a[ 3 ];
	tmp[ 27 ] += a[ 0 ] * a[ 1 ] * a[ 5 ];
	tmp[ 28 ] += a[ 0 ] * a[ 1 ] * a[ 6 ];
	tmp[ 29 ] += a[ 0 ] * a[ 1 ] * a[ 7 ];
	tmp[ 30 ] += a[ 0 ] * a[ 1 ] * a[ 8 ];
	tmp[ 31 ] += a[ 0 ] * a[ 2 ] * a[ 3 ];
	tmp[ 32 ] += a[ 0 ] * a[ 2 ] * a[ 5 ];
	tmp[ 33 ] += a[ 0 ] * a[ 2 ] * a[ 6 ];
	tmp[ 34 ] += a[ 0 ] * a[ 2 ] * a[ 7 ];
	tmp[ 35 ] += a[ 0 ] * a[ 2 ] * a[ 8 ];
	tmp[ 36 ] += a[ 0 ] * a[ 3 ] * a[ 5 ];
	tmp[ 37 ] += a[ 0 ] * a[ 3 ] * a[ 6 ];
	tmp[ 38 ] += a[ 0 ] * a[ 3 ] * a[ 7 ];
	tmp[ 39 ] += a[ 0 ] * a[ 3 ] * a[ 8 ];
	tmp[ 40 ] += a[ 0 ] * a[ 5 ] * a[ 6 ];
	tmp[ 41 ] += a[ 0 ] * a[ 5 ] * a[ 7 ];
	tmp[ 42 ] += a[ 0 ] * a[ 5 ] * a[ 8 ];
	tmp[ 43 ] += a[ 0 ] * a[ 6 ] * a[ 7 ];
	tmp[ 44 ] += a[ 0 ] * a[ 6 ] * a[ 8 ];
	tmp[ 45 ] += a[ 0 ] * a[ 7 ] * a[ 8 ];
	tmp[ 46 ] += a[ 1 ] * a[ 2 ] * a[ 3 ];
	tmp[ 47 ] += a[ 1 ] * a[ 2 ] * a[ 6 ];
	tmp[ 48 ] += a[ 1 ] * a[ 2 ] * a[ 7 ];
	tmp[ 49 ] += a[ 1 ] * a[ 2 ] * a[ 8 ];
	tmp[ 50 ] += a[ 1 ] * a[ 3 ] * a[ 5 ];
	tmp[ 51 ] += a[ 1 ] * a[ 3 ] * a[ 6 ];
	tmp[ 52 ] += a[ 1 ] * a[ 3 ] * a[ 7 ];
	tmp[ 53 ] += a[ 1 ] * a[ 3 ] * a[ 8 ];
	tmp[ 54 ] += a[ 1 ] * a[ 5 ] * a[ 6 ];
	tmp[ 55 ] += a[ 1 ] * a[ 6 ] * a[ 7 ];
	tmp[ 56 ] += a[ 1 ] * a[ 6 ] * a[ 8 ];
	tmp[ 57 ] += a[ 1 ] * a[ 7 ] * a[ 8 ];
	tmp[ 58 ] += a[ 2 ] * a[ 3 ] * a[ 5 ];
	tmp[ 59 ] += a[ 2 ] * a[ 3 ] * a[ 6 ];
	tmp[ 60 ] += a[ 2 ] * a[ 3 ] * a[ 7 ];
	tmp[ 61 ] += a[ 2 ] * a[ 3 ] * a[ 8 ];
	tmp[ 62 ] += a[ 2 ] * a[ 5 ] * a[ 6 ];
	tmp[ 63 ] += a[ 2 ] * a[ 6 ] * a[ 7 ];
	tmp[ 64 ] += a[ 2 ] * a[ 6 ] * a[ 8 ];
	tmp[ 65 ] += a[ 2 ] * a[ 7 ] * a[ 8 ];
	tmp[ 66 ] += a[ 3 ] * a[ 5 ] * a[ 6 ];
	tmp[ 67 ] += a[ 3 ] * a[ 5 ] * a[ 8 ];
	tmp[ 68 ] += a[ 3 ] * a[ 6 ] * a[ 8 ];
	tmp[ 69 ] += a[ 5 ] * a[ 6 ] * a[ 8 ];
      }
    }
  }
  
  /*** normalize ***/
#ifdef ENABLE_NORMALIZATION
  double normalVal = (double)( ( right - left ) * ( down - up ) );
  for(int i=0; i<DIM_OF_BIN_HLAC3_3; ++i) result[ i ] = tmp[ i ] / normalVal;
#else /* !ENABLE_NORMALIZATION */
  for(int i=0; i<DIM_OF_BIN_HLAC3_3; ++i) result[ i ] = tmp[ i ];  
#endif /* ENABLE_NORMALIZATION */
}

/*** 高々1次の binary HLAC, square size = 5 **********************************/
 
void HLAC::_extractBin1_5( std::vector<float> &result, const cv::Mat &img,
			      const int &rx, const int &ry, 
			      const int &left, const int &right,
			      const int &up, const int &down )
{
  const int width_step = img.step;
  
  /*** initialize ***/
  double tmp[ DIM_OF_BIN_HLAC1_5 ];
  for(int i=0; i<DIM_OF_BIN_HLAC1_5; ++i) tmp[ i ] = 0.0;
  
  /*** feature extraction ***/
  for(int j=up; j<down; ++j){
    for(int i=left; i<right; ++i){
      if( img.data[ i + j * width_step ] ){
	tmp[  0 ] += (double)img.data[   i          +   j          * width_step ];
	tmp[  1 ] += (double)img.data[ ( i - 2*rx ) + ( j - 2*ry ) * width_step ];
	tmp[  2 ] += (double)img.data[ ( i -   rx ) + ( j - 2*ry ) * width_step ];
	tmp[  3 ] += (double)img.data[   i          + ( j - 2*ry ) * width_step ];
	tmp[  4 ] += (double)img.data[ ( i +   rx ) + ( j - 2*ry ) * width_step ];
	tmp[  5 ] += (double)img.data[ ( i + 2*rx ) + ( j - 2*ry ) * width_step ];
	tmp[  6 ] += (double)img.data[ ( i - 2*rx ) + ( j -   ry ) * width_step ];
	tmp[  7 ] += (double)img.data[ ( i -   rx ) + ( j -   ry ) * width_step ];
	tmp[  8 ] += (double)img.data[   i          + ( j -   ry ) * width_step ];
	tmp[  9 ] += (double)img.data[ ( i +   rx ) + ( j -   ry ) * width_step ];
	tmp[ 10 ] += (double)img.data[ ( i + 2*rx ) + ( j -   ry ) * width_step ];
	tmp[ 11 ] += (double)img.data[ ( i - 2*rx ) +   j          * width_step ];
	tmp[ 12 ] += (double)img.data[ ( i -   rx ) +   j          * width_step ];
      }
    }
  }
  
  /*** normalize ***/
#ifdef ENABLE_NORMALIZATION
  double normalVal = (double)( ( right - left ) * ( down - up ) );
  for(int i=0; i<DIM_OF_BIN_HLAC1_5; ++i) result[ i ] = tmp[ i ] / normalVal;
#else /* !ENABLE_NORMALIZATION */
  for(int i=0; i<DIM_OF_BIN_HLAC1_5; ++i) result[ i ] = tmp[ i ];  
#endif /* ENABLE_NORMALIZATION */
}
/*** 高々2次の binary HLAC, square size = 5 **********************************/
 
void HLAC::_extractBin2_5( std::vector<float> &result, const cv::Mat &img,
			      const int &rx, const int &ry, 
			      const int &left, const int &right,
			      const int &up, const int &down )
{
  double a[ 25 ];
  const int width_step = img.step;
  
  /*** initialize ***/
  double tmp[ DIM_OF_BIN_HLAC2_5 ];
  for(int i=0; i<DIM_OF_BIN_HLAC2_5; ++i) tmp[ i ] = 0.0;
  
  /*** feature extraction ***/
  for(int j=up; j<down; ++j){
    for(int i=left; i<right; ++i){
      if( img.data[ i + j * width_step ] ){
	a[  0 ] = (double)img.data[ ( i - 2*rx ) + ( j - 2*ry ) * width_step ];
	a[  1 ] = (double)img.data[ ( i -   rx ) + ( j - 2*ry ) * width_step ];
	a[  2 ] = (double)img.data[   i          + ( j - 2*ry ) * width_step ];
	a[  3 ] = (double)img.data[ ( i +   rx ) + ( j - 2*ry ) * width_step ];
	a[  4 ] = (double)img.data[ ( i + 2*rx ) + ( j - 2*ry ) * width_step ];
	a[  5 ] = (double)img.data[ ( i - 2*rx ) + ( j -   ry ) * width_step ];
	a[  6 ] = (double)img.data[ ( i -   rx ) + ( j -   ry ) * width_step ];
	a[  7 ] = (double)img.data[   i          + ( j -   ry ) * width_step ];
	a[  8 ] = (double)img.data[ ( i +   rx ) + ( j -   ry ) * width_step ];
	a[  9 ] = (double)img.data[ ( i + 2*rx ) + ( j -   ry ) * width_step ];
	a[ 10 ] = (double)img.data[ ( i - 2*rx ) +   j          * width_step ];
	a[ 11 ] = (double)img.data[ ( i -   rx ) +   j          * width_step ];
	a[ 12 ] = (double)img.data[   i          +   j          * width_step ];
	a[ 13 ] = (double)img.data[ ( i +   rx ) +   j          * width_step ];
	a[ 14 ] = (double)img.data[ ( i + 2*rx ) +   j          * width_step ];
	a[ 15 ] = (double)img.data[ ( i - 2*rx ) + ( j +   ry ) * width_step ];
	a[ 16 ] = (double)img.data[ ( i -   rx ) + ( j +   ry ) * width_step ];
	a[ 17 ] = (double)img.data[   i          + ( j +   ry ) * width_step ];
	a[ 18 ] = (double)img.data[ ( i +   rx ) + ( j +   ry ) * width_step ];
	a[ 19 ] = (double)img.data[ ( i + 2*rx ) + ( j +   ry ) * width_step ];
	a[ 20 ] = (double)img.data[ ( i - 2*rx ) + ( j + 2*ry ) * width_step ];
	a[ 21 ] = (double)img.data[ ( i -   rx ) + ( j + 2*ry ) * width_step ];
	a[ 22 ] = (double)img.data[   i          + ( j + 2*ry ) * width_step ];
	a[ 23 ] = (double)img.data[ ( i +   rx ) + ( j + 2*ry ) * width_step ];
	a[ 24 ] = (double)img.data[ ( i + 2*rx ) + ( j + 2*ry ) * width_step ];
	tmp[ 0 ] += a[ 12 ];
	tmp[ 1 ] += a[ 0 ];
	tmp[ 2 ] += a[ 1 ];
	tmp[ 3 ] += a[ 2 ];
	tmp[ 4 ] += a[ 3 ];
	tmp[ 5 ] += a[ 4 ];
	tmp[ 6 ] += a[ 5 ];
	tmp[ 7 ] += a[ 6 ];
	tmp[ 8 ] += a[ 7 ];
	tmp[ 9 ] += a[ 8 ];
	tmp[ 10 ] += a[ 9 ];
	tmp[ 11 ] += a[ 10 ];
	tmp[ 12 ] += a[ 11 ];
	tmp[ 13 ] += a[ 0 ] * a[ 1 ];
	tmp[ 14 ] += a[ 0 ] * a[ 2 ];
	tmp[ 15 ] += a[ 0 ] * a[ 3 ];
	tmp[ 16 ] += a[ 0 ] * a[ 4 ];
	tmp[ 17 ] += a[ 0 ] * a[ 5 ];
	tmp[ 18 ] += a[ 0 ] * a[ 6 ];
	tmp[ 19 ] += a[ 0 ] * a[ 7 ];
	tmp[ 20 ] += a[ 0 ] * a[ 8 ];
	tmp[ 21 ] += a[ 0 ] * a[ 9 ];
	tmp[ 22 ] += a[ 0 ] * a[ 10 ];
	tmp[ 23 ] += a[ 0 ] * a[ 11 ];
	tmp[ 24 ] += a[ 0 ] * a[ 13 ];
	tmp[ 25 ] += a[ 0 ] * a[ 14 ];
	tmp[ 26 ] += a[ 0 ] * a[ 15 ];
	tmp[ 27 ] += a[ 0 ] * a[ 16 ];
	tmp[ 28 ] += a[ 0 ] * a[ 17 ];
	tmp[ 29 ] += a[ 0 ] * a[ 18 ];
	tmp[ 30 ] += a[ 0 ] * a[ 19 ];
	tmp[ 31 ] += a[ 0 ] * a[ 20 ];
	tmp[ 32 ] += a[ 0 ] * a[ 21 ];
	tmp[ 33 ] += a[ 0 ] * a[ 22 ];
	tmp[ 34 ] += a[ 0 ] * a[ 23 ];
	tmp[ 35 ] += a[ 0 ] * a[ 24 ];
	tmp[ 36 ] += a[ 1 ] * a[ 2 ];
	tmp[ 37 ] += a[ 1 ] * a[ 3 ];
	tmp[ 38 ] += a[ 1 ] * a[ 4 ];
	tmp[ 39 ] += a[ 1 ] * a[ 5 ];
	tmp[ 40 ] += a[ 1 ] * a[ 6 ];
	tmp[ 41 ] += a[ 1 ] * a[ 7 ];
	tmp[ 42 ] += a[ 1 ] * a[ 8 ];
	tmp[ 43 ] += a[ 1 ] * a[ 9 ];
	tmp[ 44 ] += a[ 1 ] * a[ 10 ];
	tmp[ 45 ] += a[ 1 ] * a[ 11 ];
	tmp[ 46 ] += a[ 1 ] * a[ 14 ];
	tmp[ 47 ] += a[ 1 ] * a[ 15 ];
	tmp[ 48 ] += a[ 1 ] * a[ 16 ];
	tmp[ 49 ] += a[ 1 ] * a[ 17 ];
	tmp[ 50 ] += a[ 1 ] * a[ 18 ];
	tmp[ 51 ] += a[ 1 ] * a[ 19 ];
	tmp[ 52 ] += a[ 1 ] * a[ 20 ];
	tmp[ 53 ] += a[ 1 ] * a[ 21 ];
	tmp[ 54 ] += a[ 1 ] * a[ 22 ];
	tmp[ 55 ] += a[ 1 ] * a[ 23 ];
	tmp[ 56 ] += a[ 1 ] * a[ 24 ];
	tmp[ 57 ] += a[ 2 ] * a[ 3 ];
	tmp[ 58 ] += a[ 2 ] * a[ 4 ];
	tmp[ 59 ] += a[ 2 ] * a[ 5 ];
	tmp[ 60 ] += a[ 2 ] * a[ 6 ];
	tmp[ 61 ] += a[ 2 ] * a[ 7 ];
	tmp[ 62 ] += a[ 2 ] * a[ 8 ];
	tmp[ 63 ] += a[ 2 ] * a[ 9 ];
	tmp[ 64 ] += a[ 2 ] * a[ 10 ];
	tmp[ 65 ] += a[ 2 ] * a[ 11 ];
	tmp[ 66 ] += a[ 2 ] * a[ 15 ];
	tmp[ 67 ] += a[ 2 ] * a[ 16 ];
	tmp[ 68 ] += a[ 2 ] * a[ 17 ];
	tmp[ 69 ] += a[ 2 ] * a[ 18 ];
	tmp[ 70 ] += a[ 2 ] * a[ 19 ];
	tmp[ 71 ] += a[ 2 ] * a[ 20 ];
	tmp[ 72 ] += a[ 2 ] * a[ 21 ];
	tmp[ 73 ] += a[ 2 ] * a[ 22 ];
	tmp[ 74 ] += a[ 2 ] * a[ 23 ];
	tmp[ 75 ] += a[ 2 ] * a[ 24 ];
	tmp[ 76 ] += a[ 3 ] * a[ 4 ];
	tmp[ 77 ] += a[ 3 ] * a[ 5 ];
	tmp[ 78 ] += a[ 3 ] * a[ 6 ];
	tmp[ 79 ] += a[ 3 ] * a[ 7 ];
	tmp[ 80 ] += a[ 3 ] * a[ 8 ];
	tmp[ 81 ] += a[ 3 ] * a[ 9 ];
	tmp[ 82 ] += a[ 3 ] * a[ 10 ];
	tmp[ 83 ] += a[ 3 ] * a[ 11 ];
	tmp[ 84 ] += a[ 3 ] * a[ 15 ];
	tmp[ 85 ] += a[ 3 ] * a[ 16 ];
	tmp[ 86 ] += a[ 3 ] * a[ 17 ];
	tmp[ 87 ] += a[ 3 ] * a[ 18 ];
	tmp[ 88 ] += a[ 3 ] * a[ 19 ];
	tmp[ 89 ] += a[ 3 ] * a[ 20 ];
	tmp[ 90 ] += a[ 3 ] * a[ 21 ];
	tmp[ 91 ] += a[ 3 ] * a[ 22 ];
	tmp[ 92 ] += a[ 3 ] * a[ 23 ];
	tmp[ 93 ] += a[ 3 ] * a[ 24 ];
	tmp[ 94 ] += a[ 4 ] * a[ 5 ];
	tmp[ 95 ] += a[ 4 ] * a[ 6 ];
	tmp[ 96 ] += a[ 4 ] * a[ 7 ];
	tmp[ 97 ] += a[ 4 ] * a[ 8 ];
	tmp[ 98 ] += a[ 4 ] * a[ 9 ];
	tmp[ 99 ] += a[ 4 ] * a[ 10 ];
	tmp[ 100 ] += a[ 4 ] * a[ 11 ];
	tmp[ 101 ] += a[ 4 ] * a[ 15 ];
	tmp[ 102 ] += a[ 4 ] * a[ 16 ];
	tmp[ 103 ] += a[ 4 ] * a[ 17 ];
	tmp[ 104 ] += a[ 4 ] * a[ 18 ];
	tmp[ 105 ] += a[ 4 ] * a[ 19 ];
	tmp[ 106 ] += a[ 4 ] * a[ 20 ];
	tmp[ 107 ] += a[ 4 ] * a[ 21 ];
	tmp[ 108 ] += a[ 4 ] * a[ 22 ];
	tmp[ 109 ] += a[ 4 ] * a[ 23 ];
	tmp[ 110 ] += a[ 4 ] * a[ 24 ];
	tmp[ 111 ] += a[ 5 ] * a[ 6 ];
	tmp[ 112 ] += a[ 5 ] * a[ 7 ];
	tmp[ 113 ] += a[ 5 ] * a[ 8 ];
	tmp[ 114 ] += a[ 5 ] * a[ 9 ];
	tmp[ 115 ] += a[ 5 ] * a[ 10 ];
	tmp[ 116 ] += a[ 5 ] * a[ 11 ];
	tmp[ 117 ] += a[ 5 ] * a[ 13 ];
	tmp[ 118 ] += a[ 5 ] * a[ 14 ];
	tmp[ 119 ] += a[ 5 ] * a[ 18 ];
	tmp[ 120 ] += a[ 5 ] * a[ 19 ];
	tmp[ 121 ] += a[ 5 ] * a[ 20 ];
	tmp[ 122 ] += a[ 5 ] * a[ 21 ];
	tmp[ 123 ] += a[ 5 ] * a[ 22 ];
	tmp[ 124 ] += a[ 5 ] * a[ 23 ];
	tmp[ 125 ] += a[ 5 ] * a[ 24 ];
	tmp[ 126 ] += a[ 6 ] * a[ 7 ];
	tmp[ 127 ] += a[ 6 ] * a[ 8 ];
	tmp[ 128 ] += a[ 6 ] * a[ 9 ];
	tmp[ 129 ] += a[ 6 ] * a[ 10 ];
	tmp[ 130 ] += a[ 6 ] * a[ 11 ];
	tmp[ 131 ] += a[ 6 ] * a[ 14 ];
	tmp[ 132 ] += a[ 6 ] * a[ 19 ];
	tmp[ 133 ] += a[ 6 ] * a[ 20 ];
	tmp[ 134 ] += a[ 6 ] * a[ 21 ];
	tmp[ 135 ] += a[ 6 ] * a[ 22 ];
	tmp[ 136 ] += a[ 6 ] * a[ 23 ];
	tmp[ 137 ] += a[ 6 ] * a[ 24 ];
	tmp[ 138 ] += a[ 7 ] * a[ 8 ];
	tmp[ 139 ] += a[ 7 ] * a[ 9 ];
	tmp[ 140 ] += a[ 7 ] * a[ 10 ];
	tmp[ 141 ] += a[ 7 ] * a[ 11 ];
	tmp[ 142 ] += a[ 7 ] * a[ 20 ];
	tmp[ 143 ] += a[ 7 ] * a[ 21 ];
	tmp[ 144 ] += a[ 7 ] * a[ 22 ];
	tmp[ 145 ] += a[ 7 ] * a[ 23 ];
	tmp[ 146 ] += a[ 7 ] * a[ 24 ];
	tmp[ 147 ] += a[ 8 ] * a[ 9 ];
	tmp[ 148 ] += a[ 8 ] * a[ 10 ];
	tmp[ 149 ] += a[ 8 ] * a[ 11 ];
	tmp[ 150 ] += a[ 8 ] * a[ 15 ];
	tmp[ 151 ] += a[ 8 ] * a[ 20 ];
	tmp[ 152 ] += a[ 8 ] * a[ 21 ];
	tmp[ 153 ] += a[ 8 ] * a[ 22 ];
	tmp[ 154 ] += a[ 8 ] * a[ 23 ];
	tmp[ 155 ] += a[ 8 ] * a[ 24 ];
	tmp[ 156 ] += a[ 9 ] * a[ 10 ];
	tmp[ 157 ] += a[ 9 ] * a[ 11 ];
	tmp[ 158 ] += a[ 9 ] * a[ 15 ];
	tmp[ 159 ] += a[ 9 ] * a[ 16 ];
	tmp[ 160 ] += a[ 9 ] * a[ 20 ];
	tmp[ 161 ] += a[ 9 ] * a[ 21 ];
	tmp[ 162 ] += a[ 9 ] * a[ 22 ];
	tmp[ 163 ] += a[ 9 ] * a[ 23 ];
	tmp[ 164 ] += a[ 9 ] * a[ 24 ];
	tmp[ 165 ] += a[ 10 ] * a[ 11 ];
	tmp[ 166 ] += a[ 10 ] * a[ 13 ];
	tmp[ 167 ] += a[ 10 ] * a[ 14 ];
	tmp[ 168 ] += a[ 10 ] * a[ 18 ];
	tmp[ 169 ] += a[ 10 ] * a[ 19 ];
	tmp[ 170 ] += a[ 10 ] * a[ 23 ];
	tmp[ 171 ] += a[ 10 ] * a[ 24 ];
	tmp[ 172 ] += a[ 11 ] * a[ 14 ];
	tmp[ 173 ] += a[ 11 ] * a[ 19 ];
	tmp[ 174 ] += a[ 11 ] * a[ 24 ];
	tmp[ 175 ] += a[ 13 ] * a[ 15 ];
	tmp[ 176 ] += a[ 13 ] * a[ 20 ];
	tmp[ 177 ] += a[ 14 ] * a[ 15 ];
	tmp[ 178 ] += a[ 14 ] * a[ 16 ];
	tmp[ 179 ] += a[ 14 ] * a[ 20 ];
	tmp[ 180 ] += a[ 14 ] * a[ 21 ];
	tmp[ 181 ] += a[ 15 ] * a[ 18 ];
	tmp[ 182 ] += a[ 15 ] * a[ 19 ];
	tmp[ 183 ] += a[ 15 ] * a[ 23 ];
	tmp[ 184 ] += a[ 15 ] * a[ 24 ];
	tmp[ 185 ] += a[ 16 ] * a[ 19 ];
	tmp[ 186 ] += a[ 16 ] * a[ 24 ];
	tmp[ 187 ] += a[ 18 ] * a[ 20 ];
	tmp[ 188 ] += a[ 19 ] * a[ 20 ];
	tmp[ 189 ] += a[ 19 ] * a[ 21 ];
	tmp[ 190 ] += a[ 20 ] * a[ 23 ];
	tmp[ 191 ] += a[ 20 ] * a[ 24 ];
	tmp[ 192 ] += a[ 21 ] * a[ 24 ];
      }
    }
  }
  
  /*** normalize ***/
#ifdef ENABLE_NORMALIZATION
  double normalVal = (double)( ( right - left ) * ( down - up ) );
  for(int i=0; i<DIM_OF_BIN_HLAC2_5; ++i) result[ i ] = tmp[ i ] / normalVal;
#else /* !ENABLE_NORMALIZATION */
  for(int i=0; i<DIM_OF_BIN_HLAC2_5; ++i) result[ i ] = tmp[ i ];  
#endif /* ENABLE_NORMALIZATION */
}

/*** 高々3次の binary HLAC, square size = 5 **********************************/
#ifdef ENABLE_BIN_HLAC3_5
 
void HLAC::_extractBin3_5( std::vector<float> &result, const cv::Mat &img,
			      const int &rx, const int &ry, 
			      const int &left, const int &right,
			      const int &up, const int &down )
{
  double a[ 25 ];
  const int width_step = img.step;
  
  /*** initialize ***/
  double tmp[ DIM_OF_BIN_HLAC3_5 ];
  for(int i=0; i<DIM_OF_BIN_HLAC3_5; ++i) tmp[ i ] = 0.0;
  
  /*** feature extraction ***/
  for(int j=up; j<down; ++j){
    for(int i=left; i<right; ++i){
      if( img.data[ i + j * width_step ] ){
	a[  0 ] = (double)img.data[ ( i - 2*rx ) + ( j - 2*ry ) * width_step ];
	a[  1 ] = (double)img.data[ ( i -   rx ) + ( j - 2*ry ) * width_step ];
	a[  2 ] = (double)img.data[   i          + ( j - 2*ry ) * width_step ];
	a[  3 ] = (double)img.data[ ( i +   rx ) + ( j - 2*ry ) * width_step ];
	a[  4 ] = (double)img.data[ ( i + 2*rx ) + ( j - 2*ry ) * width_step ];
	a[  5 ] = (double)img.data[ ( i - 2*rx ) + ( j -   ry ) * width_step ];
	a[  6 ] = (double)img.data[ ( i -   rx ) + ( j -   ry ) * width_step ];
	a[  7 ] = (double)img.data[   i          + ( j -   ry ) * width_step ];
	a[  8 ] = (double)img.data[ ( i +   rx ) + ( j -   ry ) * width_step ];
	a[  9 ] = (double)img.data[ ( i + 2*rx ) + ( j -   ry ) * width_step ];
	a[ 10 ] = (double)img.data[ ( i - 2*rx ) +   j          * width_step ];
	a[ 11 ] = (double)img.data[ ( i -   rx ) +   j          * width_step ];
	a[ 12 ] = (double)img.data[   i          +   j          * width_step ];
	a[ 13 ] = (double)img.data[ ( i +   rx ) +   j          * width_step ];
	a[ 14 ] = (double)img.data[ ( i + 2*rx ) +   j          * width_step ];
	a[ 15 ] = (double)img.data[ ( i - 2*rx ) + ( j +   ry ) * width_step ];
	a[ 16 ] = (double)img.data[ ( i -   rx ) + ( j +   ry ) * width_step ];
	a[ 17 ] = (double)img.data[   i          + ( j +   ry ) * width_step ];
	a[ 18 ] = (double)img.data[ ( i +   rx ) + ( j +   ry ) * width_step ];
	a[ 19 ] = (double)img.data[ ( i + 2*rx ) + ( j +   ry ) * width_step ];
	a[ 20 ] = (double)img.data[ ( i - 2*rx ) + ( j + 2*ry ) * width_step ];
	a[ 21 ] = (double)img.data[ ( i -   rx ) + ( j + 2*ry ) * width_step ];
	a[ 22 ] = (double)img.data[   i          + ( j + 2*ry ) * width_step ];
	a[ 23 ] = (double)img.data[ ( i +   rx ) + ( j + 2*ry ) * width_step ];
	a[ 24 ] = (double)img.data[ ( i + 2*rx ) + ( j + 2*ry ) * width_step ];
	tmp[ 0 ] += a[ 12 ];
	tmp[ 1 ] += a[ 0 ];
	tmp[ 2 ] += a[ 1 ];
	tmp[ 3 ] += a[ 2 ];
	tmp[ 4 ] += a[ 3 ];
	tmp[ 5 ] += a[ 4 ];
	tmp[ 6 ] += a[ 5 ];
	tmp[ 7 ] += a[ 6 ];
	tmp[ 8 ] += a[ 7 ];
	tmp[ 9 ] += a[ 8 ];
	tmp[ 10 ] += a[ 9 ];
	tmp[ 11 ] += a[ 10 ];
	tmp[ 12 ] += a[ 11 ];
	tmp[ 13 ] += a[ 0 ] * a[ 1 ];
	tmp[ 14 ] += a[ 0 ] * a[ 2 ];
	tmp[ 15 ] += a[ 0 ] * a[ 3 ];
	tmp[ 16 ] += a[ 0 ] * a[ 4 ];
	tmp[ 17 ] += a[ 0 ] * a[ 5 ];
	tmp[ 18 ] += a[ 0 ] * a[ 6 ];
	tmp[ 19 ] += a[ 0 ] * a[ 7 ];
	tmp[ 20 ] += a[ 0 ] * a[ 8 ];
	tmp[ 21 ] += a[ 0 ] * a[ 9 ];
	tmp[ 22 ] += a[ 0 ] * a[ 10 ];
	tmp[ 23 ] += a[ 0 ] * a[ 11 ];
	tmp[ 24 ] += a[ 0 ] * a[ 13 ];
	tmp[ 25 ] += a[ 0 ] * a[ 14 ];
	tmp[ 26 ] += a[ 0 ] * a[ 15 ];
	tmp[ 27 ] += a[ 0 ] * a[ 16 ];
	tmp[ 28 ] += a[ 0 ] * a[ 17 ];
	tmp[ 29 ] += a[ 0 ] * a[ 18 ];
	tmp[ 30 ] += a[ 0 ] * a[ 19 ];
	tmp[ 31 ] += a[ 0 ] * a[ 20 ];
	tmp[ 32 ] += a[ 0 ] * a[ 21 ];
	tmp[ 33 ] += a[ 0 ] * a[ 22 ];
	tmp[ 34 ] += a[ 0 ] * a[ 23 ];
	tmp[ 35 ] += a[ 0 ] * a[ 24 ];
	tmp[ 36 ] += a[ 1 ] * a[ 2 ];
	tmp[ 37 ] += a[ 1 ] * a[ 3 ];
	tmp[ 38 ] += a[ 1 ] * a[ 4 ];
	tmp[ 39 ] += a[ 1 ] * a[ 5 ];
	tmp[ 40 ] += a[ 1 ] * a[ 6 ];
	tmp[ 41 ] += a[ 1 ] * a[ 7 ];
	tmp[ 42 ] += a[ 1 ] * a[ 8 ];
	tmp[ 43 ] += a[ 1 ] * a[ 9 ];
	tmp[ 44 ] += a[ 1 ] * a[ 10 ];
	tmp[ 45 ] += a[ 1 ] * a[ 11 ];
	tmp[ 46 ] += a[ 1 ] * a[ 14 ];
	tmp[ 47 ] += a[ 1 ] * a[ 15 ];
	tmp[ 48 ] += a[ 1 ] * a[ 16 ];
	tmp[ 49 ] += a[ 1 ] * a[ 17 ];
	tmp[ 50 ] += a[ 1 ] * a[ 18 ];
	tmp[ 51 ] += a[ 1 ] * a[ 19 ];
	tmp[ 52 ] += a[ 1 ] * a[ 20 ];
	tmp[ 53 ] += a[ 1 ] * a[ 21 ];
	tmp[ 54 ] += a[ 1 ] * a[ 22 ];
	tmp[ 55 ] += a[ 1 ] * a[ 23 ];
	tmp[ 56 ] += a[ 1 ] * a[ 24 ];
	tmp[ 57 ] += a[ 2 ] * a[ 3 ];
	tmp[ 58 ] += a[ 2 ] * a[ 4 ];
	tmp[ 59 ] += a[ 2 ] * a[ 5 ];
	tmp[ 60 ] += a[ 2 ] * a[ 6 ];
	tmp[ 61 ] += a[ 2 ] * a[ 7 ];
	tmp[ 62 ] += a[ 2 ] * a[ 8 ];
	tmp[ 63 ] += a[ 2 ] * a[ 9 ];
	tmp[ 64 ] += a[ 2 ] * a[ 10 ];
	tmp[ 65 ] += a[ 2 ] * a[ 11 ];
	tmp[ 66 ] += a[ 2 ] * a[ 15 ];
	tmp[ 67 ] += a[ 2 ] * a[ 16 ];
	tmp[ 68 ] += a[ 2 ] * a[ 17 ];
	tmp[ 69 ] += a[ 2 ] * a[ 18 ];
	tmp[ 70 ] += a[ 2 ] * a[ 19 ];
	tmp[ 71 ] += a[ 2 ] * a[ 20 ];
	tmp[ 72 ] += a[ 2 ] * a[ 21 ];
	tmp[ 73 ] += a[ 2 ] * a[ 22 ];
	tmp[ 74 ] += a[ 2 ] * a[ 23 ];
	tmp[ 75 ] += a[ 2 ] * a[ 24 ];
	tmp[ 76 ] += a[ 3 ] * a[ 4 ];
	tmp[ 77 ] += a[ 3 ] * a[ 5 ];
	tmp[ 78 ] += a[ 3 ] * a[ 6 ];
	tmp[ 79 ] += a[ 3 ] * a[ 7 ];
	tmp[ 80 ] += a[ 3 ] * a[ 8 ];
	tmp[ 81 ] += a[ 3 ] * a[ 9 ];
	tmp[ 82 ] += a[ 3 ] * a[ 10 ];
	tmp[ 83 ] += a[ 3 ] * a[ 11 ];
	tmp[ 84 ] += a[ 3 ] * a[ 15 ];
	tmp[ 85 ] += a[ 3 ] * a[ 16 ];
	tmp[ 86 ] += a[ 3 ] * a[ 17 ];
	tmp[ 87 ] += a[ 3 ] * a[ 18 ];
	tmp[ 88 ] += a[ 3 ] * a[ 19 ];
	tmp[ 89 ] += a[ 3 ] * a[ 20 ];
	tmp[ 90 ] += a[ 3 ] * a[ 21 ];
	tmp[ 91 ] += a[ 3 ] * a[ 22 ];
	tmp[ 92 ] += a[ 3 ] * a[ 23 ];
	tmp[ 93 ] += a[ 3 ] * a[ 24 ];
	tmp[ 94 ] += a[ 4 ] * a[ 5 ];
	tmp[ 95 ] += a[ 4 ] * a[ 6 ];
	tmp[ 96 ] += a[ 4 ] * a[ 7 ];
	tmp[ 97 ] += a[ 4 ] * a[ 8 ];
	tmp[ 98 ] += a[ 4 ] * a[ 9 ];
	tmp[ 99 ] += a[ 4 ] * a[ 10 ];
	tmp[ 100 ] += a[ 4 ] * a[ 11 ];
	tmp[ 101 ] += a[ 4 ] * a[ 15 ];
	tmp[ 102 ] += a[ 4 ] * a[ 16 ];
	tmp[ 103 ] += a[ 4 ] * a[ 17 ];
	tmp[ 104 ] += a[ 4 ] * a[ 18 ];
	tmp[ 105 ] += a[ 4 ] * a[ 19 ];
	tmp[ 106 ] += a[ 4 ] * a[ 20 ];
	tmp[ 107 ] += a[ 4 ] * a[ 21 ];
	tmp[ 108 ] += a[ 4 ] * a[ 22 ];
	tmp[ 109 ] += a[ 4 ] * a[ 23 ];
	tmp[ 110 ] += a[ 4 ] * a[ 24 ];
	tmp[ 111 ] += a[ 5 ] * a[ 6 ];
	tmp[ 112 ] += a[ 5 ] * a[ 7 ];
	tmp[ 113 ] += a[ 5 ] * a[ 8 ];
	tmp[ 114 ] += a[ 5 ] * a[ 9 ];
	tmp[ 115 ] += a[ 5 ] * a[ 10 ];
	tmp[ 116 ] += a[ 5 ] * a[ 11 ];
	tmp[ 117 ] += a[ 5 ] * a[ 13 ];
	tmp[ 118 ] += a[ 5 ] * a[ 14 ];
	tmp[ 119 ] += a[ 5 ] * a[ 18 ];
	tmp[ 120 ] += a[ 5 ] * a[ 19 ];
	tmp[ 121 ] += a[ 5 ] * a[ 20 ];
	tmp[ 122 ] += a[ 5 ] * a[ 21 ];
	tmp[ 123 ] += a[ 5 ] * a[ 22 ];
	tmp[ 124 ] += a[ 5 ] * a[ 23 ];
	tmp[ 125 ] += a[ 5 ] * a[ 24 ];
	tmp[ 126 ] += a[ 6 ] * a[ 7 ];
	tmp[ 127 ] += a[ 6 ] * a[ 8 ];
	tmp[ 128 ] += a[ 6 ] * a[ 9 ];
	tmp[ 129 ] += a[ 6 ] * a[ 10 ];
	tmp[ 130 ] += a[ 6 ] * a[ 11 ];
	tmp[ 131 ] += a[ 6 ] * a[ 14 ];
	tmp[ 132 ] += a[ 6 ] * a[ 19 ];
	tmp[ 133 ] += a[ 6 ] * a[ 20 ];
	tmp[ 134 ] += a[ 6 ] * a[ 21 ];
	tmp[ 135 ] += a[ 6 ] * a[ 22 ];
	tmp[ 136 ] += a[ 6 ] * a[ 23 ];
	tmp[ 137 ] += a[ 6 ] * a[ 24 ];
	tmp[ 138 ] += a[ 7 ] * a[ 8 ];
	tmp[ 139 ] += a[ 7 ] * a[ 9 ];
	tmp[ 140 ] += a[ 7 ] * a[ 10 ];
	tmp[ 141 ] += a[ 7 ] * a[ 11 ];
	tmp[ 142 ] += a[ 7 ] * a[ 20 ];
	tmp[ 143 ] += a[ 7 ] * a[ 21 ];
	tmp[ 144 ] += a[ 7 ] * a[ 22 ];
	tmp[ 145 ] += a[ 7 ] * a[ 23 ];
	tmp[ 146 ] += a[ 7 ] * a[ 24 ];
	tmp[ 147 ] += a[ 8 ] * a[ 9 ];
	tmp[ 148 ] += a[ 8 ] * a[ 10 ];
	tmp[ 149 ] += a[ 8 ] * a[ 11 ];
	tmp[ 150 ] += a[ 8 ] * a[ 15 ];
	tmp[ 151 ] += a[ 8 ] * a[ 20 ];
	tmp[ 152 ] += a[ 8 ] * a[ 21 ];
	tmp[ 153 ] += a[ 8 ] * a[ 22 ];
	tmp[ 154 ] += a[ 8 ] * a[ 23 ];
	tmp[ 155 ] += a[ 8 ] * a[ 24 ];
	tmp[ 156 ] += a[ 9 ] * a[ 10 ];
	tmp[ 157 ] += a[ 9 ] * a[ 11 ];
	tmp[ 158 ] += a[ 9 ] * a[ 15 ];
	tmp[ 159 ] += a[ 9 ] * a[ 16 ];
	tmp[ 160 ] += a[ 9 ] * a[ 20 ];
	tmp[ 161 ] += a[ 9 ] * a[ 21 ];
	tmp[ 162 ] += a[ 9 ] * a[ 22 ];
	tmp[ 163 ] += a[ 9 ] * a[ 23 ];
	tmp[ 164 ] += a[ 9 ] * a[ 24 ];
	tmp[ 165 ] += a[ 10 ] * a[ 11 ];
	tmp[ 166 ] += a[ 10 ] * a[ 13 ];
	tmp[ 167 ] += a[ 10 ] * a[ 14 ];
	tmp[ 168 ] += a[ 10 ] * a[ 18 ];
	tmp[ 169 ] += a[ 10 ] * a[ 19 ];
	tmp[ 170 ] += a[ 10 ] * a[ 23 ];
	tmp[ 171 ] += a[ 10 ] * a[ 24 ];
	tmp[ 172 ] += a[ 11 ] * a[ 14 ];
	tmp[ 173 ] += a[ 11 ] * a[ 19 ];
	tmp[ 174 ] += a[ 11 ] * a[ 24 ];
	tmp[ 175 ] += a[ 13 ] * a[ 15 ];
	tmp[ 176 ] += a[ 13 ] * a[ 20 ];
	tmp[ 177 ] += a[ 14 ] * a[ 15 ];
	tmp[ 178 ] += a[ 14 ] * a[ 16 ];
	tmp[ 179 ] += a[ 14 ] * a[ 20 ];
	tmp[ 180 ] += a[ 14 ] * a[ 21 ];
	tmp[ 181 ] += a[ 15 ] * a[ 18 ];
	tmp[ 182 ] += a[ 15 ] * a[ 19 ];
	tmp[ 183 ] += a[ 15 ] * a[ 23 ];
	tmp[ 184 ] += a[ 15 ] * a[ 24 ];
	tmp[ 185 ] += a[ 16 ] * a[ 19 ];
	tmp[ 186 ] += a[ 16 ] * a[ 24 ];
	tmp[ 187 ] += a[ 18 ] * a[ 20 ];
	tmp[ 188 ] += a[ 19 ] * a[ 20 ];
	tmp[ 189 ] += a[ 19 ] * a[ 21 ];
	tmp[ 190 ] += a[ 20 ] * a[ 23 ];
	tmp[ 191 ] += a[ 20 ] * a[ 24 ];
	tmp[ 192 ] += a[ 21 ] * a[ 24 ];
	tmp[ 193 ] += a[ 0 ] * a[ 1 ] * a[ 2 ];
	tmp[ 194 ] += a[ 0 ] * a[ 1 ] * a[ 3 ];
	tmp[ 195 ] += a[ 0 ] * a[ 1 ] * a[ 4 ];
	tmp[ 196 ] += a[ 0 ] * a[ 1 ] * a[ 5 ];
	tmp[ 197 ] += a[ 0 ] * a[ 1 ] * a[ 6 ];
	tmp[ 198 ] += a[ 0 ] * a[ 1 ] * a[ 7 ];
	tmp[ 199 ] += a[ 0 ] * a[ 1 ] * a[ 8 ];
	tmp[ 200 ] += a[ 0 ] * a[ 1 ] * a[ 9 ];
	tmp[ 201 ] += a[ 0 ] * a[ 1 ] * a[ 10 ];
	tmp[ 202 ] += a[ 0 ] * a[ 1 ] * a[ 11 ];
	tmp[ 203 ] += a[ 0 ] * a[ 1 ] * a[ 13 ];
	tmp[ 204 ] += a[ 0 ] * a[ 1 ] * a[ 14 ];
	tmp[ 205 ] += a[ 0 ] * a[ 1 ] * a[ 15 ];
	tmp[ 206 ] += a[ 0 ] * a[ 1 ] * a[ 16 ];
	tmp[ 207 ] += a[ 0 ] * a[ 1 ] * a[ 17 ];
	tmp[ 208 ] += a[ 0 ] * a[ 1 ] * a[ 18 ];
	tmp[ 209 ] += a[ 0 ] * a[ 1 ] * a[ 19 ];
	tmp[ 210 ] += a[ 0 ] * a[ 1 ] * a[ 20 ];
	tmp[ 211 ] += a[ 0 ] * a[ 1 ] * a[ 21 ];
	tmp[ 212 ] += a[ 0 ] * a[ 1 ] * a[ 22 ];
	tmp[ 213 ] += a[ 0 ] * a[ 1 ] * a[ 23 ];
	tmp[ 214 ] += a[ 0 ] * a[ 1 ] * a[ 24 ];
	tmp[ 215 ] += a[ 0 ] * a[ 2 ] * a[ 3 ];
	tmp[ 216 ] += a[ 0 ] * a[ 2 ] * a[ 4 ];
	tmp[ 217 ] += a[ 0 ] * a[ 2 ] * a[ 5 ];
	tmp[ 218 ] += a[ 0 ] * a[ 2 ] * a[ 6 ];
	tmp[ 219 ] += a[ 0 ] * a[ 2 ] * a[ 7 ];
	tmp[ 220 ] += a[ 0 ] * a[ 2 ] * a[ 8 ];
	tmp[ 221 ] += a[ 0 ] * a[ 2 ] * a[ 9 ];
	tmp[ 222 ] += a[ 0 ] * a[ 2 ] * a[ 10 ];
	tmp[ 223 ] += a[ 0 ] * a[ 2 ] * a[ 11 ];
	tmp[ 224 ] += a[ 0 ] * a[ 2 ] * a[ 13 ];
	tmp[ 225 ] += a[ 0 ] * a[ 2 ] * a[ 14 ];
	tmp[ 226 ] += a[ 0 ] * a[ 2 ] * a[ 15 ];
	tmp[ 227 ] += a[ 0 ] * a[ 2 ] * a[ 16 ];
	tmp[ 228 ] += a[ 0 ] * a[ 2 ] * a[ 17 ];
	tmp[ 229 ] += a[ 0 ] * a[ 2 ] * a[ 18 ];
	tmp[ 230 ] += a[ 0 ] * a[ 2 ] * a[ 19 ];
	tmp[ 231 ] += a[ 0 ] * a[ 2 ] * a[ 20 ];
	tmp[ 232 ] += a[ 0 ] * a[ 2 ] * a[ 21 ];
	tmp[ 233 ] += a[ 0 ] * a[ 2 ] * a[ 22 ];
	tmp[ 234 ] += a[ 0 ] * a[ 2 ] * a[ 23 ];
	tmp[ 235 ] += a[ 0 ] * a[ 2 ] * a[ 24 ];
	tmp[ 236 ] += a[ 0 ] * a[ 3 ] * a[ 4 ];
	tmp[ 237 ] += a[ 0 ] * a[ 3 ] * a[ 5 ];
	tmp[ 238 ] += a[ 0 ] * a[ 3 ] * a[ 6 ];
	tmp[ 239 ] += a[ 0 ] * a[ 3 ] * a[ 7 ];
	tmp[ 240 ] += a[ 0 ] * a[ 3 ] * a[ 8 ];
	tmp[ 241 ] += a[ 0 ] * a[ 3 ] * a[ 9 ];
	tmp[ 242 ] += a[ 0 ] * a[ 3 ] * a[ 10 ];
	tmp[ 243 ] += a[ 0 ] * a[ 3 ] * a[ 11 ];
	tmp[ 244 ] += a[ 0 ] * a[ 3 ] * a[ 13 ];
	tmp[ 245 ] += a[ 0 ] * a[ 3 ] * a[ 14 ];
	tmp[ 246 ] += a[ 0 ] * a[ 3 ] * a[ 15 ];
	tmp[ 247 ] += a[ 0 ] * a[ 3 ] * a[ 16 ];
	tmp[ 248 ] += a[ 0 ] * a[ 3 ] * a[ 17 ];
	tmp[ 249 ] += a[ 0 ] * a[ 3 ] * a[ 18 ];
	tmp[ 250 ] += a[ 0 ] * a[ 3 ] * a[ 19 ];
	tmp[ 251 ] += a[ 0 ] * a[ 3 ] * a[ 20 ];
	tmp[ 252 ] += a[ 0 ] * a[ 3 ] * a[ 21 ];
	tmp[ 253 ] += a[ 0 ] * a[ 3 ] * a[ 22 ];
	tmp[ 254 ] += a[ 0 ] * a[ 3 ] * a[ 23 ];
	tmp[ 255 ] += a[ 0 ] * a[ 3 ] * a[ 24 ];
	tmp[ 256 ] += a[ 0 ] * a[ 4 ] * a[ 5 ];
	tmp[ 257 ] += a[ 0 ] * a[ 4 ] * a[ 6 ];
	tmp[ 258 ] += a[ 0 ] * a[ 4 ] * a[ 7 ];
	tmp[ 259 ] += a[ 0 ] * a[ 4 ] * a[ 8 ];
	tmp[ 260 ] += a[ 0 ] * a[ 4 ] * a[ 9 ];
	tmp[ 261 ] += a[ 0 ] * a[ 4 ] * a[ 10 ];
	tmp[ 262 ] += a[ 0 ] * a[ 4 ] * a[ 11 ];
	tmp[ 263 ] += a[ 0 ] * a[ 4 ] * a[ 13 ];
	tmp[ 264 ] += a[ 0 ] * a[ 4 ] * a[ 14 ];
	tmp[ 265 ] += a[ 0 ] * a[ 4 ] * a[ 15 ];
	tmp[ 266 ] += a[ 0 ] * a[ 4 ] * a[ 16 ];
	tmp[ 267 ] += a[ 0 ] * a[ 4 ] * a[ 17 ];
	tmp[ 268 ] += a[ 0 ] * a[ 4 ] * a[ 18 ];
	tmp[ 269 ] += a[ 0 ] * a[ 4 ] * a[ 19 ];
	tmp[ 270 ] += a[ 0 ] * a[ 4 ] * a[ 20 ];
	tmp[ 271 ] += a[ 0 ] * a[ 4 ] * a[ 21 ];
	tmp[ 272 ] += a[ 0 ] * a[ 4 ] * a[ 22 ];
	tmp[ 273 ] += a[ 0 ] * a[ 4 ] * a[ 23 ];
	tmp[ 274 ] += a[ 0 ] * a[ 4 ] * a[ 24 ];
	tmp[ 275 ] += a[ 0 ] * a[ 5 ] * a[ 6 ];
	tmp[ 276 ] += a[ 0 ] * a[ 5 ] * a[ 7 ];
	tmp[ 277 ] += a[ 0 ] * a[ 5 ] * a[ 8 ];
	tmp[ 278 ] += a[ 0 ] * a[ 5 ] * a[ 9 ];
	tmp[ 279 ] += a[ 0 ] * a[ 5 ] * a[ 10 ];
	tmp[ 280 ] += a[ 0 ] * a[ 5 ] * a[ 11 ];
	tmp[ 281 ] += a[ 0 ] * a[ 5 ] * a[ 13 ];
	tmp[ 282 ] += a[ 0 ] * a[ 5 ] * a[ 14 ];
	tmp[ 283 ] += a[ 0 ] * a[ 5 ] * a[ 15 ];
	tmp[ 284 ] += a[ 0 ] * a[ 5 ] * a[ 16 ];
	tmp[ 285 ] += a[ 0 ] * a[ 5 ] * a[ 17 ];
	tmp[ 286 ] += a[ 0 ] * a[ 5 ] * a[ 18 ];
	tmp[ 287 ] += a[ 0 ] * a[ 5 ] * a[ 19 ];
	tmp[ 288 ] += a[ 0 ] * a[ 5 ] * a[ 20 ];
	tmp[ 289 ] += a[ 0 ] * a[ 5 ] * a[ 21 ];
	tmp[ 290 ] += a[ 0 ] * a[ 5 ] * a[ 22 ];
	tmp[ 291 ] += a[ 0 ] * a[ 5 ] * a[ 23 ];
	tmp[ 292 ] += a[ 0 ] * a[ 5 ] * a[ 24 ];
	tmp[ 293 ] += a[ 0 ] * a[ 6 ] * a[ 7 ];
	tmp[ 294 ] += a[ 0 ] * a[ 6 ] * a[ 8 ];
	tmp[ 295 ] += a[ 0 ] * a[ 6 ] * a[ 9 ];
	tmp[ 296 ] += a[ 0 ] * a[ 6 ] * a[ 10 ];
	tmp[ 297 ] += a[ 0 ] * a[ 6 ] * a[ 11 ];
	tmp[ 298 ] += a[ 0 ] * a[ 6 ] * a[ 13 ];
	tmp[ 299 ] += a[ 0 ] * a[ 6 ] * a[ 14 ];
	tmp[ 300 ] += a[ 0 ] * a[ 6 ] * a[ 15 ];
	tmp[ 301 ] += a[ 0 ] * a[ 6 ] * a[ 16 ];
	tmp[ 302 ] += a[ 0 ] * a[ 6 ] * a[ 17 ];
	tmp[ 303 ] += a[ 0 ] * a[ 6 ] * a[ 18 ];
	tmp[ 304 ] += a[ 0 ] * a[ 6 ] * a[ 19 ];
	tmp[ 305 ] += a[ 0 ] * a[ 6 ] * a[ 20 ];
	tmp[ 306 ] += a[ 0 ] * a[ 6 ] * a[ 21 ];
	tmp[ 307 ] += a[ 0 ] * a[ 6 ] * a[ 22 ];
	tmp[ 308 ] += a[ 0 ] * a[ 6 ] * a[ 23 ];
	tmp[ 309 ] += a[ 0 ] * a[ 6 ] * a[ 24 ];
	tmp[ 310 ] += a[ 0 ] * a[ 7 ] * a[ 8 ];
	tmp[ 311 ] += a[ 0 ] * a[ 7 ] * a[ 9 ];
	tmp[ 312 ] += a[ 0 ] * a[ 7 ] * a[ 10 ];
	tmp[ 313 ] += a[ 0 ] * a[ 7 ] * a[ 11 ];
	tmp[ 314 ] += a[ 0 ] * a[ 7 ] * a[ 13 ];
	tmp[ 315 ] += a[ 0 ] * a[ 7 ] * a[ 14 ];
	tmp[ 316 ] += a[ 0 ] * a[ 7 ] * a[ 15 ];
	tmp[ 317 ] += a[ 0 ] * a[ 7 ] * a[ 16 ];
	tmp[ 318 ] += a[ 0 ] * a[ 7 ] * a[ 17 ];
	tmp[ 319 ] += a[ 0 ] * a[ 7 ] * a[ 18 ];
	tmp[ 320 ] += a[ 0 ] * a[ 7 ] * a[ 19 ];
	tmp[ 321 ] += a[ 0 ] * a[ 7 ] * a[ 20 ];
	tmp[ 322 ] += a[ 0 ] * a[ 7 ] * a[ 21 ];
	tmp[ 323 ] += a[ 0 ] * a[ 7 ] * a[ 22 ];
	tmp[ 324 ] += a[ 0 ] * a[ 7 ] * a[ 23 ];
	tmp[ 325 ] += a[ 0 ] * a[ 7 ] * a[ 24 ];
	tmp[ 326 ] += a[ 0 ] * a[ 8 ] * a[ 9 ];
	tmp[ 327 ] += a[ 0 ] * a[ 8 ] * a[ 10 ];
	tmp[ 328 ] += a[ 0 ] * a[ 8 ] * a[ 11 ];
	tmp[ 329 ] += a[ 0 ] * a[ 8 ] * a[ 13 ];
	tmp[ 330 ] += a[ 0 ] * a[ 8 ] * a[ 14 ];
	tmp[ 331 ] += a[ 0 ] * a[ 8 ] * a[ 15 ];
	tmp[ 332 ] += a[ 0 ] * a[ 8 ] * a[ 16 ];
	tmp[ 333 ] += a[ 0 ] * a[ 8 ] * a[ 17 ];
	tmp[ 334 ] += a[ 0 ] * a[ 8 ] * a[ 18 ];
	tmp[ 335 ] += a[ 0 ] * a[ 8 ] * a[ 19 ];
	tmp[ 336 ] += a[ 0 ] * a[ 8 ] * a[ 20 ];
	tmp[ 337 ] += a[ 0 ] * a[ 8 ] * a[ 21 ];
	tmp[ 338 ] += a[ 0 ] * a[ 8 ] * a[ 22 ];
	tmp[ 339 ] += a[ 0 ] * a[ 8 ] * a[ 23 ];
	tmp[ 340 ] += a[ 0 ] * a[ 8 ] * a[ 24 ];
	tmp[ 341 ] += a[ 0 ] * a[ 9 ] * a[ 10 ];
	tmp[ 342 ] += a[ 0 ] * a[ 9 ] * a[ 11 ];
	tmp[ 343 ] += a[ 0 ] * a[ 9 ] * a[ 13 ];
	tmp[ 344 ] += a[ 0 ] * a[ 9 ] * a[ 14 ];
	tmp[ 345 ] += a[ 0 ] * a[ 9 ] * a[ 15 ];
	tmp[ 346 ] += a[ 0 ] * a[ 9 ] * a[ 16 ];
	tmp[ 347 ] += a[ 0 ] * a[ 9 ] * a[ 17 ];
	tmp[ 348 ] += a[ 0 ] * a[ 9 ] * a[ 18 ];
	tmp[ 349 ] += a[ 0 ] * a[ 9 ] * a[ 19 ];
	tmp[ 350 ] += a[ 0 ] * a[ 9 ] * a[ 20 ];
	tmp[ 351 ] += a[ 0 ] * a[ 9 ] * a[ 21 ];
	tmp[ 352 ] += a[ 0 ] * a[ 9 ] * a[ 22 ];
	tmp[ 353 ] += a[ 0 ] * a[ 9 ] * a[ 23 ];
	tmp[ 354 ] += a[ 0 ] * a[ 9 ] * a[ 24 ];
	tmp[ 355 ] += a[ 0 ] * a[ 10 ] * a[ 11 ];
	tmp[ 356 ] += a[ 0 ] * a[ 10 ] * a[ 13 ];
	tmp[ 357 ] += a[ 0 ] * a[ 10 ] * a[ 14 ];
	tmp[ 358 ] += a[ 0 ] * a[ 10 ] * a[ 15 ];
	tmp[ 359 ] += a[ 0 ] * a[ 10 ] * a[ 16 ];
	tmp[ 360 ] += a[ 0 ] * a[ 10 ] * a[ 17 ];
	tmp[ 361 ] += a[ 0 ] * a[ 10 ] * a[ 18 ];
	tmp[ 362 ] += a[ 0 ] * a[ 10 ] * a[ 19 ];
	tmp[ 363 ] += a[ 0 ] * a[ 10 ] * a[ 20 ];
	tmp[ 364 ] += a[ 0 ] * a[ 10 ] * a[ 21 ];
	tmp[ 365 ] += a[ 0 ] * a[ 10 ] * a[ 22 ];
	tmp[ 366 ] += a[ 0 ] * a[ 10 ] * a[ 23 ];
	tmp[ 367 ] += a[ 0 ] * a[ 10 ] * a[ 24 ];
	tmp[ 368 ] += a[ 0 ] * a[ 11 ] * a[ 13 ];
	tmp[ 369 ] += a[ 0 ] * a[ 11 ] * a[ 14 ];
	tmp[ 370 ] += a[ 0 ] * a[ 11 ] * a[ 15 ];
	tmp[ 371 ] += a[ 0 ] * a[ 11 ] * a[ 16 ];
	tmp[ 372 ] += a[ 0 ] * a[ 11 ] * a[ 17 ];
	tmp[ 373 ] += a[ 0 ] * a[ 11 ] * a[ 18 ];
	tmp[ 374 ] += a[ 0 ] * a[ 11 ] * a[ 19 ];
	tmp[ 375 ] += a[ 0 ] * a[ 11 ] * a[ 20 ];
	tmp[ 376 ] += a[ 0 ] * a[ 11 ] * a[ 21 ];
	tmp[ 377 ] += a[ 0 ] * a[ 11 ] * a[ 22 ];
	tmp[ 378 ] += a[ 0 ] * a[ 11 ] * a[ 23 ];
	tmp[ 379 ] += a[ 0 ] * a[ 11 ] * a[ 24 ];
	tmp[ 380 ] += a[ 0 ] * a[ 13 ] * a[ 14 ];
	tmp[ 381 ] += a[ 0 ] * a[ 13 ] * a[ 15 ];
	tmp[ 382 ] += a[ 0 ] * a[ 13 ] * a[ 16 ];
	tmp[ 383 ] += a[ 0 ] * a[ 13 ] * a[ 17 ];
	tmp[ 384 ] += a[ 0 ] * a[ 13 ] * a[ 18 ];
	tmp[ 385 ] += a[ 0 ] * a[ 13 ] * a[ 19 ];
	tmp[ 386 ] += a[ 0 ] * a[ 13 ] * a[ 20 ];
	tmp[ 387 ] += a[ 0 ] * a[ 13 ] * a[ 21 ];
	tmp[ 388 ] += a[ 0 ] * a[ 13 ] * a[ 22 ];
	tmp[ 389 ] += a[ 0 ] * a[ 13 ] * a[ 23 ];
	tmp[ 390 ] += a[ 0 ] * a[ 13 ] * a[ 24 ];
	tmp[ 391 ] += a[ 0 ] * a[ 14 ] * a[ 15 ];
	tmp[ 392 ] += a[ 0 ] * a[ 14 ] * a[ 16 ];
	tmp[ 393 ] += a[ 0 ] * a[ 14 ] * a[ 17 ];
	tmp[ 394 ] += a[ 0 ] * a[ 14 ] * a[ 18 ];
	tmp[ 395 ] += a[ 0 ] * a[ 14 ] * a[ 19 ];
	tmp[ 396 ] += a[ 0 ] * a[ 14 ] * a[ 20 ];
	tmp[ 397 ] += a[ 0 ] * a[ 14 ] * a[ 21 ];
	tmp[ 398 ] += a[ 0 ] * a[ 14 ] * a[ 22 ];
	tmp[ 399 ] += a[ 0 ] * a[ 14 ] * a[ 23 ];
	tmp[ 400 ] += a[ 0 ] * a[ 14 ] * a[ 24 ];
	tmp[ 401 ] += a[ 0 ] * a[ 15 ] * a[ 16 ];
	tmp[ 402 ] += a[ 0 ] * a[ 15 ] * a[ 17 ];
	tmp[ 403 ] += a[ 0 ] * a[ 15 ] * a[ 18 ];
	tmp[ 404 ] += a[ 0 ] * a[ 15 ] * a[ 19 ];
	tmp[ 405 ] += a[ 0 ] * a[ 15 ] * a[ 20 ];
	tmp[ 406 ] += a[ 0 ] * a[ 15 ] * a[ 21 ];
	tmp[ 407 ] += a[ 0 ] * a[ 15 ] * a[ 22 ];
	tmp[ 408 ] += a[ 0 ] * a[ 15 ] * a[ 23 ];
	tmp[ 409 ] += a[ 0 ] * a[ 15 ] * a[ 24 ];
	tmp[ 410 ] += a[ 0 ] * a[ 16 ] * a[ 17 ];
	tmp[ 411 ] += a[ 0 ] * a[ 16 ] * a[ 18 ];
	tmp[ 412 ] += a[ 0 ] * a[ 16 ] * a[ 19 ];
	tmp[ 413 ] += a[ 0 ] * a[ 16 ] * a[ 20 ];
	tmp[ 414 ] += a[ 0 ] * a[ 16 ] * a[ 21 ];
	tmp[ 415 ] += a[ 0 ] * a[ 16 ] * a[ 22 ];
	tmp[ 416 ] += a[ 0 ] * a[ 16 ] * a[ 23 ];
	tmp[ 417 ] += a[ 0 ] * a[ 16 ] * a[ 24 ];
	tmp[ 418 ] += a[ 0 ] * a[ 17 ] * a[ 18 ];
	tmp[ 419 ] += a[ 0 ] * a[ 17 ] * a[ 19 ];
	tmp[ 420 ] += a[ 0 ] * a[ 17 ] * a[ 20 ];
	tmp[ 421 ] += a[ 0 ] * a[ 17 ] * a[ 21 ];
	tmp[ 422 ] += a[ 0 ] * a[ 17 ] * a[ 22 ];
	tmp[ 423 ] += a[ 0 ] * a[ 17 ] * a[ 23 ];
	tmp[ 424 ] += a[ 0 ] * a[ 17 ] * a[ 24 ];
	tmp[ 425 ] += a[ 0 ] * a[ 18 ] * a[ 19 ];
	tmp[ 426 ] += a[ 0 ] * a[ 18 ] * a[ 20 ];
	tmp[ 427 ] += a[ 0 ] * a[ 18 ] * a[ 21 ];
	tmp[ 428 ] += a[ 0 ] * a[ 18 ] * a[ 22 ];
	tmp[ 429 ] += a[ 0 ] * a[ 18 ] * a[ 23 ];
	tmp[ 430 ] += a[ 0 ] * a[ 18 ] * a[ 24 ];
	tmp[ 431 ] += a[ 0 ] * a[ 19 ] * a[ 20 ];
	tmp[ 432 ] += a[ 0 ] * a[ 19 ] * a[ 21 ];
	tmp[ 433 ] += a[ 0 ] * a[ 19 ] * a[ 22 ];
	tmp[ 434 ] += a[ 0 ] * a[ 19 ] * a[ 23 ];
	tmp[ 435 ] += a[ 0 ] * a[ 19 ] * a[ 24 ];
	tmp[ 436 ] += a[ 0 ] * a[ 20 ] * a[ 21 ];
	tmp[ 437 ] += a[ 0 ] * a[ 20 ] * a[ 22 ];
	tmp[ 438 ] += a[ 0 ] * a[ 20 ] * a[ 23 ];
	tmp[ 439 ] += a[ 0 ] * a[ 20 ] * a[ 24 ];
	tmp[ 440 ] += a[ 0 ] * a[ 21 ] * a[ 22 ];
	tmp[ 441 ] += a[ 0 ] * a[ 21 ] * a[ 23 ];
	tmp[ 442 ] += a[ 0 ] * a[ 21 ] * a[ 24 ];
	tmp[ 443 ] += a[ 0 ] * a[ 22 ] * a[ 23 ];
	tmp[ 444 ] += a[ 0 ] * a[ 22 ] * a[ 24 ];
	tmp[ 445 ] += a[ 0 ] * a[ 23 ] * a[ 24 ];
	tmp[ 446 ] += a[ 1 ] * a[ 2 ] * a[ 3 ];
	tmp[ 447 ] += a[ 1 ] * a[ 2 ] * a[ 4 ];
	tmp[ 448 ] += a[ 1 ] * a[ 2 ] * a[ 5 ];
	tmp[ 449 ] += a[ 1 ] * a[ 2 ] * a[ 6 ];
	tmp[ 450 ] += a[ 1 ] * a[ 2 ] * a[ 7 ];
	tmp[ 451 ] += a[ 1 ] * a[ 2 ] * a[ 8 ];
	tmp[ 452 ] += a[ 1 ] * a[ 2 ] * a[ 9 ];
	tmp[ 453 ] += a[ 1 ] * a[ 2 ] * a[ 10 ];
	tmp[ 454 ] += a[ 1 ] * a[ 2 ] * a[ 11 ];
	tmp[ 455 ] += a[ 1 ] * a[ 2 ] * a[ 14 ];
	tmp[ 456 ] += a[ 1 ] * a[ 2 ] * a[ 15 ];
	tmp[ 457 ] += a[ 1 ] * a[ 2 ] * a[ 16 ];
	tmp[ 458 ] += a[ 1 ] * a[ 2 ] * a[ 17 ];
	tmp[ 459 ] += a[ 1 ] * a[ 2 ] * a[ 18 ];
	tmp[ 460 ] += a[ 1 ] * a[ 2 ] * a[ 19 ];
	tmp[ 461 ] += a[ 1 ] * a[ 2 ] * a[ 20 ];
	tmp[ 462 ] += a[ 1 ] * a[ 2 ] * a[ 21 ];
	tmp[ 463 ] += a[ 1 ] * a[ 2 ] * a[ 22 ];
	tmp[ 464 ] += a[ 1 ] * a[ 2 ] * a[ 23 ];
	tmp[ 465 ] += a[ 1 ] * a[ 2 ] * a[ 24 ];
	tmp[ 466 ] += a[ 1 ] * a[ 3 ] * a[ 4 ];
	tmp[ 467 ] += a[ 1 ] * a[ 3 ] * a[ 5 ];
	tmp[ 468 ] += a[ 1 ] * a[ 3 ] * a[ 6 ];
	tmp[ 469 ] += a[ 1 ] * a[ 3 ] * a[ 7 ];
	tmp[ 470 ] += a[ 1 ] * a[ 3 ] * a[ 8 ];
	tmp[ 471 ] += a[ 1 ] * a[ 3 ] * a[ 9 ];
	tmp[ 472 ] += a[ 1 ] * a[ 3 ] * a[ 10 ];
	tmp[ 473 ] += a[ 1 ] * a[ 3 ] * a[ 11 ];
	tmp[ 474 ] += a[ 1 ] * a[ 3 ] * a[ 14 ];
	tmp[ 475 ] += a[ 1 ] * a[ 3 ] * a[ 15 ];
	tmp[ 476 ] += a[ 1 ] * a[ 3 ] * a[ 16 ];
	tmp[ 477 ] += a[ 1 ] * a[ 3 ] * a[ 17 ];
	tmp[ 478 ] += a[ 1 ] * a[ 3 ] * a[ 18 ];
	tmp[ 479 ] += a[ 1 ] * a[ 3 ] * a[ 19 ];
	tmp[ 480 ] += a[ 1 ] * a[ 3 ] * a[ 20 ];
	tmp[ 481 ] += a[ 1 ] * a[ 3 ] * a[ 21 ];
	tmp[ 482 ] += a[ 1 ] * a[ 3 ] * a[ 22 ];
	tmp[ 483 ] += a[ 1 ] * a[ 3 ] * a[ 23 ];
	tmp[ 484 ] += a[ 1 ] * a[ 3 ] * a[ 24 ];
	tmp[ 485 ] += a[ 1 ] * a[ 4 ] * a[ 5 ];
	tmp[ 486 ] += a[ 1 ] * a[ 4 ] * a[ 6 ];
	tmp[ 487 ] += a[ 1 ] * a[ 4 ] * a[ 7 ];
	tmp[ 488 ] += a[ 1 ] * a[ 4 ] * a[ 8 ];
	tmp[ 489 ] += a[ 1 ] * a[ 4 ] * a[ 9 ];
	tmp[ 490 ] += a[ 1 ] * a[ 4 ] * a[ 10 ];
	tmp[ 491 ] += a[ 1 ] * a[ 4 ] * a[ 11 ];
	tmp[ 492 ] += a[ 1 ] * a[ 4 ] * a[ 14 ];
	tmp[ 493 ] += a[ 1 ] * a[ 4 ] * a[ 15 ];
	tmp[ 494 ] += a[ 1 ] * a[ 4 ] * a[ 16 ];
	tmp[ 495 ] += a[ 1 ] * a[ 4 ] * a[ 17 ];
	tmp[ 496 ] += a[ 1 ] * a[ 4 ] * a[ 18 ];
	tmp[ 497 ] += a[ 1 ] * a[ 4 ] * a[ 19 ];
	tmp[ 498 ] += a[ 1 ] * a[ 4 ] * a[ 20 ];
	tmp[ 499 ] += a[ 1 ] * a[ 4 ] * a[ 21 ];
	tmp[ 500 ] += a[ 1 ] * a[ 4 ] * a[ 22 ];
	tmp[ 501 ] += a[ 1 ] * a[ 4 ] * a[ 23 ];
	tmp[ 502 ] += a[ 1 ] * a[ 4 ] * a[ 24 ];
	tmp[ 503 ] += a[ 1 ] * a[ 5 ] * a[ 6 ];
	tmp[ 504 ] += a[ 1 ] * a[ 5 ] * a[ 7 ];
	tmp[ 505 ] += a[ 1 ] * a[ 5 ] * a[ 8 ];
	tmp[ 506 ] += a[ 1 ] * a[ 5 ] * a[ 9 ];
	tmp[ 507 ] += a[ 1 ] * a[ 5 ] * a[ 10 ];
	tmp[ 508 ] += a[ 1 ] * a[ 5 ] * a[ 11 ];
	tmp[ 509 ] += a[ 1 ] * a[ 5 ] * a[ 13 ];
	tmp[ 510 ] += a[ 1 ] * a[ 5 ] * a[ 14 ];
	tmp[ 511 ] += a[ 1 ] * a[ 5 ] * a[ 15 ];
	tmp[ 512 ] += a[ 1 ] * a[ 5 ] * a[ 16 ];
	tmp[ 513 ] += a[ 1 ] * a[ 5 ] * a[ 17 ];
	tmp[ 514 ] += a[ 1 ] * a[ 5 ] * a[ 18 ];
	tmp[ 515 ] += a[ 1 ] * a[ 5 ] * a[ 19 ];
	tmp[ 516 ] += a[ 1 ] * a[ 5 ] * a[ 20 ];
	tmp[ 517 ] += a[ 1 ] * a[ 5 ] * a[ 21 ];
	tmp[ 518 ] += a[ 1 ] * a[ 5 ] * a[ 22 ];
	tmp[ 519 ] += a[ 1 ] * a[ 5 ] * a[ 23 ];
	tmp[ 520 ] += a[ 1 ] * a[ 5 ] * a[ 24 ];
	tmp[ 521 ] += a[ 1 ] * a[ 6 ] * a[ 7 ];
	tmp[ 522 ] += a[ 1 ] * a[ 6 ] * a[ 8 ];
	tmp[ 523 ] += a[ 1 ] * a[ 6 ] * a[ 9 ];
	tmp[ 524 ] += a[ 1 ] * a[ 6 ] * a[ 10 ];
	tmp[ 525 ] += a[ 1 ] * a[ 6 ] * a[ 11 ];
	tmp[ 526 ] += a[ 1 ] * a[ 6 ] * a[ 14 ];
	tmp[ 527 ] += a[ 1 ] * a[ 6 ] * a[ 15 ];
	tmp[ 528 ] += a[ 1 ] * a[ 6 ] * a[ 16 ];
	tmp[ 529 ] += a[ 1 ] * a[ 6 ] * a[ 17 ];
	tmp[ 530 ] += a[ 1 ] * a[ 6 ] * a[ 18 ];
	tmp[ 531 ] += a[ 1 ] * a[ 6 ] * a[ 19 ];
	tmp[ 532 ] += a[ 1 ] * a[ 6 ] * a[ 20 ];
	tmp[ 533 ] += a[ 1 ] * a[ 6 ] * a[ 21 ];
	tmp[ 534 ] += a[ 1 ] * a[ 6 ] * a[ 22 ];
	tmp[ 535 ] += a[ 1 ] * a[ 6 ] * a[ 23 ];
	tmp[ 536 ] += a[ 1 ] * a[ 6 ] * a[ 24 ];
	tmp[ 537 ] += a[ 1 ] * a[ 7 ] * a[ 8 ];
	tmp[ 538 ] += a[ 1 ] * a[ 7 ] * a[ 9 ];
	tmp[ 539 ] += a[ 1 ] * a[ 7 ] * a[ 10 ];
	tmp[ 540 ] += a[ 1 ] * a[ 7 ] * a[ 11 ];
	tmp[ 541 ] += a[ 1 ] * a[ 7 ] * a[ 14 ];
	tmp[ 542 ] += a[ 1 ] * a[ 7 ] * a[ 15 ];
	tmp[ 543 ] += a[ 1 ] * a[ 7 ] * a[ 16 ];
	tmp[ 544 ] += a[ 1 ] * a[ 7 ] * a[ 17 ];
	tmp[ 545 ] += a[ 1 ] * a[ 7 ] * a[ 18 ];
	tmp[ 546 ] += a[ 1 ] * a[ 7 ] * a[ 19 ];
	tmp[ 547 ] += a[ 1 ] * a[ 7 ] * a[ 20 ];
	tmp[ 548 ] += a[ 1 ] * a[ 7 ] * a[ 21 ];
	tmp[ 549 ] += a[ 1 ] * a[ 7 ] * a[ 22 ];
	tmp[ 550 ] += a[ 1 ] * a[ 7 ] * a[ 23 ];
	tmp[ 551 ] += a[ 1 ] * a[ 7 ] * a[ 24 ];
	tmp[ 552 ] += a[ 1 ] * a[ 8 ] * a[ 9 ];
	tmp[ 553 ] += a[ 1 ] * a[ 8 ] * a[ 10 ];
	tmp[ 554 ] += a[ 1 ] * a[ 8 ] * a[ 11 ];
	tmp[ 555 ] += a[ 1 ] * a[ 8 ] * a[ 14 ];
	tmp[ 556 ] += a[ 1 ] * a[ 8 ] * a[ 15 ];
	tmp[ 557 ] += a[ 1 ] * a[ 8 ] * a[ 16 ];
	tmp[ 558 ] += a[ 1 ] * a[ 8 ] * a[ 17 ];
	tmp[ 559 ] += a[ 1 ] * a[ 8 ] * a[ 18 ];
	tmp[ 560 ] += a[ 1 ] * a[ 8 ] * a[ 19 ];
	tmp[ 561 ] += a[ 1 ] * a[ 8 ] * a[ 20 ];
	tmp[ 562 ] += a[ 1 ] * a[ 8 ] * a[ 21 ];
	tmp[ 563 ] += a[ 1 ] * a[ 8 ] * a[ 22 ];
	tmp[ 564 ] += a[ 1 ] * a[ 8 ] * a[ 23 ];
	tmp[ 565 ] += a[ 1 ] * a[ 8 ] * a[ 24 ];
	tmp[ 566 ] += a[ 1 ] * a[ 9 ] * a[ 10 ];
	tmp[ 567 ] += a[ 1 ] * a[ 9 ] * a[ 11 ];
	tmp[ 568 ] += a[ 1 ] * a[ 9 ] * a[ 14 ];
	tmp[ 569 ] += a[ 1 ] * a[ 9 ] * a[ 15 ];
	tmp[ 570 ] += a[ 1 ] * a[ 9 ] * a[ 16 ];
	tmp[ 571 ] += a[ 1 ] * a[ 9 ] * a[ 17 ];
	tmp[ 572 ] += a[ 1 ] * a[ 9 ] * a[ 18 ];
	tmp[ 573 ] += a[ 1 ] * a[ 9 ] * a[ 19 ];
	tmp[ 574 ] += a[ 1 ] * a[ 9 ] * a[ 20 ];
	tmp[ 575 ] += a[ 1 ] * a[ 9 ] * a[ 21 ];
	tmp[ 576 ] += a[ 1 ] * a[ 9 ] * a[ 22 ];
	tmp[ 577 ] += a[ 1 ] * a[ 9 ] * a[ 23 ];
	tmp[ 578 ] += a[ 1 ] * a[ 9 ] * a[ 24 ];
	tmp[ 579 ] += a[ 1 ] * a[ 10 ] * a[ 11 ];
	tmp[ 580 ] += a[ 1 ] * a[ 10 ] * a[ 13 ];
	tmp[ 581 ] += a[ 1 ] * a[ 10 ] * a[ 14 ];
	tmp[ 582 ] += a[ 1 ] * a[ 10 ] * a[ 15 ];
	tmp[ 583 ] += a[ 1 ] * a[ 10 ] * a[ 16 ];
	tmp[ 584 ] += a[ 1 ] * a[ 10 ] * a[ 17 ];
	tmp[ 585 ] += a[ 1 ] * a[ 10 ] * a[ 18 ];
	tmp[ 586 ] += a[ 1 ] * a[ 10 ] * a[ 19 ];
	tmp[ 587 ] += a[ 1 ] * a[ 10 ] * a[ 20 ];
	tmp[ 588 ] += a[ 1 ] * a[ 10 ] * a[ 21 ];
	tmp[ 589 ] += a[ 1 ] * a[ 10 ] * a[ 22 ];
	tmp[ 590 ] += a[ 1 ] * a[ 10 ] * a[ 23 ];
	tmp[ 591 ] += a[ 1 ] * a[ 10 ] * a[ 24 ];
	tmp[ 592 ] += a[ 1 ] * a[ 11 ] * a[ 14 ];
	tmp[ 593 ] += a[ 1 ] * a[ 11 ] * a[ 15 ];
	tmp[ 594 ] += a[ 1 ] * a[ 11 ] * a[ 16 ];
	tmp[ 595 ] += a[ 1 ] * a[ 11 ] * a[ 17 ];
	tmp[ 596 ] += a[ 1 ] * a[ 11 ] * a[ 18 ];
	tmp[ 597 ] += a[ 1 ] * a[ 11 ] * a[ 19 ];
	tmp[ 598 ] += a[ 1 ] * a[ 11 ] * a[ 20 ];
	tmp[ 599 ] += a[ 1 ] * a[ 11 ] * a[ 21 ];
	tmp[ 600 ] += a[ 1 ] * a[ 11 ] * a[ 22 ];
	tmp[ 601 ] += a[ 1 ] * a[ 11 ] * a[ 23 ];
	tmp[ 602 ] += a[ 1 ] * a[ 11 ] * a[ 24 ];
	tmp[ 603 ] += a[ 1 ] * a[ 13 ] * a[ 15 ];
	tmp[ 604 ] += a[ 1 ] * a[ 13 ] * a[ 20 ];
	tmp[ 605 ] += a[ 1 ] * a[ 14 ] * a[ 15 ];
	tmp[ 606 ] += a[ 1 ] * a[ 14 ] * a[ 16 ];
	tmp[ 607 ] += a[ 1 ] * a[ 14 ] * a[ 17 ];
	tmp[ 608 ] += a[ 1 ] * a[ 14 ] * a[ 18 ];
	tmp[ 609 ] += a[ 1 ] * a[ 14 ] * a[ 19 ];
	tmp[ 610 ] += a[ 1 ] * a[ 14 ] * a[ 20 ];
	tmp[ 611 ] += a[ 1 ] * a[ 14 ] * a[ 21 ];
	tmp[ 612 ] += a[ 1 ] * a[ 14 ] * a[ 22 ];
	tmp[ 613 ] += a[ 1 ] * a[ 14 ] * a[ 23 ];
	tmp[ 614 ] += a[ 1 ] * a[ 14 ] * a[ 24 ];
	tmp[ 615 ] += a[ 1 ] * a[ 15 ] * a[ 16 ];
	tmp[ 616 ] += a[ 1 ] * a[ 15 ] * a[ 17 ];
	tmp[ 617 ] += a[ 1 ] * a[ 15 ] * a[ 18 ];
	tmp[ 618 ] += a[ 1 ] * a[ 15 ] * a[ 19 ];
	tmp[ 619 ] += a[ 1 ] * a[ 15 ] * a[ 20 ];
	tmp[ 620 ] += a[ 1 ] * a[ 15 ] * a[ 21 ];
	tmp[ 621 ] += a[ 1 ] * a[ 15 ] * a[ 22 ];
	tmp[ 622 ] += a[ 1 ] * a[ 15 ] * a[ 23 ];
	tmp[ 623 ] += a[ 1 ] * a[ 15 ] * a[ 24 ];
	tmp[ 624 ] += a[ 1 ] * a[ 16 ] * a[ 17 ];
	tmp[ 625 ] += a[ 1 ] * a[ 16 ] * a[ 18 ];
	tmp[ 626 ] += a[ 1 ] * a[ 16 ] * a[ 19 ];
	tmp[ 627 ] += a[ 1 ] * a[ 16 ] * a[ 20 ];
	tmp[ 628 ] += a[ 1 ] * a[ 16 ] * a[ 21 ];
	tmp[ 629 ] += a[ 1 ] * a[ 16 ] * a[ 22 ];
	tmp[ 630 ] += a[ 1 ] * a[ 16 ] * a[ 23 ];
	tmp[ 631 ] += a[ 1 ] * a[ 16 ] * a[ 24 ];
	tmp[ 632 ] += a[ 1 ] * a[ 17 ] * a[ 18 ];
	tmp[ 633 ] += a[ 1 ] * a[ 17 ] * a[ 19 ];
	tmp[ 634 ] += a[ 1 ] * a[ 17 ] * a[ 20 ];
	tmp[ 635 ] += a[ 1 ] * a[ 17 ] * a[ 21 ];
	tmp[ 636 ] += a[ 1 ] * a[ 17 ] * a[ 22 ];
	tmp[ 637 ] += a[ 1 ] * a[ 17 ] * a[ 23 ];
	tmp[ 638 ] += a[ 1 ] * a[ 17 ] * a[ 24 ];
	tmp[ 639 ] += a[ 1 ] * a[ 18 ] * a[ 19 ];
	tmp[ 640 ] += a[ 1 ] * a[ 18 ] * a[ 20 ];
	tmp[ 641 ] += a[ 1 ] * a[ 18 ] * a[ 21 ];
	tmp[ 642 ] += a[ 1 ] * a[ 18 ] * a[ 22 ];
	tmp[ 643 ] += a[ 1 ] * a[ 18 ] * a[ 23 ];
	tmp[ 644 ] += a[ 1 ] * a[ 18 ] * a[ 24 ];
	tmp[ 645 ] += a[ 1 ] * a[ 19 ] * a[ 20 ];
	tmp[ 646 ] += a[ 1 ] * a[ 19 ] * a[ 21 ];
	tmp[ 647 ] += a[ 1 ] * a[ 19 ] * a[ 22 ];
	tmp[ 648 ] += a[ 1 ] * a[ 19 ] * a[ 23 ];
	tmp[ 649 ] += a[ 1 ] * a[ 19 ] * a[ 24 ];
	tmp[ 650 ] += a[ 1 ] * a[ 20 ] * a[ 21 ];
	tmp[ 651 ] += a[ 1 ] * a[ 20 ] * a[ 22 ];
	tmp[ 652 ] += a[ 1 ] * a[ 20 ] * a[ 23 ];
	tmp[ 653 ] += a[ 1 ] * a[ 20 ] * a[ 24 ];
	tmp[ 654 ] += a[ 1 ] * a[ 21 ] * a[ 22 ];
	tmp[ 655 ] += a[ 1 ] * a[ 21 ] * a[ 23 ];
	tmp[ 656 ] += a[ 1 ] * a[ 21 ] * a[ 24 ];
	tmp[ 657 ] += a[ 1 ] * a[ 22 ] * a[ 23 ];
	tmp[ 658 ] += a[ 1 ] * a[ 22 ] * a[ 24 ];
	tmp[ 659 ] += a[ 1 ] * a[ 23 ] * a[ 24 ];
	tmp[ 660 ] += a[ 2 ] * a[ 3 ] * a[ 4 ];
	tmp[ 661 ] += a[ 2 ] * a[ 3 ] * a[ 5 ];
	tmp[ 662 ] += a[ 2 ] * a[ 3 ] * a[ 6 ];
	tmp[ 663 ] += a[ 2 ] * a[ 3 ] * a[ 7 ];
	tmp[ 664 ] += a[ 2 ] * a[ 3 ] * a[ 8 ];
	tmp[ 665 ] += a[ 2 ] * a[ 3 ] * a[ 9 ];
	tmp[ 666 ] += a[ 2 ] * a[ 3 ] * a[ 10 ];
	tmp[ 667 ] += a[ 2 ] * a[ 3 ] * a[ 11 ];
	tmp[ 668 ] += a[ 2 ] * a[ 3 ] * a[ 15 ];
	tmp[ 669 ] += a[ 2 ] * a[ 3 ] * a[ 16 ];
	tmp[ 670 ] += a[ 2 ] * a[ 3 ] * a[ 17 ];
	tmp[ 671 ] += a[ 2 ] * a[ 3 ] * a[ 18 ];
	tmp[ 672 ] += a[ 2 ] * a[ 3 ] * a[ 19 ];
	tmp[ 673 ] += a[ 2 ] * a[ 3 ] * a[ 20 ];
	tmp[ 674 ] += a[ 2 ] * a[ 3 ] * a[ 21 ];
	tmp[ 675 ] += a[ 2 ] * a[ 3 ] * a[ 22 ];
	tmp[ 676 ] += a[ 2 ] * a[ 3 ] * a[ 23 ];
	tmp[ 677 ] += a[ 2 ] * a[ 3 ] * a[ 24 ];
	tmp[ 678 ] += a[ 2 ] * a[ 4 ] * a[ 5 ];
	tmp[ 679 ] += a[ 2 ] * a[ 4 ] * a[ 6 ];
	tmp[ 680 ] += a[ 2 ] * a[ 4 ] * a[ 7 ];
	tmp[ 681 ] += a[ 2 ] * a[ 4 ] * a[ 8 ];
	tmp[ 682 ] += a[ 2 ] * a[ 4 ] * a[ 9 ];
	tmp[ 683 ] += a[ 2 ] * a[ 4 ] * a[ 10 ];
	tmp[ 684 ] += a[ 2 ] * a[ 4 ] * a[ 11 ];
	tmp[ 685 ] += a[ 2 ] * a[ 4 ] * a[ 15 ];
	tmp[ 686 ] += a[ 2 ] * a[ 4 ] * a[ 16 ];
	tmp[ 687 ] += a[ 2 ] * a[ 4 ] * a[ 17 ];
	tmp[ 688 ] += a[ 2 ] * a[ 4 ] * a[ 18 ];
	tmp[ 689 ] += a[ 2 ] * a[ 4 ] * a[ 19 ];
	tmp[ 690 ] += a[ 2 ] * a[ 4 ] * a[ 20 ];
	tmp[ 691 ] += a[ 2 ] * a[ 4 ] * a[ 21 ];
	tmp[ 692 ] += a[ 2 ] * a[ 4 ] * a[ 22 ];
	tmp[ 693 ] += a[ 2 ] * a[ 4 ] * a[ 23 ];
	tmp[ 694 ] += a[ 2 ] * a[ 4 ] * a[ 24 ];
	tmp[ 695 ] += a[ 2 ] * a[ 5 ] * a[ 6 ];
	tmp[ 696 ] += a[ 2 ] * a[ 5 ] * a[ 7 ];
	tmp[ 697 ] += a[ 2 ] * a[ 5 ] * a[ 8 ];
	tmp[ 698 ] += a[ 2 ] * a[ 5 ] * a[ 9 ];
	tmp[ 699 ] += a[ 2 ] * a[ 5 ] * a[ 10 ];
	tmp[ 700 ] += a[ 2 ] * a[ 5 ] * a[ 11 ];
	tmp[ 701 ] += a[ 2 ] * a[ 5 ] * a[ 13 ];
	tmp[ 702 ] += a[ 2 ] * a[ 5 ] * a[ 14 ];
	tmp[ 703 ] += a[ 2 ] * a[ 5 ] * a[ 15 ];
	tmp[ 704 ] += a[ 2 ] * a[ 5 ] * a[ 16 ];
	tmp[ 705 ] += a[ 2 ] * a[ 5 ] * a[ 17 ];
	tmp[ 706 ] += a[ 2 ] * a[ 5 ] * a[ 18 ];
	tmp[ 707 ] += a[ 2 ] * a[ 5 ] * a[ 19 ];
	tmp[ 708 ] += a[ 2 ] * a[ 5 ] * a[ 20 ];
	tmp[ 709 ] += a[ 2 ] * a[ 5 ] * a[ 21 ];
	tmp[ 710 ] += a[ 2 ] * a[ 5 ] * a[ 22 ];
	tmp[ 711 ] += a[ 2 ] * a[ 5 ] * a[ 23 ];
	tmp[ 712 ] += a[ 2 ] * a[ 5 ] * a[ 24 ];
	tmp[ 713 ] += a[ 2 ] * a[ 6 ] * a[ 7 ];
	tmp[ 714 ] += a[ 2 ] * a[ 6 ] * a[ 8 ];
	tmp[ 715 ] += a[ 2 ] * a[ 6 ] * a[ 9 ];
	tmp[ 716 ] += a[ 2 ] * a[ 6 ] * a[ 10 ];
	tmp[ 717 ] += a[ 2 ] * a[ 6 ] * a[ 11 ];
	tmp[ 718 ] += a[ 2 ] * a[ 6 ] * a[ 14 ];
	tmp[ 719 ] += a[ 2 ] * a[ 6 ] * a[ 15 ];
	tmp[ 720 ] += a[ 2 ] * a[ 6 ] * a[ 16 ];
	tmp[ 721 ] += a[ 2 ] * a[ 6 ] * a[ 17 ];
	tmp[ 722 ] += a[ 2 ] * a[ 6 ] * a[ 18 ];
	tmp[ 723 ] += a[ 2 ] * a[ 6 ] * a[ 19 ];
	tmp[ 724 ] += a[ 2 ] * a[ 6 ] * a[ 20 ];
	tmp[ 725 ] += a[ 2 ] * a[ 6 ] * a[ 21 ];
	tmp[ 726 ] += a[ 2 ] * a[ 6 ] * a[ 22 ];
	tmp[ 727 ] += a[ 2 ] * a[ 6 ] * a[ 23 ];
	tmp[ 728 ] += a[ 2 ] * a[ 6 ] * a[ 24 ];
	tmp[ 729 ] += a[ 2 ] * a[ 7 ] * a[ 8 ];
	tmp[ 730 ] += a[ 2 ] * a[ 7 ] * a[ 9 ];
	tmp[ 731 ] += a[ 2 ] * a[ 7 ] * a[ 10 ];
	tmp[ 732 ] += a[ 2 ] * a[ 7 ] * a[ 11 ];
	tmp[ 733 ] += a[ 2 ] * a[ 7 ] * a[ 15 ];
	tmp[ 734 ] += a[ 2 ] * a[ 7 ] * a[ 16 ];
	tmp[ 735 ] += a[ 2 ] * a[ 7 ] * a[ 17 ];
	tmp[ 736 ] += a[ 2 ] * a[ 7 ] * a[ 18 ];
	tmp[ 737 ] += a[ 2 ] * a[ 7 ] * a[ 19 ];
	tmp[ 738 ] += a[ 2 ] * a[ 7 ] * a[ 20 ];
	tmp[ 739 ] += a[ 2 ] * a[ 7 ] * a[ 21 ];
	tmp[ 740 ] += a[ 2 ] * a[ 7 ] * a[ 22 ];
	tmp[ 741 ] += a[ 2 ] * a[ 7 ] * a[ 23 ];
	tmp[ 742 ] += a[ 2 ] * a[ 7 ] * a[ 24 ];
	tmp[ 743 ] += a[ 2 ] * a[ 8 ] * a[ 9 ];
	tmp[ 744 ] += a[ 2 ] * a[ 8 ] * a[ 10 ];
	tmp[ 745 ] += a[ 2 ] * a[ 8 ] * a[ 11 ];
	tmp[ 746 ] += a[ 2 ] * a[ 8 ] * a[ 15 ];
	tmp[ 747 ] += a[ 2 ] * a[ 8 ] * a[ 16 ];
	tmp[ 748 ] += a[ 2 ] * a[ 8 ] * a[ 17 ];
	tmp[ 749 ] += a[ 2 ] * a[ 8 ] * a[ 18 ];
	tmp[ 750 ] += a[ 2 ] * a[ 8 ] * a[ 19 ];
	tmp[ 751 ] += a[ 2 ] * a[ 8 ] * a[ 20 ];
	tmp[ 752 ] += a[ 2 ] * a[ 8 ] * a[ 21 ];
	tmp[ 753 ] += a[ 2 ] * a[ 8 ] * a[ 22 ];
	tmp[ 754 ] += a[ 2 ] * a[ 8 ] * a[ 23 ];
	tmp[ 755 ] += a[ 2 ] * a[ 8 ] * a[ 24 ];
	tmp[ 756 ] += a[ 2 ] * a[ 9 ] * a[ 10 ];
	tmp[ 757 ] += a[ 2 ] * a[ 9 ] * a[ 11 ];
	tmp[ 758 ] += a[ 2 ] * a[ 9 ] * a[ 15 ];
	tmp[ 759 ] += a[ 2 ] * a[ 9 ] * a[ 16 ];
	tmp[ 760 ] += a[ 2 ] * a[ 9 ] * a[ 17 ];
	tmp[ 761 ] += a[ 2 ] * a[ 9 ] * a[ 18 ];
	tmp[ 762 ] += a[ 2 ] * a[ 9 ] * a[ 19 ];
	tmp[ 763 ] += a[ 2 ] * a[ 9 ] * a[ 20 ];
	tmp[ 764 ] += a[ 2 ] * a[ 9 ] * a[ 21 ];
	tmp[ 765 ] += a[ 2 ] * a[ 9 ] * a[ 22 ];
	tmp[ 766 ] += a[ 2 ] * a[ 9 ] * a[ 23 ];
	tmp[ 767 ] += a[ 2 ] * a[ 9 ] * a[ 24 ];
	tmp[ 768 ] += a[ 2 ] * a[ 10 ] * a[ 11 ];
	tmp[ 769 ] += a[ 2 ] * a[ 10 ] * a[ 13 ];
	tmp[ 770 ] += a[ 2 ] * a[ 10 ] * a[ 14 ];
	tmp[ 771 ] += a[ 2 ] * a[ 10 ] * a[ 15 ];
	tmp[ 772 ] += a[ 2 ] * a[ 10 ] * a[ 16 ];
	tmp[ 773 ] += a[ 2 ] * a[ 10 ] * a[ 17 ];
	tmp[ 774 ] += a[ 2 ] * a[ 10 ] * a[ 18 ];
	tmp[ 775 ] += a[ 2 ] * a[ 10 ] * a[ 19 ];
	tmp[ 776 ] += a[ 2 ] * a[ 10 ] * a[ 20 ];
	tmp[ 777 ] += a[ 2 ] * a[ 10 ] * a[ 21 ];
	tmp[ 778 ] += a[ 2 ] * a[ 10 ] * a[ 22 ];
	tmp[ 779 ] += a[ 2 ] * a[ 10 ] * a[ 23 ];
	tmp[ 780 ] += a[ 2 ] * a[ 10 ] * a[ 24 ];
	tmp[ 781 ] += a[ 2 ] * a[ 11 ] * a[ 14 ];
	tmp[ 782 ] += a[ 2 ] * a[ 11 ] * a[ 15 ];
	tmp[ 783 ] += a[ 2 ] * a[ 11 ] * a[ 16 ];
	tmp[ 784 ] += a[ 2 ] * a[ 11 ] * a[ 17 ];
	tmp[ 785 ] += a[ 2 ] * a[ 11 ] * a[ 18 ];
	tmp[ 786 ] += a[ 2 ] * a[ 11 ] * a[ 19 ];
	tmp[ 787 ] += a[ 2 ] * a[ 11 ] * a[ 20 ];
	tmp[ 788 ] += a[ 2 ] * a[ 11 ] * a[ 21 ];
	tmp[ 789 ] += a[ 2 ] * a[ 11 ] * a[ 22 ];
	tmp[ 790 ] += a[ 2 ] * a[ 11 ] * a[ 23 ];
	tmp[ 791 ] += a[ 2 ] * a[ 11 ] * a[ 24 ];
	tmp[ 792 ] += a[ 2 ] * a[ 13 ] * a[ 15 ];
	tmp[ 793 ] += a[ 2 ] * a[ 13 ] * a[ 20 ];
	tmp[ 794 ] += a[ 2 ] * a[ 14 ] * a[ 15 ];
	tmp[ 795 ] += a[ 2 ] * a[ 14 ] * a[ 16 ];
	tmp[ 796 ] += a[ 2 ] * a[ 14 ] * a[ 20 ];
	tmp[ 797 ] += a[ 2 ] * a[ 14 ] * a[ 21 ];
	tmp[ 798 ] += a[ 2 ] * a[ 15 ] * a[ 16 ];
	tmp[ 799 ] += a[ 2 ] * a[ 15 ] * a[ 17 ];
	tmp[ 800 ] += a[ 2 ] * a[ 15 ] * a[ 18 ];
	tmp[ 801 ] += a[ 2 ] * a[ 15 ] * a[ 19 ];
	tmp[ 802 ] += a[ 2 ] * a[ 15 ] * a[ 20 ];
	tmp[ 803 ] += a[ 2 ] * a[ 15 ] * a[ 21 ];
	tmp[ 804 ] += a[ 2 ] * a[ 15 ] * a[ 22 ];
	tmp[ 805 ] += a[ 2 ] * a[ 15 ] * a[ 23 ];
	tmp[ 806 ] += a[ 2 ] * a[ 15 ] * a[ 24 ];
	tmp[ 807 ] += a[ 2 ] * a[ 16 ] * a[ 17 ];
	tmp[ 808 ] += a[ 2 ] * a[ 16 ] * a[ 18 ];
	tmp[ 809 ] += a[ 2 ] * a[ 16 ] * a[ 19 ];
	tmp[ 810 ] += a[ 2 ] * a[ 16 ] * a[ 20 ];
	tmp[ 811 ] += a[ 2 ] * a[ 16 ] * a[ 21 ];
	tmp[ 812 ] += a[ 2 ] * a[ 16 ] * a[ 22 ];
	tmp[ 813 ] += a[ 2 ] * a[ 16 ] * a[ 23 ];
	tmp[ 814 ] += a[ 2 ] * a[ 16 ] * a[ 24 ];
	tmp[ 815 ] += a[ 2 ] * a[ 17 ] * a[ 18 ];
	tmp[ 816 ] += a[ 2 ] * a[ 17 ] * a[ 19 ];
	tmp[ 817 ] += a[ 2 ] * a[ 17 ] * a[ 20 ];
	tmp[ 818 ] += a[ 2 ] * a[ 17 ] * a[ 21 ];
	tmp[ 819 ] += a[ 2 ] * a[ 17 ] * a[ 22 ];
	tmp[ 820 ] += a[ 2 ] * a[ 17 ] * a[ 23 ];
	tmp[ 821 ] += a[ 2 ] * a[ 17 ] * a[ 24 ];
	tmp[ 822 ] += a[ 2 ] * a[ 18 ] * a[ 19 ];
	tmp[ 823 ] += a[ 2 ] * a[ 18 ] * a[ 20 ];
	tmp[ 824 ] += a[ 2 ] * a[ 18 ] * a[ 21 ];
	tmp[ 825 ] += a[ 2 ] * a[ 18 ] * a[ 22 ];
	tmp[ 826 ] += a[ 2 ] * a[ 18 ] * a[ 23 ];
	tmp[ 827 ] += a[ 2 ] * a[ 18 ] * a[ 24 ];
	tmp[ 828 ] += a[ 2 ] * a[ 19 ] * a[ 20 ];
	tmp[ 829 ] += a[ 2 ] * a[ 19 ] * a[ 21 ];
	tmp[ 830 ] += a[ 2 ] * a[ 19 ] * a[ 22 ];
	tmp[ 831 ] += a[ 2 ] * a[ 19 ] * a[ 23 ];
	tmp[ 832 ] += a[ 2 ] * a[ 19 ] * a[ 24 ];
	tmp[ 833 ] += a[ 2 ] * a[ 20 ] * a[ 21 ];
	tmp[ 834 ] += a[ 2 ] * a[ 20 ] * a[ 22 ];
	tmp[ 835 ] += a[ 2 ] * a[ 20 ] * a[ 23 ];
	tmp[ 836 ] += a[ 2 ] * a[ 20 ] * a[ 24 ];
	tmp[ 837 ] += a[ 2 ] * a[ 21 ] * a[ 22 ];
	tmp[ 838 ] += a[ 2 ] * a[ 21 ] * a[ 23 ];
	tmp[ 839 ] += a[ 2 ] * a[ 21 ] * a[ 24 ];
	tmp[ 840 ] += a[ 2 ] * a[ 22 ] * a[ 23 ];
	tmp[ 841 ] += a[ 2 ] * a[ 22 ] * a[ 24 ];
	tmp[ 842 ] += a[ 2 ] * a[ 23 ] * a[ 24 ];
	tmp[ 843 ] += a[ 3 ] * a[ 4 ] * a[ 5 ];
	tmp[ 844 ] += a[ 3 ] * a[ 4 ] * a[ 6 ];
	tmp[ 845 ] += a[ 3 ] * a[ 4 ] * a[ 7 ];
	tmp[ 846 ] += a[ 3 ] * a[ 4 ] * a[ 8 ];
	tmp[ 847 ] += a[ 3 ] * a[ 4 ] * a[ 9 ];
	tmp[ 848 ] += a[ 3 ] * a[ 4 ] * a[ 10 ];
	tmp[ 849 ] += a[ 3 ] * a[ 4 ] * a[ 11 ];
	tmp[ 850 ] += a[ 3 ] * a[ 4 ] * a[ 15 ];
	tmp[ 851 ] += a[ 3 ] * a[ 4 ] * a[ 16 ];
	tmp[ 852 ] += a[ 3 ] * a[ 4 ] * a[ 17 ];
	tmp[ 853 ] += a[ 3 ] * a[ 4 ] * a[ 18 ];
	tmp[ 854 ] += a[ 3 ] * a[ 4 ] * a[ 19 ];
	tmp[ 855 ] += a[ 3 ] * a[ 4 ] * a[ 20 ];
	tmp[ 856 ] += a[ 3 ] * a[ 4 ] * a[ 21 ];
	tmp[ 857 ] += a[ 3 ] * a[ 4 ] * a[ 22 ];
	tmp[ 858 ] += a[ 3 ] * a[ 4 ] * a[ 23 ];
	tmp[ 859 ] += a[ 3 ] * a[ 4 ] * a[ 24 ];
	tmp[ 860 ] += a[ 3 ] * a[ 5 ] * a[ 6 ];
	tmp[ 861 ] += a[ 3 ] * a[ 5 ] * a[ 7 ];
	tmp[ 862 ] += a[ 3 ] * a[ 5 ] * a[ 8 ];
	tmp[ 863 ] += a[ 3 ] * a[ 5 ] * a[ 9 ];
	tmp[ 864 ] += a[ 3 ] * a[ 5 ] * a[ 10 ];
	tmp[ 865 ] += a[ 3 ] * a[ 5 ] * a[ 11 ];
	tmp[ 866 ] += a[ 3 ] * a[ 5 ] * a[ 13 ];
	tmp[ 867 ] += a[ 3 ] * a[ 5 ] * a[ 14 ];
	tmp[ 868 ] += a[ 3 ] * a[ 5 ] * a[ 15 ];
	tmp[ 869 ] += a[ 3 ] * a[ 5 ] * a[ 16 ];
	tmp[ 870 ] += a[ 3 ] * a[ 5 ] * a[ 17 ];
	tmp[ 871 ] += a[ 3 ] * a[ 5 ] * a[ 18 ];
	tmp[ 872 ] += a[ 3 ] * a[ 5 ] * a[ 19 ];
	tmp[ 873 ] += a[ 3 ] * a[ 5 ] * a[ 20 ];
	tmp[ 874 ] += a[ 3 ] * a[ 5 ] * a[ 21 ];
	tmp[ 875 ] += a[ 3 ] * a[ 5 ] * a[ 22 ];
	tmp[ 876 ] += a[ 3 ] * a[ 5 ] * a[ 23 ];
	tmp[ 877 ] += a[ 3 ] * a[ 5 ] * a[ 24 ];
	tmp[ 878 ] += a[ 3 ] * a[ 6 ] * a[ 7 ];
	tmp[ 879 ] += a[ 3 ] * a[ 6 ] * a[ 8 ];
	tmp[ 880 ] += a[ 3 ] * a[ 6 ] * a[ 9 ];
	tmp[ 881 ] += a[ 3 ] * a[ 6 ] * a[ 10 ];
	tmp[ 882 ] += a[ 3 ] * a[ 6 ] * a[ 11 ];
	tmp[ 883 ] += a[ 3 ] * a[ 6 ] * a[ 14 ];
	tmp[ 884 ] += a[ 3 ] * a[ 6 ] * a[ 15 ];
	tmp[ 885 ] += a[ 3 ] * a[ 6 ] * a[ 16 ];
	tmp[ 886 ] += a[ 3 ] * a[ 6 ] * a[ 17 ];
	tmp[ 887 ] += a[ 3 ] * a[ 6 ] * a[ 18 ];
	tmp[ 888 ] += a[ 3 ] * a[ 6 ] * a[ 19 ];
	tmp[ 889 ] += a[ 3 ] * a[ 6 ] * a[ 20 ];
	tmp[ 890 ] += a[ 3 ] * a[ 6 ] * a[ 21 ];
	tmp[ 891 ] += a[ 3 ] * a[ 6 ] * a[ 22 ];
	tmp[ 892 ] += a[ 3 ] * a[ 6 ] * a[ 23 ];
	tmp[ 893 ] += a[ 3 ] * a[ 6 ] * a[ 24 ];
	tmp[ 894 ] += a[ 3 ] * a[ 7 ] * a[ 8 ];
	tmp[ 895 ] += a[ 3 ] * a[ 7 ] * a[ 9 ];
	tmp[ 896 ] += a[ 3 ] * a[ 7 ] * a[ 10 ];
	tmp[ 897 ] += a[ 3 ] * a[ 7 ] * a[ 11 ];
	tmp[ 898 ] += a[ 3 ] * a[ 7 ] * a[ 15 ];
	tmp[ 899 ] += a[ 3 ] * a[ 7 ] * a[ 16 ];
	tmp[ 900 ] += a[ 3 ] * a[ 7 ] * a[ 17 ];
	tmp[ 901 ] += a[ 3 ] * a[ 7 ] * a[ 18 ];
	tmp[ 902 ] += a[ 3 ] * a[ 7 ] * a[ 19 ];
	tmp[ 903 ] += a[ 3 ] * a[ 7 ] * a[ 20 ];
	tmp[ 904 ] += a[ 3 ] * a[ 7 ] * a[ 21 ];
	tmp[ 905 ] += a[ 3 ] * a[ 7 ] * a[ 22 ];
	tmp[ 906 ] += a[ 3 ] * a[ 7 ] * a[ 23 ];
	tmp[ 907 ] += a[ 3 ] * a[ 7 ] * a[ 24 ];
	tmp[ 908 ] += a[ 3 ] * a[ 8 ] * a[ 9 ];
	tmp[ 909 ] += a[ 3 ] * a[ 8 ] * a[ 10 ];
	tmp[ 910 ] += a[ 3 ] * a[ 8 ] * a[ 11 ];
	tmp[ 911 ] += a[ 3 ] * a[ 8 ] * a[ 15 ];
	tmp[ 912 ] += a[ 3 ] * a[ 8 ] * a[ 16 ];
	tmp[ 913 ] += a[ 3 ] * a[ 8 ] * a[ 17 ];
	tmp[ 914 ] += a[ 3 ] * a[ 8 ] * a[ 18 ];
	tmp[ 915 ] += a[ 3 ] * a[ 8 ] * a[ 19 ];
	tmp[ 916 ] += a[ 3 ] * a[ 8 ] * a[ 20 ];
	tmp[ 917 ] += a[ 3 ] * a[ 8 ] * a[ 21 ];
	tmp[ 918 ] += a[ 3 ] * a[ 8 ] * a[ 22 ];
	tmp[ 919 ] += a[ 3 ] * a[ 8 ] * a[ 23 ];
	tmp[ 920 ] += a[ 3 ] * a[ 8 ] * a[ 24 ];
	tmp[ 921 ] += a[ 3 ] * a[ 9 ] * a[ 10 ];
	tmp[ 922 ] += a[ 3 ] * a[ 9 ] * a[ 11 ];
	tmp[ 923 ] += a[ 3 ] * a[ 9 ] * a[ 15 ];
	tmp[ 924 ] += a[ 3 ] * a[ 9 ] * a[ 16 ];
	tmp[ 925 ] += a[ 3 ] * a[ 9 ] * a[ 17 ];
	tmp[ 926 ] += a[ 3 ] * a[ 9 ] * a[ 18 ];
	tmp[ 927 ] += a[ 3 ] * a[ 9 ] * a[ 19 ];
	tmp[ 928 ] += a[ 3 ] * a[ 9 ] * a[ 20 ];
	tmp[ 929 ] += a[ 3 ] * a[ 9 ] * a[ 21 ];
	tmp[ 930 ] += a[ 3 ] * a[ 9 ] * a[ 22 ];
	tmp[ 931 ] += a[ 3 ] * a[ 9 ] * a[ 23 ];
	tmp[ 932 ] += a[ 3 ] * a[ 9 ] * a[ 24 ];
	tmp[ 933 ] += a[ 3 ] * a[ 10 ] * a[ 11 ];
	tmp[ 934 ] += a[ 3 ] * a[ 10 ] * a[ 13 ];
	tmp[ 935 ] += a[ 3 ] * a[ 10 ] * a[ 14 ];
	tmp[ 936 ] += a[ 3 ] * a[ 10 ] * a[ 15 ];
	tmp[ 937 ] += a[ 3 ] * a[ 10 ] * a[ 16 ];
	tmp[ 938 ] += a[ 3 ] * a[ 10 ] * a[ 17 ];
	tmp[ 939 ] += a[ 3 ] * a[ 10 ] * a[ 18 ];
	tmp[ 940 ] += a[ 3 ] * a[ 10 ] * a[ 19 ];
	tmp[ 941 ] += a[ 3 ] * a[ 10 ] * a[ 20 ];
	tmp[ 942 ] += a[ 3 ] * a[ 10 ] * a[ 21 ];
	tmp[ 943 ] += a[ 3 ] * a[ 10 ] * a[ 22 ];
	tmp[ 944 ] += a[ 3 ] * a[ 10 ] * a[ 23 ];
	tmp[ 945 ] += a[ 3 ] * a[ 10 ] * a[ 24 ];
	tmp[ 946 ] += a[ 3 ] * a[ 11 ] * a[ 14 ];
	tmp[ 947 ] += a[ 3 ] * a[ 11 ] * a[ 15 ];
	tmp[ 948 ] += a[ 3 ] * a[ 11 ] * a[ 16 ];
	tmp[ 949 ] += a[ 3 ] * a[ 11 ] * a[ 17 ];
	tmp[ 950 ] += a[ 3 ] * a[ 11 ] * a[ 18 ];
	tmp[ 951 ] += a[ 3 ] * a[ 11 ] * a[ 19 ];
	tmp[ 952 ] += a[ 3 ] * a[ 11 ] * a[ 20 ];
	tmp[ 953 ] += a[ 3 ] * a[ 11 ] * a[ 21 ];
	tmp[ 954 ] += a[ 3 ] * a[ 11 ] * a[ 22 ];
	tmp[ 955 ] += a[ 3 ] * a[ 11 ] * a[ 23 ];
	tmp[ 956 ] += a[ 3 ] * a[ 11 ] * a[ 24 ];
	tmp[ 957 ] += a[ 3 ] * a[ 13 ] * a[ 15 ];
	tmp[ 958 ] += a[ 3 ] * a[ 13 ] * a[ 20 ];
	tmp[ 959 ] += a[ 3 ] * a[ 14 ] * a[ 15 ];
	tmp[ 960 ] += a[ 3 ] * a[ 14 ] * a[ 16 ];
	tmp[ 961 ] += a[ 3 ] * a[ 14 ] * a[ 20 ];
	tmp[ 962 ] += a[ 3 ] * a[ 14 ] * a[ 21 ];
	tmp[ 963 ] += a[ 3 ] * a[ 15 ] * a[ 16 ];
	tmp[ 964 ] += a[ 3 ] * a[ 15 ] * a[ 17 ];
	tmp[ 965 ] += a[ 3 ] * a[ 15 ] * a[ 18 ];
	tmp[ 966 ] += a[ 3 ] * a[ 15 ] * a[ 19 ];
	tmp[ 967 ] += a[ 3 ] * a[ 15 ] * a[ 20 ];
	tmp[ 968 ] += a[ 3 ] * a[ 15 ] * a[ 21 ];
	tmp[ 969 ] += a[ 3 ] * a[ 15 ] * a[ 22 ];
	tmp[ 970 ] += a[ 3 ] * a[ 15 ] * a[ 23 ];
	tmp[ 971 ] += a[ 3 ] * a[ 15 ] * a[ 24 ];
	tmp[ 972 ] += a[ 3 ] * a[ 16 ] * a[ 17 ];
	tmp[ 973 ] += a[ 3 ] * a[ 16 ] * a[ 18 ];
	tmp[ 974 ] += a[ 3 ] * a[ 16 ] * a[ 19 ];
	tmp[ 975 ] += a[ 3 ] * a[ 16 ] * a[ 20 ];
	tmp[ 976 ] += a[ 3 ] * a[ 16 ] * a[ 21 ];
	tmp[ 977 ] += a[ 3 ] * a[ 16 ] * a[ 22 ];
	tmp[ 978 ] += a[ 3 ] * a[ 16 ] * a[ 23 ];
	tmp[ 979 ] += a[ 3 ] * a[ 16 ] * a[ 24 ];
	tmp[ 980 ] += a[ 3 ] * a[ 17 ] * a[ 18 ];
	tmp[ 981 ] += a[ 3 ] * a[ 17 ] * a[ 19 ];
	tmp[ 982 ] += a[ 3 ] * a[ 17 ] * a[ 20 ];
	tmp[ 983 ] += a[ 3 ] * a[ 17 ] * a[ 21 ];
	tmp[ 984 ] += a[ 3 ] * a[ 17 ] * a[ 22 ];
	tmp[ 985 ] += a[ 3 ] * a[ 17 ] * a[ 23 ];
	tmp[ 986 ] += a[ 3 ] * a[ 17 ] * a[ 24 ];
	tmp[ 987 ] += a[ 3 ] * a[ 18 ] * a[ 19 ];
	tmp[ 988 ] += a[ 3 ] * a[ 18 ] * a[ 20 ];
	tmp[ 989 ] += a[ 3 ] * a[ 18 ] * a[ 21 ];
	tmp[ 990 ] += a[ 3 ] * a[ 18 ] * a[ 22 ];
	tmp[ 991 ] += a[ 3 ] * a[ 18 ] * a[ 23 ];
	tmp[ 992 ] += a[ 3 ] * a[ 18 ] * a[ 24 ];
	tmp[ 993 ] += a[ 3 ] * a[ 19 ] * a[ 20 ];
	tmp[ 994 ] += a[ 3 ] * a[ 19 ] * a[ 21 ];
	tmp[ 995 ] += a[ 3 ] * a[ 19 ] * a[ 22 ];
	tmp[ 996 ] += a[ 3 ] * a[ 19 ] * a[ 23 ];
	tmp[ 997 ] += a[ 3 ] * a[ 19 ] * a[ 24 ];
	tmp[ 998 ] += a[ 3 ] * a[ 20 ] * a[ 21 ];
	tmp[ 999 ] += a[ 3 ] * a[ 20 ] * a[ 22 ];
	tmp[ 1000 ] += a[ 3 ] * a[ 20 ] * a[ 23 ];
	tmp[ 1001 ] += a[ 3 ] * a[ 20 ] * a[ 24 ];
	tmp[ 1002 ] += a[ 3 ] * a[ 21 ] * a[ 22 ];
	tmp[ 1003 ] += a[ 3 ] * a[ 21 ] * a[ 23 ];
	tmp[ 1004 ] += a[ 3 ] * a[ 21 ] * a[ 24 ];
	tmp[ 1005 ] += a[ 3 ] * a[ 22 ] * a[ 23 ];
	tmp[ 1006 ] += a[ 3 ] * a[ 22 ] * a[ 24 ];
	tmp[ 1007 ] += a[ 3 ] * a[ 23 ] * a[ 24 ];
	tmp[ 1008 ] += a[ 4 ] * a[ 5 ] * a[ 6 ];
	tmp[ 1009 ] += a[ 4 ] * a[ 5 ] * a[ 7 ];
	tmp[ 1010 ] += a[ 4 ] * a[ 5 ] * a[ 8 ];
	tmp[ 1011 ] += a[ 4 ] * a[ 5 ] * a[ 9 ];
	tmp[ 1012 ] += a[ 4 ] * a[ 5 ] * a[ 10 ];
	tmp[ 1013 ] += a[ 4 ] * a[ 5 ] * a[ 11 ];
	tmp[ 1014 ] += a[ 4 ] * a[ 5 ] * a[ 13 ];
	tmp[ 1015 ] += a[ 4 ] * a[ 5 ] * a[ 14 ];
	tmp[ 1016 ] += a[ 4 ] * a[ 5 ] * a[ 15 ];
	tmp[ 1017 ] += a[ 4 ] * a[ 5 ] * a[ 16 ];
	tmp[ 1018 ] += a[ 4 ] * a[ 5 ] * a[ 17 ];
	tmp[ 1019 ] += a[ 4 ] * a[ 5 ] * a[ 18 ];
	tmp[ 1020 ] += a[ 4 ] * a[ 5 ] * a[ 19 ];
	tmp[ 1021 ] += a[ 4 ] * a[ 5 ] * a[ 20 ];
	tmp[ 1022 ] += a[ 4 ] * a[ 5 ] * a[ 21 ];
	tmp[ 1023 ] += a[ 4 ] * a[ 5 ] * a[ 22 ];
	tmp[ 1024 ] += a[ 4 ] * a[ 5 ] * a[ 23 ];
	tmp[ 1025 ] += a[ 4 ] * a[ 5 ] * a[ 24 ];
	tmp[ 1026 ] += a[ 4 ] * a[ 6 ] * a[ 7 ];
	tmp[ 1027 ] += a[ 4 ] * a[ 6 ] * a[ 8 ];
	tmp[ 1028 ] += a[ 4 ] * a[ 6 ] * a[ 9 ];
	tmp[ 1029 ] += a[ 4 ] * a[ 6 ] * a[ 10 ];
	tmp[ 1030 ] += a[ 4 ] * a[ 6 ] * a[ 11 ];
	tmp[ 1031 ] += a[ 4 ] * a[ 6 ] * a[ 14 ];
	tmp[ 1032 ] += a[ 4 ] * a[ 6 ] * a[ 15 ];
	tmp[ 1033 ] += a[ 4 ] * a[ 6 ] * a[ 16 ];
	tmp[ 1034 ] += a[ 4 ] * a[ 6 ] * a[ 17 ];
	tmp[ 1035 ] += a[ 4 ] * a[ 6 ] * a[ 18 ];
	tmp[ 1036 ] += a[ 4 ] * a[ 6 ] * a[ 19 ];
	tmp[ 1037 ] += a[ 4 ] * a[ 6 ] * a[ 20 ];
	tmp[ 1038 ] += a[ 4 ] * a[ 6 ] * a[ 21 ];
	tmp[ 1039 ] += a[ 4 ] * a[ 6 ] * a[ 22 ];
	tmp[ 1040 ] += a[ 4 ] * a[ 6 ] * a[ 23 ];
	tmp[ 1041 ] += a[ 4 ] * a[ 6 ] * a[ 24 ];
	tmp[ 1042 ] += a[ 4 ] * a[ 7 ] * a[ 8 ];
	tmp[ 1043 ] += a[ 4 ] * a[ 7 ] * a[ 9 ];
	tmp[ 1044 ] += a[ 4 ] * a[ 7 ] * a[ 10 ];
	tmp[ 1045 ] += a[ 4 ] * a[ 7 ] * a[ 11 ];
	tmp[ 1046 ] += a[ 4 ] * a[ 7 ] * a[ 15 ];
	tmp[ 1047 ] += a[ 4 ] * a[ 7 ] * a[ 16 ];
	tmp[ 1048 ] += a[ 4 ] * a[ 7 ] * a[ 17 ];
	tmp[ 1049 ] += a[ 4 ] * a[ 7 ] * a[ 18 ];
	tmp[ 1050 ] += a[ 4 ] * a[ 7 ] * a[ 19 ];
	tmp[ 1051 ] += a[ 4 ] * a[ 7 ] * a[ 20 ];
	tmp[ 1052 ] += a[ 4 ] * a[ 7 ] * a[ 21 ];
	tmp[ 1053 ] += a[ 4 ] * a[ 7 ] * a[ 22 ];
	tmp[ 1054 ] += a[ 4 ] * a[ 7 ] * a[ 23 ];
	tmp[ 1055 ] += a[ 4 ] * a[ 7 ] * a[ 24 ];
	tmp[ 1056 ] += a[ 4 ] * a[ 8 ] * a[ 9 ];
	tmp[ 1057 ] += a[ 4 ] * a[ 8 ] * a[ 10 ];
	tmp[ 1058 ] += a[ 4 ] * a[ 8 ] * a[ 11 ];
	tmp[ 1059 ] += a[ 4 ] * a[ 8 ] * a[ 15 ];
	tmp[ 1060 ] += a[ 4 ] * a[ 8 ] * a[ 16 ];
	tmp[ 1061 ] += a[ 4 ] * a[ 8 ] * a[ 17 ];
	tmp[ 1062 ] += a[ 4 ] * a[ 8 ] * a[ 18 ];
	tmp[ 1063 ] += a[ 4 ] * a[ 8 ] * a[ 19 ];
	tmp[ 1064 ] += a[ 4 ] * a[ 8 ] * a[ 20 ];
	tmp[ 1065 ] += a[ 4 ] * a[ 8 ] * a[ 21 ];
	tmp[ 1066 ] += a[ 4 ] * a[ 8 ] * a[ 22 ];
	tmp[ 1067 ] += a[ 4 ] * a[ 8 ] * a[ 23 ];
	tmp[ 1068 ] += a[ 4 ] * a[ 8 ] * a[ 24 ];
	tmp[ 1069 ] += a[ 4 ] * a[ 9 ] * a[ 10 ];
	tmp[ 1070 ] += a[ 4 ] * a[ 9 ] * a[ 11 ];
	tmp[ 1071 ] += a[ 4 ] * a[ 9 ] * a[ 15 ];
	tmp[ 1072 ] += a[ 4 ] * a[ 9 ] * a[ 16 ];
	tmp[ 1073 ] += a[ 4 ] * a[ 9 ] * a[ 17 ];
	tmp[ 1074 ] += a[ 4 ] * a[ 9 ] * a[ 18 ];
	tmp[ 1075 ] += a[ 4 ] * a[ 9 ] * a[ 19 ];
	tmp[ 1076 ] += a[ 4 ] * a[ 9 ] * a[ 20 ];
	tmp[ 1077 ] += a[ 4 ] * a[ 9 ] * a[ 21 ];
	tmp[ 1078 ] += a[ 4 ] * a[ 9 ] * a[ 22 ];
	tmp[ 1079 ] += a[ 4 ] * a[ 9 ] * a[ 23 ];
	tmp[ 1080 ] += a[ 4 ] * a[ 9 ] * a[ 24 ];
	tmp[ 1081 ] += a[ 4 ] * a[ 10 ] * a[ 11 ];
	tmp[ 1082 ] += a[ 4 ] * a[ 10 ] * a[ 13 ];
	tmp[ 1083 ] += a[ 4 ] * a[ 10 ] * a[ 14 ];
	tmp[ 1084 ] += a[ 4 ] * a[ 10 ] * a[ 15 ];
	tmp[ 1085 ] += a[ 4 ] * a[ 10 ] * a[ 16 ];
	tmp[ 1086 ] += a[ 4 ] * a[ 10 ] * a[ 17 ];
	tmp[ 1087 ] += a[ 4 ] * a[ 10 ] * a[ 18 ];
	tmp[ 1088 ] += a[ 4 ] * a[ 10 ] * a[ 19 ];
	tmp[ 1089 ] += a[ 4 ] * a[ 10 ] * a[ 20 ];
	tmp[ 1090 ] += a[ 4 ] * a[ 10 ] * a[ 21 ];
	tmp[ 1091 ] += a[ 4 ] * a[ 10 ] * a[ 22 ];
	tmp[ 1092 ] += a[ 4 ] * a[ 10 ] * a[ 23 ];
	tmp[ 1093 ] += a[ 4 ] * a[ 10 ] * a[ 24 ];
	tmp[ 1094 ] += a[ 4 ] * a[ 11 ] * a[ 14 ];
	tmp[ 1095 ] += a[ 4 ] * a[ 11 ] * a[ 15 ];
	tmp[ 1096 ] += a[ 4 ] * a[ 11 ] * a[ 16 ];
	tmp[ 1097 ] += a[ 4 ] * a[ 11 ] * a[ 17 ];
	tmp[ 1098 ] += a[ 4 ] * a[ 11 ] * a[ 18 ];
	tmp[ 1099 ] += a[ 4 ] * a[ 11 ] * a[ 19 ];
	tmp[ 1100 ] += a[ 4 ] * a[ 11 ] * a[ 20 ];
	tmp[ 1101 ] += a[ 4 ] * a[ 11 ] * a[ 21 ];
	tmp[ 1102 ] += a[ 4 ] * a[ 11 ] * a[ 22 ];
	tmp[ 1103 ] += a[ 4 ] * a[ 11 ] * a[ 23 ];
	tmp[ 1104 ] += a[ 4 ] * a[ 11 ] * a[ 24 ];
	tmp[ 1105 ] += a[ 4 ] * a[ 13 ] * a[ 15 ];
	tmp[ 1106 ] += a[ 4 ] * a[ 13 ] * a[ 20 ];
	tmp[ 1107 ] += a[ 4 ] * a[ 14 ] * a[ 15 ];
	tmp[ 1108 ] += a[ 4 ] * a[ 14 ] * a[ 16 ];
	tmp[ 1109 ] += a[ 4 ] * a[ 14 ] * a[ 20 ];
	tmp[ 1110 ] += a[ 4 ] * a[ 14 ] * a[ 21 ];
	tmp[ 1111 ] += a[ 4 ] * a[ 15 ] * a[ 16 ];
	tmp[ 1112 ] += a[ 4 ] * a[ 15 ] * a[ 17 ];
	tmp[ 1113 ] += a[ 4 ] * a[ 15 ] * a[ 18 ];
	tmp[ 1114 ] += a[ 4 ] * a[ 15 ] * a[ 19 ];
	tmp[ 1115 ] += a[ 4 ] * a[ 15 ] * a[ 20 ];
	tmp[ 1116 ] += a[ 4 ] * a[ 15 ] * a[ 21 ];
	tmp[ 1117 ] += a[ 4 ] * a[ 15 ] * a[ 22 ];
	tmp[ 1118 ] += a[ 4 ] * a[ 15 ] * a[ 23 ];
	tmp[ 1119 ] += a[ 4 ] * a[ 15 ] * a[ 24 ];
	tmp[ 1120 ] += a[ 4 ] * a[ 16 ] * a[ 17 ];
	tmp[ 1121 ] += a[ 4 ] * a[ 16 ] * a[ 18 ];
	tmp[ 1122 ] += a[ 4 ] * a[ 16 ] * a[ 19 ];
	tmp[ 1123 ] += a[ 4 ] * a[ 16 ] * a[ 20 ];
	tmp[ 1124 ] += a[ 4 ] * a[ 16 ] * a[ 21 ];
	tmp[ 1125 ] += a[ 4 ] * a[ 16 ] * a[ 22 ];
	tmp[ 1126 ] += a[ 4 ] * a[ 16 ] * a[ 23 ];
	tmp[ 1127 ] += a[ 4 ] * a[ 16 ] * a[ 24 ];
	tmp[ 1128 ] += a[ 4 ] * a[ 17 ] * a[ 18 ];
	tmp[ 1129 ] += a[ 4 ] * a[ 17 ] * a[ 19 ];
	tmp[ 1130 ] += a[ 4 ] * a[ 17 ] * a[ 20 ];
	tmp[ 1131 ] += a[ 4 ] * a[ 17 ] * a[ 21 ];
	tmp[ 1132 ] += a[ 4 ] * a[ 17 ] * a[ 22 ];
	tmp[ 1133 ] += a[ 4 ] * a[ 17 ] * a[ 23 ];
	tmp[ 1134 ] += a[ 4 ] * a[ 17 ] * a[ 24 ];
	tmp[ 1135 ] += a[ 4 ] * a[ 18 ] * a[ 19 ];
	tmp[ 1136 ] += a[ 4 ] * a[ 18 ] * a[ 20 ];
	tmp[ 1137 ] += a[ 4 ] * a[ 18 ] * a[ 21 ];
	tmp[ 1138 ] += a[ 4 ] * a[ 18 ] * a[ 22 ];
	tmp[ 1139 ] += a[ 4 ] * a[ 18 ] * a[ 23 ];
	tmp[ 1140 ] += a[ 4 ] * a[ 18 ] * a[ 24 ];
	tmp[ 1141 ] += a[ 4 ] * a[ 19 ] * a[ 20 ];
	tmp[ 1142 ] += a[ 4 ] * a[ 19 ] * a[ 21 ];
	tmp[ 1143 ] += a[ 4 ] * a[ 19 ] * a[ 22 ];
	tmp[ 1144 ] += a[ 4 ] * a[ 19 ] * a[ 23 ];
	tmp[ 1145 ] += a[ 4 ] * a[ 19 ] * a[ 24 ];
	tmp[ 1146 ] += a[ 4 ] * a[ 20 ] * a[ 21 ];
	tmp[ 1147 ] += a[ 4 ] * a[ 20 ] * a[ 22 ];
	tmp[ 1148 ] += a[ 4 ] * a[ 20 ] * a[ 23 ];
	tmp[ 1149 ] += a[ 4 ] * a[ 20 ] * a[ 24 ];
	tmp[ 1150 ] += a[ 4 ] * a[ 21 ] * a[ 22 ];
	tmp[ 1151 ] += a[ 4 ] * a[ 21 ] * a[ 23 ];
	tmp[ 1152 ] += a[ 4 ] * a[ 21 ] * a[ 24 ];
	tmp[ 1153 ] += a[ 4 ] * a[ 22 ] * a[ 23 ];
	tmp[ 1154 ] += a[ 4 ] * a[ 22 ] * a[ 24 ];
	tmp[ 1155 ] += a[ 4 ] * a[ 23 ] * a[ 24 ];
	tmp[ 1156 ] += a[ 5 ] * a[ 6 ] * a[ 7 ];
	tmp[ 1157 ] += a[ 5 ] * a[ 6 ] * a[ 8 ];
	tmp[ 1158 ] += a[ 5 ] * a[ 6 ] * a[ 9 ];
	tmp[ 1159 ] += a[ 5 ] * a[ 6 ] * a[ 10 ];
	tmp[ 1160 ] += a[ 5 ] * a[ 6 ] * a[ 11 ];
	tmp[ 1161 ] += a[ 5 ] * a[ 6 ] * a[ 13 ];
	tmp[ 1162 ] += a[ 5 ] * a[ 6 ] * a[ 14 ];
	tmp[ 1163 ] += a[ 5 ] * a[ 6 ] * a[ 18 ];
	tmp[ 1164 ] += a[ 5 ] * a[ 6 ] * a[ 19 ];
	tmp[ 1165 ] += a[ 5 ] * a[ 6 ] * a[ 20 ];
	tmp[ 1166 ] += a[ 5 ] * a[ 6 ] * a[ 21 ];
	tmp[ 1167 ] += a[ 5 ] * a[ 6 ] * a[ 22 ];
	tmp[ 1168 ] += a[ 5 ] * a[ 6 ] * a[ 23 ];
	tmp[ 1169 ] += a[ 5 ] * a[ 6 ] * a[ 24 ];
	tmp[ 1170 ] += a[ 5 ] * a[ 7 ] * a[ 8 ];
	tmp[ 1171 ] += a[ 5 ] * a[ 7 ] * a[ 9 ];
	tmp[ 1172 ] += a[ 5 ] * a[ 7 ] * a[ 10 ];
	tmp[ 1173 ] += a[ 5 ] * a[ 7 ] * a[ 11 ];
	tmp[ 1174 ] += a[ 5 ] * a[ 7 ] * a[ 13 ];
	tmp[ 1175 ] += a[ 5 ] * a[ 7 ] * a[ 14 ];
	tmp[ 1176 ] += a[ 5 ] * a[ 7 ] * a[ 18 ];
	tmp[ 1177 ] += a[ 5 ] * a[ 7 ] * a[ 19 ];
	tmp[ 1178 ] += a[ 5 ] * a[ 7 ] * a[ 20 ];
	tmp[ 1179 ] += a[ 5 ] * a[ 7 ] * a[ 21 ];
	tmp[ 1180 ] += a[ 5 ] * a[ 7 ] * a[ 22 ];
	tmp[ 1181 ] += a[ 5 ] * a[ 7 ] * a[ 23 ];
	tmp[ 1182 ] += a[ 5 ] * a[ 7 ] * a[ 24 ];
	tmp[ 1183 ] += a[ 5 ] * a[ 8 ] * a[ 9 ];
	tmp[ 1184 ] += a[ 5 ] * a[ 8 ] * a[ 10 ];
	tmp[ 1185 ] += a[ 5 ] * a[ 8 ] * a[ 11 ];
	tmp[ 1186 ] += a[ 5 ] * a[ 8 ] * a[ 13 ];
	tmp[ 1187 ] += a[ 5 ] * a[ 8 ] * a[ 14 ];
	tmp[ 1188 ] += a[ 5 ] * a[ 8 ] * a[ 15 ];
	tmp[ 1189 ] += a[ 5 ] * a[ 8 ] * a[ 18 ];
	tmp[ 1190 ] += a[ 5 ] * a[ 8 ] * a[ 19 ];
	tmp[ 1191 ] += a[ 5 ] * a[ 8 ] * a[ 20 ];
	tmp[ 1192 ] += a[ 5 ] * a[ 8 ] * a[ 21 ];
	tmp[ 1193 ] += a[ 5 ] * a[ 8 ] * a[ 22 ];
	tmp[ 1194 ] += a[ 5 ] * a[ 8 ] * a[ 23 ];
	tmp[ 1195 ] += a[ 5 ] * a[ 8 ] * a[ 24 ];
	tmp[ 1196 ] += a[ 5 ] * a[ 9 ] * a[ 10 ];
	tmp[ 1197 ] += a[ 5 ] * a[ 9 ] * a[ 11 ];
	tmp[ 1198 ] += a[ 5 ] * a[ 9 ] * a[ 13 ];
	tmp[ 1199 ] += a[ 5 ] * a[ 9 ] * a[ 14 ];
	tmp[ 1200 ] += a[ 5 ] * a[ 9 ] * a[ 15 ];
	tmp[ 1201 ] += a[ 5 ] * a[ 9 ] * a[ 16 ];
	tmp[ 1202 ] += a[ 5 ] * a[ 9 ] * a[ 18 ];
	tmp[ 1203 ] += a[ 5 ] * a[ 9 ] * a[ 19 ];
	tmp[ 1204 ] += a[ 5 ] * a[ 9 ] * a[ 20 ];
	tmp[ 1205 ] += a[ 5 ] * a[ 9 ] * a[ 21 ];
	tmp[ 1206 ] += a[ 5 ] * a[ 9 ] * a[ 22 ];
	tmp[ 1207 ] += a[ 5 ] * a[ 9 ] * a[ 23 ];
	tmp[ 1208 ] += a[ 5 ] * a[ 9 ] * a[ 24 ];
	tmp[ 1209 ] += a[ 5 ] * a[ 10 ] * a[ 11 ];
	tmp[ 1210 ] += a[ 5 ] * a[ 10 ] * a[ 13 ];
	tmp[ 1211 ] += a[ 5 ] * a[ 10 ] * a[ 14 ];
	tmp[ 1212 ] += a[ 5 ] * a[ 10 ] * a[ 18 ];
	tmp[ 1213 ] += a[ 5 ] * a[ 10 ] * a[ 19 ];
	tmp[ 1214 ] += a[ 5 ] * a[ 10 ] * a[ 20 ];
	tmp[ 1215 ] += a[ 5 ] * a[ 10 ] * a[ 21 ];
	tmp[ 1216 ] += a[ 5 ] * a[ 10 ] * a[ 22 ];
	tmp[ 1217 ] += a[ 5 ] * a[ 10 ] * a[ 23 ];
	tmp[ 1218 ] += a[ 5 ] * a[ 10 ] * a[ 24 ];
	tmp[ 1219 ] += a[ 5 ] * a[ 11 ] * a[ 13 ];
	tmp[ 1220 ] += a[ 5 ] * a[ 11 ] * a[ 14 ];
	tmp[ 1221 ] += a[ 5 ] * a[ 11 ] * a[ 18 ];
	tmp[ 1222 ] += a[ 5 ] * a[ 11 ] * a[ 19 ];
	tmp[ 1223 ] += a[ 5 ] * a[ 11 ] * a[ 20 ];
	tmp[ 1224 ] += a[ 5 ] * a[ 11 ] * a[ 21 ];
	tmp[ 1225 ] += a[ 5 ] * a[ 11 ] * a[ 22 ];
	tmp[ 1226 ] += a[ 5 ] * a[ 11 ] * a[ 23 ];
	tmp[ 1227 ] += a[ 5 ] * a[ 11 ] * a[ 24 ];
	tmp[ 1228 ] += a[ 5 ] * a[ 13 ] * a[ 14 ];
	tmp[ 1229 ] += a[ 5 ] * a[ 13 ] * a[ 15 ];
	tmp[ 1230 ] += a[ 5 ] * a[ 13 ] * a[ 18 ];
	tmp[ 1231 ] += a[ 5 ] * a[ 13 ] * a[ 19 ];
	tmp[ 1232 ] += a[ 5 ] * a[ 13 ] * a[ 20 ];
	tmp[ 1233 ] += a[ 5 ] * a[ 13 ] * a[ 21 ];
	tmp[ 1234 ] += a[ 5 ] * a[ 13 ] * a[ 22 ];
	tmp[ 1235 ] += a[ 5 ] * a[ 13 ] * a[ 23 ];
	tmp[ 1236 ] += a[ 5 ] * a[ 13 ] * a[ 24 ];
	tmp[ 1237 ] += a[ 5 ] * a[ 14 ] * a[ 15 ];
	tmp[ 1238 ] += a[ 5 ] * a[ 14 ] * a[ 16 ];
	tmp[ 1239 ] += a[ 5 ] * a[ 14 ] * a[ 18 ];
	tmp[ 1240 ] += a[ 5 ] * a[ 14 ] * a[ 19 ];
	tmp[ 1241 ] += a[ 5 ] * a[ 14 ] * a[ 20 ];
	tmp[ 1242 ] += a[ 5 ] * a[ 14 ] * a[ 21 ];
	tmp[ 1243 ] += a[ 5 ] * a[ 14 ] * a[ 22 ];
	tmp[ 1244 ] += a[ 5 ] * a[ 14 ] * a[ 23 ];
	tmp[ 1245 ] += a[ 5 ] * a[ 14 ] * a[ 24 ];
	tmp[ 1246 ] += a[ 5 ] * a[ 15 ] * a[ 18 ];
	tmp[ 1247 ] += a[ 5 ] * a[ 15 ] * a[ 19 ];
	tmp[ 1248 ] += a[ 5 ] * a[ 15 ] * a[ 23 ];
	tmp[ 1249 ] += a[ 5 ] * a[ 15 ] * a[ 24 ];
	tmp[ 1250 ] += a[ 5 ] * a[ 16 ] * a[ 19 ];
	tmp[ 1251 ] += a[ 5 ] * a[ 16 ] * a[ 24 ];
	tmp[ 1252 ] += a[ 5 ] * a[ 18 ] * a[ 19 ];
	tmp[ 1253 ] += a[ 5 ] * a[ 18 ] * a[ 20 ];
	tmp[ 1254 ] += a[ 5 ] * a[ 18 ] * a[ 21 ];
	tmp[ 1255 ] += a[ 5 ] * a[ 18 ] * a[ 22 ];
	tmp[ 1256 ] += a[ 5 ] * a[ 18 ] * a[ 23 ];
	tmp[ 1257 ] += a[ 5 ] * a[ 18 ] * a[ 24 ];
	tmp[ 1258 ] += a[ 5 ] * a[ 19 ] * a[ 20 ];
	tmp[ 1259 ] += a[ 5 ] * a[ 19 ] * a[ 21 ];
	tmp[ 1260 ] += a[ 5 ] * a[ 19 ] * a[ 22 ];
	tmp[ 1261 ] += a[ 5 ] * a[ 19 ] * a[ 23 ];
	tmp[ 1262 ] += a[ 5 ] * a[ 19 ] * a[ 24 ];
	tmp[ 1263 ] += a[ 5 ] * a[ 20 ] * a[ 21 ];
	tmp[ 1264 ] += a[ 5 ] * a[ 20 ] * a[ 22 ];
	tmp[ 1265 ] += a[ 5 ] * a[ 20 ] * a[ 23 ];
	tmp[ 1266 ] += a[ 5 ] * a[ 20 ] * a[ 24 ];
	tmp[ 1267 ] += a[ 5 ] * a[ 21 ] * a[ 22 ];
	tmp[ 1268 ] += a[ 5 ] * a[ 21 ] * a[ 23 ];
	tmp[ 1269 ] += a[ 5 ] * a[ 21 ] * a[ 24 ];
	tmp[ 1270 ] += a[ 5 ] * a[ 22 ] * a[ 23 ];
	tmp[ 1271 ] += a[ 5 ] * a[ 22 ] * a[ 24 ];
	tmp[ 1272 ] += a[ 5 ] * a[ 23 ] * a[ 24 ];
	tmp[ 1273 ] += a[ 6 ] * a[ 7 ] * a[ 8 ];
	tmp[ 1274 ] += a[ 6 ] * a[ 7 ] * a[ 9 ];
	tmp[ 1275 ] += a[ 6 ] * a[ 7 ] * a[ 10 ];
	tmp[ 1276 ] += a[ 6 ] * a[ 7 ] * a[ 11 ];
	tmp[ 1277 ] += a[ 6 ] * a[ 7 ] * a[ 14 ];
	tmp[ 1278 ] += a[ 6 ] * a[ 7 ] * a[ 19 ];
	tmp[ 1279 ] += a[ 6 ] * a[ 7 ] * a[ 20 ];
	tmp[ 1280 ] += a[ 6 ] * a[ 7 ] * a[ 21 ];
	tmp[ 1281 ] += a[ 6 ] * a[ 7 ] * a[ 22 ];
	tmp[ 1282 ] += a[ 6 ] * a[ 7 ] * a[ 23 ];
	tmp[ 1283 ] += a[ 6 ] * a[ 7 ] * a[ 24 ];
	tmp[ 1284 ] += a[ 6 ] * a[ 8 ] * a[ 9 ];
	tmp[ 1285 ] += a[ 6 ] * a[ 8 ] * a[ 10 ];
	tmp[ 1286 ] += a[ 6 ] * a[ 8 ] * a[ 11 ];
	tmp[ 1287 ] += a[ 6 ] * a[ 8 ] * a[ 14 ];
	tmp[ 1288 ] += a[ 6 ] * a[ 8 ] * a[ 15 ];
	tmp[ 1289 ] += a[ 6 ] * a[ 8 ] * a[ 19 ];
	tmp[ 1290 ] += a[ 6 ] * a[ 8 ] * a[ 20 ];
	tmp[ 1291 ] += a[ 6 ] * a[ 8 ] * a[ 21 ];
	tmp[ 1292 ] += a[ 6 ] * a[ 8 ] * a[ 22 ];
	tmp[ 1293 ] += a[ 6 ] * a[ 8 ] * a[ 23 ];
	tmp[ 1294 ] += a[ 6 ] * a[ 8 ] * a[ 24 ];
	tmp[ 1295 ] += a[ 6 ] * a[ 9 ] * a[ 10 ];
	tmp[ 1296 ] += a[ 6 ] * a[ 9 ] * a[ 11 ];
	tmp[ 1297 ] += a[ 6 ] * a[ 9 ] * a[ 14 ];
	tmp[ 1298 ] += a[ 6 ] * a[ 9 ] * a[ 15 ];
	tmp[ 1299 ] += a[ 6 ] * a[ 9 ] * a[ 16 ];
	tmp[ 1300 ] += a[ 6 ] * a[ 9 ] * a[ 19 ];
	tmp[ 1301 ] += a[ 6 ] * a[ 9 ] * a[ 20 ];
	tmp[ 1302 ] += a[ 6 ] * a[ 9 ] * a[ 21 ];
	tmp[ 1303 ] += a[ 6 ] * a[ 9 ] * a[ 22 ];
	tmp[ 1304 ] += a[ 6 ] * a[ 9 ] * a[ 23 ];
	tmp[ 1305 ] += a[ 6 ] * a[ 9 ] * a[ 24 ];
	tmp[ 1306 ] += a[ 6 ] * a[ 10 ] * a[ 11 ];
	tmp[ 1307 ] += a[ 6 ] * a[ 10 ] * a[ 13 ];
	tmp[ 1308 ] += a[ 6 ] * a[ 10 ] * a[ 14 ];
	tmp[ 1309 ] += a[ 6 ] * a[ 10 ] * a[ 18 ];
	tmp[ 1310 ] += a[ 6 ] * a[ 10 ] * a[ 19 ];
	tmp[ 1311 ] += a[ 6 ] * a[ 10 ] * a[ 20 ];
	tmp[ 1312 ] += a[ 6 ] * a[ 10 ] * a[ 21 ];
	tmp[ 1313 ] += a[ 6 ] * a[ 10 ] * a[ 22 ];
	tmp[ 1314 ] += a[ 6 ] * a[ 10 ] * a[ 23 ];
	tmp[ 1315 ] += a[ 6 ] * a[ 10 ] * a[ 24 ];
	tmp[ 1316 ] += a[ 6 ] * a[ 11 ] * a[ 14 ];
	tmp[ 1317 ] += a[ 6 ] * a[ 11 ] * a[ 19 ];
	tmp[ 1318 ] += a[ 6 ] * a[ 11 ] * a[ 20 ];
	tmp[ 1319 ] += a[ 6 ] * a[ 11 ] * a[ 21 ];
	tmp[ 1320 ] += a[ 6 ] * a[ 11 ] * a[ 22 ];
	tmp[ 1321 ] += a[ 6 ] * a[ 11 ] * a[ 23 ];
	tmp[ 1322 ] += a[ 6 ] * a[ 11 ] * a[ 24 ];
	tmp[ 1323 ] += a[ 6 ] * a[ 13 ] * a[ 15 ];
	tmp[ 1324 ] += a[ 6 ] * a[ 13 ] * a[ 20 ];
	tmp[ 1325 ] += a[ 6 ] * a[ 14 ] * a[ 15 ];
	tmp[ 1326 ] += a[ 6 ] * a[ 14 ] * a[ 16 ];
	tmp[ 1327 ] += a[ 6 ] * a[ 14 ] * a[ 19 ];
	tmp[ 1328 ] += a[ 6 ] * a[ 14 ] * a[ 20 ];
	tmp[ 1329 ] += a[ 6 ] * a[ 14 ] * a[ 21 ];
	tmp[ 1330 ] += a[ 6 ] * a[ 14 ] * a[ 22 ];
	tmp[ 1331 ] += a[ 6 ] * a[ 14 ] * a[ 23 ];
	tmp[ 1332 ] += a[ 6 ] * a[ 14 ] * a[ 24 ];
	tmp[ 1333 ] += a[ 6 ] * a[ 15 ] * a[ 18 ];
	tmp[ 1334 ] += a[ 6 ] * a[ 15 ] * a[ 19 ];
	tmp[ 1335 ] += a[ 6 ] * a[ 15 ] * a[ 23 ];
	tmp[ 1336 ] += a[ 6 ] * a[ 15 ] * a[ 24 ];
	tmp[ 1337 ] += a[ 6 ] * a[ 16 ] * a[ 19 ];
	tmp[ 1338 ] += a[ 6 ] * a[ 16 ] * a[ 24 ];
	tmp[ 1339 ] += a[ 6 ] * a[ 18 ] * a[ 20 ];
	tmp[ 1340 ] += a[ 6 ] * a[ 19 ] * a[ 20 ];
	tmp[ 1341 ] += a[ 6 ] * a[ 19 ] * a[ 21 ];
	tmp[ 1342 ] += a[ 6 ] * a[ 19 ] * a[ 22 ];
	tmp[ 1343 ] += a[ 6 ] * a[ 19 ] * a[ 23 ];
	tmp[ 1344 ] += a[ 6 ] * a[ 19 ] * a[ 24 ];
	tmp[ 1345 ] += a[ 6 ] * a[ 20 ] * a[ 21 ];
	tmp[ 1346 ] += a[ 6 ] * a[ 20 ] * a[ 22 ];
	tmp[ 1347 ] += a[ 6 ] * a[ 20 ] * a[ 23 ];
	tmp[ 1348 ] += a[ 6 ] * a[ 20 ] * a[ 24 ];
	tmp[ 1349 ] += a[ 6 ] * a[ 21 ] * a[ 22 ];
	tmp[ 1350 ] += a[ 6 ] * a[ 21 ] * a[ 23 ];
	tmp[ 1351 ] += a[ 6 ] * a[ 21 ] * a[ 24 ];
	tmp[ 1352 ] += a[ 6 ] * a[ 22 ] * a[ 23 ];
	tmp[ 1353 ] += a[ 6 ] * a[ 22 ] * a[ 24 ];
	tmp[ 1354 ] += a[ 6 ] * a[ 23 ] * a[ 24 ];
	tmp[ 1355 ] += a[ 7 ] * a[ 8 ] * a[ 9 ];
	tmp[ 1356 ] += a[ 7 ] * a[ 8 ] * a[ 10 ];
	tmp[ 1357 ] += a[ 7 ] * a[ 8 ] * a[ 11 ];
	tmp[ 1358 ] += a[ 7 ] * a[ 8 ] * a[ 15 ];
	tmp[ 1359 ] += a[ 7 ] * a[ 8 ] * a[ 20 ];
	tmp[ 1360 ] += a[ 7 ] * a[ 8 ] * a[ 21 ];
	tmp[ 1361 ] += a[ 7 ] * a[ 8 ] * a[ 22 ];
	tmp[ 1362 ] += a[ 7 ] * a[ 8 ] * a[ 23 ];
	tmp[ 1363 ] += a[ 7 ] * a[ 8 ] * a[ 24 ];
	tmp[ 1364 ] += a[ 7 ] * a[ 9 ] * a[ 10 ];
	tmp[ 1365 ] += a[ 7 ] * a[ 9 ] * a[ 11 ];
	tmp[ 1366 ] += a[ 7 ] * a[ 9 ] * a[ 15 ];
	tmp[ 1367 ] += a[ 7 ] * a[ 9 ] * a[ 16 ];
	tmp[ 1368 ] += a[ 7 ] * a[ 9 ] * a[ 20 ];
	tmp[ 1369 ] += a[ 7 ] * a[ 9 ] * a[ 21 ];
	tmp[ 1370 ] += a[ 7 ] * a[ 9 ] * a[ 22 ];
	tmp[ 1371 ] += a[ 7 ] * a[ 9 ] * a[ 23 ];
	tmp[ 1372 ] += a[ 7 ] * a[ 9 ] * a[ 24 ];
	tmp[ 1373 ] += a[ 7 ] * a[ 10 ] * a[ 11 ];
	tmp[ 1374 ] += a[ 7 ] * a[ 10 ] * a[ 13 ];
	tmp[ 1375 ] += a[ 7 ] * a[ 10 ] * a[ 14 ];
	tmp[ 1376 ] += a[ 7 ] * a[ 10 ] * a[ 18 ];
	tmp[ 1377 ] += a[ 7 ] * a[ 10 ] * a[ 19 ];
	tmp[ 1378 ] += a[ 7 ] * a[ 10 ] * a[ 20 ];
	tmp[ 1379 ] += a[ 7 ] * a[ 10 ] * a[ 21 ];
	tmp[ 1380 ] += a[ 7 ] * a[ 10 ] * a[ 22 ];
	tmp[ 1381 ] += a[ 7 ] * a[ 10 ] * a[ 23 ];
	tmp[ 1382 ] += a[ 7 ] * a[ 10 ] * a[ 24 ];
	tmp[ 1383 ] += a[ 7 ] * a[ 11 ] * a[ 14 ];
	tmp[ 1384 ] += a[ 7 ] * a[ 11 ] * a[ 19 ];
	tmp[ 1385 ] += a[ 7 ] * a[ 11 ] * a[ 20 ];
	tmp[ 1386 ] += a[ 7 ] * a[ 11 ] * a[ 21 ];
	tmp[ 1387 ] += a[ 7 ] * a[ 11 ] * a[ 22 ];
	tmp[ 1388 ] += a[ 7 ] * a[ 11 ] * a[ 23 ];
	tmp[ 1389 ] += a[ 7 ] * a[ 11 ] * a[ 24 ];
	tmp[ 1390 ] += a[ 7 ] * a[ 13 ] * a[ 15 ];
	tmp[ 1391 ] += a[ 7 ] * a[ 13 ] * a[ 20 ];
	tmp[ 1392 ] += a[ 7 ] * a[ 14 ] * a[ 15 ];
	tmp[ 1393 ] += a[ 7 ] * a[ 14 ] * a[ 16 ];
	tmp[ 1394 ] += a[ 7 ] * a[ 14 ] * a[ 20 ];
	tmp[ 1395 ] += a[ 7 ] * a[ 14 ] * a[ 21 ];
	tmp[ 1396 ] += a[ 7 ] * a[ 15 ] * a[ 18 ];
	tmp[ 1397 ] += a[ 7 ] * a[ 15 ] * a[ 19 ];
	tmp[ 1398 ] += a[ 7 ] * a[ 15 ] * a[ 23 ];
	tmp[ 1399 ] += a[ 7 ] * a[ 15 ] * a[ 24 ];
	tmp[ 1400 ] += a[ 7 ] * a[ 16 ] * a[ 19 ];
	tmp[ 1401 ] += a[ 7 ] * a[ 16 ] * a[ 24 ];
	tmp[ 1402 ] += a[ 7 ] * a[ 18 ] * a[ 20 ];
	tmp[ 1403 ] += a[ 7 ] * a[ 19 ] * a[ 20 ];
	tmp[ 1404 ] += a[ 7 ] * a[ 19 ] * a[ 21 ];
	tmp[ 1405 ] += a[ 7 ] * a[ 20 ] * a[ 21 ];
	tmp[ 1406 ] += a[ 7 ] * a[ 20 ] * a[ 22 ];
	tmp[ 1407 ] += a[ 7 ] * a[ 20 ] * a[ 23 ];
	tmp[ 1408 ] += a[ 7 ] * a[ 20 ] * a[ 24 ];
	tmp[ 1409 ] += a[ 7 ] * a[ 21 ] * a[ 22 ];
	tmp[ 1410 ] += a[ 7 ] * a[ 21 ] * a[ 23 ];
	tmp[ 1411 ] += a[ 7 ] * a[ 21 ] * a[ 24 ];
	tmp[ 1412 ] += a[ 7 ] * a[ 22 ] * a[ 23 ];
	tmp[ 1413 ] += a[ 7 ] * a[ 22 ] * a[ 24 ];
	tmp[ 1414 ] += a[ 7 ] * a[ 23 ] * a[ 24 ];
	tmp[ 1415 ] += a[ 8 ] * a[ 9 ] * a[ 10 ];
	tmp[ 1416 ] += a[ 8 ] * a[ 9 ] * a[ 11 ];
	tmp[ 1417 ] += a[ 8 ] * a[ 9 ] * a[ 15 ];
	tmp[ 1418 ] += a[ 8 ] * a[ 9 ] * a[ 16 ];
	tmp[ 1419 ] += a[ 8 ] * a[ 9 ] * a[ 20 ];
	tmp[ 1420 ] += a[ 8 ] * a[ 9 ] * a[ 21 ];
	tmp[ 1421 ] += a[ 8 ] * a[ 9 ] * a[ 22 ];
	tmp[ 1422 ] += a[ 8 ] * a[ 9 ] * a[ 23 ];
	tmp[ 1423 ] += a[ 8 ] * a[ 9 ] * a[ 24 ];
	tmp[ 1424 ] += a[ 8 ] * a[ 10 ] * a[ 11 ];
	tmp[ 1425 ] += a[ 8 ] * a[ 10 ] * a[ 13 ];
	tmp[ 1426 ] += a[ 8 ] * a[ 10 ] * a[ 14 ];
	tmp[ 1427 ] += a[ 8 ] * a[ 10 ] * a[ 15 ];
	tmp[ 1428 ] += a[ 8 ] * a[ 10 ] * a[ 18 ];
	tmp[ 1429 ] += a[ 8 ] * a[ 10 ] * a[ 19 ];
	tmp[ 1430 ] += a[ 8 ] * a[ 10 ] * a[ 20 ];
	tmp[ 1431 ] += a[ 8 ] * a[ 10 ] * a[ 21 ];
	tmp[ 1432 ] += a[ 8 ] * a[ 10 ] * a[ 22 ];
	tmp[ 1433 ] += a[ 8 ] * a[ 10 ] * a[ 23 ];
	tmp[ 1434 ] += a[ 8 ] * a[ 10 ] * a[ 24 ];
	tmp[ 1435 ] += a[ 8 ] * a[ 11 ] * a[ 14 ];
	tmp[ 1436 ] += a[ 8 ] * a[ 11 ] * a[ 15 ];
	tmp[ 1437 ] += a[ 8 ] * a[ 11 ] * a[ 19 ];
	tmp[ 1438 ] += a[ 8 ] * a[ 11 ] * a[ 20 ];
	tmp[ 1439 ] += a[ 8 ] * a[ 11 ] * a[ 21 ];
	tmp[ 1440 ] += a[ 8 ] * a[ 11 ] * a[ 22 ];
	tmp[ 1441 ] += a[ 8 ] * a[ 11 ] * a[ 23 ];
	tmp[ 1442 ] += a[ 8 ] * a[ 11 ] * a[ 24 ];
	tmp[ 1443 ] += a[ 8 ] * a[ 13 ] * a[ 15 ];
	tmp[ 1444 ] += a[ 8 ] * a[ 13 ] * a[ 20 ];
	tmp[ 1445 ] += a[ 8 ] * a[ 14 ] * a[ 15 ];
	tmp[ 1446 ] += a[ 8 ] * a[ 14 ] * a[ 16 ];
	tmp[ 1447 ] += a[ 8 ] * a[ 14 ] * a[ 20 ];
	tmp[ 1448 ] += a[ 8 ] * a[ 14 ] * a[ 21 ];
	tmp[ 1449 ] += a[ 8 ] * a[ 15 ] * a[ 18 ];
	tmp[ 1450 ] += a[ 8 ] * a[ 15 ] * a[ 19 ];
	tmp[ 1451 ] += a[ 8 ] * a[ 15 ] * a[ 20 ];
	tmp[ 1452 ] += a[ 8 ] * a[ 15 ] * a[ 21 ];
	tmp[ 1453 ] += a[ 8 ] * a[ 15 ] * a[ 22 ];
	tmp[ 1454 ] += a[ 8 ] * a[ 15 ] * a[ 23 ];
	tmp[ 1455 ] += a[ 8 ] * a[ 15 ] * a[ 24 ];
	tmp[ 1456 ] += a[ 8 ] * a[ 16 ] * a[ 19 ];
	tmp[ 1457 ] += a[ 8 ] * a[ 16 ] * a[ 24 ];
	tmp[ 1458 ] += a[ 8 ] * a[ 18 ] * a[ 20 ];
	tmp[ 1459 ] += a[ 8 ] * a[ 19 ] * a[ 20 ];
	tmp[ 1460 ] += a[ 8 ] * a[ 19 ] * a[ 21 ];
	tmp[ 1461 ] += a[ 8 ] * a[ 20 ] * a[ 21 ];
	tmp[ 1462 ] += a[ 8 ] * a[ 20 ] * a[ 22 ];
	tmp[ 1463 ] += a[ 8 ] * a[ 20 ] * a[ 23 ];
	tmp[ 1464 ] += a[ 8 ] * a[ 20 ] * a[ 24 ];
	tmp[ 1465 ] += a[ 8 ] * a[ 21 ] * a[ 22 ];
	tmp[ 1466 ] += a[ 8 ] * a[ 21 ] * a[ 23 ];
	tmp[ 1467 ] += a[ 8 ] * a[ 21 ] * a[ 24 ];
	tmp[ 1468 ] += a[ 8 ] * a[ 22 ] * a[ 23 ];
	tmp[ 1469 ] += a[ 8 ] * a[ 22 ] * a[ 24 ];
	tmp[ 1470 ] += a[ 8 ] * a[ 23 ] * a[ 24 ];
	tmp[ 1471 ] += a[ 9 ] * a[ 10 ] * a[ 11 ];
	tmp[ 1472 ] += a[ 9 ] * a[ 10 ] * a[ 13 ];
	tmp[ 1473 ] += a[ 9 ] * a[ 10 ] * a[ 14 ];
	tmp[ 1474 ] += a[ 9 ] * a[ 10 ] * a[ 15 ];
	tmp[ 1475 ] += a[ 9 ] * a[ 10 ] * a[ 16 ];
	tmp[ 1476 ] += a[ 9 ] * a[ 10 ] * a[ 18 ];
	tmp[ 1477 ] += a[ 9 ] * a[ 10 ] * a[ 19 ];
	tmp[ 1478 ] += a[ 9 ] * a[ 10 ] * a[ 20 ];
	tmp[ 1479 ] += a[ 9 ] * a[ 10 ] * a[ 21 ];
	tmp[ 1480 ] += a[ 9 ] * a[ 10 ] * a[ 22 ];
	tmp[ 1481 ] += a[ 9 ] * a[ 10 ] * a[ 23 ];
	tmp[ 1482 ] += a[ 9 ] * a[ 10 ] * a[ 24 ];
	tmp[ 1483 ] += a[ 9 ] * a[ 11 ] * a[ 14 ];
	tmp[ 1484 ] += a[ 9 ] * a[ 11 ] * a[ 15 ];
	tmp[ 1485 ] += a[ 9 ] * a[ 11 ] * a[ 16 ];
	tmp[ 1486 ] += a[ 9 ] * a[ 11 ] * a[ 19 ];
	tmp[ 1487 ] += a[ 9 ] * a[ 11 ] * a[ 20 ];
	tmp[ 1488 ] += a[ 9 ] * a[ 11 ] * a[ 21 ];
	tmp[ 1489 ] += a[ 9 ] * a[ 11 ] * a[ 22 ];
	tmp[ 1490 ] += a[ 9 ] * a[ 11 ] * a[ 23 ];
	tmp[ 1491 ] += a[ 9 ] * a[ 11 ] * a[ 24 ];
	tmp[ 1492 ] += a[ 9 ] * a[ 13 ] * a[ 15 ];
	tmp[ 1493 ] += a[ 9 ] * a[ 13 ] * a[ 20 ];
	tmp[ 1494 ] += a[ 9 ] * a[ 14 ] * a[ 15 ];
	tmp[ 1495 ] += a[ 9 ] * a[ 14 ] * a[ 16 ];
	tmp[ 1496 ] += a[ 9 ] * a[ 14 ] * a[ 20 ];
	tmp[ 1497 ] += a[ 9 ] * a[ 14 ] * a[ 21 ];
	tmp[ 1498 ] += a[ 9 ] * a[ 15 ] * a[ 16 ];
	tmp[ 1499 ] += a[ 9 ] * a[ 15 ] * a[ 18 ];
	tmp[ 1500 ] += a[ 9 ] * a[ 15 ] * a[ 19 ];
	tmp[ 1501 ] += a[ 9 ] * a[ 15 ] * a[ 20 ];
	tmp[ 1502 ] += a[ 9 ] * a[ 15 ] * a[ 21 ];
	tmp[ 1503 ] += a[ 9 ] * a[ 15 ] * a[ 22 ];
	tmp[ 1504 ] += a[ 9 ] * a[ 15 ] * a[ 23 ];
	tmp[ 1505 ] += a[ 9 ] * a[ 15 ] * a[ 24 ];
	tmp[ 1506 ] += a[ 9 ] * a[ 16 ] * a[ 19 ];
	tmp[ 1507 ] += a[ 9 ] * a[ 16 ] * a[ 20 ];
	tmp[ 1508 ] += a[ 9 ] * a[ 16 ] * a[ 21 ];
	tmp[ 1509 ] += a[ 9 ] * a[ 16 ] * a[ 22 ];
	tmp[ 1510 ] += a[ 9 ] * a[ 16 ] * a[ 23 ];
	tmp[ 1511 ] += a[ 9 ] * a[ 16 ] * a[ 24 ];
	tmp[ 1512 ] += a[ 9 ] * a[ 18 ] * a[ 20 ];
	tmp[ 1513 ] += a[ 9 ] * a[ 19 ] * a[ 20 ];
	tmp[ 1514 ] += a[ 9 ] * a[ 19 ] * a[ 21 ];
	tmp[ 1515 ] += a[ 9 ] * a[ 20 ] * a[ 21 ];
	tmp[ 1516 ] += a[ 9 ] * a[ 20 ] * a[ 22 ];
	tmp[ 1517 ] += a[ 9 ] * a[ 20 ] * a[ 23 ];
	tmp[ 1518 ] += a[ 9 ] * a[ 20 ] * a[ 24 ];
	tmp[ 1519 ] += a[ 9 ] * a[ 21 ] * a[ 22 ];
	tmp[ 1520 ] += a[ 9 ] * a[ 21 ] * a[ 23 ];
	tmp[ 1521 ] += a[ 9 ] * a[ 21 ] * a[ 24 ];
	tmp[ 1522 ] += a[ 9 ] * a[ 22 ] * a[ 23 ];
	tmp[ 1523 ] += a[ 9 ] * a[ 22 ] * a[ 24 ];
	tmp[ 1524 ] += a[ 9 ] * a[ 23 ] * a[ 24 ];
	tmp[ 1525 ] += a[ 10 ] * a[ 11 ] * a[ 13 ];
	tmp[ 1526 ] += a[ 10 ] * a[ 11 ] * a[ 14 ];
	tmp[ 1527 ] += a[ 10 ] * a[ 11 ] * a[ 18 ];
	tmp[ 1528 ] += a[ 10 ] * a[ 11 ] * a[ 19 ];
	tmp[ 1529 ] += a[ 10 ] * a[ 11 ] * a[ 23 ];
	tmp[ 1530 ] += a[ 10 ] * a[ 11 ] * a[ 24 ];
	tmp[ 1531 ] += a[ 10 ] * a[ 13 ] * a[ 14 ];
	tmp[ 1532 ] += a[ 10 ] * a[ 13 ] * a[ 15 ];
	tmp[ 1533 ] += a[ 10 ] * a[ 13 ] * a[ 18 ];
	tmp[ 1534 ] += a[ 10 ] * a[ 13 ] * a[ 19 ];
	tmp[ 1535 ] += a[ 10 ] * a[ 13 ] * a[ 20 ];
	tmp[ 1536 ] += a[ 10 ] * a[ 13 ] * a[ 23 ];
	tmp[ 1537 ] += a[ 10 ] * a[ 13 ] * a[ 24 ];
	tmp[ 1538 ] += a[ 10 ] * a[ 14 ] * a[ 15 ];
	tmp[ 1539 ] += a[ 10 ] * a[ 14 ] * a[ 16 ];
	tmp[ 1540 ] += a[ 10 ] * a[ 14 ] * a[ 18 ];
	tmp[ 1541 ] += a[ 10 ] * a[ 14 ] * a[ 19 ];
	tmp[ 1542 ] += a[ 10 ] * a[ 14 ] * a[ 20 ];
	tmp[ 1543 ] += a[ 10 ] * a[ 14 ] * a[ 21 ];
	tmp[ 1544 ] += a[ 10 ] * a[ 14 ] * a[ 23 ];
	tmp[ 1545 ] += a[ 10 ] * a[ 14 ] * a[ 24 ];
	tmp[ 1546 ] += a[ 10 ] * a[ 15 ] * a[ 18 ];
	tmp[ 1547 ] += a[ 10 ] * a[ 15 ] * a[ 19 ];
	tmp[ 1548 ] += a[ 10 ] * a[ 15 ] * a[ 23 ];
	tmp[ 1549 ] += a[ 10 ] * a[ 15 ] * a[ 24 ];
	tmp[ 1550 ] += a[ 10 ] * a[ 16 ] * a[ 19 ];
	tmp[ 1551 ] += a[ 10 ] * a[ 16 ] * a[ 24 ];
	tmp[ 1552 ] += a[ 10 ] * a[ 18 ] * a[ 19 ];
	tmp[ 1553 ] += a[ 10 ] * a[ 18 ] * a[ 20 ];
	tmp[ 1554 ] += a[ 10 ] * a[ 18 ] * a[ 23 ];
	tmp[ 1555 ] += a[ 10 ] * a[ 18 ] * a[ 24 ];
	tmp[ 1556 ] += a[ 10 ] * a[ 19 ] * a[ 20 ];
	tmp[ 1557 ] += a[ 10 ] * a[ 19 ] * a[ 21 ];
	tmp[ 1558 ] += a[ 10 ] * a[ 19 ] * a[ 23 ];
	tmp[ 1559 ] += a[ 10 ] * a[ 19 ] * a[ 24 ];
	tmp[ 1560 ] += a[ 10 ] * a[ 20 ] * a[ 23 ];
	tmp[ 1561 ] += a[ 10 ] * a[ 20 ] * a[ 24 ];
	tmp[ 1562 ] += a[ 10 ] * a[ 21 ] * a[ 24 ];
	tmp[ 1563 ] += a[ 10 ] * a[ 23 ] * a[ 24 ];
	tmp[ 1564 ] += a[ 11 ] * a[ 13 ] * a[ 15 ];
	tmp[ 1565 ] += a[ 11 ] * a[ 13 ] * a[ 20 ];
	tmp[ 1566 ] += a[ 11 ] * a[ 14 ] * a[ 15 ];
	tmp[ 1567 ] += a[ 11 ] * a[ 14 ] * a[ 16 ];
	tmp[ 1568 ] += a[ 11 ] * a[ 14 ] * a[ 19 ];
	tmp[ 1569 ] += a[ 11 ] * a[ 14 ] * a[ 20 ];
	tmp[ 1570 ] += a[ 11 ] * a[ 14 ] * a[ 21 ];
	tmp[ 1571 ] += a[ 11 ] * a[ 14 ] * a[ 24 ];
	tmp[ 1572 ] += a[ 11 ] * a[ 15 ] * a[ 18 ];
	tmp[ 1573 ] += a[ 11 ] * a[ 15 ] * a[ 19 ];
	tmp[ 1574 ] += a[ 11 ] * a[ 15 ] * a[ 23 ];
	tmp[ 1575 ] += a[ 11 ] * a[ 15 ] * a[ 24 ];
	tmp[ 1576 ] += a[ 11 ] * a[ 16 ] * a[ 19 ];
	tmp[ 1577 ] += a[ 11 ] * a[ 16 ] * a[ 24 ];
	tmp[ 1578 ] += a[ 11 ] * a[ 18 ] * a[ 20 ];
	tmp[ 1579 ] += a[ 11 ] * a[ 19 ] * a[ 20 ];
	tmp[ 1580 ] += a[ 11 ] * a[ 19 ] * a[ 21 ];
	tmp[ 1581 ] += a[ 11 ] * a[ 19 ] * a[ 24 ];
	tmp[ 1582 ] += a[ 11 ] * a[ 20 ] * a[ 23 ];
	tmp[ 1583 ] += a[ 11 ] * a[ 20 ] * a[ 24 ];
	tmp[ 1584 ] += a[ 11 ] * a[ 21 ] * a[ 24 ];
	tmp[ 1585 ] += a[ 13 ] * a[ 14 ] * a[ 15 ];
	tmp[ 1586 ] += a[ 13 ] * a[ 14 ] * a[ 20 ];
	tmp[ 1587 ] += a[ 13 ] * a[ 15 ] * a[ 18 ];
	tmp[ 1588 ] += a[ 13 ] * a[ 15 ] * a[ 19 ];
	tmp[ 1589 ] += a[ 13 ] * a[ 15 ] * a[ 20 ];
	tmp[ 1590 ] += a[ 13 ] * a[ 15 ] * a[ 23 ];
	tmp[ 1591 ] += a[ 13 ] * a[ 15 ] * a[ 24 ];
	tmp[ 1592 ] += a[ 13 ] * a[ 18 ] * a[ 20 ];
	tmp[ 1593 ] += a[ 13 ] * a[ 19 ] * a[ 20 ];
	tmp[ 1594 ] += a[ 13 ] * a[ 20 ] * a[ 23 ];
	tmp[ 1595 ] += a[ 13 ] * a[ 20 ] * a[ 24 ];
	tmp[ 1596 ] += a[ 14 ] * a[ 15 ] * a[ 16 ];
	tmp[ 1597 ] += a[ 14 ] * a[ 15 ] * a[ 18 ];
	tmp[ 1598 ] += a[ 14 ] * a[ 15 ] * a[ 19 ];
	tmp[ 1599 ] += a[ 14 ] * a[ 15 ] * a[ 20 ];
	tmp[ 1600 ] += a[ 14 ] * a[ 15 ] * a[ 21 ];
	tmp[ 1601 ] += a[ 14 ] * a[ 15 ] * a[ 23 ];
	tmp[ 1602 ] += a[ 14 ] * a[ 15 ] * a[ 24 ];
	tmp[ 1603 ] += a[ 14 ] * a[ 16 ] * a[ 19 ];
	tmp[ 1604 ] += a[ 14 ] * a[ 16 ] * a[ 20 ];
	tmp[ 1605 ] += a[ 14 ] * a[ 16 ] * a[ 21 ];
	tmp[ 1606 ] += a[ 14 ] * a[ 16 ] * a[ 24 ];
	tmp[ 1607 ] += a[ 14 ] * a[ 18 ] * a[ 20 ];
	tmp[ 1608 ] += a[ 14 ] * a[ 19 ] * a[ 20 ];
	tmp[ 1609 ] += a[ 14 ] * a[ 19 ] * a[ 21 ];
	tmp[ 1610 ] += a[ 14 ] * a[ 20 ] * a[ 21 ];
	tmp[ 1611 ] += a[ 14 ] * a[ 20 ] * a[ 23 ];
	tmp[ 1612 ] += a[ 14 ] * a[ 20 ] * a[ 24 ];
	tmp[ 1613 ] += a[ 14 ] * a[ 21 ] * a[ 24 ];
	tmp[ 1614 ] += a[ 15 ] * a[ 16 ] * a[ 19 ];
	tmp[ 1615 ] += a[ 15 ] * a[ 16 ] * a[ 24 ];
	tmp[ 1616 ] += a[ 15 ] * a[ 18 ] * a[ 19 ];
	tmp[ 1617 ] += a[ 15 ] * a[ 18 ] * a[ 20 ];
	tmp[ 1618 ] += a[ 15 ] * a[ 18 ] * a[ 23 ];
	tmp[ 1619 ] += a[ 15 ] * a[ 18 ] * a[ 24 ];
	tmp[ 1620 ] += a[ 15 ] * a[ 19 ] * a[ 20 ];
	tmp[ 1621 ] += a[ 15 ] * a[ 19 ] * a[ 21 ];
	tmp[ 1622 ] += a[ 15 ] * a[ 19 ] * a[ 23 ];
	tmp[ 1623 ] += a[ 15 ] * a[ 19 ] * a[ 24 ];
	tmp[ 1624 ] += a[ 15 ] * a[ 20 ] * a[ 23 ];
	tmp[ 1625 ] += a[ 15 ] * a[ 20 ] * a[ 24 ];
	tmp[ 1626 ] += a[ 15 ] * a[ 21 ] * a[ 24 ];
	tmp[ 1627 ] += a[ 15 ] * a[ 23 ] * a[ 24 ];
	tmp[ 1628 ] += a[ 16 ] * a[ 19 ] * a[ 20 ];
	tmp[ 1629 ] += a[ 16 ] * a[ 19 ] * a[ 21 ];
	tmp[ 1630 ] += a[ 16 ] * a[ 19 ] * a[ 24 ];
	tmp[ 1631 ] += a[ 16 ] * a[ 20 ] * a[ 24 ];
	tmp[ 1632 ] += a[ 16 ] * a[ 21 ] * a[ 24 ];
	tmp[ 1633 ] += a[ 18 ] * a[ 19 ] * a[ 20 ];
	tmp[ 1634 ] += a[ 18 ] * a[ 20 ] * a[ 23 ];
	tmp[ 1635 ] += a[ 18 ] * a[ 20 ] * a[ 24 ];
	tmp[ 1636 ] += a[ 19 ] * a[ 20 ] * a[ 21 ];
	tmp[ 1637 ] += a[ 19 ] * a[ 20 ] * a[ 23 ];
	tmp[ 1638 ] += a[ 19 ] * a[ 20 ] * a[ 24 ];
	tmp[ 1639 ] += a[ 19 ] * a[ 21 ] * a[ 24 ];
	tmp[ 1640 ] += a[ 20 ] * a[ 21 ] * a[ 24 ];
	tmp[ 1641 ] += a[ 20 ] * a[ 23 ] * a[ 24 ];
      }
    }
  }
  
  /*** normalize ***/
#ifdef ENABLE_NORMALIZATION
  double normalVal = (double)( ( right - left ) * ( down - up ) );
  for(int i=0; i<DIM_OF_BIN_HLAC3_5; ++i) result[ i ] = tmp[ i ] / normalVal;
#else /* !ENABLE_NORMALIZATION */
  for(int i=0; i<DIM_OF_BIN_HLAC3_5; ++i) result[ i ] = tmp[ i ];  
#endif /* ENABLE_NORMALIZATION */
}
#endif /* ENABLE_BIN_HLAC3_5 */


/*------------------------*
 *  for gray-scale image  *
 *------------------------*/
/*** 高々1次の gray HLAC, square size = 3 ************************************/
 
void HLAC::_extractGray1_3( std::vector<float> &result, const cv::Mat &img,
			       const int &rx, const int &ry, 
			       const int &left, const int &right,
			       const int &up, const int &down )
{
  double a;
  const int width_step = img.step;
  
  /*** initialize ***/
  double tmp[ DIM_OF_GRAY_HLAC1_3 ];
  for(int i=0; i<DIM_OF_GRAY_HLAC1_3; ++i) tmp[ i ] = 0.0;
  
  /*** feature extraction ***/
  for(int j=up; j<down; ++j){
    for(int i=left; i<right; ++i){
      a = (double)img.data[ i + j * width_step ];
      tmp[ 0 ] += a;
      tmp[ 1 ] += a * (double)img.data[ ( i - rx ) + ( j - ry ) * width_step ];
      tmp[ 2 ] += a * (double)img.data[   i        + ( j - ry ) * width_step ];
      tmp[ 3 ] += a * (double)img.data[ ( i + rx ) + ( j - ry ) * width_step ];
      tmp[ 4 ] += a * (double)img.data[ ( i - rx ) +   j        * width_step ];
      tmp[ 5 ] += a * a;
    }    
  }
  
  /*** normalize ***/  
#ifdef ENABLE_NORMALIZATION
  double normalVal = (double)( 255 * ( right - left ) * ( down - up ) );
  result[ 0 ] = tmp[ 0 ] / normalVal;
  normalVal *= 255.0;
  for(int i=1; i<DIM_OF_GRAY_HLAC1_3; ++i) result[ i ] = tmp[ i ] / normalVal;
#else /* !ENABLE_NORMALIZATION */
  result[ 0 ] = tmp[ 0 ] / 255.0;
  for(int i=1; i<DIM_OF_GRAY_HLAC1_3; ++i) result[ i ] = tmp[ i ] / 65025.0;
#endif /* ENABLE_NORMALIZATION */
}

/*** 高々2次の gray HLAC, square size = 3 ************************************/
 
void HLAC::_extractGray2_3( std::vector<float> &result, const cv::Mat &img,
			       const int &rx, const int &ry, 
			       const int &left, const int &right,
			       const int &up, const int &down )
{
  double a[ 9 ];
  const int width_step = img.step;
  
  /*** initialize ***/
  double tmp[ DIM_OF_GRAY_HLAC2_3 ];
  for(int i=0; i<DIM_OF_GRAY_HLAC2_3; ++i) tmp[ i ] = 0.0;
  
  /*** feature extraction ***/
  for(int j=up; j<down; ++j){
    for(int i=left; i<right; ++i){
      a[  0 ] = (double)img.data[ ( i - rx ) + ( j - ry ) * width_step ];
      a[  1 ] = (double)img.data[   i        + ( j - ry ) * width_step ];
      a[  2 ] = (double)img.data[ ( i + rx ) + ( j - ry ) * width_step ];
      a[  3 ] = (double)img.data[ ( i - rx ) +   j        * width_step ];
      a[  4 ] = (double)img.data[   i        +   j        * width_step ];
      a[  5 ] = (double)img.data[ ( i + rx ) +   j        * width_step ];
      a[  6 ] = (double)img.data[ ( i - rx ) + ( j + ry ) * width_step ];
      a[  7 ] = (double)img.data[   i        + ( j + ry ) * width_step ];
      a[  8 ] = (double)img.data[ ( i + rx ) + ( j + ry ) * width_step ];
      tmp[ 0 ] += a[ 4 ];
      tmp[ 1 ] += a[ 4 ] * a[ 0 ];
      tmp[ 2 ] += a[ 4 ] * a[ 1 ];
      tmp[ 3 ] += a[ 4 ] * a[ 2 ];
      tmp[ 4 ] += a[ 4 ] * a[ 3 ];
      tmp[ 5 ] += a[ 4 ] * a[ 4 ];
      tmp[ 6 ] += a[ 4 ] * a[ 0 ] * a[ 0 ];
      tmp[ 7 ] += a[ 4 ] * a[ 0 ] * a[ 1 ];
      tmp[ 8 ] += a[ 4 ] * a[ 0 ] * a[ 2 ];
      tmp[ 9 ] += a[ 4 ] * a[ 0 ] * a[ 3 ];
      tmp[ 10 ] += a[ 4 ] * a[ 0 ] * a[ 4 ];
      tmp[ 11 ] += a[ 4 ] * a[ 0 ] * a[ 5 ];
      tmp[ 12 ] += a[ 4 ] * a[ 0 ] * a[ 6 ];
      tmp[ 13 ] += a[ 4 ] * a[ 0 ] * a[ 7 ];
      tmp[ 14 ] += a[ 4 ] * a[ 0 ] * a[ 8 ];
      tmp[ 15 ] += a[ 4 ] * a[ 1 ] * a[ 1 ];
      tmp[ 16 ] += a[ 4 ] * a[ 1 ] * a[ 2 ];
      tmp[ 17 ] += a[ 4 ] * a[ 1 ] * a[ 3 ];
      tmp[ 18 ] += a[ 4 ] * a[ 1 ] * a[ 4 ];
      tmp[ 19 ] += a[ 4 ] * a[ 1 ] * a[ 6 ];
      tmp[ 20 ] += a[ 4 ] * a[ 1 ] * a[ 7 ];
      tmp[ 21 ] += a[ 4 ] * a[ 1 ] * a[ 8 ];
      tmp[ 22 ] += a[ 4 ] * a[ 2 ] * a[ 2 ];
      tmp[ 23 ] += a[ 4 ] * a[ 2 ] * a[ 3 ];
      tmp[ 24 ] += a[ 4 ] * a[ 2 ] * a[ 4 ];
      tmp[ 25 ] += a[ 4 ] * a[ 2 ] * a[ 6 ];
      tmp[ 26 ] += a[ 4 ] * a[ 2 ] * a[ 7 ];
      tmp[ 27 ] += a[ 4 ] * a[ 2 ] * a[ 8 ];
      tmp[ 28 ] += a[ 4 ] * a[ 3 ] * a[ 3 ];
      tmp[ 29 ] += a[ 4 ] * a[ 3 ] * a[ 4 ];
      tmp[ 30 ] += a[ 4 ] * a[ 3 ] * a[ 5 ];
      tmp[ 31 ] += a[ 4 ] * a[ 3 ] * a[ 8 ];
      tmp[ 32 ] += a[ 4 ] * a[ 4 ] * a[ 4 ];
      tmp[ 33 ] += a[ 4 ] * a[ 5 ] * a[ 6 ];
      tmp[ 34 ] += a[ 4 ] * a[ 6 ] * a[ 8 ];
    }    
  }
  
  /*** normalize ***/  
#ifdef ENABLE_NORMALIZATION
  double normalVal = (double)( 255 * ( right - left ) * ( down - up ) );
  result[ 0 ] = tmp[ 0 ] / normalVal;
  normalVal *= 255.0;
  for(int i=1; i<DIM_OF_GRAY_HLAC1_3; ++i){
    result[ i ] = tmp[ i ] / normalVal;
  }
  normalVal *= 255.0;
  for(int i=DIM_OF_GRAY_HLAC1_3; i<DIM_OF_GRAY_HLAC2_3; ++i){
    result[ i ] = tmp[ i ] / normalVal;
  }
#else /* !ENABLE_NORMALIZATION */
  result[ 0 ] = tmp[ 0 ] / 255.0;
  for(int i=1; i<DIM_OF_GRAY_HLAC1_3; ++i){
    result[ i ] = tmp[ i ] / 65025.0;
  }
  for(int i=DIM_OF_GRAY_HLAC1_3; i<DIM_OF_GRAY_HLAC2_3; ++i){
    result[ i ] = tmp[ i ] / 16581375.0;
  }
#endif /* ENABLE_NORMALIZATION */
}

/*** 高々3次の gray HLAC, square size = 3 ************************************/
 
void HLAC::_extractGray3_3( std::vector<float> &result, const cv::Mat &img,
			       const int &rx, const int &ry, 
			       const int &left, const int &right,
			       const int &up, const int &down )
{
  double a[ 9 ];
  const int width_step = img.step;
  
  /*** initialize ***/
  double tmp[ DIM_OF_GRAY_HLAC3_3 ];
  for(int i=0; i<DIM_OF_GRAY_HLAC3_3; ++i) tmp[ i ] = 0.0;
  
  /*** feature extraction ***/
  for(int j=up; j<down; ++j){
    for(int i=left; i<right; ++i){
      a[  0 ] = (double)img.data[ ( i - rx ) + ( j - ry ) * width_step ];
      a[  1 ] = (double)img.data[   i        + ( j - ry ) * width_step ];
      a[  2 ] = (double)img.data[ ( i + rx ) + ( j - ry ) * width_step ];
      a[  3 ] = (double)img.data[ ( i - rx ) +   j        * width_step ];
      a[  4 ] = (double)img.data[   i        +   j        * width_step ];
      a[  5 ] = (double)img.data[ ( i + rx ) +   j        * width_step ];
      a[  6 ] = (double)img.data[ ( i - rx ) + ( j + ry ) * width_step ];
      a[  7 ] = (double)img.data[   i        + ( j + ry ) * width_step ];
      a[  8 ] = (double)img.data[ ( i + rx ) + ( j + ry ) * width_step ];
      tmp[ 0 ] += a[ 4 ];
      tmp[ 1 ] += a[ 4 ] * a[ 0 ];
      tmp[ 2 ] += a[ 4 ] * a[ 1 ];
      tmp[ 3 ] += a[ 4 ] * a[ 2 ];
      tmp[ 4 ] += a[ 4 ] * a[ 3 ];
      tmp[ 5 ] += a[ 4 ] * a[ 4 ];
      tmp[ 6 ] += a[ 4 ] * a[ 0 ] * a[ 0 ];
      tmp[ 7 ] += a[ 4 ] * a[ 0 ] * a[ 1 ];
      tmp[ 8 ] += a[ 4 ] * a[ 0 ] * a[ 2 ];
      tmp[ 9 ] += a[ 4 ] * a[ 0 ] * a[ 3 ];
      tmp[ 10 ] += a[ 4 ] * a[ 0 ] * a[ 4 ];
      tmp[ 11 ] += a[ 4 ] * a[ 0 ] * a[ 5 ];
      tmp[ 12 ] += a[ 4 ] * a[ 0 ] * a[ 6 ];
      tmp[ 13 ] += a[ 4 ] * a[ 0 ] * a[ 7 ];
      tmp[ 14 ] += a[ 4 ] * a[ 0 ] * a[ 8 ];
      tmp[ 15 ] += a[ 4 ] * a[ 1 ] * a[ 1 ];
      tmp[ 16 ] += a[ 4 ] * a[ 1 ] * a[ 2 ];
      tmp[ 17 ] += a[ 4 ] * a[ 1 ] * a[ 3 ];
      tmp[ 18 ] += a[ 4 ] * a[ 1 ] * a[ 4 ];
      tmp[ 19 ] += a[ 4 ] * a[ 1 ] * a[ 6 ];
      tmp[ 20 ] += a[ 4 ] * a[ 1 ] * a[ 7 ];
      tmp[ 21 ] += a[ 4 ] * a[ 1 ] * a[ 8 ];
      tmp[ 22 ] += a[ 4 ] * a[ 2 ] * a[ 2 ];
      tmp[ 23 ] += a[ 4 ] * a[ 2 ] * a[ 3 ];
      tmp[ 24 ] += a[ 4 ] * a[ 2 ] * a[ 4 ];
      tmp[ 25 ] += a[ 4 ] * a[ 2 ] * a[ 6 ];
      tmp[ 26 ] += a[ 4 ] * a[ 2 ] * a[ 7 ];
      tmp[ 27 ] += a[ 4 ] * a[ 2 ] * a[ 8 ];
      tmp[ 28 ] += a[ 4 ] * a[ 3 ] * a[ 3 ];
      tmp[ 29 ] += a[ 4 ] * a[ 3 ] * a[ 4 ];
      tmp[ 30 ] += a[ 4 ] * a[ 3 ] * a[ 5 ];
      tmp[ 31 ] += a[ 4 ] * a[ 3 ] * a[ 8 ];
      tmp[ 32 ] += a[ 4 ] * a[ 4 ] * a[ 4 ];
      tmp[ 33 ] += a[ 4 ] * a[ 5 ] * a[ 6 ];
      tmp[ 34 ] += a[ 4 ] * a[ 6 ] * a[ 8 ];
      tmp[ 35 ] += a[ 4 ] * a[ 0 ] * a[ 0 ] * a[ 0 ];
      tmp[ 36 ] += a[ 4 ] * a[ 0 ] * a[ 0 ] * a[ 1 ];
      tmp[ 37 ] += a[ 4 ] * a[ 0 ] * a[ 0 ] * a[ 2 ];
      tmp[ 38 ] += a[ 4 ] * a[ 0 ] * a[ 0 ] * a[ 3 ];
      tmp[ 39 ] += a[ 4 ] * a[ 0 ] * a[ 0 ] * a[ 4 ];
      tmp[ 40 ] += a[ 4 ] * a[ 0 ] * a[ 0 ] * a[ 5 ];
      tmp[ 41 ] += a[ 4 ] * a[ 0 ] * a[ 0 ] * a[ 6 ];
      tmp[ 42 ] += a[ 4 ] * a[ 0 ] * a[ 0 ] * a[ 7 ];
      tmp[ 43 ] += a[ 4 ] * a[ 0 ] * a[ 0 ] * a[ 8 ];
      tmp[ 44 ] += a[ 4 ] * a[ 0 ] * a[ 1 ] * a[ 1 ];
      tmp[ 45 ] += a[ 4 ] * a[ 0 ] * a[ 1 ] * a[ 2 ];
      tmp[ 46 ] += a[ 4 ] * a[ 0 ] * a[ 1 ] * a[ 3 ];
      tmp[ 47 ] += a[ 4 ] * a[ 0 ] * a[ 1 ] * a[ 4 ];
      tmp[ 48 ] += a[ 4 ] * a[ 0 ] * a[ 1 ] * a[ 5 ];
      tmp[ 49 ] += a[ 4 ] * a[ 0 ] * a[ 1 ] * a[ 6 ];
      tmp[ 50 ] += a[ 4 ] * a[ 0 ] * a[ 1 ] * a[ 7 ];
      tmp[ 51 ] += a[ 4 ] * a[ 0 ] * a[ 1 ] * a[ 8 ];
      tmp[ 52 ] += a[ 4 ] * a[ 0 ] * a[ 2 ] * a[ 2 ];
      tmp[ 53 ] += a[ 4 ] * a[ 0 ] * a[ 2 ] * a[ 3 ];
      tmp[ 54 ] += a[ 4 ] * a[ 0 ] * a[ 2 ] * a[ 4 ];
      tmp[ 55 ] += a[ 4 ] * a[ 0 ] * a[ 2 ] * a[ 5 ];
      tmp[ 56 ] += a[ 4 ] * a[ 0 ] * a[ 2 ] * a[ 6 ];
      tmp[ 57 ] += a[ 4 ] * a[ 0 ] * a[ 2 ] * a[ 7 ];
      tmp[ 58 ] += a[ 4 ] * a[ 0 ] * a[ 2 ] * a[ 8 ];
      tmp[ 59 ] += a[ 4 ] * a[ 0 ] * a[ 3 ] * a[ 3 ];
      tmp[ 60 ] += a[ 4 ] * a[ 0 ] * a[ 3 ] * a[ 4 ];
      tmp[ 61 ] += a[ 4 ] * a[ 0 ] * a[ 3 ] * a[ 5 ];
      tmp[ 62 ] += a[ 4 ] * a[ 0 ] * a[ 3 ] * a[ 6 ];
      tmp[ 63 ] += a[ 4 ] * a[ 0 ] * a[ 3 ] * a[ 7 ];
      tmp[ 64 ] += a[ 4 ] * a[ 0 ] * a[ 3 ] * a[ 8 ];
      tmp[ 65 ] += a[ 4 ] * a[ 0 ] * a[ 4 ] * a[ 4 ];
      tmp[ 66 ] += a[ 4 ] * a[ 0 ] * a[ 4 ] * a[ 5 ];
      tmp[ 67 ] += a[ 4 ] * a[ 0 ] * a[ 4 ] * a[ 6 ];
      tmp[ 68 ] += a[ 4 ] * a[ 0 ] * a[ 4 ] * a[ 7 ];
      tmp[ 69 ] += a[ 4 ] * a[ 0 ] * a[ 4 ] * a[ 8 ];
      tmp[ 70 ] += a[ 4 ] * a[ 0 ] * a[ 5 ] * a[ 5 ];
      tmp[ 71 ] += a[ 4 ] * a[ 0 ] * a[ 5 ] * a[ 6 ];
      tmp[ 72 ] += a[ 4 ] * a[ 0 ] * a[ 5 ] * a[ 7 ];
      tmp[ 73 ] += a[ 4 ] * a[ 0 ] * a[ 5 ] * a[ 8 ];
      tmp[ 74 ] += a[ 4 ] * a[ 0 ] * a[ 6 ] * a[ 6 ];
      tmp[ 75 ] += a[ 4 ] * a[ 0 ] * a[ 6 ] * a[ 7 ];
      tmp[ 76 ] += a[ 4 ] * a[ 0 ] * a[ 6 ] * a[ 8 ];
      tmp[ 77 ] += a[ 4 ] * a[ 0 ] * a[ 7 ] * a[ 7 ];
      tmp[ 78 ] += a[ 4 ] * a[ 0 ] * a[ 7 ] * a[ 8 ];
      tmp[ 79 ] += a[ 4 ] * a[ 0 ] * a[ 8 ] * a[ 8 ];
      tmp[ 80 ] += a[ 4 ] * a[ 1 ] * a[ 1 ] * a[ 1 ];
      tmp[ 81 ] += a[ 4 ] * a[ 1 ] * a[ 1 ] * a[ 2 ];
      tmp[ 82 ] += a[ 4 ] * a[ 1 ] * a[ 1 ] * a[ 3 ];
      tmp[ 83 ] += a[ 4 ] * a[ 1 ] * a[ 1 ] * a[ 4 ];
      tmp[ 84 ] += a[ 4 ] * a[ 1 ] * a[ 1 ] * a[ 6 ];
      tmp[ 85 ] += a[ 4 ] * a[ 1 ] * a[ 1 ] * a[ 7 ];
      tmp[ 86 ] += a[ 4 ] * a[ 1 ] * a[ 1 ] * a[ 8 ];
      tmp[ 87 ] += a[ 4 ] * a[ 1 ] * a[ 2 ] * a[ 2 ];
      tmp[ 88 ] += a[ 4 ] * a[ 1 ] * a[ 2 ] * a[ 3 ];
      tmp[ 89 ] += a[ 4 ] * a[ 1 ] * a[ 2 ] * a[ 4 ];
      tmp[ 90 ] += a[ 4 ] * a[ 1 ] * a[ 2 ] * a[ 6 ];
      tmp[ 91 ] += a[ 4 ] * a[ 1 ] * a[ 2 ] * a[ 7 ];
      tmp[ 92 ] += a[ 4 ] * a[ 1 ] * a[ 2 ] * a[ 8 ];
      tmp[ 93 ] += a[ 4 ] * a[ 1 ] * a[ 3 ] * a[ 3 ];
      tmp[ 94 ] += a[ 4 ] * a[ 1 ] * a[ 3 ] * a[ 4 ];
      tmp[ 95 ] += a[ 4 ] * a[ 1 ] * a[ 3 ] * a[ 5 ];
      tmp[ 96 ] += a[ 4 ] * a[ 1 ] * a[ 3 ] * a[ 6 ];
      tmp[ 97 ] += a[ 4 ] * a[ 1 ] * a[ 3 ] * a[ 7 ];
      tmp[ 98 ] += a[ 4 ] * a[ 1 ] * a[ 3 ] * a[ 8 ];
      tmp[ 99 ] += a[ 4 ] * a[ 1 ] * a[ 4 ] * a[ 4 ];
      tmp[ 100 ] += a[ 4 ] * a[ 1 ] * a[ 4 ] * a[ 6 ];
      tmp[ 101 ] += a[ 4 ] * a[ 1 ] * a[ 4 ] * a[ 7 ];
      tmp[ 102 ] += a[ 4 ] * a[ 1 ] * a[ 4 ] * a[ 8 ];
      tmp[ 103 ] += a[ 4 ] * a[ 1 ] * a[ 5 ] * a[ 6 ];
      tmp[ 104 ] += a[ 4 ] * a[ 1 ] * a[ 6 ] * a[ 6 ];
      tmp[ 105 ] += a[ 4 ] * a[ 1 ] * a[ 6 ] * a[ 7 ];
      tmp[ 106 ] += a[ 4 ] * a[ 1 ] * a[ 6 ] * a[ 8 ];
      tmp[ 107 ] += a[ 4 ] * a[ 1 ] * a[ 7 ] * a[ 7 ];
      tmp[ 108 ] += a[ 4 ] * a[ 1 ] * a[ 7 ] * a[ 8 ];
      tmp[ 109 ] += a[ 4 ] * a[ 1 ] * a[ 8 ] * a[ 8 ];
      tmp[ 110 ] += a[ 4 ] * a[ 2 ] * a[ 2 ] * a[ 2 ];
      tmp[ 111 ] += a[ 4 ] * a[ 2 ] * a[ 2 ] * a[ 3 ];
      tmp[ 112 ] += a[ 4 ] * a[ 2 ] * a[ 2 ] * a[ 4 ];
      tmp[ 113 ] += a[ 4 ] * a[ 2 ] * a[ 2 ] * a[ 6 ];
      tmp[ 114 ] += a[ 4 ] * a[ 2 ] * a[ 2 ] * a[ 7 ];
      tmp[ 115 ] += a[ 4 ] * a[ 2 ] * a[ 2 ] * a[ 8 ];
      tmp[ 116 ] += a[ 4 ] * a[ 2 ] * a[ 3 ] * a[ 3 ];
      tmp[ 117 ] += a[ 4 ] * a[ 2 ] * a[ 3 ] * a[ 4 ];
      tmp[ 118 ] += a[ 4 ] * a[ 2 ] * a[ 3 ] * a[ 5 ];
      tmp[ 119 ] += a[ 4 ] * a[ 2 ] * a[ 3 ] * a[ 6 ];
      tmp[ 120 ] += a[ 4 ] * a[ 2 ] * a[ 3 ] * a[ 7 ];
      tmp[ 121 ] += a[ 4 ] * a[ 2 ] * a[ 3 ] * a[ 8 ];
      tmp[ 122 ] += a[ 4 ] * a[ 2 ] * a[ 4 ] * a[ 4 ];
      tmp[ 123 ] += a[ 4 ] * a[ 2 ] * a[ 4 ] * a[ 6 ];
      tmp[ 124 ] += a[ 4 ] * a[ 2 ] * a[ 4 ] * a[ 7 ];
      tmp[ 125 ] += a[ 4 ] * a[ 2 ] * a[ 4 ] * a[ 8 ];
      tmp[ 126 ] += a[ 4 ] * a[ 2 ] * a[ 5 ] * a[ 6 ];
      tmp[ 127 ] += a[ 4 ] * a[ 2 ] * a[ 6 ] * a[ 6 ];
      tmp[ 128 ] += a[ 4 ] * a[ 2 ] * a[ 6 ] * a[ 7 ];
      tmp[ 129 ] += a[ 4 ] * a[ 2 ] * a[ 6 ] * a[ 8 ];
      tmp[ 130 ] += a[ 4 ] * a[ 2 ] * a[ 7 ] * a[ 7 ];
      tmp[ 131 ] += a[ 4 ] * a[ 2 ] * a[ 7 ] * a[ 8 ];
      tmp[ 132 ] += a[ 4 ] * a[ 2 ] * a[ 8 ] * a[ 8 ];
      tmp[ 133 ] += a[ 4 ] * a[ 3 ] * a[ 3 ] * a[ 3 ];
      tmp[ 134 ] += a[ 4 ] * a[ 3 ] * a[ 3 ] * a[ 4 ];
      tmp[ 135 ] += a[ 4 ] * a[ 3 ] * a[ 3 ] * a[ 5 ];
      tmp[ 136 ] += a[ 4 ] * a[ 3 ] * a[ 3 ] * a[ 8 ];
      tmp[ 137 ] += a[ 4 ] * a[ 3 ] * a[ 4 ] * a[ 4 ];
      tmp[ 138 ] += a[ 4 ] * a[ 3 ] * a[ 4 ] * a[ 5 ];
      tmp[ 139 ] += a[ 4 ] * a[ 3 ] * a[ 4 ] * a[ 8 ];
      tmp[ 140 ] += a[ 4 ] * a[ 3 ] * a[ 5 ] * a[ 5 ];
      tmp[ 141 ] += a[ 4 ] * a[ 3 ] * a[ 5 ] * a[ 6 ];
      tmp[ 142 ] += a[ 4 ] * a[ 3 ] * a[ 5 ] * a[ 8 ];
      tmp[ 143 ] += a[ 4 ] * a[ 3 ] * a[ 6 ] * a[ 8 ];
      tmp[ 144 ] += a[ 4 ] * a[ 3 ] * a[ 8 ] * a[ 8 ];
      tmp[ 145 ] += a[ 4 ] * a[ 4 ] * a[ 4 ] * a[ 4 ];
      tmp[ 146 ] += a[ 4 ] * a[ 4 ] * a[ 5 ] * a[ 6 ];
      tmp[ 147 ] += a[ 4 ] * a[ 4 ] * a[ 6 ] * a[ 8 ];
      tmp[ 148 ] += a[ 4 ] * a[ 5 ] * a[ 5 ] * a[ 6 ];
      tmp[ 149 ] += a[ 4 ] * a[ 5 ] * a[ 6 ] * a[ 6 ];
      tmp[ 150 ] += a[ 4 ] * a[ 5 ] * a[ 6 ] * a[ 8 ];
      tmp[ 151 ] += a[ 4 ] * a[ 6 ] * a[ 6 ] * a[ 8 ];
      tmp[ 152 ] += a[ 4 ] * a[ 6 ] * a[ 8 ] * a[ 8 ];
    }    
  }
  
  /*** normalize ***/  
#ifdef ENABLE_NORMALIZATION
  double normalVal = (double)( 255 * ( right - left ) * ( down - up ) );
  result[ 0 ] = tmp[ 0 ] / normalVal;
  normalVal *= 255.0;
  for(int i=1; i<DIM_OF_GRAY_HLAC1_3; ++i){
    result[ i ] = tmp[ i ] / normalVal;
  }
  normalVal *= 255.0;
  for(int i=DIM_OF_GRAY_HLAC1_3; i<DIM_OF_GRAY_HLAC2_3; ++i){
    result[ i ] = tmp[ i ] / normalVal;
  }
  normalVal *= 255.0;
  for(int i=DIM_OF_GRAY_HLAC2_3; i<DIM_OF_GRAY_HLAC3_3; ++i){
    result[ i ] = tmp[ i ] / normalVal;
  }
#else /* !ENABLE_NORMALIZATION */
  result[ 0 ] = tmp[ 0 ] / 255.0;
  for(int i=1; i<DIM_OF_GRAY_HLAC1_3; ++i){
    result[ i ] = tmp[ i ] / 65025.0;
  }
  for(int i=DIM_OF_GRAY_HLAC1_3; i<DIM_OF_GRAY_HLAC2_3; ++i){
    result[ i ] = tmp[ i ] / 16581375.0;
  }
  for(int i=DIM_OF_GRAY_HLAC2_3; i<DIM_OF_GRAY_HLAC3_3; ++i){
    result[ i ] = tmp[ i ] / 4228250625.0;
  }
#endif /* ENABLE_NORMALIZATION */
}

/*** 高々1次の gray HLAC, square size = 5 ************************************/
 
void HLAC::_extractGray1_5( std::vector<float> &result, const cv::Mat &img,
			       const int &rx, const int &ry, 
			       const int &left, const int &right,
			       const int &up, const int &down )
{
  double a;
  const int width_step = img.step;
  
  /*** initialize ***/
  double tmp[ DIM_OF_GRAY_HLAC1_5 ];
  for(int i=0; i<DIM_OF_GRAY_HLAC1_5; ++i) tmp[ i ] = 0.0;
  
  /*** feature extraction ***/
  for(int j=up; j<down; ++j){
    for(int i=left; i<right; ++i){
      a = (double)img.data[ i + j * width_step ];
      tmp[ 0 ] += a;
      tmp[ 1 ] += a * (double)img.data[ ( i - 2*rx ) + ( j - 2*ry ) * width_step ];
      tmp[ 2 ] += a * (double)img.data[ ( i -   rx ) + ( j - 2*ry ) * width_step ];
      tmp[ 3 ] += a * (double)img.data[   i          + ( j - 2*ry ) * width_step ];
      tmp[ 4 ] += a * (double)img.data[ ( i +   rx ) + ( j - 2*ry ) * width_step ];
      tmp[ 5 ] += a * (double)img.data[ ( i + 2*rx ) + ( j - 2*ry ) * width_step ];
      tmp[ 6 ] += a * (double)img.data[ ( i - 2*rx ) + ( j -   ry ) * width_step ];
      tmp[ 7 ] += a * (double)img.data[ ( i -   rx ) + ( j -   ry ) * width_step ];
      tmp[ 8 ] += a * (double)img.data[   i          + ( j -   ry ) * width_step ];
      tmp[ 9 ] += a * (double)img.data[ ( i +   rx ) + ( j -   ry ) * width_step ];
      tmp[ 10 ] += a * (double)img.data[ ( i + 2*rx ) + ( j -   ry ) * width_step ];
      tmp[ 11 ] += a * (double)img.data[ ( i - 2*rx ) +   j          * width_step ];
      tmp[ 12 ] += a * (double)img.data[ ( i -   rx ) +   j          * width_step ];
      tmp[ 13 ] += a * a;
    }
  }
  
  /*** normalize ***/  
#ifdef ENABLE_NORMALIZATION
  double normalVal = (double)( 255 * ( right - left ) * ( down - up ) );
  result[ 0 ] = tmp[ 0 ] / normalVal;
  normalVal *= 255.0;
  for(int i=1; i<DIM_OF_GRAY_HLAC1_5; ++i){
    result[ i ] = tmp[ i ] / normalVal;
  }
#else /* !ENABLE_NORMALIZATION */
  result[ 0 ] = tmp[ 0 ] / 255.0;
  for(int i=1; i<DIM_OF_GRAY_HLAC1_5; ++i){
    result[ i ] = tmp[ i ] / 65025.0;
  }
#endif /* ENABLE_NORMALIZATION */
}

/*** 高々2次の gray HLAC, square size = 5 ************************************/
 
void HLAC::_extractGray2_5( std::vector<float> &result, const cv::Mat &img,
			       const int &rx, const int &ry, 
			       const int &left, const int &right,
			       const int &up, const int &down )
{
  double a[ 25 ];
  const int width_step = img.step;
  
  /*** initialize ***/
  double tmp[ DIM_OF_GRAY_HLAC2_5 ];
  for(int i=0; i<DIM_OF_GRAY_HLAC2_5; ++i) tmp[ i ] = 0.0;
  
  /*** feature extraction ***/
  for(int j=up; j<down; ++j){
    for(int i=left; i<right; ++i){
      a[  0 ] = (double)img.data[ ( i - 2*rx ) + ( j - 2*ry ) * width_step ];
      a[  1 ] = (double)img.data[ ( i -   rx ) + ( j - 2*ry ) * width_step ];
      a[  2 ] = (double)img.data[   i          + ( j - 2*ry ) * width_step ];
      a[  3 ] = (double)img.data[ ( i +   rx ) + ( j - 2*ry ) * width_step ];
      a[  4 ] = (double)img.data[ ( i + 2*rx ) + ( j - 2*ry ) * width_step ];
      a[  5 ] = (double)img.data[ ( i - 2*rx ) + ( j -   ry ) * width_step ];
      a[  6 ] = (double)img.data[ ( i -   rx ) + ( j -   ry ) * width_step ];
      a[  7 ] = (double)img.data[   i          + ( j -   ry ) * width_step ];
      a[  8 ] = (double)img.data[ ( i +   rx ) + ( j -   ry ) * width_step ];
      a[  9 ] = (double)img.data[ ( i + 2*rx ) + ( j -   ry ) * width_step ];
      a[ 10 ] = (double)img.data[ ( i - 2*rx ) +   j          * width_step ];
      a[ 11 ] = (double)img.data[ ( i -   rx ) +   j          * width_step ];
      a[ 12 ] = (double)img.data[   i          +   j          * width_step ];
      a[ 13 ] = (double)img.data[ ( i +   rx ) +   j          * width_step ];
      a[ 14 ] = (double)img.data[ ( i + 2*rx ) +   j          * width_step ];
      a[ 15 ] = (double)img.data[ ( i - 2*rx ) + ( j +   ry ) * width_step ];
      a[ 16 ] = (double)img.data[ ( i -   rx ) + ( j +   ry ) * width_step ];
      a[ 17 ] = (double)img.data[   i          + ( j +   ry ) * width_step ];
      a[ 18 ] = (double)img.data[ ( i +   rx ) + ( j +   ry ) * width_step ];
      a[ 19 ] = (double)img.data[ ( i + 2*rx ) + ( j +   ry ) * width_step ];
      a[ 20 ] = (double)img.data[ ( i - 2*rx ) + ( j + 2*ry ) * width_step ];
      a[ 21 ] = (double)img.data[ ( i -   rx ) + ( j + 2*ry ) * width_step ];
      a[ 22 ] = (double)img.data[   i          + ( j + 2*ry ) * width_step ];
      a[ 23 ] = (double)img.data[ ( i +   rx ) + ( j + 2*ry ) * width_step ];
      a[ 24 ] = (double)img.data[ ( i + 2*rx ) + ( j + 2*ry ) * width_step ];
      tmp[ 0 ] += a[ 12 ];
      tmp[ 1 ] += a[ 12 ] * a[ 0 ];
      tmp[ 2 ] += a[ 12 ] * a[ 1 ];
      tmp[ 3 ] += a[ 12 ] * a[ 2 ];
      tmp[ 4 ] += a[ 12 ] * a[ 3 ];
      tmp[ 5 ] += a[ 12 ] * a[ 4 ];
      tmp[ 6 ] += a[ 12 ] * a[ 5 ];
      tmp[ 7 ] += a[ 12 ] * a[ 6 ];
      tmp[ 8 ] += a[ 12 ] * a[ 7 ];
      tmp[ 9 ] += a[ 12 ] * a[ 8 ];
      tmp[ 10 ] += a[ 12 ] * a[ 9 ];
      tmp[ 11 ] += a[ 12 ] * a[ 10 ];
      tmp[ 12 ] += a[ 12 ] * a[ 11 ];
      tmp[ 13 ] += a[ 12 ] * a[ 12 ];
      tmp[ 14 ] += a[ 12 ] * a[ 0 ] * a[ 0 ];
      tmp[ 15 ] += a[ 12 ] * a[ 0 ] * a[ 1 ];
      tmp[ 16 ] += a[ 12 ] * a[ 0 ] * a[ 2 ];
      tmp[ 17 ] += a[ 12 ] * a[ 0 ] * a[ 3 ];
      tmp[ 18 ] += a[ 12 ] * a[ 0 ] * a[ 4 ];
      tmp[ 19 ] += a[ 12 ] * a[ 0 ] * a[ 5 ];
      tmp[ 20 ] += a[ 12 ] * a[ 0 ] * a[ 6 ];
      tmp[ 21 ] += a[ 12 ] * a[ 0 ] * a[ 7 ];
      tmp[ 22 ] += a[ 12 ] * a[ 0 ] * a[ 8 ];
      tmp[ 23 ] += a[ 12 ] * a[ 0 ] * a[ 9 ];
      tmp[ 24 ] += a[ 12 ] * a[ 0 ] * a[ 10 ];
      tmp[ 25 ] += a[ 12 ] * a[ 0 ] * a[ 11 ];
      tmp[ 26 ] += a[ 12 ] * a[ 0 ] * a[ 12 ];
      tmp[ 27 ] += a[ 12 ] * a[ 0 ] * a[ 13 ];
      tmp[ 28 ] += a[ 12 ] * a[ 0 ] * a[ 14 ];
      tmp[ 29 ] += a[ 12 ] * a[ 0 ] * a[ 15 ];
      tmp[ 30 ] += a[ 12 ] * a[ 0 ] * a[ 16 ];
      tmp[ 31 ] += a[ 12 ] * a[ 0 ] * a[ 17 ];
      tmp[ 32 ] += a[ 12 ] * a[ 0 ] * a[ 18 ];
      tmp[ 33 ] += a[ 12 ] * a[ 0 ] * a[ 19 ];
      tmp[ 34 ] += a[ 12 ] * a[ 0 ] * a[ 20 ];
      tmp[ 35 ] += a[ 12 ] * a[ 0 ] * a[ 21 ];
      tmp[ 36 ] += a[ 12 ] * a[ 0 ] * a[ 22 ];
      tmp[ 37 ] += a[ 12 ] * a[ 0 ] * a[ 23 ];
      tmp[ 38 ] += a[ 12 ] * a[ 0 ] * a[ 24 ];
      tmp[ 39 ] += a[ 12 ] * a[ 1 ] * a[ 1 ];
      tmp[ 40 ] += a[ 12 ] * a[ 1 ] * a[ 2 ];
      tmp[ 41 ] += a[ 12 ] * a[ 1 ] * a[ 3 ];
      tmp[ 42 ] += a[ 12 ] * a[ 1 ] * a[ 4 ];
      tmp[ 43 ] += a[ 12 ] * a[ 1 ] * a[ 5 ];
      tmp[ 44 ] += a[ 12 ] * a[ 1 ] * a[ 6 ];
      tmp[ 45 ] += a[ 12 ] * a[ 1 ] * a[ 7 ];
      tmp[ 46 ] += a[ 12 ] * a[ 1 ] * a[ 8 ];
      tmp[ 47 ] += a[ 12 ] * a[ 1 ] * a[ 9 ];
      tmp[ 48 ] += a[ 12 ] * a[ 1 ] * a[ 10 ];
      tmp[ 49 ] += a[ 12 ] * a[ 1 ] * a[ 11 ];
      tmp[ 50 ] += a[ 12 ] * a[ 1 ] * a[ 12 ];
      tmp[ 51 ] += a[ 12 ] * a[ 1 ] * a[ 14 ];
      tmp[ 52 ] += a[ 12 ] * a[ 1 ] * a[ 15 ];
      tmp[ 53 ] += a[ 12 ] * a[ 1 ] * a[ 16 ];
      tmp[ 54 ] += a[ 12 ] * a[ 1 ] * a[ 17 ];
      tmp[ 55 ] += a[ 12 ] * a[ 1 ] * a[ 18 ];
      tmp[ 56 ] += a[ 12 ] * a[ 1 ] * a[ 19 ];
      tmp[ 57 ] += a[ 12 ] * a[ 1 ] * a[ 20 ];
      tmp[ 58 ] += a[ 12 ] * a[ 1 ] * a[ 21 ];
      tmp[ 59 ] += a[ 12 ] * a[ 1 ] * a[ 22 ];
      tmp[ 60 ] += a[ 12 ] * a[ 1 ] * a[ 23 ];
      tmp[ 61 ] += a[ 12 ] * a[ 1 ] * a[ 24 ];
      tmp[ 62 ] += a[ 12 ] * a[ 2 ] * a[ 2 ];
      tmp[ 63 ] += a[ 12 ] * a[ 2 ] * a[ 3 ];
      tmp[ 64 ] += a[ 12 ] * a[ 2 ] * a[ 4 ];
      tmp[ 65 ] += a[ 12 ] * a[ 2 ] * a[ 5 ];
      tmp[ 66 ] += a[ 12 ] * a[ 2 ] * a[ 6 ];
      tmp[ 67 ] += a[ 12 ] * a[ 2 ] * a[ 7 ];
      tmp[ 68 ] += a[ 12 ] * a[ 2 ] * a[ 8 ];
      tmp[ 69 ] += a[ 12 ] * a[ 2 ] * a[ 9 ];
      tmp[ 70 ] += a[ 12 ] * a[ 2 ] * a[ 10 ];
      tmp[ 71 ] += a[ 12 ] * a[ 2 ] * a[ 11 ];
      tmp[ 72 ] += a[ 12 ] * a[ 2 ] * a[ 12 ];
      tmp[ 73 ] += a[ 12 ] * a[ 2 ] * a[ 15 ];
      tmp[ 74 ] += a[ 12 ] * a[ 2 ] * a[ 16 ];
      tmp[ 75 ] += a[ 12 ] * a[ 2 ] * a[ 17 ];
      tmp[ 76 ] += a[ 12 ] * a[ 2 ] * a[ 18 ];
      tmp[ 77 ] += a[ 12 ] * a[ 2 ] * a[ 19 ];
      tmp[ 78 ] += a[ 12 ] * a[ 2 ] * a[ 20 ];
      tmp[ 79 ] += a[ 12 ] * a[ 2 ] * a[ 21 ];
      tmp[ 80 ] += a[ 12 ] * a[ 2 ] * a[ 22 ];
      tmp[ 81 ] += a[ 12 ] * a[ 2 ] * a[ 23 ];
      tmp[ 82 ] += a[ 12 ] * a[ 2 ] * a[ 24 ];
      tmp[ 83 ] += a[ 12 ] * a[ 3 ] * a[ 3 ];
      tmp[ 84 ] += a[ 12 ] * a[ 3 ] * a[ 4 ];
      tmp[ 85 ] += a[ 12 ] * a[ 3 ] * a[ 5 ];
      tmp[ 86 ] += a[ 12 ] * a[ 3 ] * a[ 6 ];
      tmp[ 87 ] += a[ 12 ] * a[ 3 ] * a[ 7 ];
      tmp[ 88 ] += a[ 12 ] * a[ 3 ] * a[ 8 ];
      tmp[ 89 ] += a[ 12 ] * a[ 3 ] * a[ 9 ];
      tmp[ 90 ] += a[ 12 ] * a[ 3 ] * a[ 10 ];
      tmp[ 91 ] += a[ 12 ] * a[ 3 ] * a[ 11 ];
      tmp[ 92 ] += a[ 12 ] * a[ 3 ] * a[ 12 ];
      tmp[ 93 ] += a[ 12 ] * a[ 3 ] * a[ 15 ];
      tmp[ 94 ] += a[ 12 ] * a[ 3 ] * a[ 16 ];
      tmp[ 95 ] += a[ 12 ] * a[ 3 ] * a[ 17 ];
      tmp[ 96 ] += a[ 12 ] * a[ 3 ] * a[ 18 ];
      tmp[ 97 ] += a[ 12 ] * a[ 3 ] * a[ 19 ];
      tmp[ 98 ] += a[ 12 ] * a[ 3 ] * a[ 20 ];
      tmp[ 99 ] += a[ 12 ] * a[ 3 ] * a[ 21 ];
      tmp[ 100 ] += a[ 12 ] * a[ 3 ] * a[ 22 ];
      tmp[ 101 ] += a[ 12 ] * a[ 3 ] * a[ 23 ];
      tmp[ 102 ] += a[ 12 ] * a[ 3 ] * a[ 24 ];
      tmp[ 103 ] += a[ 12 ] * a[ 4 ] * a[ 4 ];
      tmp[ 104 ] += a[ 12 ] * a[ 4 ] * a[ 5 ];
      tmp[ 105 ] += a[ 12 ] * a[ 4 ] * a[ 6 ];
      tmp[ 106 ] += a[ 12 ] * a[ 4 ] * a[ 7 ];
      tmp[ 107 ] += a[ 12 ] * a[ 4 ] * a[ 8 ];
      tmp[ 108 ] += a[ 12 ] * a[ 4 ] * a[ 9 ];
      tmp[ 109 ] += a[ 12 ] * a[ 4 ] * a[ 10 ];
      tmp[ 110 ] += a[ 12 ] * a[ 4 ] * a[ 11 ];
      tmp[ 111 ] += a[ 12 ] * a[ 4 ] * a[ 12 ];
      tmp[ 112 ] += a[ 12 ] * a[ 4 ] * a[ 15 ];
      tmp[ 113 ] += a[ 12 ] * a[ 4 ] * a[ 16 ];
      tmp[ 114 ] += a[ 12 ] * a[ 4 ] * a[ 17 ];
      tmp[ 115 ] += a[ 12 ] * a[ 4 ] * a[ 18 ];
      tmp[ 116 ] += a[ 12 ] * a[ 4 ] * a[ 19 ];
      tmp[ 117 ] += a[ 12 ] * a[ 4 ] * a[ 20 ];
      tmp[ 118 ] += a[ 12 ] * a[ 4 ] * a[ 21 ];
      tmp[ 119 ] += a[ 12 ] * a[ 4 ] * a[ 22 ];
      tmp[ 120 ] += a[ 12 ] * a[ 4 ] * a[ 23 ];
      tmp[ 121 ] += a[ 12 ] * a[ 4 ] * a[ 24 ];
      tmp[ 122 ] += a[ 12 ] * a[ 5 ] * a[ 5 ];
      tmp[ 123 ] += a[ 12 ] * a[ 5 ] * a[ 6 ];
      tmp[ 124 ] += a[ 12 ] * a[ 5 ] * a[ 7 ];
      tmp[ 125 ] += a[ 12 ] * a[ 5 ] * a[ 8 ];
      tmp[ 126 ] += a[ 12 ] * a[ 5 ] * a[ 9 ];
      tmp[ 127 ] += a[ 12 ] * a[ 5 ] * a[ 10 ];
      tmp[ 128 ] += a[ 12 ] * a[ 5 ] * a[ 11 ];
      tmp[ 129 ] += a[ 12 ] * a[ 5 ] * a[ 12 ];
      tmp[ 130 ] += a[ 12 ] * a[ 5 ] * a[ 13 ];
      tmp[ 131 ] += a[ 12 ] * a[ 5 ] * a[ 14 ];
      tmp[ 132 ] += a[ 12 ] * a[ 5 ] * a[ 18 ];
      tmp[ 133 ] += a[ 12 ] * a[ 5 ] * a[ 19 ];
      tmp[ 134 ] += a[ 12 ] * a[ 5 ] * a[ 20 ];
      tmp[ 135 ] += a[ 12 ] * a[ 5 ] * a[ 21 ];
      tmp[ 136 ] += a[ 12 ] * a[ 5 ] * a[ 22 ];
      tmp[ 137 ] += a[ 12 ] * a[ 5 ] * a[ 23 ];
      tmp[ 138 ] += a[ 12 ] * a[ 5 ] * a[ 24 ];
      tmp[ 139 ] += a[ 12 ] * a[ 6 ] * a[ 6 ];
      tmp[ 140 ] += a[ 12 ] * a[ 6 ] * a[ 7 ];
      tmp[ 141 ] += a[ 12 ] * a[ 6 ] * a[ 8 ];
      tmp[ 142 ] += a[ 12 ] * a[ 6 ] * a[ 9 ];
      tmp[ 143 ] += a[ 12 ] * a[ 6 ] * a[ 10 ];
      tmp[ 144 ] += a[ 12 ] * a[ 6 ] * a[ 11 ];
      tmp[ 145 ] += a[ 12 ] * a[ 6 ] * a[ 12 ];
      tmp[ 146 ] += a[ 12 ] * a[ 6 ] * a[ 14 ];
      tmp[ 147 ] += a[ 12 ] * a[ 6 ] * a[ 19 ];
      tmp[ 148 ] += a[ 12 ] * a[ 6 ] * a[ 20 ];
      tmp[ 149 ] += a[ 12 ] * a[ 6 ] * a[ 21 ];
      tmp[ 150 ] += a[ 12 ] * a[ 6 ] * a[ 22 ];
      tmp[ 151 ] += a[ 12 ] * a[ 6 ] * a[ 23 ];
      tmp[ 152 ] += a[ 12 ] * a[ 6 ] * a[ 24 ];
      tmp[ 153 ] += a[ 12 ] * a[ 7 ] * a[ 7 ];
      tmp[ 154 ] += a[ 12 ] * a[ 7 ] * a[ 8 ];
      tmp[ 155 ] += a[ 12 ] * a[ 7 ] * a[ 9 ];
      tmp[ 156 ] += a[ 12 ] * a[ 7 ] * a[ 10 ];
      tmp[ 157 ] += a[ 12 ] * a[ 7 ] * a[ 11 ];
      tmp[ 158 ] += a[ 12 ] * a[ 7 ] * a[ 12 ];
      tmp[ 159 ] += a[ 12 ] * a[ 7 ] * a[ 20 ];
      tmp[ 160 ] += a[ 12 ] * a[ 7 ] * a[ 21 ];
      tmp[ 161 ] += a[ 12 ] * a[ 7 ] * a[ 22 ];
      tmp[ 162 ] += a[ 12 ] * a[ 7 ] * a[ 23 ];
      tmp[ 163 ] += a[ 12 ] * a[ 7 ] * a[ 24 ];
      tmp[ 164 ] += a[ 12 ] * a[ 8 ] * a[ 8 ];
      tmp[ 165 ] += a[ 12 ] * a[ 8 ] * a[ 9 ];
      tmp[ 166 ] += a[ 12 ] * a[ 8 ] * a[ 10 ];
      tmp[ 167 ] += a[ 12 ] * a[ 8 ] * a[ 11 ];
      tmp[ 168 ] += a[ 12 ] * a[ 8 ] * a[ 12 ];
      tmp[ 169 ] += a[ 12 ] * a[ 8 ] * a[ 15 ];
      tmp[ 170 ] += a[ 12 ] * a[ 8 ] * a[ 20 ];
      tmp[ 171 ] += a[ 12 ] * a[ 8 ] * a[ 21 ];
      tmp[ 172 ] += a[ 12 ] * a[ 8 ] * a[ 22 ];
      tmp[ 173 ] += a[ 12 ] * a[ 8 ] * a[ 23 ];
      tmp[ 174 ] += a[ 12 ] * a[ 8 ] * a[ 24 ];
      tmp[ 175 ] += a[ 12 ] * a[ 9 ] * a[ 9 ];
      tmp[ 176 ] += a[ 12 ] * a[ 9 ] * a[ 10 ];
      tmp[ 177 ] += a[ 12 ] * a[ 9 ] * a[ 11 ];
      tmp[ 178 ] += a[ 12 ] * a[ 9 ] * a[ 12 ];
      tmp[ 179 ] += a[ 12 ] * a[ 9 ] * a[ 15 ];
      tmp[ 180 ] += a[ 12 ] * a[ 9 ] * a[ 16 ];
      tmp[ 181 ] += a[ 12 ] * a[ 9 ] * a[ 20 ];
      tmp[ 182 ] += a[ 12 ] * a[ 9 ] * a[ 21 ];
      tmp[ 183 ] += a[ 12 ] * a[ 9 ] * a[ 22 ];
      tmp[ 184 ] += a[ 12 ] * a[ 9 ] * a[ 23 ];
      tmp[ 185 ] += a[ 12 ] * a[ 9 ] * a[ 24 ];
      tmp[ 186 ] += a[ 12 ] * a[ 10 ] * a[ 10 ];
      tmp[ 187 ] += a[ 12 ] * a[ 10 ] * a[ 11 ];
      tmp[ 188 ] += a[ 12 ] * a[ 10 ] * a[ 12 ];
      tmp[ 189 ] += a[ 12 ] * a[ 10 ] * a[ 13 ];
      tmp[ 190 ] += a[ 12 ] * a[ 10 ] * a[ 14 ];
      tmp[ 191 ] += a[ 12 ] * a[ 10 ] * a[ 18 ];
      tmp[ 192 ] += a[ 12 ] * a[ 10 ] * a[ 19 ];
      tmp[ 193 ] += a[ 12 ] * a[ 10 ] * a[ 23 ];
      tmp[ 194 ] += a[ 12 ] * a[ 10 ] * a[ 24 ];
      tmp[ 195 ] += a[ 12 ] * a[ 11 ] * a[ 11 ];
      tmp[ 196 ] += a[ 12 ] * a[ 11 ] * a[ 12 ];
      tmp[ 197 ] += a[ 12 ] * a[ 11 ] * a[ 14 ];
      tmp[ 198 ] += a[ 12 ] * a[ 11 ] * a[ 19 ];
      tmp[ 199 ] += a[ 12 ] * a[ 11 ] * a[ 24 ];
      tmp[ 200 ] += a[ 12 ] * a[ 12 ] * a[ 12 ];
      tmp[ 201 ] += a[ 12 ] * a[ 13 ] * a[ 15 ];
      tmp[ 202 ] += a[ 12 ] * a[ 13 ] * a[ 20 ];
      tmp[ 203 ] += a[ 12 ] * a[ 14 ] * a[ 15 ];
      tmp[ 204 ] += a[ 12 ] * a[ 14 ] * a[ 16 ];
      tmp[ 205 ] += a[ 12 ] * a[ 14 ] * a[ 20 ];
      tmp[ 206 ] += a[ 12 ] * a[ 14 ] * a[ 21 ];
      tmp[ 207 ] += a[ 12 ] * a[ 15 ] * a[ 18 ];
      tmp[ 208 ] += a[ 12 ] * a[ 15 ] * a[ 19 ];
      tmp[ 209 ] += a[ 12 ] * a[ 15 ] * a[ 23 ];
      tmp[ 210 ] += a[ 12 ] * a[ 15 ] * a[ 24 ];
      tmp[ 211 ] += a[ 12 ] * a[ 16 ] * a[ 19 ];
      tmp[ 212 ] += a[ 12 ] * a[ 16 ] * a[ 24 ];
      tmp[ 213 ] += a[ 12 ] * a[ 18 ] * a[ 20 ];
      tmp[ 214 ] += a[ 12 ] * a[ 19 ] * a[ 20 ];
      tmp[ 215 ] += a[ 12 ] * a[ 19 ] * a[ 21 ];
      tmp[ 216 ] += a[ 12 ] * a[ 20 ] * a[ 23 ];
      tmp[ 217 ] += a[ 12 ] * a[ 20 ] * a[ 24 ];
      tmp[ 218 ] += a[ 12 ] * a[ 21 ] * a[ 24 ];
    }    
  }
  
  /*** normalize ***/  
#ifdef ENABLE_NORMALIZATION
  double normalVal = (double)( 255 * ( right - left ) * ( down - up ) );
  result[ 0 ] = tmp[ 0 ] / normalVal;
  normalVal *= 255.0;
  for(int i=1; i<DIM_OF_GRAY_HLAC1_5; ++i){
    result[ i ] = tmp[ i ] / normalVal;
  }
  normalVal *= 255.0;
  for(int i=DIM_OF_GRAY_HLAC1_5; i<DIM_OF_GRAY_HLAC2_5; ++i){
    result[ i ] = tmp[ i ] / normalVal;
  }
#else /* !ENABLE_NORMALIZATION */
  result[ 0 ] = tmp[ 0 ] / 255.0;
  for(int i=1; i<DIM_OF_GRAY_HLAC1_5; ++i){
    result[ i ] = tmp[ i ] / 65025.0;
  }
  for(int i=DIM_OF_GRAY_HLAC1_5; i<DIM_OF_GRAY_HLAC2_5; ++i){
    result[ i ] = tmp[ i ] / 16581375.0;
  }
#endif /* ENABLE_NORMALIZATION */
}

/*** 高々3次の gray HLAC, square size = 5 ************************************/
#ifdef ENABLE_GRAY_HLAC3_5
 
void HLAC::_extractGray3_5( std::vector<float> &result, const cv::Mat &img,
			       const int &rx, const int &ry, 
			       const int &left, const int &right,
			       const int &up, const int &down )
{
  double a[ 25 ];
  const int width_step = img.step;
  
  /*** initialize ***/
  double tmp[ DIM_OF_GRAY_HLAC3_5 ];
  for(int i=0; i<DIM_OF_GRAY_HLAC3_5; ++i) tmp[ i ] = 0.0;
  
  /*** feature extraction ***/
  for(int j=up; j<down; ++j){
    for(int i=left; i<right; ++i){
      a[  0 ] = (double)img.data[ ( i - 2*rx ) + ( j - 2*ry ) * width_step ];
      a[  1 ] = (double)img.data[ ( i -   rx ) + ( j - 2*ry ) * width_step ];
      a[  2 ] = (double)img.data[   i          + ( j - 2*ry ) * width_step ];
      a[  3 ] = (double)img.data[ ( i +   rx ) + ( j - 2*ry ) * width_step ];
      a[  4 ] = (double)img.data[ ( i + 2*rx ) + ( j - 2*ry ) * width_step ];
      a[  5 ] = (double)img.data[ ( i - 2*rx ) + ( j -   ry ) * width_step ];
      a[  6 ] = (double)img.data[ ( i -   rx ) + ( j -   ry ) * width_step ];
      a[  7 ] = (double)img.data[   i          + ( j -   ry ) * width_step ];
      a[  8 ] = (double)img.data[ ( i +   rx ) + ( j -   ry ) * width_step ];
      a[  9 ] = (double)img.data[ ( i + 2*rx ) + ( j -   ry ) * width_step ];
      a[ 10 ] = (double)img.data[ ( i - 2*rx ) +   j          * width_step ];
      a[ 11 ] = (double)img.data[ ( i -   rx ) +   j          * width_step ];
      a[ 12 ] = (double)img.data[   i          +   j          * width_step ];
      a[ 13 ] = (double)img.data[ ( i +   rx ) +   j          * width_step ];
      a[ 14 ] = (double)img.data[ ( i + 2*rx ) +   j          * width_step ];
      a[ 15 ] = (double)img.data[ ( i - 2*rx ) + ( j +   ry ) * width_step ];
      a[ 16 ] = (double)img.data[ ( i -   rx ) + ( j +   ry ) * width_step ];
      a[ 17 ] = (double)img.data[   i          + ( j +   ry ) * width_step ];
      a[ 18 ] = (double)img.data[ ( i +   rx ) + ( j +   ry ) * width_step ];
      a[ 19 ] = (double)img.data[ ( i + 2*rx ) + ( j +   ry ) * width_step ];
      a[ 20 ] = (double)img.data[ ( i - 2*rx ) + ( j + 2*ry ) * width_step ];
      a[ 21 ] = (double)img.data[ ( i -   rx ) + ( j + 2*ry ) * width_step ];
      a[ 22 ] = (double)img.data[   i          + ( j + 2*ry ) * width_step ];
      a[ 23 ] = (double)img.data[ ( i +   rx ) + ( j + 2*ry ) * width_step ];
      a[ 24 ] = (double)img.data[ ( i + 2*rx ) + ( j + 2*ry ) * width_step ];
      tmp[ 0 ] += a[ 12 ];
      tmp[ 1 ] += a[ 12 ] * a[ 0 ];
      tmp[ 2 ] += a[ 12 ] * a[ 1 ];
      tmp[ 3 ] += a[ 12 ] * a[ 2 ];
      tmp[ 4 ] += a[ 12 ] * a[ 3 ];
      tmp[ 5 ] += a[ 12 ] * a[ 4 ];
      tmp[ 6 ] += a[ 12 ] * a[ 5 ];
      tmp[ 7 ] += a[ 12 ] * a[ 6 ];
      tmp[ 8 ] += a[ 12 ] * a[ 7 ];
      tmp[ 9 ] += a[ 12 ] * a[ 8 ];
      tmp[ 10 ] += a[ 12 ] * a[ 9 ];
      tmp[ 11 ] += a[ 12 ] * a[ 10 ];
      tmp[ 12 ] += a[ 12 ] * a[ 11 ];
      tmp[ 13 ] += a[ 12 ] * a[ 12 ];
      tmp[ 14 ] += a[ 12 ] * a[ 0 ] * a[ 0 ];
      tmp[ 15 ] += a[ 12 ] * a[ 0 ] * a[ 1 ];
      tmp[ 16 ] += a[ 12 ] * a[ 0 ] * a[ 2 ];
      tmp[ 17 ] += a[ 12 ] * a[ 0 ] * a[ 3 ];
      tmp[ 18 ] += a[ 12 ] * a[ 0 ] * a[ 4 ];
      tmp[ 19 ] += a[ 12 ] * a[ 0 ] * a[ 5 ];
      tmp[ 20 ] += a[ 12 ] * a[ 0 ] * a[ 6 ];
      tmp[ 21 ] += a[ 12 ] * a[ 0 ] * a[ 7 ];
      tmp[ 22 ] += a[ 12 ] * a[ 0 ] * a[ 8 ];
      tmp[ 23 ] += a[ 12 ] * a[ 0 ] * a[ 9 ];
      tmp[ 24 ] += a[ 12 ] * a[ 0 ] * a[ 10 ];
      tmp[ 25 ] += a[ 12 ] * a[ 0 ] * a[ 11 ];
      tmp[ 26 ] += a[ 12 ] * a[ 0 ] * a[ 12 ];
      tmp[ 27 ] += a[ 12 ] * a[ 0 ] * a[ 13 ];
      tmp[ 28 ] += a[ 12 ] * a[ 0 ] * a[ 14 ];
      tmp[ 29 ] += a[ 12 ] * a[ 0 ] * a[ 15 ];
      tmp[ 30 ] += a[ 12 ] * a[ 0 ] * a[ 16 ];
      tmp[ 31 ] += a[ 12 ] * a[ 0 ] * a[ 17 ];
      tmp[ 32 ] += a[ 12 ] * a[ 0 ] * a[ 18 ];
      tmp[ 33 ] += a[ 12 ] * a[ 0 ] * a[ 19 ];
      tmp[ 34 ] += a[ 12 ] * a[ 0 ] * a[ 20 ];
      tmp[ 35 ] += a[ 12 ] * a[ 0 ] * a[ 21 ];
      tmp[ 36 ] += a[ 12 ] * a[ 0 ] * a[ 22 ];
      tmp[ 37 ] += a[ 12 ] * a[ 0 ] * a[ 23 ];
      tmp[ 38 ] += a[ 12 ] * a[ 0 ] * a[ 24 ];
      tmp[ 39 ] += a[ 12 ] * a[ 1 ] * a[ 1 ];
      tmp[ 40 ] += a[ 12 ] * a[ 1 ] * a[ 2 ];
      tmp[ 41 ] += a[ 12 ] * a[ 1 ] * a[ 3 ];
      tmp[ 42 ] += a[ 12 ] * a[ 1 ] * a[ 4 ];
      tmp[ 43 ] += a[ 12 ] * a[ 1 ] * a[ 5 ];
      tmp[ 44 ] += a[ 12 ] * a[ 1 ] * a[ 6 ];
      tmp[ 45 ] += a[ 12 ] * a[ 1 ] * a[ 7 ];
      tmp[ 46 ] += a[ 12 ] * a[ 1 ] * a[ 8 ];
      tmp[ 47 ] += a[ 12 ] * a[ 1 ] * a[ 9 ];
      tmp[ 48 ] += a[ 12 ] * a[ 1 ] * a[ 10 ];
      tmp[ 49 ] += a[ 12 ] * a[ 1 ] * a[ 11 ];
      tmp[ 50 ] += a[ 12 ] * a[ 1 ] * a[ 12 ];
      tmp[ 51 ] += a[ 12 ] * a[ 1 ] * a[ 14 ];
      tmp[ 52 ] += a[ 12 ] * a[ 1 ] * a[ 15 ];
      tmp[ 53 ] += a[ 12 ] * a[ 1 ] * a[ 16 ];
      tmp[ 54 ] += a[ 12 ] * a[ 1 ] * a[ 17 ];
      tmp[ 55 ] += a[ 12 ] * a[ 1 ] * a[ 18 ];
      tmp[ 56 ] += a[ 12 ] * a[ 1 ] * a[ 19 ];
      tmp[ 57 ] += a[ 12 ] * a[ 1 ] * a[ 20 ];
      tmp[ 58 ] += a[ 12 ] * a[ 1 ] * a[ 21 ];
      tmp[ 59 ] += a[ 12 ] * a[ 1 ] * a[ 22 ];
      tmp[ 60 ] += a[ 12 ] * a[ 1 ] * a[ 23 ];
      tmp[ 61 ] += a[ 12 ] * a[ 1 ] * a[ 24 ];
      tmp[ 62 ] += a[ 12 ] * a[ 2 ] * a[ 2 ];
      tmp[ 63 ] += a[ 12 ] * a[ 2 ] * a[ 3 ];
      tmp[ 64 ] += a[ 12 ] * a[ 2 ] * a[ 4 ];
      tmp[ 65 ] += a[ 12 ] * a[ 2 ] * a[ 5 ];
      tmp[ 66 ] += a[ 12 ] * a[ 2 ] * a[ 6 ];
      tmp[ 67 ] += a[ 12 ] * a[ 2 ] * a[ 7 ];
      tmp[ 68 ] += a[ 12 ] * a[ 2 ] * a[ 8 ];
      tmp[ 69 ] += a[ 12 ] * a[ 2 ] * a[ 9 ];
      tmp[ 70 ] += a[ 12 ] * a[ 2 ] * a[ 10 ];
      tmp[ 71 ] += a[ 12 ] * a[ 2 ] * a[ 11 ];
      tmp[ 72 ] += a[ 12 ] * a[ 2 ] * a[ 12 ];
      tmp[ 73 ] += a[ 12 ] * a[ 2 ] * a[ 15 ];
      tmp[ 74 ] += a[ 12 ] * a[ 2 ] * a[ 16 ];
      tmp[ 75 ] += a[ 12 ] * a[ 2 ] * a[ 17 ];
      tmp[ 76 ] += a[ 12 ] * a[ 2 ] * a[ 18 ];
      tmp[ 77 ] += a[ 12 ] * a[ 2 ] * a[ 19 ];
      tmp[ 78 ] += a[ 12 ] * a[ 2 ] * a[ 20 ];
      tmp[ 79 ] += a[ 12 ] * a[ 2 ] * a[ 21 ];
      tmp[ 80 ] += a[ 12 ] * a[ 2 ] * a[ 22 ];
      tmp[ 81 ] += a[ 12 ] * a[ 2 ] * a[ 23 ];
      tmp[ 82 ] += a[ 12 ] * a[ 2 ] * a[ 24 ];
      tmp[ 83 ] += a[ 12 ] * a[ 3 ] * a[ 3 ];
      tmp[ 84 ] += a[ 12 ] * a[ 3 ] * a[ 4 ];
      tmp[ 85 ] += a[ 12 ] * a[ 3 ] * a[ 5 ];
      tmp[ 86 ] += a[ 12 ] * a[ 3 ] * a[ 6 ];
      tmp[ 87 ] += a[ 12 ] * a[ 3 ] * a[ 7 ];
      tmp[ 88 ] += a[ 12 ] * a[ 3 ] * a[ 8 ];
      tmp[ 89 ] += a[ 12 ] * a[ 3 ] * a[ 9 ];
      tmp[ 90 ] += a[ 12 ] * a[ 3 ] * a[ 10 ];
      tmp[ 91 ] += a[ 12 ] * a[ 3 ] * a[ 11 ];
      tmp[ 92 ] += a[ 12 ] * a[ 3 ] * a[ 12 ];
      tmp[ 93 ] += a[ 12 ] * a[ 3 ] * a[ 15 ];
      tmp[ 94 ] += a[ 12 ] * a[ 3 ] * a[ 16 ];
      tmp[ 95 ] += a[ 12 ] * a[ 3 ] * a[ 17 ];
      tmp[ 96 ] += a[ 12 ] * a[ 3 ] * a[ 18 ];
      tmp[ 97 ] += a[ 12 ] * a[ 3 ] * a[ 19 ];
      tmp[ 98 ] += a[ 12 ] * a[ 3 ] * a[ 20 ];
      tmp[ 99 ] += a[ 12 ] * a[ 3 ] * a[ 21 ];
      tmp[ 100 ] += a[ 12 ] * a[ 3 ] * a[ 22 ];
      tmp[ 101 ] += a[ 12 ] * a[ 3 ] * a[ 23 ];
      tmp[ 102 ] += a[ 12 ] * a[ 3 ] * a[ 24 ];
      tmp[ 103 ] += a[ 12 ] * a[ 4 ] * a[ 4 ];
      tmp[ 104 ] += a[ 12 ] * a[ 4 ] * a[ 5 ];
      tmp[ 105 ] += a[ 12 ] * a[ 4 ] * a[ 6 ];
      tmp[ 106 ] += a[ 12 ] * a[ 4 ] * a[ 7 ];
      tmp[ 107 ] += a[ 12 ] * a[ 4 ] * a[ 8 ];
      tmp[ 108 ] += a[ 12 ] * a[ 4 ] * a[ 9 ];
      tmp[ 109 ] += a[ 12 ] * a[ 4 ] * a[ 10 ];
      tmp[ 110 ] += a[ 12 ] * a[ 4 ] * a[ 11 ];
      tmp[ 111 ] += a[ 12 ] * a[ 4 ] * a[ 12 ];
      tmp[ 112 ] += a[ 12 ] * a[ 4 ] * a[ 15 ];
      tmp[ 113 ] += a[ 12 ] * a[ 4 ] * a[ 16 ];
      tmp[ 114 ] += a[ 12 ] * a[ 4 ] * a[ 17 ];
      tmp[ 115 ] += a[ 12 ] * a[ 4 ] * a[ 18 ];
      tmp[ 116 ] += a[ 12 ] * a[ 4 ] * a[ 19 ];
      tmp[ 117 ] += a[ 12 ] * a[ 4 ] * a[ 20 ];
      tmp[ 118 ] += a[ 12 ] * a[ 4 ] * a[ 21 ];
      tmp[ 119 ] += a[ 12 ] * a[ 4 ] * a[ 22 ];
      tmp[ 120 ] += a[ 12 ] * a[ 4 ] * a[ 23 ];
      tmp[ 121 ] += a[ 12 ] * a[ 4 ] * a[ 24 ];
      tmp[ 122 ] += a[ 12 ] * a[ 5 ] * a[ 5 ];
      tmp[ 123 ] += a[ 12 ] * a[ 5 ] * a[ 6 ];
      tmp[ 124 ] += a[ 12 ] * a[ 5 ] * a[ 7 ];
      tmp[ 125 ] += a[ 12 ] * a[ 5 ] * a[ 8 ];
      tmp[ 126 ] += a[ 12 ] * a[ 5 ] * a[ 9 ];
      tmp[ 127 ] += a[ 12 ] * a[ 5 ] * a[ 10 ];
      tmp[ 128 ] += a[ 12 ] * a[ 5 ] * a[ 11 ];
      tmp[ 129 ] += a[ 12 ] * a[ 5 ] * a[ 12 ];
      tmp[ 130 ] += a[ 12 ] * a[ 5 ] * a[ 13 ];
      tmp[ 131 ] += a[ 12 ] * a[ 5 ] * a[ 14 ];
      tmp[ 132 ] += a[ 12 ] * a[ 5 ] * a[ 18 ];
      tmp[ 133 ] += a[ 12 ] * a[ 5 ] * a[ 19 ];
      tmp[ 134 ] += a[ 12 ] * a[ 5 ] * a[ 20 ];
      tmp[ 135 ] += a[ 12 ] * a[ 5 ] * a[ 21 ];
      tmp[ 136 ] += a[ 12 ] * a[ 5 ] * a[ 22 ];
      tmp[ 137 ] += a[ 12 ] * a[ 5 ] * a[ 23 ];
      tmp[ 138 ] += a[ 12 ] * a[ 5 ] * a[ 24 ];
      tmp[ 139 ] += a[ 12 ] * a[ 6 ] * a[ 6 ];
      tmp[ 140 ] += a[ 12 ] * a[ 6 ] * a[ 7 ];
      tmp[ 141 ] += a[ 12 ] * a[ 6 ] * a[ 8 ];
      tmp[ 142 ] += a[ 12 ] * a[ 6 ] * a[ 9 ];
      tmp[ 143 ] += a[ 12 ] * a[ 6 ] * a[ 10 ];
      tmp[ 144 ] += a[ 12 ] * a[ 6 ] * a[ 11 ];
      tmp[ 145 ] += a[ 12 ] * a[ 6 ] * a[ 12 ];
      tmp[ 146 ] += a[ 12 ] * a[ 6 ] * a[ 14 ];
      tmp[ 147 ] += a[ 12 ] * a[ 6 ] * a[ 19 ];
      tmp[ 148 ] += a[ 12 ] * a[ 6 ] * a[ 20 ];
      tmp[ 149 ] += a[ 12 ] * a[ 6 ] * a[ 21 ];
      tmp[ 150 ] += a[ 12 ] * a[ 6 ] * a[ 22 ];
      tmp[ 151 ] += a[ 12 ] * a[ 6 ] * a[ 23 ];
      tmp[ 152 ] += a[ 12 ] * a[ 6 ] * a[ 24 ];
      tmp[ 153 ] += a[ 12 ] * a[ 7 ] * a[ 7 ];
      tmp[ 154 ] += a[ 12 ] * a[ 7 ] * a[ 8 ];
      tmp[ 155 ] += a[ 12 ] * a[ 7 ] * a[ 9 ];
      tmp[ 156 ] += a[ 12 ] * a[ 7 ] * a[ 10 ];
      tmp[ 157 ] += a[ 12 ] * a[ 7 ] * a[ 11 ];
      tmp[ 158 ] += a[ 12 ] * a[ 7 ] * a[ 12 ];
      tmp[ 159 ] += a[ 12 ] * a[ 7 ] * a[ 20 ];
      tmp[ 160 ] += a[ 12 ] * a[ 7 ] * a[ 21 ];
      tmp[ 161 ] += a[ 12 ] * a[ 7 ] * a[ 22 ];
      tmp[ 162 ] += a[ 12 ] * a[ 7 ] * a[ 23 ];
      tmp[ 163 ] += a[ 12 ] * a[ 7 ] * a[ 24 ];
      tmp[ 164 ] += a[ 12 ] * a[ 8 ] * a[ 8 ];
      tmp[ 165 ] += a[ 12 ] * a[ 8 ] * a[ 9 ];
      tmp[ 166 ] += a[ 12 ] * a[ 8 ] * a[ 10 ];
      tmp[ 167 ] += a[ 12 ] * a[ 8 ] * a[ 11 ];
      tmp[ 168 ] += a[ 12 ] * a[ 8 ] * a[ 12 ];
      tmp[ 169 ] += a[ 12 ] * a[ 8 ] * a[ 15 ];
      tmp[ 170 ] += a[ 12 ] * a[ 8 ] * a[ 20 ];
      tmp[ 171 ] += a[ 12 ] * a[ 8 ] * a[ 21 ];
      tmp[ 172 ] += a[ 12 ] * a[ 8 ] * a[ 22 ];
      tmp[ 173 ] += a[ 12 ] * a[ 8 ] * a[ 23 ];
      tmp[ 174 ] += a[ 12 ] * a[ 8 ] * a[ 24 ];
      tmp[ 175 ] += a[ 12 ] * a[ 9 ] * a[ 9 ];
      tmp[ 176 ] += a[ 12 ] * a[ 9 ] * a[ 10 ];
      tmp[ 177 ] += a[ 12 ] * a[ 9 ] * a[ 11 ];
      tmp[ 178 ] += a[ 12 ] * a[ 9 ] * a[ 12 ];
      tmp[ 179 ] += a[ 12 ] * a[ 9 ] * a[ 15 ];
      tmp[ 180 ] += a[ 12 ] * a[ 9 ] * a[ 16 ];
      tmp[ 181 ] += a[ 12 ] * a[ 9 ] * a[ 20 ];
      tmp[ 182 ] += a[ 12 ] * a[ 9 ] * a[ 21 ];
      tmp[ 183 ] += a[ 12 ] * a[ 9 ] * a[ 22 ];
      tmp[ 184 ] += a[ 12 ] * a[ 9 ] * a[ 23 ];
      tmp[ 185 ] += a[ 12 ] * a[ 9 ] * a[ 24 ];
      tmp[ 186 ] += a[ 12 ] * a[ 10 ] * a[ 10 ];
      tmp[ 187 ] += a[ 12 ] * a[ 10 ] * a[ 11 ];
      tmp[ 188 ] += a[ 12 ] * a[ 10 ] * a[ 12 ];
      tmp[ 189 ] += a[ 12 ] * a[ 10 ] * a[ 13 ];
      tmp[ 190 ] += a[ 12 ] * a[ 10 ] * a[ 14 ];
      tmp[ 191 ] += a[ 12 ] * a[ 10 ] * a[ 18 ];
      tmp[ 192 ] += a[ 12 ] * a[ 10 ] * a[ 19 ];
      tmp[ 193 ] += a[ 12 ] * a[ 10 ] * a[ 23 ];
      tmp[ 194 ] += a[ 12 ] * a[ 10 ] * a[ 24 ];
      tmp[ 195 ] += a[ 12 ] * a[ 11 ] * a[ 11 ];
      tmp[ 196 ] += a[ 12 ] * a[ 11 ] * a[ 12 ];
      tmp[ 197 ] += a[ 12 ] * a[ 11 ] * a[ 14 ];
      tmp[ 198 ] += a[ 12 ] * a[ 11 ] * a[ 19 ];
      tmp[ 199 ] += a[ 12 ] * a[ 11 ] * a[ 24 ];
      tmp[ 200 ] += a[ 12 ] * a[ 12 ] * a[ 12 ];
      tmp[ 201 ] += a[ 12 ] * a[ 13 ] * a[ 15 ];
      tmp[ 202 ] += a[ 12 ] * a[ 13 ] * a[ 20 ];
      tmp[ 203 ] += a[ 12 ] * a[ 14 ] * a[ 15 ];
      tmp[ 204 ] += a[ 12 ] * a[ 14 ] * a[ 16 ];
      tmp[ 205 ] += a[ 12 ] * a[ 14 ] * a[ 20 ];
      tmp[ 206 ] += a[ 12 ] * a[ 14 ] * a[ 21 ];
      tmp[ 207 ] += a[ 12 ] * a[ 15 ] * a[ 18 ];
      tmp[ 208 ] += a[ 12 ] * a[ 15 ] * a[ 19 ];
      tmp[ 209 ] += a[ 12 ] * a[ 15 ] * a[ 23 ];
      tmp[ 210 ] += a[ 12 ] * a[ 15 ] * a[ 24 ];
      tmp[ 211 ] += a[ 12 ] * a[ 16 ] * a[ 19 ];
      tmp[ 212 ] += a[ 12 ] * a[ 16 ] * a[ 24 ];
      tmp[ 213 ] += a[ 12 ] * a[ 18 ] * a[ 20 ];
      tmp[ 214 ] += a[ 12 ] * a[ 19 ] * a[ 20 ];
      tmp[ 215 ] += a[ 12 ] * a[ 19 ] * a[ 21 ];
      tmp[ 216 ] += a[ 12 ] * a[ 20 ] * a[ 23 ];
      tmp[ 217 ] += a[ 12 ] * a[ 20 ] * a[ 24 ];
      tmp[ 218 ] += a[ 12 ] * a[ 21 ] * a[ 24 ];
      tmp[ 219 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 0 ];
      tmp[ 220 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 1 ];
      tmp[ 221 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 2 ];
      tmp[ 222 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 3 ];
      tmp[ 223 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 4 ];
      tmp[ 224 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 5 ];
      tmp[ 225 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 6 ];
      tmp[ 226 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 7 ];
      tmp[ 227 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 8 ];
      tmp[ 228 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 9 ];
      tmp[ 229 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 10 ];
      tmp[ 230 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 11 ];
      tmp[ 231 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 12 ];
      tmp[ 232 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 13 ];
      tmp[ 233 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 14 ];
      tmp[ 234 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 15 ];
      tmp[ 235 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 16 ];
      tmp[ 236 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 17 ];
      tmp[ 237 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 18 ];
      tmp[ 238 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 19 ];
      tmp[ 239 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 20 ];
      tmp[ 240 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 21 ];
      tmp[ 241 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 22 ];
      tmp[ 242 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 23 ];
      tmp[ 243 ] += a[ 12 ] * a[ 0 ] * a[ 0 ] * a[ 24 ];
      tmp[ 244 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 1 ];
      tmp[ 245 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 2 ];
      tmp[ 246 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 3 ];
      tmp[ 247 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 4 ];
      tmp[ 248 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 5 ];
      tmp[ 249 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 6 ];
      tmp[ 250 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 7 ];
      tmp[ 251 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 8 ];
      tmp[ 252 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 9 ];
      tmp[ 253 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 10 ];
      tmp[ 254 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 11 ];
      tmp[ 255 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 12 ];
      tmp[ 256 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 13 ];
      tmp[ 257 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 14 ];
      tmp[ 258 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 15 ];
      tmp[ 259 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 16 ];
      tmp[ 260 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 17 ];
      tmp[ 261 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 18 ];
      tmp[ 262 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 19 ];
      tmp[ 263 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 20 ];
      tmp[ 264 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 21 ];
      tmp[ 265 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 22 ];
      tmp[ 266 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 23 ];
      tmp[ 267 ] += a[ 12 ] * a[ 0 ] * a[ 1 ] * a[ 24 ];
      tmp[ 268 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 2 ];
      tmp[ 269 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 3 ];
      tmp[ 270 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 4 ];
      tmp[ 271 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 5 ];
      tmp[ 272 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 6 ];
      tmp[ 273 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 7 ];
      tmp[ 274 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 8 ];
      tmp[ 275 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 9 ];
      tmp[ 276 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 10 ];
      tmp[ 277 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 11 ];
      tmp[ 278 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 12 ];
      tmp[ 279 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 13 ];
      tmp[ 280 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 14 ];
      tmp[ 281 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 15 ];
      tmp[ 282 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 16 ];
      tmp[ 283 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 17 ];
      tmp[ 284 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 18 ];
      tmp[ 285 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 19 ];
      tmp[ 286 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 20 ];
      tmp[ 287 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 21 ];
      tmp[ 288 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 22 ];
      tmp[ 289 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 23 ];
      tmp[ 290 ] += a[ 12 ] * a[ 0 ] * a[ 2 ] * a[ 24 ];
      tmp[ 291 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 3 ];
      tmp[ 292 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 4 ];
      tmp[ 293 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 5 ];
      tmp[ 294 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 6 ];
      tmp[ 295 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 7 ];
      tmp[ 296 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 8 ];
      tmp[ 297 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 9 ];
      tmp[ 298 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 10 ];
      tmp[ 299 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 11 ];
      tmp[ 300 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 12 ];
      tmp[ 301 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 13 ];
      tmp[ 302 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 14 ];
      tmp[ 303 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 15 ];
      tmp[ 304 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 16 ];
      tmp[ 305 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 17 ];
      tmp[ 306 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 18 ];
      tmp[ 307 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 19 ];
      tmp[ 308 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 20 ];
      tmp[ 309 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 21 ];
      tmp[ 310 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 22 ];
      tmp[ 311 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 23 ];
      tmp[ 312 ] += a[ 12 ] * a[ 0 ] * a[ 3 ] * a[ 24 ];
      tmp[ 313 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 4 ];
      tmp[ 314 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 5 ];
      tmp[ 315 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 6 ];
      tmp[ 316 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 7 ];
      tmp[ 317 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 8 ];
      tmp[ 318 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 9 ];
      tmp[ 319 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 10 ];
      tmp[ 320 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 11 ];
      tmp[ 321 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 12 ];
      tmp[ 322 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 13 ];
      tmp[ 323 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 14 ];
      tmp[ 324 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 15 ];
      tmp[ 325 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 16 ];
      tmp[ 326 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 17 ];
      tmp[ 327 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 18 ];
      tmp[ 328 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 19 ];
      tmp[ 329 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 20 ];
      tmp[ 330 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 21 ];
      tmp[ 331 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 22 ];
      tmp[ 332 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 23 ];
      tmp[ 333 ] += a[ 12 ] * a[ 0 ] * a[ 4 ] * a[ 24 ];
      tmp[ 334 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 5 ];
      tmp[ 335 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 6 ];
      tmp[ 336 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 7 ];
      tmp[ 337 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 8 ];
      tmp[ 338 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 9 ];
      tmp[ 339 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 10 ];
      tmp[ 340 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 11 ];
      tmp[ 341 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 12 ];
      tmp[ 342 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 13 ];
      tmp[ 343 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 14 ];
      tmp[ 344 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 15 ];
      tmp[ 345 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 16 ];
      tmp[ 346 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 17 ];
      tmp[ 347 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 18 ];
      tmp[ 348 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 19 ];
      tmp[ 349 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 20 ];
      tmp[ 350 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 21 ];
      tmp[ 351 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 22 ];
      tmp[ 352 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 23 ];
      tmp[ 353 ] += a[ 12 ] * a[ 0 ] * a[ 5 ] * a[ 24 ];
      tmp[ 354 ] += a[ 12 ] * a[ 0 ] * a[ 6 ] * a[ 6 ];
      tmp[ 355 ] += a[ 12 ] * a[ 0 ] * a[ 6 ] * a[ 7 ];
      tmp[ 356 ] += a[ 12 ] * a[ 0 ] * a[ 6 ] * a[ 8 ];
      tmp[ 357 ] += a[ 12 ] * a[ 0 ] * a[ 6 ] * a[ 9 ];
      tmp[ 358 ] += a[ 12 ] * a[ 0 ] * a[ 6 ] * a[ 10 ];
      tmp[ 359 ] += a[ 12 ] * a[ 0 ] * a[ 6 ] * a[ 11 ];
      tmp[ 360 ] += a[ 12 ] * a[ 0 ] * a[ 6 ] * a[ 12 ];
      tmp[ 361 ] += a[ 12 ] * a[ 0 ] * a[ 6 ] * a[ 13 ];
      tmp[ 362 ] += a[ 12 ] * a[ 0 ] * a[ 6 ] * a[ 14 ];
      tmp[ 363 ] += a[ 12 ] * a[ 0 ] * a[ 6 ] * a[ 15 ];
      tmp[ 364 ] += a[ 12 ] * a[ 0 ] * a[ 6 ] * a[ 16 ];
      tmp[ 365 ] += a[ 12 ] * a[ 0 ] * a[ 6 ] * a[ 17 ];
      tmp[ 366 ] += a[ 12 ] * a[ 0 ] * a[ 6 ] * a[ 18 ];
      tmp[ 367 ] += a[ 12 ] * a[ 0 ] * a[ 6 ] * a[ 19 ];
      tmp[ 368 ] += a[ 12 ] * a[ 0 ] * a[ 6 ] * a[ 20 ];
      tmp[ 369 ] += a[ 12 ] * a[ 0 ] * a[ 6 ] * a[ 21 ];
      tmp[ 370 ] += a[ 12 ] * a[ 0 ] * a[ 6 ] * a[ 22 ];
      tmp[ 371 ] += a[ 12 ] * a[ 0 ] * a[ 6 ] * a[ 23 ];
      tmp[ 372 ] += a[ 12 ] * a[ 0 ] * a[ 6 ] * a[ 24 ];
      tmp[ 373 ] += a[ 12 ] * a[ 0 ] * a[ 7 ] * a[ 7 ];
      tmp[ 374 ] += a[ 12 ] * a[ 0 ] * a[ 7 ] * a[ 8 ];
      tmp[ 375 ] += a[ 12 ] * a[ 0 ] * a[ 7 ] * a[ 9 ];
      tmp[ 376 ] += a[ 12 ] * a[ 0 ] * a[ 7 ] * a[ 10 ];
      tmp[ 377 ] += a[ 12 ] * a[ 0 ] * a[ 7 ] * a[ 11 ];
      tmp[ 378 ] += a[ 12 ] * a[ 0 ] * a[ 7 ] * a[ 12 ];
      tmp[ 379 ] += a[ 12 ] * a[ 0 ] * a[ 7 ] * a[ 13 ];
      tmp[ 380 ] += a[ 12 ] * a[ 0 ] * a[ 7 ] * a[ 14 ];
      tmp[ 381 ] += a[ 12 ] * a[ 0 ] * a[ 7 ] * a[ 15 ];
      tmp[ 382 ] += a[ 12 ] * a[ 0 ] * a[ 7 ] * a[ 16 ];
      tmp[ 383 ] += a[ 12 ] * a[ 0 ] * a[ 7 ] * a[ 17 ];
      tmp[ 384 ] += a[ 12 ] * a[ 0 ] * a[ 7 ] * a[ 18 ];
      tmp[ 385 ] += a[ 12 ] * a[ 0 ] * a[ 7 ] * a[ 19 ];
      tmp[ 386 ] += a[ 12 ] * a[ 0 ] * a[ 7 ] * a[ 20 ];
      tmp[ 387 ] += a[ 12 ] * a[ 0 ] * a[ 7 ] * a[ 21 ];
      tmp[ 388 ] += a[ 12 ] * a[ 0 ] * a[ 7 ] * a[ 22 ];
      tmp[ 389 ] += a[ 12 ] * a[ 0 ] * a[ 7 ] * a[ 23 ];
      tmp[ 390 ] += a[ 12 ] * a[ 0 ] * a[ 7 ] * a[ 24 ];
      tmp[ 391 ] += a[ 12 ] * a[ 0 ] * a[ 8 ] * a[ 8 ];
      tmp[ 392 ] += a[ 12 ] * a[ 0 ] * a[ 8 ] * a[ 9 ];
      tmp[ 393 ] += a[ 12 ] * a[ 0 ] * a[ 8 ] * a[ 10 ];
      tmp[ 394 ] += a[ 12 ] * a[ 0 ] * a[ 8 ] * a[ 11 ];
      tmp[ 395 ] += a[ 12 ] * a[ 0 ] * a[ 8 ] * a[ 12 ];
      tmp[ 396 ] += a[ 12 ] * a[ 0 ] * a[ 8 ] * a[ 13 ];
      tmp[ 397 ] += a[ 12 ] * a[ 0 ] * a[ 8 ] * a[ 14 ];
      tmp[ 398 ] += a[ 12 ] * a[ 0 ] * a[ 8 ] * a[ 15 ];
      tmp[ 399 ] += a[ 12 ] * a[ 0 ] * a[ 8 ] * a[ 16 ];
      tmp[ 400 ] += a[ 12 ] * a[ 0 ] * a[ 8 ] * a[ 17 ];
      tmp[ 401 ] += a[ 12 ] * a[ 0 ] * a[ 8 ] * a[ 18 ];
      tmp[ 402 ] += a[ 12 ] * a[ 0 ] * a[ 8 ] * a[ 19 ];
      tmp[ 403 ] += a[ 12 ] * a[ 0 ] * a[ 8 ] * a[ 20 ];
      tmp[ 404 ] += a[ 12 ] * a[ 0 ] * a[ 8 ] * a[ 21 ];
      tmp[ 405 ] += a[ 12 ] * a[ 0 ] * a[ 8 ] * a[ 22 ];
      tmp[ 406 ] += a[ 12 ] * a[ 0 ] * a[ 8 ] * a[ 23 ];
      tmp[ 407 ] += a[ 12 ] * a[ 0 ] * a[ 8 ] * a[ 24 ];
      tmp[ 408 ] += a[ 12 ] * a[ 0 ] * a[ 9 ] * a[ 9 ];
      tmp[ 409 ] += a[ 12 ] * a[ 0 ] * a[ 9 ] * a[ 10 ];
      tmp[ 410 ] += a[ 12 ] * a[ 0 ] * a[ 9 ] * a[ 11 ];
      tmp[ 411 ] += a[ 12 ] * a[ 0 ] * a[ 9 ] * a[ 12 ];
      tmp[ 412 ] += a[ 12 ] * a[ 0 ] * a[ 9 ] * a[ 13 ];
      tmp[ 413 ] += a[ 12 ] * a[ 0 ] * a[ 9 ] * a[ 14 ];
      tmp[ 414 ] += a[ 12 ] * a[ 0 ] * a[ 9 ] * a[ 15 ];
      tmp[ 415 ] += a[ 12 ] * a[ 0 ] * a[ 9 ] * a[ 16 ];
      tmp[ 416 ] += a[ 12 ] * a[ 0 ] * a[ 9 ] * a[ 17 ];
      tmp[ 417 ] += a[ 12 ] * a[ 0 ] * a[ 9 ] * a[ 18 ];
      tmp[ 418 ] += a[ 12 ] * a[ 0 ] * a[ 9 ] * a[ 19 ];
      tmp[ 419 ] += a[ 12 ] * a[ 0 ] * a[ 9 ] * a[ 20 ];
      tmp[ 420 ] += a[ 12 ] * a[ 0 ] * a[ 9 ] * a[ 21 ];
      tmp[ 421 ] += a[ 12 ] * a[ 0 ] * a[ 9 ] * a[ 22 ];
      tmp[ 422 ] += a[ 12 ] * a[ 0 ] * a[ 9 ] * a[ 23 ];
      tmp[ 423 ] += a[ 12 ] * a[ 0 ] * a[ 9 ] * a[ 24 ];
      tmp[ 424 ] += a[ 12 ] * a[ 0 ] * a[ 10 ] * a[ 10 ];
      tmp[ 425 ] += a[ 12 ] * a[ 0 ] * a[ 10 ] * a[ 11 ];
      tmp[ 426 ] += a[ 12 ] * a[ 0 ] * a[ 10 ] * a[ 12 ];
      tmp[ 427 ] += a[ 12 ] * a[ 0 ] * a[ 10 ] * a[ 13 ];
      tmp[ 428 ] += a[ 12 ] * a[ 0 ] * a[ 10 ] * a[ 14 ];
      tmp[ 429 ] += a[ 12 ] * a[ 0 ] * a[ 10 ] * a[ 15 ];
      tmp[ 430 ] += a[ 12 ] * a[ 0 ] * a[ 10 ] * a[ 16 ];
      tmp[ 431 ] += a[ 12 ] * a[ 0 ] * a[ 10 ] * a[ 17 ];
      tmp[ 432 ] += a[ 12 ] * a[ 0 ] * a[ 10 ] * a[ 18 ];
      tmp[ 433 ] += a[ 12 ] * a[ 0 ] * a[ 10 ] * a[ 19 ];
      tmp[ 434 ] += a[ 12 ] * a[ 0 ] * a[ 10 ] * a[ 20 ];
      tmp[ 435 ] += a[ 12 ] * a[ 0 ] * a[ 10 ] * a[ 21 ];
      tmp[ 436 ] += a[ 12 ] * a[ 0 ] * a[ 10 ] * a[ 22 ];
      tmp[ 437 ] += a[ 12 ] * a[ 0 ] * a[ 10 ] * a[ 23 ];
      tmp[ 438 ] += a[ 12 ] * a[ 0 ] * a[ 10 ] * a[ 24 ];
      tmp[ 439 ] += a[ 12 ] * a[ 0 ] * a[ 11 ] * a[ 11 ];
      tmp[ 440 ] += a[ 12 ] * a[ 0 ] * a[ 11 ] * a[ 12 ];
      tmp[ 441 ] += a[ 12 ] * a[ 0 ] * a[ 11 ] * a[ 13 ];
      tmp[ 442 ] += a[ 12 ] * a[ 0 ] * a[ 11 ] * a[ 14 ];
      tmp[ 443 ] += a[ 12 ] * a[ 0 ] * a[ 11 ] * a[ 15 ];
      tmp[ 444 ] += a[ 12 ] * a[ 0 ] * a[ 11 ] * a[ 16 ];
      tmp[ 445 ] += a[ 12 ] * a[ 0 ] * a[ 11 ] * a[ 17 ];
      tmp[ 446 ] += a[ 12 ] * a[ 0 ] * a[ 11 ] * a[ 18 ];
      tmp[ 447 ] += a[ 12 ] * a[ 0 ] * a[ 11 ] * a[ 19 ];
      tmp[ 448 ] += a[ 12 ] * a[ 0 ] * a[ 11 ] * a[ 20 ];
      tmp[ 449 ] += a[ 12 ] * a[ 0 ] * a[ 11 ] * a[ 21 ];
      tmp[ 450 ] += a[ 12 ] * a[ 0 ] * a[ 11 ] * a[ 22 ];
      tmp[ 451 ] += a[ 12 ] * a[ 0 ] * a[ 11 ] * a[ 23 ];
      tmp[ 452 ] += a[ 12 ] * a[ 0 ] * a[ 11 ] * a[ 24 ];
      tmp[ 453 ] += a[ 12 ] * a[ 0 ] * a[ 12 ] * a[ 12 ];
      tmp[ 454 ] += a[ 12 ] * a[ 0 ] * a[ 12 ] * a[ 13 ];
      tmp[ 455 ] += a[ 12 ] * a[ 0 ] * a[ 12 ] * a[ 14 ];
      tmp[ 456 ] += a[ 12 ] * a[ 0 ] * a[ 12 ] * a[ 15 ];
      tmp[ 457 ] += a[ 12 ] * a[ 0 ] * a[ 12 ] * a[ 16 ];
      tmp[ 458 ] += a[ 12 ] * a[ 0 ] * a[ 12 ] * a[ 17 ];
      tmp[ 459 ] += a[ 12 ] * a[ 0 ] * a[ 12 ] * a[ 18 ];
      tmp[ 460 ] += a[ 12 ] * a[ 0 ] * a[ 12 ] * a[ 19 ];
      tmp[ 461 ] += a[ 12 ] * a[ 0 ] * a[ 12 ] * a[ 20 ];
      tmp[ 462 ] += a[ 12 ] * a[ 0 ] * a[ 12 ] * a[ 21 ];
      tmp[ 463 ] += a[ 12 ] * a[ 0 ] * a[ 12 ] * a[ 22 ];
      tmp[ 464 ] += a[ 12 ] * a[ 0 ] * a[ 12 ] * a[ 23 ];
      tmp[ 465 ] += a[ 12 ] * a[ 0 ] * a[ 12 ] * a[ 24 ];
      tmp[ 466 ] += a[ 12 ] * a[ 0 ] * a[ 13 ] * a[ 13 ];
      tmp[ 467 ] += a[ 12 ] * a[ 0 ] * a[ 13 ] * a[ 14 ];
      tmp[ 468 ] += a[ 12 ] * a[ 0 ] * a[ 13 ] * a[ 15 ];
      tmp[ 469 ] += a[ 12 ] * a[ 0 ] * a[ 13 ] * a[ 16 ];
      tmp[ 470 ] += a[ 12 ] * a[ 0 ] * a[ 13 ] * a[ 17 ];
      tmp[ 471 ] += a[ 12 ] * a[ 0 ] * a[ 13 ] * a[ 18 ];
      tmp[ 472 ] += a[ 12 ] * a[ 0 ] * a[ 13 ] * a[ 19 ];
      tmp[ 473 ] += a[ 12 ] * a[ 0 ] * a[ 13 ] * a[ 20 ];
      tmp[ 474 ] += a[ 12 ] * a[ 0 ] * a[ 13 ] * a[ 21 ];
      tmp[ 475 ] += a[ 12 ] * a[ 0 ] * a[ 13 ] * a[ 22 ];
      tmp[ 476 ] += a[ 12 ] * a[ 0 ] * a[ 13 ] * a[ 23 ];
      tmp[ 477 ] += a[ 12 ] * a[ 0 ] * a[ 13 ] * a[ 24 ];
      tmp[ 478 ] += a[ 12 ] * a[ 0 ] * a[ 14 ] * a[ 14 ];
      tmp[ 479 ] += a[ 12 ] * a[ 0 ] * a[ 14 ] * a[ 15 ];
      tmp[ 480 ] += a[ 12 ] * a[ 0 ] * a[ 14 ] * a[ 16 ];
      tmp[ 481 ] += a[ 12 ] * a[ 0 ] * a[ 14 ] * a[ 17 ];
      tmp[ 482 ] += a[ 12 ] * a[ 0 ] * a[ 14 ] * a[ 18 ];
      tmp[ 483 ] += a[ 12 ] * a[ 0 ] * a[ 14 ] * a[ 19 ];
      tmp[ 484 ] += a[ 12 ] * a[ 0 ] * a[ 14 ] * a[ 20 ];
      tmp[ 485 ] += a[ 12 ] * a[ 0 ] * a[ 14 ] * a[ 21 ];
      tmp[ 486 ] += a[ 12 ] * a[ 0 ] * a[ 14 ] * a[ 22 ];
      tmp[ 487 ] += a[ 12 ] * a[ 0 ] * a[ 14 ] * a[ 23 ];
      tmp[ 488 ] += a[ 12 ] * a[ 0 ] * a[ 14 ] * a[ 24 ];
      tmp[ 489 ] += a[ 12 ] * a[ 0 ] * a[ 15 ] * a[ 15 ];
      tmp[ 490 ] += a[ 12 ] * a[ 0 ] * a[ 15 ] * a[ 16 ];
      tmp[ 491 ] += a[ 12 ] * a[ 0 ] * a[ 15 ] * a[ 17 ];
      tmp[ 492 ] += a[ 12 ] * a[ 0 ] * a[ 15 ] * a[ 18 ];
      tmp[ 493 ] += a[ 12 ] * a[ 0 ] * a[ 15 ] * a[ 19 ];
      tmp[ 494 ] += a[ 12 ] * a[ 0 ] * a[ 15 ] * a[ 20 ];
      tmp[ 495 ] += a[ 12 ] * a[ 0 ] * a[ 15 ] * a[ 21 ];
      tmp[ 496 ] += a[ 12 ] * a[ 0 ] * a[ 15 ] * a[ 22 ];
      tmp[ 497 ] += a[ 12 ] * a[ 0 ] * a[ 15 ] * a[ 23 ];
      tmp[ 498 ] += a[ 12 ] * a[ 0 ] * a[ 15 ] * a[ 24 ];
      tmp[ 499 ] += a[ 12 ] * a[ 0 ] * a[ 16 ] * a[ 16 ];
      tmp[ 500 ] += a[ 12 ] * a[ 0 ] * a[ 16 ] * a[ 17 ];
      tmp[ 501 ] += a[ 12 ] * a[ 0 ] * a[ 16 ] * a[ 18 ];
      tmp[ 502 ] += a[ 12 ] * a[ 0 ] * a[ 16 ] * a[ 19 ];
      tmp[ 503 ] += a[ 12 ] * a[ 0 ] * a[ 16 ] * a[ 20 ];
      tmp[ 504 ] += a[ 12 ] * a[ 0 ] * a[ 16 ] * a[ 21 ];
      tmp[ 505 ] += a[ 12 ] * a[ 0 ] * a[ 16 ] * a[ 22 ];
      tmp[ 506 ] += a[ 12 ] * a[ 0 ] * a[ 16 ] * a[ 23 ];
      tmp[ 507 ] += a[ 12 ] * a[ 0 ] * a[ 16 ] * a[ 24 ];
      tmp[ 508 ] += a[ 12 ] * a[ 0 ] * a[ 17 ] * a[ 17 ];
      tmp[ 509 ] += a[ 12 ] * a[ 0 ] * a[ 17 ] * a[ 18 ];
      tmp[ 510 ] += a[ 12 ] * a[ 0 ] * a[ 17 ] * a[ 19 ];
      tmp[ 511 ] += a[ 12 ] * a[ 0 ] * a[ 17 ] * a[ 20 ];
      tmp[ 512 ] += a[ 12 ] * a[ 0 ] * a[ 17 ] * a[ 21 ];
      tmp[ 513 ] += a[ 12 ] * a[ 0 ] * a[ 17 ] * a[ 22 ];
      tmp[ 514 ] += a[ 12 ] * a[ 0 ] * a[ 17 ] * a[ 23 ];
      tmp[ 515 ] += a[ 12 ] * a[ 0 ] * a[ 17 ] * a[ 24 ];
      tmp[ 516 ] += a[ 12 ] * a[ 0 ] * a[ 18 ] * a[ 18 ];
      tmp[ 517 ] += a[ 12 ] * a[ 0 ] * a[ 18 ] * a[ 19 ];
      tmp[ 518 ] += a[ 12 ] * a[ 0 ] * a[ 18 ] * a[ 20 ];
      tmp[ 519 ] += a[ 12 ] * a[ 0 ] * a[ 18 ] * a[ 21 ];
      tmp[ 520 ] += a[ 12 ] * a[ 0 ] * a[ 18 ] * a[ 22 ];
      tmp[ 521 ] += a[ 12 ] * a[ 0 ] * a[ 18 ] * a[ 23 ];
      tmp[ 522 ] += a[ 12 ] * a[ 0 ] * a[ 18 ] * a[ 24 ];
      tmp[ 523 ] += a[ 12 ] * a[ 0 ] * a[ 19 ] * a[ 19 ];
      tmp[ 524 ] += a[ 12 ] * a[ 0 ] * a[ 19 ] * a[ 20 ];
      tmp[ 525 ] += a[ 12 ] * a[ 0 ] * a[ 19 ] * a[ 21 ];
      tmp[ 526 ] += a[ 12 ] * a[ 0 ] * a[ 19 ] * a[ 22 ];
      tmp[ 527 ] += a[ 12 ] * a[ 0 ] * a[ 19 ] * a[ 23 ];
      tmp[ 528 ] += a[ 12 ] * a[ 0 ] * a[ 19 ] * a[ 24 ];
      tmp[ 529 ] += a[ 12 ] * a[ 0 ] * a[ 20 ] * a[ 20 ];
      tmp[ 530 ] += a[ 12 ] * a[ 0 ] * a[ 20 ] * a[ 21 ];
      tmp[ 531 ] += a[ 12 ] * a[ 0 ] * a[ 20 ] * a[ 22 ];
      tmp[ 532 ] += a[ 12 ] * a[ 0 ] * a[ 20 ] * a[ 23 ];
      tmp[ 533 ] += a[ 12 ] * a[ 0 ] * a[ 20 ] * a[ 24 ];
      tmp[ 534 ] += a[ 12 ] * a[ 0 ] * a[ 21 ] * a[ 21 ];
      tmp[ 535 ] += a[ 12 ] * a[ 0 ] * a[ 21 ] * a[ 22 ];
      tmp[ 536 ] += a[ 12 ] * a[ 0 ] * a[ 21 ] * a[ 23 ];
      tmp[ 537 ] += a[ 12 ] * a[ 0 ] * a[ 21 ] * a[ 24 ];
      tmp[ 538 ] += a[ 12 ] * a[ 0 ] * a[ 22 ] * a[ 22 ];
      tmp[ 539 ] += a[ 12 ] * a[ 0 ] * a[ 22 ] * a[ 23 ];
      tmp[ 540 ] += a[ 12 ] * a[ 0 ] * a[ 22 ] * a[ 24 ];
      tmp[ 541 ] += a[ 12 ] * a[ 0 ] * a[ 23 ] * a[ 23 ];
      tmp[ 542 ] += a[ 12 ] * a[ 0 ] * a[ 23 ] * a[ 24 ];
      tmp[ 543 ] += a[ 12 ] * a[ 0 ] * a[ 24 ] * a[ 24 ];
      tmp[ 544 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 1 ];
      tmp[ 545 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 2 ];
      tmp[ 546 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 3 ];
      tmp[ 547 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 4 ];
      tmp[ 548 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 5 ];
      tmp[ 549 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 6 ];
      tmp[ 550 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 7 ];
      tmp[ 551 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 8 ];
      tmp[ 552 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 9 ];
      tmp[ 553 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 10 ];
      tmp[ 554 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 11 ];
      tmp[ 555 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 12 ];
      tmp[ 556 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 14 ];
      tmp[ 557 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 15 ];
      tmp[ 558 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 16 ];
      tmp[ 559 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 17 ];
      tmp[ 560 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 18 ];
      tmp[ 561 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 19 ];
      tmp[ 562 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 20 ];
      tmp[ 563 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 21 ];
      tmp[ 564 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 22 ];
      tmp[ 565 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 23 ];
      tmp[ 566 ] += a[ 12 ] * a[ 1 ] * a[ 1 ] * a[ 24 ];
      tmp[ 567 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 2 ];
      tmp[ 568 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 3 ];
      tmp[ 569 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 4 ];
      tmp[ 570 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 5 ];
      tmp[ 571 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 6 ];
      tmp[ 572 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 7 ];
      tmp[ 573 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 8 ];
      tmp[ 574 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 9 ];
      tmp[ 575 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 10 ];
      tmp[ 576 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 11 ];
      tmp[ 577 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 12 ];
      tmp[ 578 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 14 ];
      tmp[ 579 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 15 ];
      tmp[ 580 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 16 ];
      tmp[ 581 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 17 ];
      tmp[ 582 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 18 ];
      tmp[ 583 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 19 ];
      tmp[ 584 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 20 ];
      tmp[ 585 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 21 ];
      tmp[ 586 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 22 ];
      tmp[ 587 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 23 ];
      tmp[ 588 ] += a[ 12 ] * a[ 1 ] * a[ 2 ] * a[ 24 ];
      tmp[ 589 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 3 ];
      tmp[ 590 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 4 ];
      tmp[ 591 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 5 ];
      tmp[ 592 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 6 ];
      tmp[ 593 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 7 ];
      tmp[ 594 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 8 ];
      tmp[ 595 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 9 ];
      tmp[ 596 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 10 ];
      tmp[ 597 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 11 ];
      tmp[ 598 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 12 ];
      tmp[ 599 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 14 ];
      tmp[ 600 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 15 ];
      tmp[ 601 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 16 ];
      tmp[ 602 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 17 ];
      tmp[ 603 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 18 ];
      tmp[ 604 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 19 ];
      tmp[ 605 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 20 ];
      tmp[ 606 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 21 ];
      tmp[ 607 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 22 ];
      tmp[ 608 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 23 ];
      tmp[ 609 ] += a[ 12 ] * a[ 1 ] * a[ 3 ] * a[ 24 ];
      tmp[ 610 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 4 ];
      tmp[ 611 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 5 ];
      tmp[ 612 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 6 ];
      tmp[ 613 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 7 ];
      tmp[ 614 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 8 ];
      tmp[ 615 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 9 ];
      tmp[ 616 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 10 ];
      tmp[ 617 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 11 ];
      tmp[ 618 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 12 ];
      tmp[ 619 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 14 ];
      tmp[ 620 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 15 ];
      tmp[ 621 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 16 ];
      tmp[ 622 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 17 ];
      tmp[ 623 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 18 ];
      tmp[ 624 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 19 ];
      tmp[ 625 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 20 ];
      tmp[ 626 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 21 ];
      tmp[ 627 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 22 ];
      tmp[ 628 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 23 ];
      tmp[ 629 ] += a[ 12 ] * a[ 1 ] * a[ 4 ] * a[ 24 ];
      tmp[ 630 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 5 ];
      tmp[ 631 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 6 ];
      tmp[ 632 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 7 ];
      tmp[ 633 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 8 ];
      tmp[ 634 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 9 ];
      tmp[ 635 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 10 ];
      tmp[ 636 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 11 ];
      tmp[ 637 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 12 ];
      tmp[ 638 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 13 ];
      tmp[ 639 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 14 ];
      tmp[ 640 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 15 ];
      tmp[ 641 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 16 ];
      tmp[ 642 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 17 ];
      tmp[ 643 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 18 ];
      tmp[ 644 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 19 ];
      tmp[ 645 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 20 ];
      tmp[ 646 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 21 ];
      tmp[ 647 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 22 ];
      tmp[ 648 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 23 ];
      tmp[ 649 ] += a[ 12 ] * a[ 1 ] * a[ 5 ] * a[ 24 ];
      tmp[ 650 ] += a[ 12 ] * a[ 1 ] * a[ 6 ] * a[ 6 ];
      tmp[ 651 ] += a[ 12 ] * a[ 1 ] * a[ 6 ] * a[ 7 ];
      tmp[ 652 ] += a[ 12 ] * a[ 1 ] * a[ 6 ] * a[ 8 ];
      tmp[ 653 ] += a[ 12 ] * a[ 1 ] * a[ 6 ] * a[ 9 ];
      tmp[ 654 ] += a[ 12 ] * a[ 1 ] * a[ 6 ] * a[ 10 ];
      tmp[ 655 ] += a[ 12 ] * a[ 1 ] * a[ 6 ] * a[ 11 ];
      tmp[ 656 ] += a[ 12 ] * a[ 1 ] * a[ 6 ] * a[ 12 ];
      tmp[ 657 ] += a[ 12 ] * a[ 1 ] * a[ 6 ] * a[ 14 ];
      tmp[ 658 ] += a[ 12 ] * a[ 1 ] * a[ 6 ] * a[ 15 ];
      tmp[ 659 ] += a[ 12 ] * a[ 1 ] * a[ 6 ] * a[ 16 ];
      tmp[ 660 ] += a[ 12 ] * a[ 1 ] * a[ 6 ] * a[ 17 ];
      tmp[ 661 ] += a[ 12 ] * a[ 1 ] * a[ 6 ] * a[ 18 ];
      tmp[ 662 ] += a[ 12 ] * a[ 1 ] * a[ 6 ] * a[ 19 ];
      tmp[ 663 ] += a[ 12 ] * a[ 1 ] * a[ 6 ] * a[ 20 ];
      tmp[ 664 ] += a[ 12 ] * a[ 1 ] * a[ 6 ] * a[ 21 ];
      tmp[ 665 ] += a[ 12 ] * a[ 1 ] * a[ 6 ] * a[ 22 ];
      tmp[ 666 ] += a[ 12 ] * a[ 1 ] * a[ 6 ] * a[ 23 ];
      tmp[ 667 ] += a[ 12 ] * a[ 1 ] * a[ 6 ] * a[ 24 ];
      tmp[ 668 ] += a[ 12 ] * a[ 1 ] * a[ 7 ] * a[ 7 ];
      tmp[ 669 ] += a[ 12 ] * a[ 1 ] * a[ 7 ] * a[ 8 ];
      tmp[ 670 ] += a[ 12 ] * a[ 1 ] * a[ 7 ] * a[ 9 ];
      tmp[ 671 ] += a[ 12 ] * a[ 1 ] * a[ 7 ] * a[ 10 ];
      tmp[ 672 ] += a[ 12 ] * a[ 1 ] * a[ 7 ] * a[ 11 ];
      tmp[ 673 ] += a[ 12 ] * a[ 1 ] * a[ 7 ] * a[ 12 ];
      tmp[ 674 ] += a[ 12 ] * a[ 1 ] * a[ 7 ] * a[ 14 ];
      tmp[ 675 ] += a[ 12 ] * a[ 1 ] * a[ 7 ] * a[ 15 ];
      tmp[ 676 ] += a[ 12 ] * a[ 1 ] * a[ 7 ] * a[ 16 ];
      tmp[ 677 ] += a[ 12 ] * a[ 1 ] * a[ 7 ] * a[ 17 ];
      tmp[ 678 ] += a[ 12 ] * a[ 1 ] * a[ 7 ] * a[ 18 ];
      tmp[ 679 ] += a[ 12 ] * a[ 1 ] * a[ 7 ] * a[ 19 ];
      tmp[ 680 ] += a[ 12 ] * a[ 1 ] * a[ 7 ] * a[ 20 ];
      tmp[ 681 ] += a[ 12 ] * a[ 1 ] * a[ 7 ] * a[ 21 ];
      tmp[ 682 ] += a[ 12 ] * a[ 1 ] * a[ 7 ] * a[ 22 ];
      tmp[ 683 ] += a[ 12 ] * a[ 1 ] * a[ 7 ] * a[ 23 ];
      tmp[ 684 ] += a[ 12 ] * a[ 1 ] * a[ 7 ] * a[ 24 ];
      tmp[ 685 ] += a[ 12 ] * a[ 1 ] * a[ 8 ] * a[ 8 ];
      tmp[ 686 ] += a[ 12 ] * a[ 1 ] * a[ 8 ] * a[ 9 ];
      tmp[ 687 ] += a[ 12 ] * a[ 1 ] * a[ 8 ] * a[ 10 ];
      tmp[ 688 ] += a[ 12 ] * a[ 1 ] * a[ 8 ] * a[ 11 ];
      tmp[ 689 ] += a[ 12 ] * a[ 1 ] * a[ 8 ] * a[ 12 ];
      tmp[ 690 ] += a[ 12 ] * a[ 1 ] * a[ 8 ] * a[ 14 ];
      tmp[ 691 ] += a[ 12 ] * a[ 1 ] * a[ 8 ] * a[ 15 ];
      tmp[ 692 ] += a[ 12 ] * a[ 1 ] * a[ 8 ] * a[ 16 ];
      tmp[ 693 ] += a[ 12 ] * a[ 1 ] * a[ 8 ] * a[ 17 ];
      tmp[ 694 ] += a[ 12 ] * a[ 1 ] * a[ 8 ] * a[ 18 ];
      tmp[ 695 ] += a[ 12 ] * a[ 1 ] * a[ 8 ] * a[ 19 ];
      tmp[ 696 ] += a[ 12 ] * a[ 1 ] * a[ 8 ] * a[ 20 ];
      tmp[ 697 ] += a[ 12 ] * a[ 1 ] * a[ 8 ] * a[ 21 ];
      tmp[ 698 ] += a[ 12 ] * a[ 1 ] * a[ 8 ] * a[ 22 ];
      tmp[ 699 ] += a[ 12 ] * a[ 1 ] * a[ 8 ] * a[ 23 ];
      tmp[ 700 ] += a[ 12 ] * a[ 1 ] * a[ 8 ] * a[ 24 ];
      tmp[ 701 ] += a[ 12 ] * a[ 1 ] * a[ 9 ] * a[ 9 ];
      tmp[ 702 ] += a[ 12 ] * a[ 1 ] * a[ 9 ] * a[ 10 ];
      tmp[ 703 ] += a[ 12 ] * a[ 1 ] * a[ 9 ] * a[ 11 ];
      tmp[ 704 ] += a[ 12 ] * a[ 1 ] * a[ 9 ] * a[ 12 ];
      tmp[ 705 ] += a[ 12 ] * a[ 1 ] * a[ 9 ] * a[ 14 ];
      tmp[ 706 ] += a[ 12 ] * a[ 1 ] * a[ 9 ] * a[ 15 ];
      tmp[ 707 ] += a[ 12 ] * a[ 1 ] * a[ 9 ] * a[ 16 ];
      tmp[ 708 ] += a[ 12 ] * a[ 1 ] * a[ 9 ] * a[ 17 ];
      tmp[ 709 ] += a[ 12 ] * a[ 1 ] * a[ 9 ] * a[ 18 ];
      tmp[ 710 ] += a[ 12 ] * a[ 1 ] * a[ 9 ] * a[ 19 ];
      tmp[ 711 ] += a[ 12 ] * a[ 1 ] * a[ 9 ] * a[ 20 ];
      tmp[ 712 ] += a[ 12 ] * a[ 1 ] * a[ 9 ] * a[ 21 ];
      tmp[ 713 ] += a[ 12 ] * a[ 1 ] * a[ 9 ] * a[ 22 ];
      tmp[ 714 ] += a[ 12 ] * a[ 1 ] * a[ 9 ] * a[ 23 ];
      tmp[ 715 ] += a[ 12 ] * a[ 1 ] * a[ 9 ] * a[ 24 ];
      tmp[ 716 ] += a[ 12 ] * a[ 1 ] * a[ 10 ] * a[ 10 ];
      tmp[ 717 ] += a[ 12 ] * a[ 1 ] * a[ 10 ] * a[ 11 ];
      tmp[ 718 ] += a[ 12 ] * a[ 1 ] * a[ 10 ] * a[ 12 ];
      tmp[ 719 ] += a[ 12 ] * a[ 1 ] * a[ 10 ] * a[ 13 ];
      tmp[ 720 ] += a[ 12 ] * a[ 1 ] * a[ 10 ] * a[ 14 ];
      tmp[ 721 ] += a[ 12 ] * a[ 1 ] * a[ 10 ] * a[ 15 ];
      tmp[ 722 ] += a[ 12 ] * a[ 1 ] * a[ 10 ] * a[ 16 ];
      tmp[ 723 ] += a[ 12 ] * a[ 1 ] * a[ 10 ] * a[ 17 ];
      tmp[ 724 ] += a[ 12 ] * a[ 1 ] * a[ 10 ] * a[ 18 ];
      tmp[ 725 ] += a[ 12 ] * a[ 1 ] * a[ 10 ] * a[ 19 ];
      tmp[ 726 ] += a[ 12 ] * a[ 1 ] * a[ 10 ] * a[ 20 ];
      tmp[ 727 ] += a[ 12 ] * a[ 1 ] * a[ 10 ] * a[ 21 ];
      tmp[ 728 ] += a[ 12 ] * a[ 1 ] * a[ 10 ] * a[ 22 ];
      tmp[ 729 ] += a[ 12 ] * a[ 1 ] * a[ 10 ] * a[ 23 ];
      tmp[ 730 ] += a[ 12 ] * a[ 1 ] * a[ 10 ] * a[ 24 ];
      tmp[ 731 ] += a[ 12 ] * a[ 1 ] * a[ 11 ] * a[ 11 ];
      tmp[ 732 ] += a[ 12 ] * a[ 1 ] * a[ 11 ] * a[ 12 ];
      tmp[ 733 ] += a[ 12 ] * a[ 1 ] * a[ 11 ] * a[ 14 ];
      tmp[ 734 ] += a[ 12 ] * a[ 1 ] * a[ 11 ] * a[ 15 ];
      tmp[ 735 ] += a[ 12 ] * a[ 1 ] * a[ 11 ] * a[ 16 ];
      tmp[ 736 ] += a[ 12 ] * a[ 1 ] * a[ 11 ] * a[ 17 ];
      tmp[ 737 ] += a[ 12 ] * a[ 1 ] * a[ 11 ] * a[ 18 ];
      tmp[ 738 ] += a[ 12 ] * a[ 1 ] * a[ 11 ] * a[ 19 ];
      tmp[ 739 ] += a[ 12 ] * a[ 1 ] * a[ 11 ] * a[ 20 ];
      tmp[ 740 ] += a[ 12 ] * a[ 1 ] * a[ 11 ] * a[ 21 ];
      tmp[ 741 ] += a[ 12 ] * a[ 1 ] * a[ 11 ] * a[ 22 ];
      tmp[ 742 ] += a[ 12 ] * a[ 1 ] * a[ 11 ] * a[ 23 ];
      tmp[ 743 ] += a[ 12 ] * a[ 1 ] * a[ 11 ] * a[ 24 ];
      tmp[ 744 ] += a[ 12 ] * a[ 1 ] * a[ 12 ] * a[ 12 ];
      tmp[ 745 ] += a[ 12 ] * a[ 1 ] * a[ 12 ] * a[ 14 ];
      tmp[ 746 ] += a[ 12 ] * a[ 1 ] * a[ 12 ] * a[ 15 ];
      tmp[ 747 ] += a[ 12 ] * a[ 1 ] * a[ 12 ] * a[ 16 ];
      tmp[ 748 ] += a[ 12 ] * a[ 1 ] * a[ 12 ] * a[ 17 ];
      tmp[ 749 ] += a[ 12 ] * a[ 1 ] * a[ 12 ] * a[ 18 ];
      tmp[ 750 ] += a[ 12 ] * a[ 1 ] * a[ 12 ] * a[ 19 ];
      tmp[ 751 ] += a[ 12 ] * a[ 1 ] * a[ 12 ] * a[ 20 ];
      tmp[ 752 ] += a[ 12 ] * a[ 1 ] * a[ 12 ] * a[ 21 ];
      tmp[ 753 ] += a[ 12 ] * a[ 1 ] * a[ 12 ] * a[ 22 ];
      tmp[ 754 ] += a[ 12 ] * a[ 1 ] * a[ 12 ] * a[ 23 ];
      tmp[ 755 ] += a[ 12 ] * a[ 1 ] * a[ 12 ] * a[ 24 ];
      tmp[ 756 ] += a[ 12 ] * a[ 1 ] * a[ 13 ] * a[ 15 ];
      tmp[ 757 ] += a[ 12 ] * a[ 1 ] * a[ 13 ] * a[ 20 ];
      tmp[ 758 ] += a[ 12 ] * a[ 1 ] * a[ 14 ] * a[ 14 ];
      tmp[ 759 ] += a[ 12 ] * a[ 1 ] * a[ 14 ] * a[ 15 ];
      tmp[ 760 ] += a[ 12 ] * a[ 1 ] * a[ 14 ] * a[ 16 ];
      tmp[ 761 ] += a[ 12 ] * a[ 1 ] * a[ 14 ] * a[ 17 ];
      tmp[ 762 ] += a[ 12 ] * a[ 1 ] * a[ 14 ] * a[ 18 ];
      tmp[ 763 ] += a[ 12 ] * a[ 1 ] * a[ 14 ] * a[ 19 ];
      tmp[ 764 ] += a[ 12 ] * a[ 1 ] * a[ 14 ] * a[ 20 ];
      tmp[ 765 ] += a[ 12 ] * a[ 1 ] * a[ 14 ] * a[ 21 ];
      tmp[ 766 ] += a[ 12 ] * a[ 1 ] * a[ 14 ] * a[ 22 ];
      tmp[ 767 ] += a[ 12 ] * a[ 1 ] * a[ 14 ] * a[ 23 ];
      tmp[ 768 ] += a[ 12 ] * a[ 1 ] * a[ 14 ] * a[ 24 ];
      tmp[ 769 ] += a[ 12 ] * a[ 1 ] * a[ 15 ] * a[ 15 ];
      tmp[ 770 ] += a[ 12 ] * a[ 1 ] * a[ 15 ] * a[ 16 ];
      tmp[ 771 ] += a[ 12 ] * a[ 1 ] * a[ 15 ] * a[ 17 ];
      tmp[ 772 ] += a[ 12 ] * a[ 1 ] * a[ 15 ] * a[ 18 ];
      tmp[ 773 ] += a[ 12 ] * a[ 1 ] * a[ 15 ] * a[ 19 ];
      tmp[ 774 ] += a[ 12 ] * a[ 1 ] * a[ 15 ] * a[ 20 ];
      tmp[ 775 ] += a[ 12 ] * a[ 1 ] * a[ 15 ] * a[ 21 ];
      tmp[ 776 ] += a[ 12 ] * a[ 1 ] * a[ 15 ] * a[ 22 ];
      tmp[ 777 ] += a[ 12 ] * a[ 1 ] * a[ 15 ] * a[ 23 ];
      tmp[ 778 ] += a[ 12 ] * a[ 1 ] * a[ 15 ] * a[ 24 ];
      tmp[ 779 ] += a[ 12 ] * a[ 1 ] * a[ 16 ] * a[ 16 ];
      tmp[ 780 ] += a[ 12 ] * a[ 1 ] * a[ 16 ] * a[ 17 ];
      tmp[ 781 ] += a[ 12 ] * a[ 1 ] * a[ 16 ] * a[ 18 ];
      tmp[ 782 ] += a[ 12 ] * a[ 1 ] * a[ 16 ] * a[ 19 ];
      tmp[ 783 ] += a[ 12 ] * a[ 1 ] * a[ 16 ] * a[ 20 ];
      tmp[ 784 ] += a[ 12 ] * a[ 1 ] * a[ 16 ] * a[ 21 ];
      tmp[ 785 ] += a[ 12 ] * a[ 1 ] * a[ 16 ] * a[ 22 ];
      tmp[ 786 ] += a[ 12 ] * a[ 1 ] * a[ 16 ] * a[ 23 ];
      tmp[ 787 ] += a[ 12 ] * a[ 1 ] * a[ 16 ] * a[ 24 ];
      tmp[ 788 ] += a[ 12 ] * a[ 1 ] * a[ 17 ] * a[ 17 ];
      tmp[ 789 ] += a[ 12 ] * a[ 1 ] * a[ 17 ] * a[ 18 ];
      tmp[ 790 ] += a[ 12 ] * a[ 1 ] * a[ 17 ] * a[ 19 ];
      tmp[ 791 ] += a[ 12 ] * a[ 1 ] * a[ 17 ] * a[ 20 ];
      tmp[ 792 ] += a[ 12 ] * a[ 1 ] * a[ 17 ] * a[ 21 ];
      tmp[ 793 ] += a[ 12 ] * a[ 1 ] * a[ 17 ] * a[ 22 ];
      tmp[ 794 ] += a[ 12 ] * a[ 1 ] * a[ 17 ] * a[ 23 ];
      tmp[ 795 ] += a[ 12 ] * a[ 1 ] * a[ 17 ] * a[ 24 ];
      tmp[ 796 ] += a[ 12 ] * a[ 1 ] * a[ 18 ] * a[ 18 ];
      tmp[ 797 ] += a[ 12 ] * a[ 1 ] * a[ 18 ] * a[ 19 ];
      tmp[ 798 ] += a[ 12 ] * a[ 1 ] * a[ 18 ] * a[ 20 ];
      tmp[ 799 ] += a[ 12 ] * a[ 1 ] * a[ 18 ] * a[ 21 ];
      tmp[ 800 ] += a[ 12 ] * a[ 1 ] * a[ 18 ] * a[ 22 ];
      tmp[ 801 ] += a[ 12 ] * a[ 1 ] * a[ 18 ] * a[ 23 ];
      tmp[ 802 ] += a[ 12 ] * a[ 1 ] * a[ 18 ] * a[ 24 ];
      tmp[ 803 ] += a[ 12 ] * a[ 1 ] * a[ 19 ] * a[ 19 ];
      tmp[ 804 ] += a[ 12 ] * a[ 1 ] * a[ 19 ] * a[ 20 ];
      tmp[ 805 ] += a[ 12 ] * a[ 1 ] * a[ 19 ] * a[ 21 ];
      tmp[ 806 ] += a[ 12 ] * a[ 1 ] * a[ 19 ] * a[ 22 ];
      tmp[ 807 ] += a[ 12 ] * a[ 1 ] * a[ 19 ] * a[ 23 ];
      tmp[ 808 ] += a[ 12 ] * a[ 1 ] * a[ 19 ] * a[ 24 ];
      tmp[ 809 ] += a[ 12 ] * a[ 1 ] * a[ 20 ] * a[ 20 ];
      tmp[ 810 ] += a[ 12 ] * a[ 1 ] * a[ 20 ] * a[ 21 ];
      tmp[ 811 ] += a[ 12 ] * a[ 1 ] * a[ 20 ] * a[ 22 ];
      tmp[ 812 ] += a[ 12 ] * a[ 1 ] * a[ 20 ] * a[ 23 ];
      tmp[ 813 ] += a[ 12 ] * a[ 1 ] * a[ 20 ] * a[ 24 ];
      tmp[ 814 ] += a[ 12 ] * a[ 1 ] * a[ 21 ] * a[ 21 ];
      tmp[ 815 ] += a[ 12 ] * a[ 1 ] * a[ 21 ] * a[ 22 ];
      tmp[ 816 ] += a[ 12 ] * a[ 1 ] * a[ 21 ] * a[ 23 ];
      tmp[ 817 ] += a[ 12 ] * a[ 1 ] * a[ 21 ] * a[ 24 ];
      tmp[ 818 ] += a[ 12 ] * a[ 1 ] * a[ 22 ] * a[ 22 ];
      tmp[ 819 ] += a[ 12 ] * a[ 1 ] * a[ 22 ] * a[ 23 ];
      tmp[ 820 ] += a[ 12 ] * a[ 1 ] * a[ 22 ] * a[ 24 ];
      tmp[ 821 ] += a[ 12 ] * a[ 1 ] * a[ 23 ] * a[ 23 ];
      tmp[ 822 ] += a[ 12 ] * a[ 1 ] * a[ 23 ] * a[ 24 ];
      tmp[ 823 ] += a[ 12 ] * a[ 1 ] * a[ 24 ] * a[ 24 ];
      tmp[ 824 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 2 ];
      tmp[ 825 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 3 ];
      tmp[ 826 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 4 ];
      tmp[ 827 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 5 ];
      tmp[ 828 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 6 ];
      tmp[ 829 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 7 ];
      tmp[ 830 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 8 ];
      tmp[ 831 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 9 ];
      tmp[ 832 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 10 ];
      tmp[ 833 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 11 ];
      tmp[ 834 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 12 ];
      tmp[ 835 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 15 ];
      tmp[ 836 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 16 ];
      tmp[ 837 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 17 ];
      tmp[ 838 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 18 ];
      tmp[ 839 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 19 ];
      tmp[ 840 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 20 ];
      tmp[ 841 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 21 ];
      tmp[ 842 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 22 ];
      tmp[ 843 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 23 ];
      tmp[ 844 ] += a[ 12 ] * a[ 2 ] * a[ 2 ] * a[ 24 ];
      tmp[ 845 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 3 ];
      tmp[ 846 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 4 ];
      tmp[ 847 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 5 ];
      tmp[ 848 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 6 ];
      tmp[ 849 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 7 ];
      tmp[ 850 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 8 ];
      tmp[ 851 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 9 ];
      tmp[ 852 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 10 ];
      tmp[ 853 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 11 ];
      tmp[ 854 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 12 ];
      tmp[ 855 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 15 ];
      tmp[ 856 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 16 ];
      tmp[ 857 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 17 ];
      tmp[ 858 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 18 ];
      tmp[ 859 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 19 ];
      tmp[ 860 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 20 ];
      tmp[ 861 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 21 ];
      tmp[ 862 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 22 ];
      tmp[ 863 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 23 ];
      tmp[ 864 ] += a[ 12 ] * a[ 2 ] * a[ 3 ] * a[ 24 ];
      tmp[ 865 ] += a[ 12 ] * a[ 2 ] * a[ 4 ] * a[ 4 ];
      tmp[ 866 ] += a[ 12 ] * a[ 2 ] * a[ 4 ] * a[ 5 ];
      tmp[ 867 ] += a[ 12 ] * a[ 2 ] * a[ 4 ] * a[ 6 ];
      tmp[ 868 ] += a[ 12 ] * a[ 2 ] * a[ 4 ] * a[ 7 ];
      tmp[ 869 ] += a[ 12 ] * a[ 2 ] * a[ 4 ] * a[ 8 ];
      tmp[ 870 ] += a[ 12 ] * a[ 2 ] * a[ 4 ] * a[ 9 ];
      tmp[ 871 ] += a[ 12 ] * a[ 2 ] * a[ 4 ] * a[ 10 ];
      tmp[ 872 ] += a[ 12 ] * a[ 2 ] * a[ 4 ] * a[ 11 ];
      tmp[ 873 ] += a[ 12 ] * a[ 2 ] * a[ 4 ] * a[ 12 ];
      tmp[ 874 ] += a[ 12 ] * a[ 2 ] * a[ 4 ] * a[ 15 ];
      tmp[ 875 ] += a[ 12 ] * a[ 2 ] * a[ 4 ] * a[ 16 ];
      tmp[ 876 ] += a[ 12 ] * a[ 2 ] * a[ 4 ] * a[ 17 ];
      tmp[ 877 ] += a[ 12 ] * a[ 2 ] * a[ 4 ] * a[ 18 ];
      tmp[ 878 ] += a[ 12 ] * a[ 2 ] * a[ 4 ] * a[ 19 ];
      tmp[ 879 ] += a[ 12 ] * a[ 2 ] * a[ 4 ] * a[ 20 ];
      tmp[ 880 ] += a[ 12 ] * a[ 2 ] * a[ 4 ] * a[ 21 ];
      tmp[ 881 ] += a[ 12 ] * a[ 2 ] * a[ 4 ] * a[ 22 ];
      tmp[ 882 ] += a[ 12 ] * a[ 2 ] * a[ 4 ] * a[ 23 ];
      tmp[ 883 ] += a[ 12 ] * a[ 2 ] * a[ 4 ] * a[ 24 ];
      tmp[ 884 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 5 ];
      tmp[ 885 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 6 ];
      tmp[ 886 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 7 ];
      tmp[ 887 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 8 ];
      tmp[ 888 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 9 ];
      tmp[ 889 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 10 ];
      tmp[ 890 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 11 ];
      tmp[ 891 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 12 ];
      tmp[ 892 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 13 ];
      tmp[ 893 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 14 ];
      tmp[ 894 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 15 ];
      tmp[ 895 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 16 ];
      tmp[ 896 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 17 ];
      tmp[ 897 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 18 ];
      tmp[ 898 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 19 ];
      tmp[ 899 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 20 ];
      tmp[ 900 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 21 ];
      tmp[ 901 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 22 ];
      tmp[ 902 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 23 ];
      tmp[ 903 ] += a[ 12 ] * a[ 2 ] * a[ 5 ] * a[ 24 ];
      tmp[ 904 ] += a[ 12 ] * a[ 2 ] * a[ 6 ] * a[ 6 ];
      tmp[ 905 ] += a[ 12 ] * a[ 2 ] * a[ 6 ] * a[ 7 ];
      tmp[ 906 ] += a[ 12 ] * a[ 2 ] * a[ 6 ] * a[ 8 ];
      tmp[ 907 ] += a[ 12 ] * a[ 2 ] * a[ 6 ] * a[ 9 ];
      tmp[ 908 ] += a[ 12 ] * a[ 2 ] * a[ 6 ] * a[ 10 ];
      tmp[ 909 ] += a[ 12 ] * a[ 2 ] * a[ 6 ] * a[ 11 ];
      tmp[ 910 ] += a[ 12 ] * a[ 2 ] * a[ 6 ] * a[ 12 ];
      tmp[ 911 ] += a[ 12 ] * a[ 2 ] * a[ 6 ] * a[ 14 ];
      tmp[ 912 ] += a[ 12 ] * a[ 2 ] * a[ 6 ] * a[ 15 ];
      tmp[ 913 ] += a[ 12 ] * a[ 2 ] * a[ 6 ] * a[ 16 ];
      tmp[ 914 ] += a[ 12 ] * a[ 2 ] * a[ 6 ] * a[ 17 ];
      tmp[ 915 ] += a[ 12 ] * a[ 2 ] * a[ 6 ] * a[ 18 ];
      tmp[ 916 ] += a[ 12 ] * a[ 2 ] * a[ 6 ] * a[ 19 ];
      tmp[ 917 ] += a[ 12 ] * a[ 2 ] * a[ 6 ] * a[ 20 ];
      tmp[ 918 ] += a[ 12 ] * a[ 2 ] * a[ 6 ] * a[ 21 ];
      tmp[ 919 ] += a[ 12 ] * a[ 2 ] * a[ 6 ] * a[ 22 ];
      tmp[ 920 ] += a[ 12 ] * a[ 2 ] * a[ 6 ] * a[ 23 ];
      tmp[ 921 ] += a[ 12 ] * a[ 2 ] * a[ 6 ] * a[ 24 ];
      tmp[ 922 ] += a[ 12 ] * a[ 2 ] * a[ 7 ] * a[ 7 ];
      tmp[ 923 ] += a[ 12 ] * a[ 2 ] * a[ 7 ] * a[ 8 ];
      tmp[ 924 ] += a[ 12 ] * a[ 2 ] * a[ 7 ] * a[ 9 ];
      tmp[ 925 ] += a[ 12 ] * a[ 2 ] * a[ 7 ] * a[ 10 ];
      tmp[ 926 ] += a[ 12 ] * a[ 2 ] * a[ 7 ] * a[ 11 ];
      tmp[ 927 ] += a[ 12 ] * a[ 2 ] * a[ 7 ] * a[ 12 ];
      tmp[ 928 ] += a[ 12 ] * a[ 2 ] * a[ 7 ] * a[ 15 ];
      tmp[ 929 ] += a[ 12 ] * a[ 2 ] * a[ 7 ] * a[ 16 ];
      tmp[ 930 ] += a[ 12 ] * a[ 2 ] * a[ 7 ] * a[ 17 ];
      tmp[ 931 ] += a[ 12 ] * a[ 2 ] * a[ 7 ] * a[ 18 ];
      tmp[ 932 ] += a[ 12 ] * a[ 2 ] * a[ 7 ] * a[ 19 ];
      tmp[ 933 ] += a[ 12 ] * a[ 2 ] * a[ 7 ] * a[ 20 ];
      tmp[ 934 ] += a[ 12 ] * a[ 2 ] * a[ 7 ] * a[ 21 ];
      tmp[ 935 ] += a[ 12 ] * a[ 2 ] * a[ 7 ] * a[ 22 ];
      tmp[ 936 ] += a[ 12 ] * a[ 2 ] * a[ 7 ] * a[ 23 ];
      tmp[ 937 ] += a[ 12 ] * a[ 2 ] * a[ 7 ] * a[ 24 ];
      tmp[ 938 ] += a[ 12 ] * a[ 2 ] * a[ 8 ] * a[ 8 ];
      tmp[ 939 ] += a[ 12 ] * a[ 2 ] * a[ 8 ] * a[ 9 ];
      tmp[ 940 ] += a[ 12 ] * a[ 2 ] * a[ 8 ] * a[ 10 ];
      tmp[ 941 ] += a[ 12 ] * a[ 2 ] * a[ 8 ] * a[ 11 ];
      tmp[ 942 ] += a[ 12 ] * a[ 2 ] * a[ 8 ] * a[ 12 ];
      tmp[ 943 ] += a[ 12 ] * a[ 2 ] * a[ 8 ] * a[ 15 ];
      tmp[ 944 ] += a[ 12 ] * a[ 2 ] * a[ 8 ] * a[ 16 ];
      tmp[ 945 ] += a[ 12 ] * a[ 2 ] * a[ 8 ] * a[ 17 ];
      tmp[ 946 ] += a[ 12 ] * a[ 2 ] * a[ 8 ] * a[ 18 ];
      tmp[ 947 ] += a[ 12 ] * a[ 2 ] * a[ 8 ] * a[ 19 ];
      tmp[ 948 ] += a[ 12 ] * a[ 2 ] * a[ 8 ] * a[ 20 ];
      tmp[ 949 ] += a[ 12 ] * a[ 2 ] * a[ 8 ] * a[ 21 ];
      tmp[ 950 ] += a[ 12 ] * a[ 2 ] * a[ 8 ] * a[ 22 ];
      tmp[ 951 ] += a[ 12 ] * a[ 2 ] * a[ 8 ] * a[ 23 ];
      tmp[ 952 ] += a[ 12 ] * a[ 2 ] * a[ 8 ] * a[ 24 ];
      tmp[ 953 ] += a[ 12 ] * a[ 2 ] * a[ 9 ] * a[ 9 ];
      tmp[ 954 ] += a[ 12 ] * a[ 2 ] * a[ 9 ] * a[ 10 ];
      tmp[ 955 ] += a[ 12 ] * a[ 2 ] * a[ 9 ] * a[ 11 ];
      tmp[ 956 ] += a[ 12 ] * a[ 2 ] * a[ 9 ] * a[ 12 ];
      tmp[ 957 ] += a[ 12 ] * a[ 2 ] * a[ 9 ] * a[ 15 ];
      tmp[ 958 ] += a[ 12 ] * a[ 2 ] * a[ 9 ] * a[ 16 ];
      tmp[ 959 ] += a[ 12 ] * a[ 2 ] * a[ 9 ] * a[ 17 ];
      tmp[ 960 ] += a[ 12 ] * a[ 2 ] * a[ 9 ] * a[ 18 ];
      tmp[ 961 ] += a[ 12 ] * a[ 2 ] * a[ 9 ] * a[ 19 ];
      tmp[ 962 ] += a[ 12 ] * a[ 2 ] * a[ 9 ] * a[ 20 ];
      tmp[ 963 ] += a[ 12 ] * a[ 2 ] * a[ 9 ] * a[ 21 ];
      tmp[ 964 ] += a[ 12 ] * a[ 2 ] * a[ 9 ] * a[ 22 ];
      tmp[ 965 ] += a[ 12 ] * a[ 2 ] * a[ 9 ] * a[ 23 ];
      tmp[ 966 ] += a[ 12 ] * a[ 2 ] * a[ 9 ] * a[ 24 ];
      tmp[ 967 ] += a[ 12 ] * a[ 2 ] * a[ 10 ] * a[ 10 ];
      tmp[ 968 ] += a[ 12 ] * a[ 2 ] * a[ 10 ] * a[ 11 ];
      tmp[ 969 ] += a[ 12 ] * a[ 2 ] * a[ 10 ] * a[ 12 ];
      tmp[ 970 ] += a[ 12 ] * a[ 2 ] * a[ 10 ] * a[ 13 ];
      tmp[ 971 ] += a[ 12 ] * a[ 2 ] * a[ 10 ] * a[ 14 ];
      tmp[ 972 ] += a[ 12 ] * a[ 2 ] * a[ 10 ] * a[ 15 ];
      tmp[ 973 ] += a[ 12 ] * a[ 2 ] * a[ 10 ] * a[ 16 ];
      tmp[ 974 ] += a[ 12 ] * a[ 2 ] * a[ 10 ] * a[ 17 ];
      tmp[ 975 ] += a[ 12 ] * a[ 2 ] * a[ 10 ] * a[ 18 ];
      tmp[ 976 ] += a[ 12 ] * a[ 2 ] * a[ 10 ] * a[ 19 ];
      tmp[ 977 ] += a[ 12 ] * a[ 2 ] * a[ 10 ] * a[ 20 ];
      tmp[ 978 ] += a[ 12 ] * a[ 2 ] * a[ 10 ] * a[ 21 ];
      tmp[ 979 ] += a[ 12 ] * a[ 2 ] * a[ 10 ] * a[ 22 ];
      tmp[ 980 ] += a[ 12 ] * a[ 2 ] * a[ 10 ] * a[ 23 ];
      tmp[ 981 ] += a[ 12 ] * a[ 2 ] * a[ 10 ] * a[ 24 ];
      tmp[ 982 ] += a[ 12 ] * a[ 2 ] * a[ 11 ] * a[ 11 ];
      tmp[ 983 ] += a[ 12 ] * a[ 2 ] * a[ 11 ] * a[ 12 ];
      tmp[ 984 ] += a[ 12 ] * a[ 2 ] * a[ 11 ] * a[ 14 ];
      tmp[ 985 ] += a[ 12 ] * a[ 2 ] * a[ 11 ] * a[ 15 ];
      tmp[ 986 ] += a[ 12 ] * a[ 2 ] * a[ 11 ] * a[ 16 ];
      tmp[ 987 ] += a[ 12 ] * a[ 2 ] * a[ 11 ] * a[ 17 ];
      tmp[ 988 ] += a[ 12 ] * a[ 2 ] * a[ 11 ] * a[ 18 ];
      tmp[ 989 ] += a[ 12 ] * a[ 2 ] * a[ 11 ] * a[ 19 ];
      tmp[ 990 ] += a[ 12 ] * a[ 2 ] * a[ 11 ] * a[ 20 ];
      tmp[ 991 ] += a[ 12 ] * a[ 2 ] * a[ 11 ] * a[ 21 ];
      tmp[ 992 ] += a[ 12 ] * a[ 2 ] * a[ 11 ] * a[ 22 ];
      tmp[ 993 ] += a[ 12 ] * a[ 2 ] * a[ 11 ] * a[ 23 ];
      tmp[ 994 ] += a[ 12 ] * a[ 2 ] * a[ 11 ] * a[ 24 ];
      tmp[ 995 ] += a[ 12 ] * a[ 2 ] * a[ 12 ] * a[ 12 ];
      tmp[ 996 ] += a[ 12 ] * a[ 2 ] * a[ 12 ] * a[ 15 ];
      tmp[ 997 ] += a[ 12 ] * a[ 2 ] * a[ 12 ] * a[ 16 ];
      tmp[ 998 ] += a[ 12 ] * a[ 2 ] * a[ 12 ] * a[ 17 ];
      tmp[ 999 ] += a[ 12 ] * a[ 2 ] * a[ 12 ] * a[ 18 ];
      tmp[ 1000 ] += a[ 12 ] * a[ 2 ] * a[ 12 ] * a[ 19 ];
      tmp[ 1001 ] += a[ 12 ] * a[ 2 ] * a[ 12 ] * a[ 20 ];
      tmp[ 1002 ] += a[ 12 ] * a[ 2 ] * a[ 12 ] * a[ 21 ];
      tmp[ 1003 ] += a[ 12 ] * a[ 2 ] * a[ 12 ] * a[ 22 ];
      tmp[ 1004 ] += a[ 12 ] * a[ 2 ] * a[ 12 ] * a[ 23 ];
      tmp[ 1005 ] += a[ 12 ] * a[ 2 ] * a[ 12 ] * a[ 24 ];
      tmp[ 1006 ] += a[ 12 ] * a[ 2 ] * a[ 13 ] * a[ 15 ];
      tmp[ 1007 ] += a[ 12 ] * a[ 2 ] * a[ 13 ] * a[ 20 ];
      tmp[ 1008 ] += a[ 12 ] * a[ 2 ] * a[ 14 ] * a[ 15 ];
      tmp[ 1009 ] += a[ 12 ] * a[ 2 ] * a[ 14 ] * a[ 16 ];
      tmp[ 1010 ] += a[ 12 ] * a[ 2 ] * a[ 14 ] * a[ 20 ];
      tmp[ 1011 ] += a[ 12 ] * a[ 2 ] * a[ 14 ] * a[ 21 ];
      tmp[ 1012 ] += a[ 12 ] * a[ 2 ] * a[ 15 ] * a[ 15 ];
      tmp[ 1013 ] += a[ 12 ] * a[ 2 ] * a[ 15 ] * a[ 16 ];
      tmp[ 1014 ] += a[ 12 ] * a[ 2 ] * a[ 15 ] * a[ 17 ];
      tmp[ 1015 ] += a[ 12 ] * a[ 2 ] * a[ 15 ] * a[ 18 ];
      tmp[ 1016 ] += a[ 12 ] * a[ 2 ] * a[ 15 ] * a[ 19 ];
      tmp[ 1017 ] += a[ 12 ] * a[ 2 ] * a[ 15 ] * a[ 20 ];
      tmp[ 1018 ] += a[ 12 ] * a[ 2 ] * a[ 15 ] * a[ 21 ];
      tmp[ 1019 ] += a[ 12 ] * a[ 2 ] * a[ 15 ] * a[ 22 ];
      tmp[ 1020 ] += a[ 12 ] * a[ 2 ] * a[ 15 ] * a[ 23 ];
      tmp[ 1021 ] += a[ 12 ] * a[ 2 ] * a[ 15 ] * a[ 24 ];
      tmp[ 1022 ] += a[ 12 ] * a[ 2 ] * a[ 16 ] * a[ 16 ];
      tmp[ 1023 ] += a[ 12 ] * a[ 2 ] * a[ 16 ] * a[ 17 ];
      tmp[ 1024 ] += a[ 12 ] * a[ 2 ] * a[ 16 ] * a[ 18 ];
      tmp[ 1025 ] += a[ 12 ] * a[ 2 ] * a[ 16 ] * a[ 19 ];
      tmp[ 1026 ] += a[ 12 ] * a[ 2 ] * a[ 16 ] * a[ 20 ];
      tmp[ 1027 ] += a[ 12 ] * a[ 2 ] * a[ 16 ] * a[ 21 ];
      tmp[ 1028 ] += a[ 12 ] * a[ 2 ] * a[ 16 ] * a[ 22 ];
      tmp[ 1029 ] += a[ 12 ] * a[ 2 ] * a[ 16 ] * a[ 23 ];
      tmp[ 1030 ] += a[ 12 ] * a[ 2 ] * a[ 16 ] * a[ 24 ];
      tmp[ 1031 ] += a[ 12 ] * a[ 2 ] * a[ 17 ] * a[ 17 ];
      tmp[ 1032 ] += a[ 12 ] * a[ 2 ] * a[ 17 ] * a[ 18 ];
      tmp[ 1033 ] += a[ 12 ] * a[ 2 ] * a[ 17 ] * a[ 19 ];
      tmp[ 1034 ] += a[ 12 ] * a[ 2 ] * a[ 17 ] * a[ 20 ];
      tmp[ 1035 ] += a[ 12 ] * a[ 2 ] * a[ 17 ] * a[ 21 ];
      tmp[ 1036 ] += a[ 12 ] * a[ 2 ] * a[ 17 ] * a[ 22 ];
      tmp[ 1037 ] += a[ 12 ] * a[ 2 ] * a[ 17 ] * a[ 23 ];
      tmp[ 1038 ] += a[ 12 ] * a[ 2 ] * a[ 17 ] * a[ 24 ];
      tmp[ 1039 ] += a[ 12 ] * a[ 2 ] * a[ 18 ] * a[ 18 ];
      tmp[ 1040 ] += a[ 12 ] * a[ 2 ] * a[ 18 ] * a[ 19 ];
      tmp[ 1041 ] += a[ 12 ] * a[ 2 ] * a[ 18 ] * a[ 20 ];
      tmp[ 1042 ] += a[ 12 ] * a[ 2 ] * a[ 18 ] * a[ 21 ];
      tmp[ 1043 ] += a[ 12 ] * a[ 2 ] * a[ 18 ] * a[ 22 ];
      tmp[ 1044 ] += a[ 12 ] * a[ 2 ] * a[ 18 ] * a[ 23 ];
      tmp[ 1045 ] += a[ 12 ] * a[ 2 ] * a[ 18 ] * a[ 24 ];
      tmp[ 1046 ] += a[ 12 ] * a[ 2 ] * a[ 19 ] * a[ 19 ];
      tmp[ 1047 ] += a[ 12 ] * a[ 2 ] * a[ 19 ] * a[ 20 ];
      tmp[ 1048 ] += a[ 12 ] * a[ 2 ] * a[ 19 ] * a[ 21 ];
      tmp[ 1049 ] += a[ 12 ] * a[ 2 ] * a[ 19 ] * a[ 22 ];
      tmp[ 1050 ] += a[ 12 ] * a[ 2 ] * a[ 19 ] * a[ 23 ];
      tmp[ 1051 ] += a[ 12 ] * a[ 2 ] * a[ 19 ] * a[ 24 ];
      tmp[ 1052 ] += a[ 12 ] * a[ 2 ] * a[ 20 ] * a[ 20 ];
      tmp[ 1053 ] += a[ 12 ] * a[ 2 ] * a[ 20 ] * a[ 21 ];
      tmp[ 1054 ] += a[ 12 ] * a[ 2 ] * a[ 20 ] * a[ 22 ];
      tmp[ 1055 ] += a[ 12 ] * a[ 2 ] * a[ 20 ] * a[ 23 ];
      tmp[ 1056 ] += a[ 12 ] * a[ 2 ] * a[ 20 ] * a[ 24 ];
      tmp[ 1057 ] += a[ 12 ] * a[ 2 ] * a[ 21 ] * a[ 21 ];
      tmp[ 1058 ] += a[ 12 ] * a[ 2 ] * a[ 21 ] * a[ 22 ];
      tmp[ 1059 ] += a[ 12 ] * a[ 2 ] * a[ 21 ] * a[ 23 ];
      tmp[ 1060 ] += a[ 12 ] * a[ 2 ] * a[ 21 ] * a[ 24 ];
      tmp[ 1061 ] += a[ 12 ] * a[ 2 ] * a[ 22 ] * a[ 22 ];
      tmp[ 1062 ] += a[ 12 ] * a[ 2 ] * a[ 22 ] * a[ 23 ];
      tmp[ 1063 ] += a[ 12 ] * a[ 2 ] * a[ 22 ] * a[ 24 ];
      tmp[ 1064 ] += a[ 12 ] * a[ 2 ] * a[ 23 ] * a[ 23 ];
      tmp[ 1065 ] += a[ 12 ] * a[ 2 ] * a[ 23 ] * a[ 24 ];
      tmp[ 1066 ] += a[ 12 ] * a[ 2 ] * a[ 24 ] * a[ 24 ];
      tmp[ 1067 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 3 ];
      tmp[ 1068 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 4 ];
      tmp[ 1069 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 5 ];
      tmp[ 1070 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 6 ];
      tmp[ 1071 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 7 ];
      tmp[ 1072 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 8 ];
      tmp[ 1073 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 9 ];
      tmp[ 1074 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 10 ];
      tmp[ 1075 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 11 ];
      tmp[ 1076 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 12 ];
      tmp[ 1077 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 15 ];
      tmp[ 1078 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 16 ];
      tmp[ 1079 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 17 ];
      tmp[ 1080 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 18 ];
      tmp[ 1081 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 19 ];
      tmp[ 1082 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 20 ];
      tmp[ 1083 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 21 ];
      tmp[ 1084 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 22 ];
      tmp[ 1085 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 23 ];
      tmp[ 1086 ] += a[ 12 ] * a[ 3 ] * a[ 3 ] * a[ 24 ];
      tmp[ 1087 ] += a[ 12 ] * a[ 3 ] * a[ 4 ] * a[ 4 ];
      tmp[ 1088 ] += a[ 12 ] * a[ 3 ] * a[ 4 ] * a[ 5 ];
      tmp[ 1089 ] += a[ 12 ] * a[ 3 ] * a[ 4 ] * a[ 6 ];
      tmp[ 1090 ] += a[ 12 ] * a[ 3 ] * a[ 4 ] * a[ 7 ];
      tmp[ 1091 ] += a[ 12 ] * a[ 3 ] * a[ 4 ] * a[ 8 ];
      tmp[ 1092 ] += a[ 12 ] * a[ 3 ] * a[ 4 ] * a[ 9 ];
      tmp[ 1093 ] += a[ 12 ] * a[ 3 ] * a[ 4 ] * a[ 10 ];
      tmp[ 1094 ] += a[ 12 ] * a[ 3 ] * a[ 4 ] * a[ 11 ];
      tmp[ 1095 ] += a[ 12 ] * a[ 3 ] * a[ 4 ] * a[ 12 ];
      tmp[ 1096 ] += a[ 12 ] * a[ 3 ] * a[ 4 ] * a[ 15 ];
      tmp[ 1097 ] += a[ 12 ] * a[ 3 ] * a[ 4 ] * a[ 16 ];
      tmp[ 1098 ] += a[ 12 ] * a[ 3 ] * a[ 4 ] * a[ 17 ];
      tmp[ 1099 ] += a[ 12 ] * a[ 3 ] * a[ 4 ] * a[ 18 ];
      tmp[ 1100 ] += a[ 12 ] * a[ 3 ] * a[ 4 ] * a[ 19 ];
      tmp[ 1101 ] += a[ 12 ] * a[ 3 ] * a[ 4 ] * a[ 20 ];
      tmp[ 1102 ] += a[ 12 ] * a[ 3 ] * a[ 4 ] * a[ 21 ];
      tmp[ 1103 ] += a[ 12 ] * a[ 3 ] * a[ 4 ] * a[ 22 ];
      tmp[ 1104 ] += a[ 12 ] * a[ 3 ] * a[ 4 ] * a[ 23 ];
      tmp[ 1105 ] += a[ 12 ] * a[ 3 ] * a[ 4 ] * a[ 24 ];
      tmp[ 1106 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 5 ];
      tmp[ 1107 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 6 ];
      tmp[ 1108 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 7 ];
      tmp[ 1109 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 8 ];
      tmp[ 1110 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 9 ];
      tmp[ 1111 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 10 ];
      tmp[ 1112 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 11 ];
      tmp[ 1113 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 12 ];
      tmp[ 1114 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 13 ];
      tmp[ 1115 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 14 ];
      tmp[ 1116 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 15 ];
      tmp[ 1117 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 16 ];
      tmp[ 1118 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 17 ];
      tmp[ 1119 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 18 ];
      tmp[ 1120 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 19 ];
      tmp[ 1121 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 20 ];
      tmp[ 1122 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 21 ];
      tmp[ 1123 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 22 ];
      tmp[ 1124 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 23 ];
      tmp[ 1125 ] += a[ 12 ] * a[ 3 ] * a[ 5 ] * a[ 24 ];
      tmp[ 1126 ] += a[ 12 ] * a[ 3 ] * a[ 6 ] * a[ 6 ];
      tmp[ 1127 ] += a[ 12 ] * a[ 3 ] * a[ 6 ] * a[ 7 ];
      tmp[ 1128 ] += a[ 12 ] * a[ 3 ] * a[ 6 ] * a[ 8 ];
      tmp[ 1129 ] += a[ 12 ] * a[ 3 ] * a[ 6 ] * a[ 9 ];
      tmp[ 1130 ] += a[ 12 ] * a[ 3 ] * a[ 6 ] * a[ 10 ];
      tmp[ 1131 ] += a[ 12 ] * a[ 3 ] * a[ 6 ] * a[ 11 ];
      tmp[ 1132 ] += a[ 12 ] * a[ 3 ] * a[ 6 ] * a[ 12 ];
      tmp[ 1133 ] += a[ 12 ] * a[ 3 ] * a[ 6 ] * a[ 14 ];
      tmp[ 1134 ] += a[ 12 ] * a[ 3 ] * a[ 6 ] * a[ 15 ];
      tmp[ 1135 ] += a[ 12 ] * a[ 3 ] * a[ 6 ] * a[ 16 ];
      tmp[ 1136 ] += a[ 12 ] * a[ 3 ] * a[ 6 ] * a[ 17 ];
      tmp[ 1137 ] += a[ 12 ] * a[ 3 ] * a[ 6 ] * a[ 18 ];
      tmp[ 1138 ] += a[ 12 ] * a[ 3 ] * a[ 6 ] * a[ 19 ];
      tmp[ 1139 ] += a[ 12 ] * a[ 3 ] * a[ 6 ] * a[ 20 ];
      tmp[ 1140 ] += a[ 12 ] * a[ 3 ] * a[ 6 ] * a[ 21 ];
      tmp[ 1141 ] += a[ 12 ] * a[ 3 ] * a[ 6 ] * a[ 22 ];
      tmp[ 1142 ] += a[ 12 ] * a[ 3 ] * a[ 6 ] * a[ 23 ];
      tmp[ 1143 ] += a[ 12 ] * a[ 3 ] * a[ 6 ] * a[ 24 ];
      tmp[ 1144 ] += a[ 12 ] * a[ 3 ] * a[ 7 ] * a[ 7 ];
      tmp[ 1145 ] += a[ 12 ] * a[ 3 ] * a[ 7 ] * a[ 8 ];
      tmp[ 1146 ] += a[ 12 ] * a[ 3 ] * a[ 7 ] * a[ 9 ];
      tmp[ 1147 ] += a[ 12 ] * a[ 3 ] * a[ 7 ] * a[ 10 ];
      tmp[ 1148 ] += a[ 12 ] * a[ 3 ] * a[ 7 ] * a[ 11 ];
      tmp[ 1149 ] += a[ 12 ] * a[ 3 ] * a[ 7 ] * a[ 12 ];
      tmp[ 1150 ] += a[ 12 ] * a[ 3 ] * a[ 7 ] * a[ 15 ];
      tmp[ 1151 ] += a[ 12 ] * a[ 3 ] * a[ 7 ] * a[ 16 ];
      tmp[ 1152 ] += a[ 12 ] * a[ 3 ] * a[ 7 ] * a[ 17 ];
      tmp[ 1153 ] += a[ 12 ] * a[ 3 ] * a[ 7 ] * a[ 18 ];
      tmp[ 1154 ] += a[ 12 ] * a[ 3 ] * a[ 7 ] * a[ 19 ];
      tmp[ 1155 ] += a[ 12 ] * a[ 3 ] * a[ 7 ] * a[ 20 ];
      tmp[ 1156 ] += a[ 12 ] * a[ 3 ] * a[ 7 ] * a[ 21 ];
      tmp[ 1157 ] += a[ 12 ] * a[ 3 ] * a[ 7 ] * a[ 22 ];
      tmp[ 1158 ] += a[ 12 ] * a[ 3 ] * a[ 7 ] * a[ 23 ];
      tmp[ 1159 ] += a[ 12 ] * a[ 3 ] * a[ 7 ] * a[ 24 ];
      tmp[ 1160 ] += a[ 12 ] * a[ 3 ] * a[ 8 ] * a[ 8 ];
      tmp[ 1161 ] += a[ 12 ] * a[ 3 ] * a[ 8 ] * a[ 9 ];
      tmp[ 1162 ] += a[ 12 ] * a[ 3 ] * a[ 8 ] * a[ 10 ];
      tmp[ 1163 ] += a[ 12 ] * a[ 3 ] * a[ 8 ] * a[ 11 ];
      tmp[ 1164 ] += a[ 12 ] * a[ 3 ] * a[ 8 ] * a[ 12 ];
      tmp[ 1165 ] += a[ 12 ] * a[ 3 ] * a[ 8 ] * a[ 15 ];
      tmp[ 1166 ] += a[ 12 ] * a[ 3 ] * a[ 8 ] * a[ 16 ];
      tmp[ 1167 ] += a[ 12 ] * a[ 3 ] * a[ 8 ] * a[ 17 ];
      tmp[ 1168 ] += a[ 12 ] * a[ 3 ] * a[ 8 ] * a[ 18 ];
      tmp[ 1169 ] += a[ 12 ] * a[ 3 ] * a[ 8 ] * a[ 19 ];
      tmp[ 1170 ] += a[ 12 ] * a[ 3 ] * a[ 8 ] * a[ 20 ];
      tmp[ 1171 ] += a[ 12 ] * a[ 3 ] * a[ 8 ] * a[ 21 ];
      tmp[ 1172 ] += a[ 12 ] * a[ 3 ] * a[ 8 ] * a[ 22 ];
      tmp[ 1173 ] += a[ 12 ] * a[ 3 ] * a[ 8 ] * a[ 23 ];
      tmp[ 1174 ] += a[ 12 ] * a[ 3 ] * a[ 8 ] * a[ 24 ];
      tmp[ 1175 ] += a[ 12 ] * a[ 3 ] * a[ 9 ] * a[ 9 ];
      tmp[ 1176 ] += a[ 12 ] * a[ 3 ] * a[ 9 ] * a[ 10 ];
      tmp[ 1177 ] += a[ 12 ] * a[ 3 ] * a[ 9 ] * a[ 11 ];
      tmp[ 1178 ] += a[ 12 ] * a[ 3 ] * a[ 9 ] * a[ 12 ];
      tmp[ 1179 ] += a[ 12 ] * a[ 3 ] * a[ 9 ] * a[ 15 ];
      tmp[ 1180 ] += a[ 12 ] * a[ 3 ] * a[ 9 ] * a[ 16 ];
      tmp[ 1181 ] += a[ 12 ] * a[ 3 ] * a[ 9 ] * a[ 17 ];
      tmp[ 1182 ] += a[ 12 ] * a[ 3 ] * a[ 9 ] * a[ 18 ];
      tmp[ 1183 ] += a[ 12 ] * a[ 3 ] * a[ 9 ] * a[ 19 ];
      tmp[ 1184 ] += a[ 12 ] * a[ 3 ] * a[ 9 ] * a[ 20 ];
      tmp[ 1185 ] += a[ 12 ] * a[ 3 ] * a[ 9 ] * a[ 21 ];
      tmp[ 1186 ] += a[ 12 ] * a[ 3 ] * a[ 9 ] * a[ 22 ];
      tmp[ 1187 ] += a[ 12 ] * a[ 3 ] * a[ 9 ] * a[ 23 ];
      tmp[ 1188 ] += a[ 12 ] * a[ 3 ] * a[ 9 ] * a[ 24 ];
      tmp[ 1189 ] += a[ 12 ] * a[ 3 ] * a[ 10 ] * a[ 10 ];
      tmp[ 1190 ] += a[ 12 ] * a[ 3 ] * a[ 10 ] * a[ 11 ];
      tmp[ 1191 ] += a[ 12 ] * a[ 3 ] * a[ 10 ] * a[ 12 ];
      tmp[ 1192 ] += a[ 12 ] * a[ 3 ] * a[ 10 ] * a[ 13 ];
      tmp[ 1193 ] += a[ 12 ] * a[ 3 ] * a[ 10 ] * a[ 14 ];
      tmp[ 1194 ] += a[ 12 ] * a[ 3 ] * a[ 10 ] * a[ 15 ];
      tmp[ 1195 ] += a[ 12 ] * a[ 3 ] * a[ 10 ] * a[ 16 ];
      tmp[ 1196 ] += a[ 12 ] * a[ 3 ] * a[ 10 ] * a[ 17 ];
      tmp[ 1197 ] += a[ 12 ] * a[ 3 ] * a[ 10 ] * a[ 18 ];
      tmp[ 1198 ] += a[ 12 ] * a[ 3 ] * a[ 10 ] * a[ 19 ];
      tmp[ 1199 ] += a[ 12 ] * a[ 3 ] * a[ 10 ] * a[ 20 ];
      tmp[ 1200 ] += a[ 12 ] * a[ 3 ] * a[ 10 ] * a[ 21 ];
      tmp[ 1201 ] += a[ 12 ] * a[ 3 ] * a[ 10 ] * a[ 22 ];
      tmp[ 1202 ] += a[ 12 ] * a[ 3 ] * a[ 10 ] * a[ 23 ];
      tmp[ 1203 ] += a[ 12 ] * a[ 3 ] * a[ 10 ] * a[ 24 ];
      tmp[ 1204 ] += a[ 12 ] * a[ 3 ] * a[ 11 ] * a[ 11 ];
      tmp[ 1205 ] += a[ 12 ] * a[ 3 ] * a[ 11 ] * a[ 12 ];
      tmp[ 1206 ] += a[ 12 ] * a[ 3 ] * a[ 11 ] * a[ 14 ];
      tmp[ 1207 ] += a[ 12 ] * a[ 3 ] * a[ 11 ] * a[ 15 ];
      tmp[ 1208 ] += a[ 12 ] * a[ 3 ] * a[ 11 ] * a[ 16 ];
      tmp[ 1209 ] += a[ 12 ] * a[ 3 ] * a[ 11 ] * a[ 17 ];
      tmp[ 1210 ] += a[ 12 ] * a[ 3 ] * a[ 11 ] * a[ 18 ];
      tmp[ 1211 ] += a[ 12 ] * a[ 3 ] * a[ 11 ] * a[ 19 ];
      tmp[ 1212 ] += a[ 12 ] * a[ 3 ] * a[ 11 ] * a[ 20 ];
      tmp[ 1213 ] += a[ 12 ] * a[ 3 ] * a[ 11 ] * a[ 21 ];
      tmp[ 1214 ] += a[ 12 ] * a[ 3 ] * a[ 11 ] * a[ 22 ];
      tmp[ 1215 ] += a[ 12 ] * a[ 3 ] * a[ 11 ] * a[ 23 ];
      tmp[ 1216 ] += a[ 12 ] * a[ 3 ] * a[ 11 ] * a[ 24 ];
      tmp[ 1217 ] += a[ 12 ] * a[ 3 ] * a[ 12 ] * a[ 12 ];
      tmp[ 1218 ] += a[ 12 ] * a[ 3 ] * a[ 12 ] * a[ 15 ];
      tmp[ 1219 ] += a[ 12 ] * a[ 3 ] * a[ 12 ] * a[ 16 ];
      tmp[ 1220 ] += a[ 12 ] * a[ 3 ] * a[ 12 ] * a[ 17 ];
      tmp[ 1221 ] += a[ 12 ] * a[ 3 ] * a[ 12 ] * a[ 18 ];
      tmp[ 1222 ] += a[ 12 ] * a[ 3 ] * a[ 12 ] * a[ 19 ];
      tmp[ 1223 ] += a[ 12 ] * a[ 3 ] * a[ 12 ] * a[ 20 ];
      tmp[ 1224 ] += a[ 12 ] * a[ 3 ] * a[ 12 ] * a[ 21 ];
      tmp[ 1225 ] += a[ 12 ] * a[ 3 ] * a[ 12 ] * a[ 22 ];
      tmp[ 1226 ] += a[ 12 ] * a[ 3 ] * a[ 12 ] * a[ 23 ];
      tmp[ 1227 ] += a[ 12 ] * a[ 3 ] * a[ 12 ] * a[ 24 ];
      tmp[ 1228 ] += a[ 12 ] * a[ 3 ] * a[ 13 ] * a[ 15 ];
      tmp[ 1229 ] += a[ 12 ] * a[ 3 ] * a[ 13 ] * a[ 20 ];
      tmp[ 1230 ] += a[ 12 ] * a[ 3 ] * a[ 14 ] * a[ 15 ];
      tmp[ 1231 ] += a[ 12 ] * a[ 3 ] * a[ 14 ] * a[ 16 ];
      tmp[ 1232 ] += a[ 12 ] * a[ 3 ] * a[ 14 ] * a[ 20 ];
      tmp[ 1233 ] += a[ 12 ] * a[ 3 ] * a[ 14 ] * a[ 21 ];
      tmp[ 1234 ] += a[ 12 ] * a[ 3 ] * a[ 15 ] * a[ 15 ];
      tmp[ 1235 ] += a[ 12 ] * a[ 3 ] * a[ 15 ] * a[ 16 ];
      tmp[ 1236 ] += a[ 12 ] * a[ 3 ] * a[ 15 ] * a[ 17 ];
      tmp[ 1237 ] += a[ 12 ] * a[ 3 ] * a[ 15 ] * a[ 18 ];
      tmp[ 1238 ] += a[ 12 ] * a[ 3 ] * a[ 15 ] * a[ 19 ];
      tmp[ 1239 ] += a[ 12 ] * a[ 3 ] * a[ 15 ] * a[ 20 ];
      tmp[ 1240 ] += a[ 12 ] * a[ 3 ] * a[ 15 ] * a[ 21 ];
      tmp[ 1241 ] += a[ 12 ] * a[ 3 ] * a[ 15 ] * a[ 22 ];
      tmp[ 1242 ] += a[ 12 ] * a[ 3 ] * a[ 15 ] * a[ 23 ];
      tmp[ 1243 ] += a[ 12 ] * a[ 3 ] * a[ 15 ] * a[ 24 ];
      tmp[ 1244 ] += a[ 12 ] * a[ 3 ] * a[ 16 ] * a[ 16 ];
      tmp[ 1245 ] += a[ 12 ] * a[ 3 ] * a[ 16 ] * a[ 17 ];
      tmp[ 1246 ] += a[ 12 ] * a[ 3 ] * a[ 16 ] * a[ 18 ];
      tmp[ 1247 ] += a[ 12 ] * a[ 3 ] * a[ 16 ] * a[ 19 ];
      tmp[ 1248 ] += a[ 12 ] * a[ 3 ] * a[ 16 ] * a[ 20 ];
      tmp[ 1249 ] += a[ 12 ] * a[ 3 ] * a[ 16 ] * a[ 21 ];
      tmp[ 1250 ] += a[ 12 ] * a[ 3 ] * a[ 16 ] * a[ 22 ];
      tmp[ 1251 ] += a[ 12 ] * a[ 3 ] * a[ 16 ] * a[ 23 ];
      tmp[ 1252 ] += a[ 12 ] * a[ 3 ] * a[ 16 ] * a[ 24 ];
      tmp[ 1253 ] += a[ 12 ] * a[ 3 ] * a[ 17 ] * a[ 17 ];
      tmp[ 1254 ] += a[ 12 ] * a[ 3 ] * a[ 17 ] * a[ 18 ];
      tmp[ 1255 ] += a[ 12 ] * a[ 3 ] * a[ 17 ] * a[ 19 ];
      tmp[ 1256 ] += a[ 12 ] * a[ 3 ] * a[ 17 ] * a[ 20 ];
      tmp[ 1257 ] += a[ 12 ] * a[ 3 ] * a[ 17 ] * a[ 21 ];
      tmp[ 1258 ] += a[ 12 ] * a[ 3 ] * a[ 17 ] * a[ 22 ];
      tmp[ 1259 ] += a[ 12 ] * a[ 3 ] * a[ 17 ] * a[ 23 ];
      tmp[ 1260 ] += a[ 12 ] * a[ 3 ] * a[ 17 ] * a[ 24 ];
      tmp[ 1261 ] += a[ 12 ] * a[ 3 ] * a[ 18 ] * a[ 18 ];
      tmp[ 1262 ] += a[ 12 ] * a[ 3 ] * a[ 18 ] * a[ 19 ];
      tmp[ 1263 ] += a[ 12 ] * a[ 3 ] * a[ 18 ] * a[ 20 ];
      tmp[ 1264 ] += a[ 12 ] * a[ 3 ] * a[ 18 ] * a[ 21 ];
      tmp[ 1265 ] += a[ 12 ] * a[ 3 ] * a[ 18 ] * a[ 22 ];
      tmp[ 1266 ] += a[ 12 ] * a[ 3 ] * a[ 18 ] * a[ 23 ];
      tmp[ 1267 ] += a[ 12 ] * a[ 3 ] * a[ 18 ] * a[ 24 ];
      tmp[ 1268 ] += a[ 12 ] * a[ 3 ] * a[ 19 ] * a[ 19 ];
      tmp[ 1269 ] += a[ 12 ] * a[ 3 ] * a[ 19 ] * a[ 20 ];
      tmp[ 1270 ] += a[ 12 ] * a[ 3 ] * a[ 19 ] * a[ 21 ];
      tmp[ 1271 ] += a[ 12 ] * a[ 3 ] * a[ 19 ] * a[ 22 ];
      tmp[ 1272 ] += a[ 12 ] * a[ 3 ] * a[ 19 ] * a[ 23 ];
      tmp[ 1273 ] += a[ 12 ] * a[ 3 ] * a[ 19 ] * a[ 24 ];
      tmp[ 1274 ] += a[ 12 ] * a[ 3 ] * a[ 20 ] * a[ 20 ];
      tmp[ 1275 ] += a[ 12 ] * a[ 3 ] * a[ 20 ] * a[ 21 ];
      tmp[ 1276 ] += a[ 12 ] * a[ 3 ] * a[ 20 ] * a[ 22 ];
      tmp[ 1277 ] += a[ 12 ] * a[ 3 ] * a[ 20 ] * a[ 23 ];
      tmp[ 1278 ] += a[ 12 ] * a[ 3 ] * a[ 20 ] * a[ 24 ];
      tmp[ 1279 ] += a[ 12 ] * a[ 3 ] * a[ 21 ] * a[ 21 ];
      tmp[ 1280 ] += a[ 12 ] * a[ 3 ] * a[ 21 ] * a[ 22 ];
      tmp[ 1281 ] += a[ 12 ] * a[ 3 ] * a[ 21 ] * a[ 23 ];
      tmp[ 1282 ] += a[ 12 ] * a[ 3 ] * a[ 21 ] * a[ 24 ];
      tmp[ 1283 ] += a[ 12 ] * a[ 3 ] * a[ 22 ] * a[ 22 ];
      tmp[ 1284 ] += a[ 12 ] * a[ 3 ] * a[ 22 ] * a[ 23 ];
      tmp[ 1285 ] += a[ 12 ] * a[ 3 ] * a[ 22 ] * a[ 24 ];
      tmp[ 1286 ] += a[ 12 ] * a[ 3 ] * a[ 23 ] * a[ 23 ];
      tmp[ 1287 ] += a[ 12 ] * a[ 3 ] * a[ 23 ] * a[ 24 ];
      tmp[ 1288 ] += a[ 12 ] * a[ 3 ] * a[ 24 ] * a[ 24 ];
      tmp[ 1289 ] += a[ 12 ] * a[ 4 ] * a[ 4 ] * a[ 4 ];
      tmp[ 1290 ] += a[ 12 ] * a[ 4 ] * a[ 4 ] * a[ 5 ];
      tmp[ 1291 ] += a[ 12 ] * a[ 4 ] * a[ 4 ] * a[ 6 ];
      tmp[ 1292 ] += a[ 12 ] * a[ 4 ] * a[ 4 ] * a[ 7 ];
      tmp[ 1293 ] += a[ 12 ] * a[ 4 ] * a[ 4 ] * a[ 8 ];
      tmp[ 1294 ] += a[ 12 ] * a[ 4 ] * a[ 4 ] * a[ 9 ];
      tmp[ 1295 ] += a[ 12 ] * a[ 4 ] * a[ 4 ] * a[ 10 ];
      tmp[ 1296 ] += a[ 12 ] * a[ 4 ] * a[ 4 ] * a[ 11 ];
      tmp[ 1297 ] += a[ 12 ] * a[ 4 ] * a[ 4 ] * a[ 12 ];
      tmp[ 1298 ] += a[ 12 ] * a[ 4 ] * a[ 4 ] * a[ 15 ];
      tmp[ 1299 ] += a[ 12 ] * a[ 4 ] * a[ 4 ] * a[ 16 ];
      tmp[ 1300 ] += a[ 12 ] * a[ 4 ] * a[ 4 ] * a[ 17 ];
      tmp[ 1301 ] += a[ 12 ] * a[ 4 ] * a[ 4 ] * a[ 18 ];
      tmp[ 1302 ] += a[ 12 ] * a[ 4 ] * a[ 4 ] * a[ 19 ];
      tmp[ 1303 ] += a[ 12 ] * a[ 4 ] * a[ 4 ] * a[ 20 ];
      tmp[ 1304 ] += a[ 12 ] * a[ 4 ] * a[ 4 ] * a[ 21 ];
      tmp[ 1305 ] += a[ 12 ] * a[ 4 ] * a[ 4 ] * a[ 22 ];
      tmp[ 1306 ] += a[ 12 ] * a[ 4 ] * a[ 4 ] * a[ 23 ];
      tmp[ 1307 ] += a[ 12 ] * a[ 4 ] * a[ 4 ] * a[ 24 ];
      tmp[ 1308 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 5 ];
      tmp[ 1309 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 6 ];
      tmp[ 1310 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 7 ];
      tmp[ 1311 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 8 ];
      tmp[ 1312 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 9 ];
      tmp[ 1313 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 10 ];
      tmp[ 1314 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 11 ];
      tmp[ 1315 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 12 ];
      tmp[ 1316 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 13 ];
      tmp[ 1317 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 14 ];
      tmp[ 1318 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 15 ];
      tmp[ 1319 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 16 ];
      tmp[ 1320 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 17 ];
      tmp[ 1321 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 18 ];
      tmp[ 1322 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 19 ];
      tmp[ 1323 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 20 ];
      tmp[ 1324 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 21 ];
      tmp[ 1325 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 22 ];
      tmp[ 1326 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 23 ];
      tmp[ 1327 ] += a[ 12 ] * a[ 4 ] * a[ 5 ] * a[ 24 ];
      tmp[ 1328 ] += a[ 12 ] * a[ 4 ] * a[ 6 ] * a[ 6 ];
      tmp[ 1329 ] += a[ 12 ] * a[ 4 ] * a[ 6 ] * a[ 7 ];
      tmp[ 1330 ] += a[ 12 ] * a[ 4 ] * a[ 6 ] * a[ 8 ];
      tmp[ 1331 ] += a[ 12 ] * a[ 4 ] * a[ 6 ] * a[ 9 ];
      tmp[ 1332 ] += a[ 12 ] * a[ 4 ] * a[ 6 ] * a[ 10 ];
      tmp[ 1333 ] += a[ 12 ] * a[ 4 ] * a[ 6 ] * a[ 11 ];
      tmp[ 1334 ] += a[ 12 ] * a[ 4 ] * a[ 6 ] * a[ 12 ];
      tmp[ 1335 ] += a[ 12 ] * a[ 4 ] * a[ 6 ] * a[ 14 ];
      tmp[ 1336 ] += a[ 12 ] * a[ 4 ] * a[ 6 ] * a[ 15 ];
      tmp[ 1337 ] += a[ 12 ] * a[ 4 ] * a[ 6 ] * a[ 16 ];
      tmp[ 1338 ] += a[ 12 ] * a[ 4 ] * a[ 6 ] * a[ 17 ];
      tmp[ 1339 ] += a[ 12 ] * a[ 4 ] * a[ 6 ] * a[ 18 ];
      tmp[ 1340 ] += a[ 12 ] * a[ 4 ] * a[ 6 ] * a[ 19 ];
      tmp[ 1341 ] += a[ 12 ] * a[ 4 ] * a[ 6 ] * a[ 20 ];
      tmp[ 1342 ] += a[ 12 ] * a[ 4 ] * a[ 6 ] * a[ 21 ];
      tmp[ 1343 ] += a[ 12 ] * a[ 4 ] * a[ 6 ] * a[ 22 ];
      tmp[ 1344 ] += a[ 12 ] * a[ 4 ] * a[ 6 ] * a[ 23 ];
      tmp[ 1345 ] += a[ 12 ] * a[ 4 ] * a[ 6 ] * a[ 24 ];
      tmp[ 1346 ] += a[ 12 ] * a[ 4 ] * a[ 7 ] * a[ 7 ];
      tmp[ 1347 ] += a[ 12 ] * a[ 4 ] * a[ 7 ] * a[ 8 ];
      tmp[ 1348 ] += a[ 12 ] * a[ 4 ] * a[ 7 ] * a[ 9 ];
      tmp[ 1349 ] += a[ 12 ] * a[ 4 ] * a[ 7 ] * a[ 10 ];
      tmp[ 1350 ] += a[ 12 ] * a[ 4 ] * a[ 7 ] * a[ 11 ];
      tmp[ 1351 ] += a[ 12 ] * a[ 4 ] * a[ 7 ] * a[ 12 ];
      tmp[ 1352 ] += a[ 12 ] * a[ 4 ] * a[ 7 ] * a[ 15 ];
      tmp[ 1353 ] += a[ 12 ] * a[ 4 ] * a[ 7 ] * a[ 16 ];
      tmp[ 1354 ] += a[ 12 ] * a[ 4 ] * a[ 7 ] * a[ 17 ];
      tmp[ 1355 ] += a[ 12 ] * a[ 4 ] * a[ 7 ] * a[ 18 ];
      tmp[ 1356 ] += a[ 12 ] * a[ 4 ] * a[ 7 ] * a[ 19 ];
      tmp[ 1357 ] += a[ 12 ] * a[ 4 ] * a[ 7 ] * a[ 20 ];
      tmp[ 1358 ] += a[ 12 ] * a[ 4 ] * a[ 7 ] * a[ 21 ];
      tmp[ 1359 ] += a[ 12 ] * a[ 4 ] * a[ 7 ] * a[ 22 ];
      tmp[ 1360 ] += a[ 12 ] * a[ 4 ] * a[ 7 ] * a[ 23 ];
      tmp[ 1361 ] += a[ 12 ] * a[ 4 ] * a[ 7 ] * a[ 24 ];
      tmp[ 1362 ] += a[ 12 ] * a[ 4 ] * a[ 8 ] * a[ 8 ];
      tmp[ 1363 ] += a[ 12 ] * a[ 4 ] * a[ 8 ] * a[ 9 ];
      tmp[ 1364 ] += a[ 12 ] * a[ 4 ] * a[ 8 ] * a[ 10 ];
      tmp[ 1365 ] += a[ 12 ] * a[ 4 ] * a[ 8 ] * a[ 11 ];
      tmp[ 1366 ] += a[ 12 ] * a[ 4 ] * a[ 8 ] * a[ 12 ];
      tmp[ 1367 ] += a[ 12 ] * a[ 4 ] * a[ 8 ] * a[ 15 ];
      tmp[ 1368 ] += a[ 12 ] * a[ 4 ] * a[ 8 ] * a[ 16 ];
      tmp[ 1369 ] += a[ 12 ] * a[ 4 ] * a[ 8 ] * a[ 17 ];
      tmp[ 1370 ] += a[ 12 ] * a[ 4 ] * a[ 8 ] * a[ 18 ];
      tmp[ 1371 ] += a[ 12 ] * a[ 4 ] * a[ 8 ] * a[ 19 ];
      tmp[ 1372 ] += a[ 12 ] * a[ 4 ] * a[ 8 ] * a[ 20 ];
      tmp[ 1373 ] += a[ 12 ] * a[ 4 ] * a[ 8 ] * a[ 21 ];
      tmp[ 1374 ] += a[ 12 ] * a[ 4 ] * a[ 8 ] * a[ 22 ];
      tmp[ 1375 ] += a[ 12 ] * a[ 4 ] * a[ 8 ] * a[ 23 ];
      tmp[ 1376 ] += a[ 12 ] * a[ 4 ] * a[ 8 ] * a[ 24 ];
      tmp[ 1377 ] += a[ 12 ] * a[ 4 ] * a[ 9 ] * a[ 9 ];
      tmp[ 1378 ] += a[ 12 ] * a[ 4 ] * a[ 9 ] * a[ 10 ];
      tmp[ 1379 ] += a[ 12 ] * a[ 4 ] * a[ 9 ] * a[ 11 ];
      tmp[ 1380 ] += a[ 12 ] * a[ 4 ] * a[ 9 ] * a[ 12 ];
      tmp[ 1381 ] += a[ 12 ] * a[ 4 ] * a[ 9 ] * a[ 15 ];
      tmp[ 1382 ] += a[ 12 ] * a[ 4 ] * a[ 9 ] * a[ 16 ];
      tmp[ 1383 ] += a[ 12 ] * a[ 4 ] * a[ 9 ] * a[ 17 ];
      tmp[ 1384 ] += a[ 12 ] * a[ 4 ] * a[ 9 ] * a[ 18 ];
      tmp[ 1385 ] += a[ 12 ] * a[ 4 ] * a[ 9 ] * a[ 19 ];
      tmp[ 1386 ] += a[ 12 ] * a[ 4 ] * a[ 9 ] * a[ 20 ];
      tmp[ 1387 ] += a[ 12 ] * a[ 4 ] * a[ 9 ] * a[ 21 ];
      tmp[ 1388 ] += a[ 12 ] * a[ 4 ] * a[ 9 ] * a[ 22 ];
      tmp[ 1389 ] += a[ 12 ] * a[ 4 ] * a[ 9 ] * a[ 23 ];
      tmp[ 1390 ] += a[ 12 ] * a[ 4 ] * a[ 9 ] * a[ 24 ];
      tmp[ 1391 ] += a[ 12 ] * a[ 4 ] * a[ 10 ] * a[ 10 ];
      tmp[ 1392 ] += a[ 12 ] * a[ 4 ] * a[ 10 ] * a[ 11 ];
      tmp[ 1393 ] += a[ 12 ] * a[ 4 ] * a[ 10 ] * a[ 12 ];
      tmp[ 1394 ] += a[ 12 ] * a[ 4 ] * a[ 10 ] * a[ 13 ];
      tmp[ 1395 ] += a[ 12 ] * a[ 4 ] * a[ 10 ] * a[ 14 ];
      tmp[ 1396 ] += a[ 12 ] * a[ 4 ] * a[ 10 ] * a[ 15 ];
      tmp[ 1397 ] += a[ 12 ] * a[ 4 ] * a[ 10 ] * a[ 16 ];
      tmp[ 1398 ] += a[ 12 ] * a[ 4 ] * a[ 10 ] * a[ 17 ];
      tmp[ 1399 ] += a[ 12 ] * a[ 4 ] * a[ 10 ] * a[ 18 ];
      tmp[ 1400 ] += a[ 12 ] * a[ 4 ] * a[ 10 ] * a[ 19 ];
      tmp[ 1401 ] += a[ 12 ] * a[ 4 ] * a[ 10 ] * a[ 20 ];
      tmp[ 1402 ] += a[ 12 ] * a[ 4 ] * a[ 10 ] * a[ 21 ];
      tmp[ 1403 ] += a[ 12 ] * a[ 4 ] * a[ 10 ] * a[ 22 ];
      tmp[ 1404 ] += a[ 12 ] * a[ 4 ] * a[ 10 ] * a[ 23 ];
      tmp[ 1405 ] += a[ 12 ] * a[ 4 ] * a[ 10 ] * a[ 24 ];
      tmp[ 1406 ] += a[ 12 ] * a[ 4 ] * a[ 11 ] * a[ 11 ];
      tmp[ 1407 ] += a[ 12 ] * a[ 4 ] * a[ 11 ] * a[ 12 ];
      tmp[ 1408 ] += a[ 12 ] * a[ 4 ] * a[ 11 ] * a[ 14 ];
      tmp[ 1409 ] += a[ 12 ] * a[ 4 ] * a[ 11 ] * a[ 15 ];
      tmp[ 1410 ] += a[ 12 ] * a[ 4 ] * a[ 11 ] * a[ 16 ];
      tmp[ 1411 ] += a[ 12 ] * a[ 4 ] * a[ 11 ] * a[ 17 ];
      tmp[ 1412 ] += a[ 12 ] * a[ 4 ] * a[ 11 ] * a[ 18 ];
      tmp[ 1413 ] += a[ 12 ] * a[ 4 ] * a[ 11 ] * a[ 19 ];
      tmp[ 1414 ] += a[ 12 ] * a[ 4 ] * a[ 11 ] * a[ 20 ];
      tmp[ 1415 ] += a[ 12 ] * a[ 4 ] * a[ 11 ] * a[ 21 ];
      tmp[ 1416 ] += a[ 12 ] * a[ 4 ] * a[ 11 ] * a[ 22 ];
      tmp[ 1417 ] += a[ 12 ] * a[ 4 ] * a[ 11 ] * a[ 23 ];
      tmp[ 1418 ] += a[ 12 ] * a[ 4 ] * a[ 11 ] * a[ 24 ];
      tmp[ 1419 ] += a[ 12 ] * a[ 4 ] * a[ 12 ] * a[ 12 ];
      tmp[ 1420 ] += a[ 12 ] * a[ 4 ] * a[ 12 ] * a[ 15 ];
      tmp[ 1421 ] += a[ 12 ] * a[ 4 ] * a[ 12 ] * a[ 16 ];
      tmp[ 1422 ] += a[ 12 ] * a[ 4 ] * a[ 12 ] * a[ 17 ];
      tmp[ 1423 ] += a[ 12 ] * a[ 4 ] * a[ 12 ] * a[ 18 ];
      tmp[ 1424 ] += a[ 12 ] * a[ 4 ] * a[ 12 ] * a[ 19 ];
      tmp[ 1425 ] += a[ 12 ] * a[ 4 ] * a[ 12 ] * a[ 20 ];
      tmp[ 1426 ] += a[ 12 ] * a[ 4 ] * a[ 12 ] * a[ 21 ];
      tmp[ 1427 ] += a[ 12 ] * a[ 4 ] * a[ 12 ] * a[ 22 ];
      tmp[ 1428 ] += a[ 12 ] * a[ 4 ] * a[ 12 ] * a[ 23 ];
      tmp[ 1429 ] += a[ 12 ] * a[ 4 ] * a[ 12 ] * a[ 24 ];
      tmp[ 1430 ] += a[ 12 ] * a[ 4 ] * a[ 13 ] * a[ 15 ];
      tmp[ 1431 ] += a[ 12 ] * a[ 4 ] * a[ 13 ] * a[ 20 ];
      tmp[ 1432 ] += a[ 12 ] * a[ 4 ] * a[ 14 ] * a[ 15 ];
      tmp[ 1433 ] += a[ 12 ] * a[ 4 ] * a[ 14 ] * a[ 16 ];
      tmp[ 1434 ] += a[ 12 ] * a[ 4 ] * a[ 14 ] * a[ 20 ];
      tmp[ 1435 ] += a[ 12 ] * a[ 4 ] * a[ 14 ] * a[ 21 ];
      tmp[ 1436 ] += a[ 12 ] * a[ 4 ] * a[ 15 ] * a[ 15 ];
      tmp[ 1437 ] += a[ 12 ] * a[ 4 ] * a[ 15 ] * a[ 16 ];
      tmp[ 1438 ] += a[ 12 ] * a[ 4 ] * a[ 15 ] * a[ 17 ];
      tmp[ 1439 ] += a[ 12 ] * a[ 4 ] * a[ 15 ] * a[ 18 ];
      tmp[ 1440 ] += a[ 12 ] * a[ 4 ] * a[ 15 ] * a[ 19 ];
      tmp[ 1441 ] += a[ 12 ] * a[ 4 ] * a[ 15 ] * a[ 20 ];
      tmp[ 1442 ] += a[ 12 ] * a[ 4 ] * a[ 15 ] * a[ 21 ];
      tmp[ 1443 ] += a[ 12 ] * a[ 4 ] * a[ 15 ] * a[ 22 ];
      tmp[ 1444 ] += a[ 12 ] * a[ 4 ] * a[ 15 ] * a[ 23 ];
      tmp[ 1445 ] += a[ 12 ] * a[ 4 ] * a[ 15 ] * a[ 24 ];
      tmp[ 1446 ] += a[ 12 ] * a[ 4 ] * a[ 16 ] * a[ 16 ];
      tmp[ 1447 ] += a[ 12 ] * a[ 4 ] * a[ 16 ] * a[ 17 ];
      tmp[ 1448 ] += a[ 12 ] * a[ 4 ] * a[ 16 ] * a[ 18 ];
      tmp[ 1449 ] += a[ 12 ] * a[ 4 ] * a[ 16 ] * a[ 19 ];
      tmp[ 1450 ] += a[ 12 ] * a[ 4 ] * a[ 16 ] * a[ 20 ];
      tmp[ 1451 ] += a[ 12 ] * a[ 4 ] * a[ 16 ] * a[ 21 ];
      tmp[ 1452 ] += a[ 12 ] * a[ 4 ] * a[ 16 ] * a[ 22 ];
      tmp[ 1453 ] += a[ 12 ] * a[ 4 ] * a[ 16 ] * a[ 23 ];
      tmp[ 1454 ] += a[ 12 ] * a[ 4 ] * a[ 16 ] * a[ 24 ];
      tmp[ 1455 ] += a[ 12 ] * a[ 4 ] * a[ 17 ] * a[ 17 ];
      tmp[ 1456 ] += a[ 12 ] * a[ 4 ] * a[ 17 ] * a[ 18 ];
      tmp[ 1457 ] += a[ 12 ] * a[ 4 ] * a[ 17 ] * a[ 19 ];
      tmp[ 1458 ] += a[ 12 ] * a[ 4 ] * a[ 17 ] * a[ 20 ];
      tmp[ 1459 ] += a[ 12 ] * a[ 4 ] * a[ 17 ] * a[ 21 ];
      tmp[ 1460 ] += a[ 12 ] * a[ 4 ] * a[ 17 ] * a[ 22 ];
      tmp[ 1461 ] += a[ 12 ] * a[ 4 ] * a[ 17 ] * a[ 23 ];
      tmp[ 1462 ] += a[ 12 ] * a[ 4 ] * a[ 17 ] * a[ 24 ];
      tmp[ 1463 ] += a[ 12 ] * a[ 4 ] * a[ 18 ] * a[ 18 ];
      tmp[ 1464 ] += a[ 12 ] * a[ 4 ] * a[ 18 ] * a[ 19 ];
      tmp[ 1465 ] += a[ 12 ] * a[ 4 ] * a[ 18 ] * a[ 20 ];
      tmp[ 1466 ] += a[ 12 ] * a[ 4 ] * a[ 18 ] * a[ 21 ];
      tmp[ 1467 ] += a[ 12 ] * a[ 4 ] * a[ 18 ] * a[ 22 ];
      tmp[ 1468 ] += a[ 12 ] * a[ 4 ] * a[ 18 ] * a[ 23 ];
      tmp[ 1469 ] += a[ 12 ] * a[ 4 ] * a[ 18 ] * a[ 24 ];
      tmp[ 1470 ] += a[ 12 ] * a[ 4 ] * a[ 19 ] * a[ 19 ];
      tmp[ 1471 ] += a[ 12 ] * a[ 4 ] * a[ 19 ] * a[ 20 ];
      tmp[ 1472 ] += a[ 12 ] * a[ 4 ] * a[ 19 ] * a[ 21 ];
      tmp[ 1473 ] += a[ 12 ] * a[ 4 ] * a[ 19 ] * a[ 22 ];
      tmp[ 1474 ] += a[ 12 ] * a[ 4 ] * a[ 19 ] * a[ 23 ];
      tmp[ 1475 ] += a[ 12 ] * a[ 4 ] * a[ 19 ] * a[ 24 ];
      tmp[ 1476 ] += a[ 12 ] * a[ 4 ] * a[ 20 ] * a[ 20 ];
      tmp[ 1477 ] += a[ 12 ] * a[ 4 ] * a[ 20 ] * a[ 21 ];
      tmp[ 1478 ] += a[ 12 ] * a[ 4 ] * a[ 20 ] * a[ 22 ];
      tmp[ 1479 ] += a[ 12 ] * a[ 4 ] * a[ 20 ] * a[ 23 ];
      tmp[ 1480 ] += a[ 12 ] * a[ 4 ] * a[ 20 ] * a[ 24 ];
      tmp[ 1481 ] += a[ 12 ] * a[ 4 ] * a[ 21 ] * a[ 21 ];
      tmp[ 1482 ] += a[ 12 ] * a[ 4 ] * a[ 21 ] * a[ 22 ];
      tmp[ 1483 ] += a[ 12 ] * a[ 4 ] * a[ 21 ] * a[ 23 ];
      tmp[ 1484 ] += a[ 12 ] * a[ 4 ] * a[ 21 ] * a[ 24 ];
      tmp[ 1485 ] += a[ 12 ] * a[ 4 ] * a[ 22 ] * a[ 22 ];
      tmp[ 1486 ] += a[ 12 ] * a[ 4 ] * a[ 22 ] * a[ 23 ];
      tmp[ 1487 ] += a[ 12 ] * a[ 4 ] * a[ 22 ] * a[ 24 ];
      tmp[ 1488 ] += a[ 12 ] * a[ 4 ] * a[ 23 ] * a[ 23 ];
      tmp[ 1489 ] += a[ 12 ] * a[ 4 ] * a[ 23 ] * a[ 24 ];
      tmp[ 1490 ] += a[ 12 ] * a[ 4 ] * a[ 24 ] * a[ 24 ];
      tmp[ 1491 ] += a[ 12 ] * a[ 5 ] * a[ 5 ] * a[ 5 ];
      tmp[ 1492 ] += a[ 12 ] * a[ 5 ] * a[ 5 ] * a[ 6 ];
      tmp[ 1493 ] += a[ 12 ] * a[ 5 ] * a[ 5 ] * a[ 7 ];
      tmp[ 1494 ] += a[ 12 ] * a[ 5 ] * a[ 5 ] * a[ 8 ];
      tmp[ 1495 ] += a[ 12 ] * a[ 5 ] * a[ 5 ] * a[ 9 ];
      tmp[ 1496 ] += a[ 12 ] * a[ 5 ] * a[ 5 ] * a[ 10 ];
      tmp[ 1497 ] += a[ 12 ] * a[ 5 ] * a[ 5 ] * a[ 11 ];
      tmp[ 1498 ] += a[ 12 ] * a[ 5 ] * a[ 5 ] * a[ 12 ];
      tmp[ 1499 ] += a[ 12 ] * a[ 5 ] * a[ 5 ] * a[ 13 ];
      tmp[ 1500 ] += a[ 12 ] * a[ 5 ] * a[ 5 ] * a[ 14 ];
      tmp[ 1501 ] += a[ 12 ] * a[ 5 ] * a[ 5 ] * a[ 18 ];
      tmp[ 1502 ] += a[ 12 ] * a[ 5 ] * a[ 5 ] * a[ 19 ];
      tmp[ 1503 ] += a[ 12 ] * a[ 5 ] * a[ 5 ] * a[ 20 ];
      tmp[ 1504 ] += a[ 12 ] * a[ 5 ] * a[ 5 ] * a[ 21 ];
      tmp[ 1505 ] += a[ 12 ] * a[ 5 ] * a[ 5 ] * a[ 22 ];
      tmp[ 1506 ] += a[ 12 ] * a[ 5 ] * a[ 5 ] * a[ 23 ];
      tmp[ 1507 ] += a[ 12 ] * a[ 5 ] * a[ 5 ] * a[ 24 ];
      tmp[ 1508 ] += a[ 12 ] * a[ 5 ] * a[ 6 ] * a[ 6 ];
      tmp[ 1509 ] += a[ 12 ] * a[ 5 ] * a[ 6 ] * a[ 7 ];
      tmp[ 1510 ] += a[ 12 ] * a[ 5 ] * a[ 6 ] * a[ 8 ];
      tmp[ 1511 ] += a[ 12 ] * a[ 5 ] * a[ 6 ] * a[ 9 ];
      tmp[ 1512 ] += a[ 12 ] * a[ 5 ] * a[ 6 ] * a[ 10 ];
      tmp[ 1513 ] += a[ 12 ] * a[ 5 ] * a[ 6 ] * a[ 11 ];
      tmp[ 1514 ] += a[ 12 ] * a[ 5 ] * a[ 6 ] * a[ 12 ];
      tmp[ 1515 ] += a[ 12 ] * a[ 5 ] * a[ 6 ] * a[ 13 ];
      tmp[ 1516 ] += a[ 12 ] * a[ 5 ] * a[ 6 ] * a[ 14 ];
      tmp[ 1517 ] += a[ 12 ] * a[ 5 ] * a[ 6 ] * a[ 18 ];
      tmp[ 1518 ] += a[ 12 ] * a[ 5 ] * a[ 6 ] * a[ 19 ];
      tmp[ 1519 ] += a[ 12 ] * a[ 5 ] * a[ 6 ] * a[ 20 ];
      tmp[ 1520 ] += a[ 12 ] * a[ 5 ] * a[ 6 ] * a[ 21 ];
      tmp[ 1521 ] += a[ 12 ] * a[ 5 ] * a[ 6 ] * a[ 22 ];
      tmp[ 1522 ] += a[ 12 ] * a[ 5 ] * a[ 6 ] * a[ 23 ];
      tmp[ 1523 ] += a[ 12 ] * a[ 5 ] * a[ 6 ] * a[ 24 ];
      tmp[ 1524 ] += a[ 12 ] * a[ 5 ] * a[ 7 ] * a[ 7 ];
      tmp[ 1525 ] += a[ 12 ] * a[ 5 ] * a[ 7 ] * a[ 8 ];
      tmp[ 1526 ] += a[ 12 ] * a[ 5 ] * a[ 7 ] * a[ 9 ];
      tmp[ 1527 ] += a[ 12 ] * a[ 5 ] * a[ 7 ] * a[ 10 ];
      tmp[ 1528 ] += a[ 12 ] * a[ 5 ] * a[ 7 ] * a[ 11 ];
      tmp[ 1529 ] += a[ 12 ] * a[ 5 ] * a[ 7 ] * a[ 12 ];
      tmp[ 1530 ] += a[ 12 ] * a[ 5 ] * a[ 7 ] * a[ 13 ];
      tmp[ 1531 ] += a[ 12 ] * a[ 5 ] * a[ 7 ] * a[ 14 ];
      tmp[ 1532 ] += a[ 12 ] * a[ 5 ] * a[ 7 ] * a[ 18 ];
      tmp[ 1533 ] += a[ 12 ] * a[ 5 ] * a[ 7 ] * a[ 19 ];
      tmp[ 1534 ] += a[ 12 ] * a[ 5 ] * a[ 7 ] * a[ 20 ];
      tmp[ 1535 ] += a[ 12 ] * a[ 5 ] * a[ 7 ] * a[ 21 ];
      tmp[ 1536 ] += a[ 12 ] * a[ 5 ] * a[ 7 ] * a[ 22 ];
      tmp[ 1537 ] += a[ 12 ] * a[ 5 ] * a[ 7 ] * a[ 23 ];
      tmp[ 1538 ] += a[ 12 ] * a[ 5 ] * a[ 7 ] * a[ 24 ];
      tmp[ 1539 ] += a[ 12 ] * a[ 5 ] * a[ 8 ] * a[ 8 ];
      tmp[ 1540 ] += a[ 12 ] * a[ 5 ] * a[ 8 ] * a[ 9 ];
      tmp[ 1541 ] += a[ 12 ] * a[ 5 ] * a[ 8 ] * a[ 10 ];
      tmp[ 1542 ] += a[ 12 ] * a[ 5 ] * a[ 8 ] * a[ 11 ];
      tmp[ 1543 ] += a[ 12 ] * a[ 5 ] * a[ 8 ] * a[ 12 ];
      tmp[ 1544 ] += a[ 12 ] * a[ 5 ] * a[ 8 ] * a[ 13 ];
      tmp[ 1545 ] += a[ 12 ] * a[ 5 ] * a[ 8 ] * a[ 14 ];
      tmp[ 1546 ] += a[ 12 ] * a[ 5 ] * a[ 8 ] * a[ 15 ];
      tmp[ 1547 ] += a[ 12 ] * a[ 5 ] * a[ 8 ] * a[ 18 ];
      tmp[ 1548 ] += a[ 12 ] * a[ 5 ] * a[ 8 ] * a[ 19 ];
      tmp[ 1549 ] += a[ 12 ] * a[ 5 ] * a[ 8 ] * a[ 20 ];
      tmp[ 1550 ] += a[ 12 ] * a[ 5 ] * a[ 8 ] * a[ 21 ];
      tmp[ 1551 ] += a[ 12 ] * a[ 5 ] * a[ 8 ] * a[ 22 ];
      tmp[ 1552 ] += a[ 12 ] * a[ 5 ] * a[ 8 ] * a[ 23 ];
      tmp[ 1553 ] += a[ 12 ] * a[ 5 ] * a[ 8 ] * a[ 24 ];
      tmp[ 1554 ] += a[ 12 ] * a[ 5 ] * a[ 9 ] * a[ 9 ];
      tmp[ 1555 ] += a[ 12 ] * a[ 5 ] * a[ 9 ] * a[ 10 ];
      tmp[ 1556 ] += a[ 12 ] * a[ 5 ] * a[ 9 ] * a[ 11 ];
      tmp[ 1557 ] += a[ 12 ] * a[ 5 ] * a[ 9 ] * a[ 12 ];
      tmp[ 1558 ] += a[ 12 ] * a[ 5 ] * a[ 9 ] * a[ 13 ];
      tmp[ 1559 ] += a[ 12 ] * a[ 5 ] * a[ 9 ] * a[ 14 ];
      tmp[ 1560 ] += a[ 12 ] * a[ 5 ] * a[ 9 ] * a[ 15 ];
      tmp[ 1561 ] += a[ 12 ] * a[ 5 ] * a[ 9 ] * a[ 16 ];
      tmp[ 1562 ] += a[ 12 ] * a[ 5 ] * a[ 9 ] * a[ 18 ];
      tmp[ 1563 ] += a[ 12 ] * a[ 5 ] * a[ 9 ] * a[ 19 ];
      tmp[ 1564 ] += a[ 12 ] * a[ 5 ] * a[ 9 ] * a[ 20 ];
      tmp[ 1565 ] += a[ 12 ] * a[ 5 ] * a[ 9 ] * a[ 21 ];
      tmp[ 1566 ] += a[ 12 ] * a[ 5 ] * a[ 9 ] * a[ 22 ];
      tmp[ 1567 ] += a[ 12 ] * a[ 5 ] * a[ 9 ] * a[ 23 ];
      tmp[ 1568 ] += a[ 12 ] * a[ 5 ] * a[ 9 ] * a[ 24 ];
      tmp[ 1569 ] += a[ 12 ] * a[ 5 ] * a[ 10 ] * a[ 10 ];
      tmp[ 1570 ] += a[ 12 ] * a[ 5 ] * a[ 10 ] * a[ 11 ];
      tmp[ 1571 ] += a[ 12 ] * a[ 5 ] * a[ 10 ] * a[ 12 ];
      tmp[ 1572 ] += a[ 12 ] * a[ 5 ] * a[ 10 ] * a[ 13 ];
      tmp[ 1573 ] += a[ 12 ] * a[ 5 ] * a[ 10 ] * a[ 14 ];
      tmp[ 1574 ] += a[ 12 ] * a[ 5 ] * a[ 10 ] * a[ 18 ];
      tmp[ 1575 ] += a[ 12 ] * a[ 5 ] * a[ 10 ] * a[ 19 ];
      tmp[ 1576 ] += a[ 12 ] * a[ 5 ] * a[ 10 ] * a[ 20 ];
      tmp[ 1577 ] += a[ 12 ] * a[ 5 ] * a[ 10 ] * a[ 21 ];
      tmp[ 1578 ] += a[ 12 ] * a[ 5 ] * a[ 10 ] * a[ 22 ];
      tmp[ 1579 ] += a[ 12 ] * a[ 5 ] * a[ 10 ] * a[ 23 ];
      tmp[ 1580 ] += a[ 12 ] * a[ 5 ] * a[ 10 ] * a[ 24 ];
      tmp[ 1581 ] += a[ 12 ] * a[ 5 ] * a[ 11 ] * a[ 11 ];
      tmp[ 1582 ] += a[ 12 ] * a[ 5 ] * a[ 11 ] * a[ 12 ];
      tmp[ 1583 ] += a[ 12 ] * a[ 5 ] * a[ 11 ] * a[ 13 ];
      tmp[ 1584 ] += a[ 12 ] * a[ 5 ] * a[ 11 ] * a[ 14 ];
      tmp[ 1585 ] += a[ 12 ] * a[ 5 ] * a[ 11 ] * a[ 18 ];
      tmp[ 1586 ] += a[ 12 ] * a[ 5 ] * a[ 11 ] * a[ 19 ];
      tmp[ 1587 ] += a[ 12 ] * a[ 5 ] * a[ 11 ] * a[ 20 ];
      tmp[ 1588 ] += a[ 12 ] * a[ 5 ] * a[ 11 ] * a[ 21 ];
      tmp[ 1589 ] += a[ 12 ] * a[ 5 ] * a[ 11 ] * a[ 22 ];
      tmp[ 1590 ] += a[ 12 ] * a[ 5 ] * a[ 11 ] * a[ 23 ];
      tmp[ 1591 ] += a[ 12 ] * a[ 5 ] * a[ 11 ] * a[ 24 ];
      tmp[ 1592 ] += a[ 12 ] * a[ 5 ] * a[ 12 ] * a[ 12 ];
      tmp[ 1593 ] += a[ 12 ] * a[ 5 ] * a[ 12 ] * a[ 13 ];
      tmp[ 1594 ] += a[ 12 ] * a[ 5 ] * a[ 12 ] * a[ 14 ];
      tmp[ 1595 ] += a[ 12 ] * a[ 5 ] * a[ 12 ] * a[ 18 ];
      tmp[ 1596 ] += a[ 12 ] * a[ 5 ] * a[ 12 ] * a[ 19 ];
      tmp[ 1597 ] += a[ 12 ] * a[ 5 ] * a[ 12 ] * a[ 20 ];
      tmp[ 1598 ] += a[ 12 ] * a[ 5 ] * a[ 12 ] * a[ 21 ];
      tmp[ 1599 ] += a[ 12 ] * a[ 5 ] * a[ 12 ] * a[ 22 ];
      tmp[ 1600 ] += a[ 12 ] * a[ 5 ] * a[ 12 ] * a[ 23 ];
      tmp[ 1601 ] += a[ 12 ] * a[ 5 ] * a[ 12 ] * a[ 24 ];
      tmp[ 1602 ] += a[ 12 ] * a[ 5 ] * a[ 13 ] * a[ 13 ];
      tmp[ 1603 ] += a[ 12 ] * a[ 5 ] * a[ 13 ] * a[ 14 ];
      tmp[ 1604 ] += a[ 12 ] * a[ 5 ] * a[ 13 ] * a[ 15 ];
      tmp[ 1605 ] += a[ 12 ] * a[ 5 ] * a[ 13 ] * a[ 18 ];
      tmp[ 1606 ] += a[ 12 ] * a[ 5 ] * a[ 13 ] * a[ 19 ];
      tmp[ 1607 ] += a[ 12 ] * a[ 5 ] * a[ 13 ] * a[ 20 ];
      tmp[ 1608 ] += a[ 12 ] * a[ 5 ] * a[ 13 ] * a[ 21 ];
      tmp[ 1609 ] += a[ 12 ] * a[ 5 ] * a[ 13 ] * a[ 22 ];
      tmp[ 1610 ] += a[ 12 ] * a[ 5 ] * a[ 13 ] * a[ 23 ];
      tmp[ 1611 ] += a[ 12 ] * a[ 5 ] * a[ 13 ] * a[ 24 ];
      tmp[ 1612 ] += a[ 12 ] * a[ 5 ] * a[ 14 ] * a[ 14 ];
      tmp[ 1613 ] += a[ 12 ] * a[ 5 ] * a[ 14 ] * a[ 15 ];
      tmp[ 1614 ] += a[ 12 ] * a[ 5 ] * a[ 14 ] * a[ 16 ];
      tmp[ 1615 ] += a[ 12 ] * a[ 5 ] * a[ 14 ] * a[ 18 ];
      tmp[ 1616 ] += a[ 12 ] * a[ 5 ] * a[ 14 ] * a[ 19 ];
      tmp[ 1617 ] += a[ 12 ] * a[ 5 ] * a[ 14 ] * a[ 20 ];
      tmp[ 1618 ] += a[ 12 ] * a[ 5 ] * a[ 14 ] * a[ 21 ];
      tmp[ 1619 ] += a[ 12 ] * a[ 5 ] * a[ 14 ] * a[ 22 ];
      tmp[ 1620 ] += a[ 12 ] * a[ 5 ] * a[ 14 ] * a[ 23 ];
      tmp[ 1621 ] += a[ 12 ] * a[ 5 ] * a[ 14 ] * a[ 24 ];
      tmp[ 1622 ] += a[ 12 ] * a[ 5 ] * a[ 15 ] * a[ 18 ];
      tmp[ 1623 ] += a[ 12 ] * a[ 5 ] * a[ 15 ] * a[ 19 ];
      tmp[ 1624 ] += a[ 12 ] * a[ 5 ] * a[ 15 ] * a[ 23 ];
      tmp[ 1625 ] += a[ 12 ] * a[ 5 ] * a[ 15 ] * a[ 24 ];
      tmp[ 1626 ] += a[ 12 ] * a[ 5 ] * a[ 16 ] * a[ 19 ];
      tmp[ 1627 ] += a[ 12 ] * a[ 5 ] * a[ 16 ] * a[ 24 ];
      tmp[ 1628 ] += a[ 12 ] * a[ 5 ] * a[ 18 ] * a[ 18 ];
      tmp[ 1629 ] += a[ 12 ] * a[ 5 ] * a[ 18 ] * a[ 19 ];
      tmp[ 1630 ] += a[ 12 ] * a[ 5 ] * a[ 18 ] * a[ 20 ];
      tmp[ 1631 ] += a[ 12 ] * a[ 5 ] * a[ 18 ] * a[ 21 ];
      tmp[ 1632 ] += a[ 12 ] * a[ 5 ] * a[ 18 ] * a[ 22 ];
      tmp[ 1633 ] += a[ 12 ] * a[ 5 ] * a[ 18 ] * a[ 23 ];
      tmp[ 1634 ] += a[ 12 ] * a[ 5 ] * a[ 18 ] * a[ 24 ];
      tmp[ 1635 ] += a[ 12 ] * a[ 5 ] * a[ 19 ] * a[ 19 ];
      tmp[ 1636 ] += a[ 12 ] * a[ 5 ] * a[ 19 ] * a[ 20 ];
      tmp[ 1637 ] += a[ 12 ] * a[ 5 ] * a[ 19 ] * a[ 21 ];
      tmp[ 1638 ] += a[ 12 ] * a[ 5 ] * a[ 19 ] * a[ 22 ];
      tmp[ 1639 ] += a[ 12 ] * a[ 5 ] * a[ 19 ] * a[ 23 ];
      tmp[ 1640 ] += a[ 12 ] * a[ 5 ] * a[ 19 ] * a[ 24 ];
      tmp[ 1641 ] += a[ 12 ] * a[ 5 ] * a[ 20 ] * a[ 20 ];
      tmp[ 1642 ] += a[ 12 ] * a[ 5 ] * a[ 20 ] * a[ 21 ];
      tmp[ 1643 ] += a[ 12 ] * a[ 5 ] * a[ 20 ] * a[ 22 ];
      tmp[ 1644 ] += a[ 12 ] * a[ 5 ] * a[ 20 ] * a[ 23 ];
      tmp[ 1645 ] += a[ 12 ] * a[ 5 ] * a[ 20 ] * a[ 24 ];
      tmp[ 1646 ] += a[ 12 ] * a[ 5 ] * a[ 21 ] * a[ 21 ];
      tmp[ 1647 ] += a[ 12 ] * a[ 5 ] * a[ 21 ] * a[ 22 ];
      tmp[ 1648 ] += a[ 12 ] * a[ 5 ] * a[ 21 ] * a[ 23 ];
      tmp[ 1649 ] += a[ 12 ] * a[ 5 ] * a[ 21 ] * a[ 24 ];
      tmp[ 1650 ] += a[ 12 ] * a[ 5 ] * a[ 22 ] * a[ 22 ];
      tmp[ 1651 ] += a[ 12 ] * a[ 5 ] * a[ 22 ] * a[ 23 ];
      tmp[ 1652 ] += a[ 12 ] * a[ 5 ] * a[ 22 ] * a[ 24 ];
      tmp[ 1653 ] += a[ 12 ] * a[ 5 ] * a[ 23 ] * a[ 23 ];
      tmp[ 1654 ] += a[ 12 ] * a[ 5 ] * a[ 23 ] * a[ 24 ];
      tmp[ 1655 ] += a[ 12 ] * a[ 5 ] * a[ 24 ] * a[ 24 ];
      tmp[ 1656 ] += a[ 12 ] * a[ 6 ] * a[ 6 ] * a[ 6 ];
      tmp[ 1657 ] += a[ 12 ] * a[ 6 ] * a[ 6 ] * a[ 7 ];
      tmp[ 1658 ] += a[ 12 ] * a[ 6 ] * a[ 6 ] * a[ 8 ];
      tmp[ 1659 ] += a[ 12 ] * a[ 6 ] * a[ 6 ] * a[ 9 ];
      tmp[ 1660 ] += a[ 12 ] * a[ 6 ] * a[ 6 ] * a[ 10 ];
      tmp[ 1661 ] += a[ 12 ] * a[ 6 ] * a[ 6 ] * a[ 11 ];
      tmp[ 1662 ] += a[ 12 ] * a[ 6 ] * a[ 6 ] * a[ 12 ];
      tmp[ 1663 ] += a[ 12 ] * a[ 6 ] * a[ 6 ] * a[ 14 ];
      tmp[ 1664 ] += a[ 12 ] * a[ 6 ] * a[ 6 ] * a[ 19 ];
      tmp[ 1665 ] += a[ 12 ] * a[ 6 ] * a[ 6 ] * a[ 20 ];
      tmp[ 1666 ] += a[ 12 ] * a[ 6 ] * a[ 6 ] * a[ 21 ];
      tmp[ 1667 ] += a[ 12 ] * a[ 6 ] * a[ 6 ] * a[ 22 ];
      tmp[ 1668 ] += a[ 12 ] * a[ 6 ] * a[ 6 ] * a[ 23 ];
      tmp[ 1669 ] += a[ 12 ] * a[ 6 ] * a[ 6 ] * a[ 24 ];
      tmp[ 1670 ] += a[ 12 ] * a[ 6 ] * a[ 7 ] * a[ 7 ];
      tmp[ 1671 ] += a[ 12 ] * a[ 6 ] * a[ 7 ] * a[ 8 ];
      tmp[ 1672 ] += a[ 12 ] * a[ 6 ] * a[ 7 ] * a[ 9 ];
      tmp[ 1673 ] += a[ 12 ] * a[ 6 ] * a[ 7 ] * a[ 10 ];
      tmp[ 1674 ] += a[ 12 ] * a[ 6 ] * a[ 7 ] * a[ 11 ];
      tmp[ 1675 ] += a[ 12 ] * a[ 6 ] * a[ 7 ] * a[ 12 ];
      tmp[ 1676 ] += a[ 12 ] * a[ 6 ] * a[ 7 ] * a[ 14 ];
      tmp[ 1677 ] += a[ 12 ] * a[ 6 ] * a[ 7 ] * a[ 19 ];
      tmp[ 1678 ] += a[ 12 ] * a[ 6 ] * a[ 7 ] * a[ 20 ];
      tmp[ 1679 ] += a[ 12 ] * a[ 6 ] * a[ 7 ] * a[ 21 ];
      tmp[ 1680 ] += a[ 12 ] * a[ 6 ] * a[ 7 ] * a[ 22 ];
      tmp[ 1681 ] += a[ 12 ] * a[ 6 ] * a[ 7 ] * a[ 23 ];
      tmp[ 1682 ] += a[ 12 ] * a[ 6 ] * a[ 7 ] * a[ 24 ];
      tmp[ 1683 ] += a[ 12 ] * a[ 6 ] * a[ 8 ] * a[ 8 ];
      tmp[ 1684 ] += a[ 12 ] * a[ 6 ] * a[ 8 ] * a[ 9 ];
      tmp[ 1685 ] += a[ 12 ] * a[ 6 ] * a[ 8 ] * a[ 10 ];
      tmp[ 1686 ] += a[ 12 ] * a[ 6 ] * a[ 8 ] * a[ 11 ];
      tmp[ 1687 ] += a[ 12 ] * a[ 6 ] * a[ 8 ] * a[ 12 ];
      tmp[ 1688 ] += a[ 12 ] * a[ 6 ] * a[ 8 ] * a[ 14 ];
      tmp[ 1689 ] += a[ 12 ] * a[ 6 ] * a[ 8 ] * a[ 15 ];
      tmp[ 1690 ] += a[ 12 ] * a[ 6 ] * a[ 8 ] * a[ 19 ];
      tmp[ 1691 ] += a[ 12 ] * a[ 6 ] * a[ 8 ] * a[ 20 ];
      tmp[ 1692 ] += a[ 12 ] * a[ 6 ] * a[ 8 ] * a[ 21 ];
      tmp[ 1693 ] += a[ 12 ] * a[ 6 ] * a[ 8 ] * a[ 22 ];
      tmp[ 1694 ] += a[ 12 ] * a[ 6 ] * a[ 8 ] * a[ 23 ];
      tmp[ 1695 ] += a[ 12 ] * a[ 6 ] * a[ 8 ] * a[ 24 ];
      tmp[ 1696 ] += a[ 12 ] * a[ 6 ] * a[ 9 ] * a[ 9 ];
      tmp[ 1697 ] += a[ 12 ] * a[ 6 ] * a[ 9 ] * a[ 10 ];
      tmp[ 1698 ] += a[ 12 ] * a[ 6 ] * a[ 9 ] * a[ 11 ];
      tmp[ 1699 ] += a[ 12 ] * a[ 6 ] * a[ 9 ] * a[ 12 ];
      tmp[ 1700 ] += a[ 12 ] * a[ 6 ] * a[ 9 ] * a[ 14 ];
      tmp[ 1701 ] += a[ 12 ] * a[ 6 ] * a[ 9 ] * a[ 15 ];
      tmp[ 1702 ] += a[ 12 ] * a[ 6 ] * a[ 9 ] * a[ 16 ];
      tmp[ 1703 ] += a[ 12 ] * a[ 6 ] * a[ 9 ] * a[ 19 ];
      tmp[ 1704 ] += a[ 12 ] * a[ 6 ] * a[ 9 ] * a[ 20 ];
      tmp[ 1705 ] += a[ 12 ] * a[ 6 ] * a[ 9 ] * a[ 21 ];
      tmp[ 1706 ] += a[ 12 ] * a[ 6 ] * a[ 9 ] * a[ 22 ];
      tmp[ 1707 ] += a[ 12 ] * a[ 6 ] * a[ 9 ] * a[ 23 ];
      tmp[ 1708 ] += a[ 12 ] * a[ 6 ] * a[ 9 ] * a[ 24 ];
      tmp[ 1709 ] += a[ 12 ] * a[ 6 ] * a[ 10 ] * a[ 10 ];
      tmp[ 1710 ] += a[ 12 ] * a[ 6 ] * a[ 10 ] * a[ 11 ];
      tmp[ 1711 ] += a[ 12 ] * a[ 6 ] * a[ 10 ] * a[ 12 ];
      tmp[ 1712 ] += a[ 12 ] * a[ 6 ] * a[ 10 ] * a[ 13 ];
      tmp[ 1713 ] += a[ 12 ] * a[ 6 ] * a[ 10 ] * a[ 14 ];
      tmp[ 1714 ] += a[ 12 ] * a[ 6 ] * a[ 10 ] * a[ 18 ];
      tmp[ 1715 ] += a[ 12 ] * a[ 6 ] * a[ 10 ] * a[ 19 ];
      tmp[ 1716 ] += a[ 12 ] * a[ 6 ] * a[ 10 ] * a[ 20 ];
      tmp[ 1717 ] += a[ 12 ] * a[ 6 ] * a[ 10 ] * a[ 21 ];
      tmp[ 1718 ] += a[ 12 ] * a[ 6 ] * a[ 10 ] * a[ 22 ];
      tmp[ 1719 ] += a[ 12 ] * a[ 6 ] * a[ 10 ] * a[ 23 ];
      tmp[ 1720 ] += a[ 12 ] * a[ 6 ] * a[ 10 ] * a[ 24 ];
      tmp[ 1721 ] += a[ 12 ] * a[ 6 ] * a[ 11 ] * a[ 11 ];
      tmp[ 1722 ] += a[ 12 ] * a[ 6 ] * a[ 11 ] * a[ 12 ];
      tmp[ 1723 ] += a[ 12 ] * a[ 6 ] * a[ 11 ] * a[ 14 ];
      tmp[ 1724 ] += a[ 12 ] * a[ 6 ] * a[ 11 ] * a[ 19 ];
      tmp[ 1725 ] += a[ 12 ] * a[ 6 ] * a[ 11 ] * a[ 20 ];
      tmp[ 1726 ] += a[ 12 ] * a[ 6 ] * a[ 11 ] * a[ 21 ];
      tmp[ 1727 ] += a[ 12 ] * a[ 6 ] * a[ 11 ] * a[ 22 ];
      tmp[ 1728 ] += a[ 12 ] * a[ 6 ] * a[ 11 ] * a[ 23 ];
      tmp[ 1729 ] += a[ 12 ] * a[ 6 ] * a[ 11 ] * a[ 24 ];
      tmp[ 1730 ] += a[ 12 ] * a[ 6 ] * a[ 12 ] * a[ 12 ];
      tmp[ 1731 ] += a[ 12 ] * a[ 6 ] * a[ 12 ] * a[ 14 ];
      tmp[ 1732 ] += a[ 12 ] * a[ 6 ] * a[ 12 ] * a[ 19 ];
      tmp[ 1733 ] += a[ 12 ] * a[ 6 ] * a[ 12 ] * a[ 20 ];
      tmp[ 1734 ] += a[ 12 ] * a[ 6 ] * a[ 12 ] * a[ 21 ];
      tmp[ 1735 ] += a[ 12 ] * a[ 6 ] * a[ 12 ] * a[ 22 ];
      tmp[ 1736 ] += a[ 12 ] * a[ 6 ] * a[ 12 ] * a[ 23 ];
      tmp[ 1737 ] += a[ 12 ] * a[ 6 ] * a[ 12 ] * a[ 24 ];
      tmp[ 1738 ] += a[ 12 ] * a[ 6 ] * a[ 13 ] * a[ 15 ];
      tmp[ 1739 ] += a[ 12 ] * a[ 6 ] * a[ 13 ] * a[ 20 ];
      tmp[ 1740 ] += a[ 12 ] * a[ 6 ] * a[ 14 ] * a[ 14 ];
      tmp[ 1741 ] += a[ 12 ] * a[ 6 ] * a[ 14 ] * a[ 15 ];
      tmp[ 1742 ] += a[ 12 ] * a[ 6 ] * a[ 14 ] * a[ 16 ];
      tmp[ 1743 ] += a[ 12 ] * a[ 6 ] * a[ 14 ] * a[ 19 ];
      tmp[ 1744 ] += a[ 12 ] * a[ 6 ] * a[ 14 ] * a[ 20 ];
      tmp[ 1745 ] += a[ 12 ] * a[ 6 ] * a[ 14 ] * a[ 21 ];
      tmp[ 1746 ] += a[ 12 ] * a[ 6 ] * a[ 14 ] * a[ 22 ];
      tmp[ 1747 ] += a[ 12 ] * a[ 6 ] * a[ 14 ] * a[ 23 ];
      tmp[ 1748 ] += a[ 12 ] * a[ 6 ] * a[ 14 ] * a[ 24 ];
      tmp[ 1749 ] += a[ 12 ] * a[ 6 ] * a[ 15 ] * a[ 18 ];
      tmp[ 1750 ] += a[ 12 ] * a[ 6 ] * a[ 15 ] * a[ 19 ];
      tmp[ 1751 ] += a[ 12 ] * a[ 6 ] * a[ 15 ] * a[ 23 ];
      tmp[ 1752 ] += a[ 12 ] * a[ 6 ] * a[ 15 ] * a[ 24 ];
      tmp[ 1753 ] += a[ 12 ] * a[ 6 ] * a[ 16 ] * a[ 19 ];
      tmp[ 1754 ] += a[ 12 ] * a[ 6 ] * a[ 16 ] * a[ 24 ];
      tmp[ 1755 ] += a[ 12 ] * a[ 6 ] * a[ 18 ] * a[ 20 ];
      tmp[ 1756 ] += a[ 12 ] * a[ 6 ] * a[ 19 ] * a[ 19 ];
      tmp[ 1757 ] += a[ 12 ] * a[ 6 ] * a[ 19 ] * a[ 20 ];
      tmp[ 1758 ] += a[ 12 ] * a[ 6 ] * a[ 19 ] * a[ 21 ];
      tmp[ 1759 ] += a[ 12 ] * a[ 6 ] * a[ 19 ] * a[ 22 ];
      tmp[ 1760 ] += a[ 12 ] * a[ 6 ] * a[ 19 ] * a[ 23 ];
      tmp[ 1761 ] += a[ 12 ] * a[ 6 ] * a[ 19 ] * a[ 24 ];
      tmp[ 1762 ] += a[ 12 ] * a[ 6 ] * a[ 20 ] * a[ 20 ];
      tmp[ 1763 ] += a[ 12 ] * a[ 6 ] * a[ 20 ] * a[ 21 ];
      tmp[ 1764 ] += a[ 12 ] * a[ 6 ] * a[ 20 ] * a[ 22 ];
      tmp[ 1765 ] += a[ 12 ] * a[ 6 ] * a[ 20 ] * a[ 23 ];
      tmp[ 1766 ] += a[ 12 ] * a[ 6 ] * a[ 20 ] * a[ 24 ];
      tmp[ 1767 ] += a[ 12 ] * a[ 6 ] * a[ 21 ] * a[ 21 ];
      tmp[ 1768 ] += a[ 12 ] * a[ 6 ] * a[ 21 ] * a[ 22 ];
      tmp[ 1769 ] += a[ 12 ] * a[ 6 ] * a[ 21 ] * a[ 23 ];
      tmp[ 1770 ] += a[ 12 ] * a[ 6 ] * a[ 21 ] * a[ 24 ];
      tmp[ 1771 ] += a[ 12 ] * a[ 6 ] * a[ 22 ] * a[ 22 ];
      tmp[ 1772 ] += a[ 12 ] * a[ 6 ] * a[ 22 ] * a[ 23 ];
      tmp[ 1773 ] += a[ 12 ] * a[ 6 ] * a[ 22 ] * a[ 24 ];
      tmp[ 1774 ] += a[ 12 ] * a[ 6 ] * a[ 23 ] * a[ 23 ];
      tmp[ 1775 ] += a[ 12 ] * a[ 6 ] * a[ 23 ] * a[ 24 ];
      tmp[ 1776 ] += a[ 12 ] * a[ 6 ] * a[ 24 ] * a[ 24 ];
      tmp[ 1777 ] += a[ 12 ] * a[ 7 ] * a[ 7 ] * a[ 7 ];
      tmp[ 1778 ] += a[ 12 ] * a[ 7 ] * a[ 7 ] * a[ 8 ];
      tmp[ 1779 ] += a[ 12 ] * a[ 7 ] * a[ 7 ] * a[ 9 ];
      tmp[ 1780 ] += a[ 12 ] * a[ 7 ] * a[ 7 ] * a[ 10 ];
      tmp[ 1781 ] += a[ 12 ] * a[ 7 ] * a[ 7 ] * a[ 11 ];
      tmp[ 1782 ] += a[ 12 ] * a[ 7 ] * a[ 7 ] * a[ 12 ];
      tmp[ 1783 ] += a[ 12 ] * a[ 7 ] * a[ 7 ] * a[ 20 ];
      tmp[ 1784 ] += a[ 12 ] * a[ 7 ] * a[ 7 ] * a[ 21 ];
      tmp[ 1785 ] += a[ 12 ] * a[ 7 ] * a[ 7 ] * a[ 22 ];
      tmp[ 1786 ] += a[ 12 ] * a[ 7 ] * a[ 7 ] * a[ 23 ];
      tmp[ 1787 ] += a[ 12 ] * a[ 7 ] * a[ 7 ] * a[ 24 ];
      tmp[ 1788 ] += a[ 12 ] * a[ 7 ] * a[ 8 ] * a[ 8 ];
      tmp[ 1789 ] += a[ 12 ] * a[ 7 ] * a[ 8 ] * a[ 9 ];
      tmp[ 1790 ] += a[ 12 ] * a[ 7 ] * a[ 8 ] * a[ 10 ];
      tmp[ 1791 ] += a[ 12 ] * a[ 7 ] * a[ 8 ] * a[ 11 ];
      tmp[ 1792 ] += a[ 12 ] * a[ 7 ] * a[ 8 ] * a[ 12 ];
      tmp[ 1793 ] += a[ 12 ] * a[ 7 ] * a[ 8 ] * a[ 15 ];
      tmp[ 1794 ] += a[ 12 ] * a[ 7 ] * a[ 8 ] * a[ 20 ];
      tmp[ 1795 ] += a[ 12 ] * a[ 7 ] * a[ 8 ] * a[ 21 ];
      tmp[ 1796 ] += a[ 12 ] * a[ 7 ] * a[ 8 ] * a[ 22 ];
      tmp[ 1797 ] += a[ 12 ] * a[ 7 ] * a[ 8 ] * a[ 23 ];
      tmp[ 1798 ] += a[ 12 ] * a[ 7 ] * a[ 8 ] * a[ 24 ];
      tmp[ 1799 ] += a[ 12 ] * a[ 7 ] * a[ 9 ] * a[ 9 ];
      tmp[ 1800 ] += a[ 12 ] * a[ 7 ] * a[ 9 ] * a[ 10 ];
      tmp[ 1801 ] += a[ 12 ] * a[ 7 ] * a[ 9 ] * a[ 11 ];
      tmp[ 1802 ] += a[ 12 ] * a[ 7 ] * a[ 9 ] * a[ 12 ];
      tmp[ 1803 ] += a[ 12 ] * a[ 7 ] * a[ 9 ] * a[ 15 ];
      tmp[ 1804 ] += a[ 12 ] * a[ 7 ] * a[ 9 ] * a[ 16 ];
      tmp[ 1805 ] += a[ 12 ] * a[ 7 ] * a[ 9 ] * a[ 20 ];
      tmp[ 1806 ] += a[ 12 ] * a[ 7 ] * a[ 9 ] * a[ 21 ];
      tmp[ 1807 ] += a[ 12 ] * a[ 7 ] * a[ 9 ] * a[ 22 ];
      tmp[ 1808 ] += a[ 12 ] * a[ 7 ] * a[ 9 ] * a[ 23 ];
      tmp[ 1809 ] += a[ 12 ] * a[ 7 ] * a[ 9 ] * a[ 24 ];
      tmp[ 1810 ] += a[ 12 ] * a[ 7 ] * a[ 10 ] * a[ 10 ];
      tmp[ 1811 ] += a[ 12 ] * a[ 7 ] * a[ 10 ] * a[ 11 ];
      tmp[ 1812 ] += a[ 12 ] * a[ 7 ] * a[ 10 ] * a[ 12 ];
      tmp[ 1813 ] += a[ 12 ] * a[ 7 ] * a[ 10 ] * a[ 13 ];
      tmp[ 1814 ] += a[ 12 ] * a[ 7 ] * a[ 10 ] * a[ 14 ];
      tmp[ 1815 ] += a[ 12 ] * a[ 7 ] * a[ 10 ] * a[ 18 ];
      tmp[ 1816 ] += a[ 12 ] * a[ 7 ] * a[ 10 ] * a[ 19 ];
      tmp[ 1817 ] += a[ 12 ] * a[ 7 ] * a[ 10 ] * a[ 20 ];
      tmp[ 1818 ] += a[ 12 ] * a[ 7 ] * a[ 10 ] * a[ 21 ];
      tmp[ 1819 ] += a[ 12 ] * a[ 7 ] * a[ 10 ] * a[ 22 ];
      tmp[ 1820 ] += a[ 12 ] * a[ 7 ] * a[ 10 ] * a[ 23 ];
      tmp[ 1821 ] += a[ 12 ] * a[ 7 ] * a[ 10 ] * a[ 24 ];
      tmp[ 1822 ] += a[ 12 ] * a[ 7 ] * a[ 11 ] * a[ 11 ];
      tmp[ 1823 ] += a[ 12 ] * a[ 7 ] * a[ 11 ] * a[ 12 ];
      tmp[ 1824 ] += a[ 12 ] * a[ 7 ] * a[ 11 ] * a[ 14 ];
      tmp[ 1825 ] += a[ 12 ] * a[ 7 ] * a[ 11 ] * a[ 19 ];
      tmp[ 1826 ] += a[ 12 ] * a[ 7 ] * a[ 11 ] * a[ 20 ];
      tmp[ 1827 ] += a[ 12 ] * a[ 7 ] * a[ 11 ] * a[ 21 ];
      tmp[ 1828 ] += a[ 12 ] * a[ 7 ] * a[ 11 ] * a[ 22 ];
      tmp[ 1829 ] += a[ 12 ] * a[ 7 ] * a[ 11 ] * a[ 23 ];
      tmp[ 1830 ] += a[ 12 ] * a[ 7 ] * a[ 11 ] * a[ 24 ];
      tmp[ 1831 ] += a[ 12 ] * a[ 7 ] * a[ 12 ] * a[ 12 ];
      tmp[ 1832 ] += a[ 12 ] * a[ 7 ] * a[ 12 ] * a[ 20 ];
      tmp[ 1833 ] += a[ 12 ] * a[ 7 ] * a[ 12 ] * a[ 21 ];
      tmp[ 1834 ] += a[ 12 ] * a[ 7 ] * a[ 12 ] * a[ 22 ];
      tmp[ 1835 ] += a[ 12 ] * a[ 7 ] * a[ 12 ] * a[ 23 ];
      tmp[ 1836 ] += a[ 12 ] * a[ 7 ] * a[ 12 ] * a[ 24 ];
      tmp[ 1837 ] += a[ 12 ] * a[ 7 ] * a[ 13 ] * a[ 15 ];
      tmp[ 1838 ] += a[ 12 ] * a[ 7 ] * a[ 13 ] * a[ 20 ];
      tmp[ 1839 ] += a[ 12 ] * a[ 7 ] * a[ 14 ] * a[ 15 ];
      tmp[ 1840 ] += a[ 12 ] * a[ 7 ] * a[ 14 ] * a[ 16 ];
      tmp[ 1841 ] += a[ 12 ] * a[ 7 ] * a[ 14 ] * a[ 20 ];
      tmp[ 1842 ] += a[ 12 ] * a[ 7 ] * a[ 14 ] * a[ 21 ];
      tmp[ 1843 ] += a[ 12 ] * a[ 7 ] * a[ 15 ] * a[ 18 ];
      tmp[ 1844 ] += a[ 12 ] * a[ 7 ] * a[ 15 ] * a[ 19 ];
      tmp[ 1845 ] += a[ 12 ] * a[ 7 ] * a[ 15 ] * a[ 23 ];
      tmp[ 1846 ] += a[ 12 ] * a[ 7 ] * a[ 15 ] * a[ 24 ];
      tmp[ 1847 ] += a[ 12 ] * a[ 7 ] * a[ 16 ] * a[ 19 ];
      tmp[ 1848 ] += a[ 12 ] * a[ 7 ] * a[ 16 ] * a[ 24 ];
      tmp[ 1849 ] += a[ 12 ] * a[ 7 ] * a[ 18 ] * a[ 20 ];
      tmp[ 1850 ] += a[ 12 ] * a[ 7 ] * a[ 19 ] * a[ 20 ];
      tmp[ 1851 ] += a[ 12 ] * a[ 7 ] * a[ 19 ] * a[ 21 ];
      tmp[ 1852 ] += a[ 12 ] * a[ 7 ] * a[ 20 ] * a[ 20 ];
      tmp[ 1853 ] += a[ 12 ] * a[ 7 ] * a[ 20 ] * a[ 21 ];
      tmp[ 1854 ] += a[ 12 ] * a[ 7 ] * a[ 20 ] * a[ 22 ];
      tmp[ 1855 ] += a[ 12 ] * a[ 7 ] * a[ 20 ] * a[ 23 ];
      tmp[ 1856 ] += a[ 12 ] * a[ 7 ] * a[ 20 ] * a[ 24 ];
      tmp[ 1857 ] += a[ 12 ] * a[ 7 ] * a[ 21 ] * a[ 21 ];
      tmp[ 1858 ] += a[ 12 ] * a[ 7 ] * a[ 21 ] * a[ 22 ];
      tmp[ 1859 ] += a[ 12 ] * a[ 7 ] * a[ 21 ] * a[ 23 ];
      tmp[ 1860 ] += a[ 12 ] * a[ 7 ] * a[ 21 ] * a[ 24 ];
      tmp[ 1861 ] += a[ 12 ] * a[ 7 ] * a[ 22 ] * a[ 22 ];
      tmp[ 1862 ] += a[ 12 ] * a[ 7 ] * a[ 22 ] * a[ 23 ];
      tmp[ 1863 ] += a[ 12 ] * a[ 7 ] * a[ 22 ] * a[ 24 ];
      tmp[ 1864 ] += a[ 12 ] * a[ 7 ] * a[ 23 ] * a[ 23 ];
      tmp[ 1865 ] += a[ 12 ] * a[ 7 ] * a[ 23 ] * a[ 24 ];
      tmp[ 1866 ] += a[ 12 ] * a[ 7 ] * a[ 24 ] * a[ 24 ];
      tmp[ 1867 ] += a[ 12 ] * a[ 8 ] * a[ 8 ] * a[ 8 ];
      tmp[ 1868 ] += a[ 12 ] * a[ 8 ] * a[ 8 ] * a[ 9 ];
      tmp[ 1869 ] += a[ 12 ] * a[ 8 ] * a[ 8 ] * a[ 10 ];
      tmp[ 1870 ] += a[ 12 ] * a[ 8 ] * a[ 8 ] * a[ 11 ];
      tmp[ 1871 ] += a[ 12 ] * a[ 8 ] * a[ 8 ] * a[ 12 ];
      tmp[ 1872 ] += a[ 12 ] * a[ 8 ] * a[ 8 ] * a[ 15 ];
      tmp[ 1873 ] += a[ 12 ] * a[ 8 ] * a[ 8 ] * a[ 20 ];
      tmp[ 1874 ] += a[ 12 ] * a[ 8 ] * a[ 8 ] * a[ 21 ];
      tmp[ 1875 ] += a[ 12 ] * a[ 8 ] * a[ 8 ] * a[ 22 ];
      tmp[ 1876 ] += a[ 12 ] * a[ 8 ] * a[ 8 ] * a[ 23 ];
      tmp[ 1877 ] += a[ 12 ] * a[ 8 ] * a[ 8 ] * a[ 24 ];
      tmp[ 1878 ] += a[ 12 ] * a[ 8 ] * a[ 9 ] * a[ 9 ];
      tmp[ 1879 ] += a[ 12 ] * a[ 8 ] * a[ 9 ] * a[ 10 ];
      tmp[ 1880 ] += a[ 12 ] * a[ 8 ] * a[ 9 ] * a[ 11 ];
      tmp[ 1881 ] += a[ 12 ] * a[ 8 ] * a[ 9 ] * a[ 12 ];
      tmp[ 1882 ] += a[ 12 ] * a[ 8 ] * a[ 9 ] * a[ 15 ];
      tmp[ 1883 ] += a[ 12 ] * a[ 8 ] * a[ 9 ] * a[ 16 ];
      tmp[ 1884 ] += a[ 12 ] * a[ 8 ] * a[ 9 ] * a[ 20 ];
      tmp[ 1885 ] += a[ 12 ] * a[ 8 ] * a[ 9 ] * a[ 21 ];
      tmp[ 1886 ] += a[ 12 ] * a[ 8 ] * a[ 9 ] * a[ 22 ];
      tmp[ 1887 ] += a[ 12 ] * a[ 8 ] * a[ 9 ] * a[ 23 ];
      tmp[ 1888 ] += a[ 12 ] * a[ 8 ] * a[ 9 ] * a[ 24 ];
      tmp[ 1889 ] += a[ 12 ] * a[ 8 ] * a[ 10 ] * a[ 10 ];
      tmp[ 1890 ] += a[ 12 ] * a[ 8 ] * a[ 10 ] * a[ 11 ];
      tmp[ 1891 ] += a[ 12 ] * a[ 8 ] * a[ 10 ] * a[ 12 ];
      tmp[ 1892 ] += a[ 12 ] * a[ 8 ] * a[ 10 ] * a[ 13 ];
      tmp[ 1893 ] += a[ 12 ] * a[ 8 ] * a[ 10 ] * a[ 14 ];
      tmp[ 1894 ] += a[ 12 ] * a[ 8 ] * a[ 10 ] * a[ 15 ];
      tmp[ 1895 ] += a[ 12 ] * a[ 8 ] * a[ 10 ] * a[ 18 ];
      tmp[ 1896 ] += a[ 12 ] * a[ 8 ] * a[ 10 ] * a[ 19 ];
      tmp[ 1897 ] += a[ 12 ] * a[ 8 ] * a[ 10 ] * a[ 20 ];
      tmp[ 1898 ] += a[ 12 ] * a[ 8 ] * a[ 10 ] * a[ 21 ];
      tmp[ 1899 ] += a[ 12 ] * a[ 8 ] * a[ 10 ] * a[ 22 ];
      tmp[ 1900 ] += a[ 12 ] * a[ 8 ] * a[ 10 ] * a[ 23 ];
      tmp[ 1901 ] += a[ 12 ] * a[ 8 ] * a[ 10 ] * a[ 24 ];
      tmp[ 1902 ] += a[ 12 ] * a[ 8 ] * a[ 11 ] * a[ 11 ];
      tmp[ 1903 ] += a[ 12 ] * a[ 8 ] * a[ 11 ] * a[ 12 ];
      tmp[ 1904 ] += a[ 12 ] * a[ 8 ] * a[ 11 ] * a[ 14 ];
      tmp[ 1905 ] += a[ 12 ] * a[ 8 ] * a[ 11 ] * a[ 15 ];
      tmp[ 1906 ] += a[ 12 ] * a[ 8 ] * a[ 11 ] * a[ 19 ];
      tmp[ 1907 ] += a[ 12 ] * a[ 8 ] * a[ 11 ] * a[ 20 ];
      tmp[ 1908 ] += a[ 12 ] * a[ 8 ] * a[ 11 ] * a[ 21 ];
      tmp[ 1909 ] += a[ 12 ] * a[ 8 ] * a[ 11 ] * a[ 22 ];
      tmp[ 1910 ] += a[ 12 ] * a[ 8 ] * a[ 11 ] * a[ 23 ];
      tmp[ 1911 ] += a[ 12 ] * a[ 8 ] * a[ 11 ] * a[ 24 ];
      tmp[ 1912 ] += a[ 12 ] * a[ 8 ] * a[ 12 ] * a[ 12 ];
      tmp[ 1913 ] += a[ 12 ] * a[ 8 ] * a[ 12 ] * a[ 15 ];
      tmp[ 1914 ] += a[ 12 ] * a[ 8 ] * a[ 12 ] * a[ 20 ];
      tmp[ 1915 ] += a[ 12 ] * a[ 8 ] * a[ 12 ] * a[ 21 ];
      tmp[ 1916 ] += a[ 12 ] * a[ 8 ] * a[ 12 ] * a[ 22 ];
      tmp[ 1917 ] += a[ 12 ] * a[ 8 ] * a[ 12 ] * a[ 23 ];
      tmp[ 1918 ] += a[ 12 ] * a[ 8 ] * a[ 12 ] * a[ 24 ];
      tmp[ 1919 ] += a[ 12 ] * a[ 8 ] * a[ 13 ] * a[ 15 ];
      tmp[ 1920 ] += a[ 12 ] * a[ 8 ] * a[ 13 ] * a[ 20 ];
      tmp[ 1921 ] += a[ 12 ] * a[ 8 ] * a[ 14 ] * a[ 15 ];
      tmp[ 1922 ] += a[ 12 ] * a[ 8 ] * a[ 14 ] * a[ 16 ];
      tmp[ 1923 ] += a[ 12 ] * a[ 8 ] * a[ 14 ] * a[ 20 ];
      tmp[ 1924 ] += a[ 12 ] * a[ 8 ] * a[ 14 ] * a[ 21 ];
      tmp[ 1925 ] += a[ 12 ] * a[ 8 ] * a[ 15 ] * a[ 15 ];
      tmp[ 1926 ] += a[ 12 ] * a[ 8 ] * a[ 15 ] * a[ 18 ];
      tmp[ 1927 ] += a[ 12 ] * a[ 8 ] * a[ 15 ] * a[ 19 ];
      tmp[ 1928 ] += a[ 12 ] * a[ 8 ] * a[ 15 ] * a[ 20 ];
      tmp[ 1929 ] += a[ 12 ] * a[ 8 ] * a[ 15 ] * a[ 21 ];
      tmp[ 1930 ] += a[ 12 ] * a[ 8 ] * a[ 15 ] * a[ 22 ];
      tmp[ 1931 ] += a[ 12 ] * a[ 8 ] * a[ 15 ] * a[ 23 ];
      tmp[ 1932 ] += a[ 12 ] * a[ 8 ] * a[ 15 ] * a[ 24 ];
      tmp[ 1933 ] += a[ 12 ] * a[ 8 ] * a[ 16 ] * a[ 19 ];
      tmp[ 1934 ] += a[ 12 ] * a[ 8 ] * a[ 16 ] * a[ 24 ];
      tmp[ 1935 ] += a[ 12 ] * a[ 8 ] * a[ 18 ] * a[ 20 ];
      tmp[ 1936 ] += a[ 12 ] * a[ 8 ] * a[ 19 ] * a[ 20 ];
      tmp[ 1937 ] += a[ 12 ] * a[ 8 ] * a[ 19 ] * a[ 21 ];
      tmp[ 1938 ] += a[ 12 ] * a[ 8 ] * a[ 20 ] * a[ 20 ];
      tmp[ 1939 ] += a[ 12 ] * a[ 8 ] * a[ 20 ] * a[ 21 ];
      tmp[ 1940 ] += a[ 12 ] * a[ 8 ] * a[ 20 ] * a[ 22 ];
      tmp[ 1941 ] += a[ 12 ] * a[ 8 ] * a[ 20 ] * a[ 23 ];
      tmp[ 1942 ] += a[ 12 ] * a[ 8 ] * a[ 20 ] * a[ 24 ];
      tmp[ 1943 ] += a[ 12 ] * a[ 8 ] * a[ 21 ] * a[ 21 ];
      tmp[ 1944 ] += a[ 12 ] * a[ 8 ] * a[ 21 ] * a[ 22 ];
      tmp[ 1945 ] += a[ 12 ] * a[ 8 ] * a[ 21 ] * a[ 23 ];
      tmp[ 1946 ] += a[ 12 ] * a[ 8 ] * a[ 21 ] * a[ 24 ];
      tmp[ 1947 ] += a[ 12 ] * a[ 8 ] * a[ 22 ] * a[ 22 ];
      tmp[ 1948 ] += a[ 12 ] * a[ 8 ] * a[ 22 ] * a[ 23 ];
      tmp[ 1949 ] += a[ 12 ] * a[ 8 ] * a[ 22 ] * a[ 24 ];
      tmp[ 1950 ] += a[ 12 ] * a[ 8 ] * a[ 23 ] * a[ 23 ];
      tmp[ 1951 ] += a[ 12 ] * a[ 8 ] * a[ 23 ] * a[ 24 ];
      tmp[ 1952 ] += a[ 12 ] * a[ 8 ] * a[ 24 ] * a[ 24 ];
      tmp[ 1953 ] += a[ 12 ] * a[ 9 ] * a[ 9 ] * a[ 9 ];
      tmp[ 1954 ] += a[ 12 ] * a[ 9 ] * a[ 9 ] * a[ 10 ];
      tmp[ 1955 ] += a[ 12 ] * a[ 9 ] * a[ 9 ] * a[ 11 ];
      tmp[ 1956 ] += a[ 12 ] * a[ 9 ] * a[ 9 ] * a[ 12 ];
      tmp[ 1957 ] += a[ 12 ] * a[ 9 ] * a[ 9 ] * a[ 15 ];
      tmp[ 1958 ] += a[ 12 ] * a[ 9 ] * a[ 9 ] * a[ 16 ];
      tmp[ 1959 ] += a[ 12 ] * a[ 9 ] * a[ 9 ] * a[ 20 ];
      tmp[ 1960 ] += a[ 12 ] * a[ 9 ] * a[ 9 ] * a[ 21 ];
      tmp[ 1961 ] += a[ 12 ] * a[ 9 ] * a[ 9 ] * a[ 22 ];
      tmp[ 1962 ] += a[ 12 ] * a[ 9 ] * a[ 9 ] * a[ 23 ];
      tmp[ 1963 ] += a[ 12 ] * a[ 9 ] * a[ 9 ] * a[ 24 ];
      tmp[ 1964 ] += a[ 12 ] * a[ 9 ] * a[ 10 ] * a[ 10 ];
      tmp[ 1965 ] += a[ 12 ] * a[ 9 ] * a[ 10 ] * a[ 11 ];
      tmp[ 1966 ] += a[ 12 ] * a[ 9 ] * a[ 10 ] * a[ 12 ];
      tmp[ 1967 ] += a[ 12 ] * a[ 9 ] * a[ 10 ] * a[ 13 ];
      tmp[ 1968 ] += a[ 12 ] * a[ 9 ] * a[ 10 ] * a[ 14 ];
      tmp[ 1969 ] += a[ 12 ] * a[ 9 ] * a[ 10 ] * a[ 15 ];
      tmp[ 1970 ] += a[ 12 ] * a[ 9 ] * a[ 10 ] * a[ 16 ];
      tmp[ 1971 ] += a[ 12 ] * a[ 9 ] * a[ 10 ] * a[ 18 ];
      tmp[ 1972 ] += a[ 12 ] * a[ 9 ] * a[ 10 ] * a[ 19 ];
      tmp[ 1973 ] += a[ 12 ] * a[ 9 ] * a[ 10 ] * a[ 20 ];
      tmp[ 1974 ] += a[ 12 ] * a[ 9 ] * a[ 10 ] * a[ 21 ];
      tmp[ 1975 ] += a[ 12 ] * a[ 9 ] * a[ 10 ] * a[ 22 ];
      tmp[ 1976 ] += a[ 12 ] * a[ 9 ] * a[ 10 ] * a[ 23 ];
      tmp[ 1977 ] += a[ 12 ] * a[ 9 ] * a[ 10 ] * a[ 24 ];
      tmp[ 1978 ] += a[ 12 ] * a[ 9 ] * a[ 11 ] * a[ 11 ];
      tmp[ 1979 ] += a[ 12 ] * a[ 9 ] * a[ 11 ] * a[ 12 ];
      tmp[ 1980 ] += a[ 12 ] * a[ 9 ] * a[ 11 ] * a[ 14 ];
      tmp[ 1981 ] += a[ 12 ] * a[ 9 ] * a[ 11 ] * a[ 15 ];
      tmp[ 1982 ] += a[ 12 ] * a[ 9 ] * a[ 11 ] * a[ 16 ];
      tmp[ 1983 ] += a[ 12 ] * a[ 9 ] * a[ 11 ] * a[ 19 ];
      tmp[ 1984 ] += a[ 12 ] * a[ 9 ] * a[ 11 ] * a[ 20 ];
      tmp[ 1985 ] += a[ 12 ] * a[ 9 ] * a[ 11 ] * a[ 21 ];
      tmp[ 1986 ] += a[ 12 ] * a[ 9 ] * a[ 11 ] * a[ 22 ];
      tmp[ 1987 ] += a[ 12 ] * a[ 9 ] * a[ 11 ] * a[ 23 ];
      tmp[ 1988 ] += a[ 12 ] * a[ 9 ] * a[ 11 ] * a[ 24 ];
      tmp[ 1989 ] += a[ 12 ] * a[ 9 ] * a[ 12 ] * a[ 12 ];
      tmp[ 1990 ] += a[ 12 ] * a[ 9 ] * a[ 12 ] * a[ 15 ];
      tmp[ 1991 ] += a[ 12 ] * a[ 9 ] * a[ 12 ] * a[ 16 ];
      tmp[ 1992 ] += a[ 12 ] * a[ 9 ] * a[ 12 ] * a[ 20 ];
      tmp[ 1993 ] += a[ 12 ] * a[ 9 ] * a[ 12 ] * a[ 21 ];
      tmp[ 1994 ] += a[ 12 ] * a[ 9 ] * a[ 12 ] * a[ 22 ];
      tmp[ 1995 ] += a[ 12 ] * a[ 9 ] * a[ 12 ] * a[ 23 ];
      tmp[ 1996 ] += a[ 12 ] * a[ 9 ] * a[ 12 ] * a[ 24 ];
      tmp[ 1997 ] += a[ 12 ] * a[ 9 ] * a[ 13 ] * a[ 15 ];
      tmp[ 1998 ] += a[ 12 ] * a[ 9 ] * a[ 13 ] * a[ 20 ];
      tmp[ 1999 ] += a[ 12 ] * a[ 9 ] * a[ 14 ] * a[ 15 ];
      tmp[ 2000 ] += a[ 12 ] * a[ 9 ] * a[ 14 ] * a[ 16 ];
      tmp[ 2001 ] += a[ 12 ] * a[ 9 ] * a[ 14 ] * a[ 20 ];
      tmp[ 2002 ] += a[ 12 ] * a[ 9 ] * a[ 14 ] * a[ 21 ];
      tmp[ 2003 ] += a[ 12 ] * a[ 9 ] * a[ 15 ] * a[ 15 ];
      tmp[ 2004 ] += a[ 12 ] * a[ 9 ] * a[ 15 ] * a[ 16 ];
      tmp[ 2005 ] += a[ 12 ] * a[ 9 ] * a[ 15 ] * a[ 18 ];
      tmp[ 2006 ] += a[ 12 ] * a[ 9 ] * a[ 15 ] * a[ 19 ];
      tmp[ 2007 ] += a[ 12 ] * a[ 9 ] * a[ 15 ] * a[ 20 ];
      tmp[ 2008 ] += a[ 12 ] * a[ 9 ] * a[ 15 ] * a[ 21 ];
      tmp[ 2009 ] += a[ 12 ] * a[ 9 ] * a[ 15 ] * a[ 22 ];
      tmp[ 2010 ] += a[ 12 ] * a[ 9 ] * a[ 15 ] * a[ 23 ];
      tmp[ 2011 ] += a[ 12 ] * a[ 9 ] * a[ 15 ] * a[ 24 ];
      tmp[ 2012 ] += a[ 12 ] * a[ 9 ] * a[ 16 ] * a[ 16 ];
      tmp[ 2013 ] += a[ 12 ] * a[ 9 ] * a[ 16 ] * a[ 19 ];
      tmp[ 2014 ] += a[ 12 ] * a[ 9 ] * a[ 16 ] * a[ 20 ];
      tmp[ 2015 ] += a[ 12 ] * a[ 9 ] * a[ 16 ] * a[ 21 ];
      tmp[ 2016 ] += a[ 12 ] * a[ 9 ] * a[ 16 ] * a[ 22 ];
      tmp[ 2017 ] += a[ 12 ] * a[ 9 ] * a[ 16 ] * a[ 23 ];
      tmp[ 2018 ] += a[ 12 ] * a[ 9 ] * a[ 16 ] * a[ 24 ];
      tmp[ 2019 ] += a[ 12 ] * a[ 9 ] * a[ 18 ] * a[ 20 ];
      tmp[ 2020 ] += a[ 12 ] * a[ 9 ] * a[ 19 ] * a[ 20 ];
      tmp[ 2021 ] += a[ 12 ] * a[ 9 ] * a[ 19 ] * a[ 21 ];
      tmp[ 2022 ] += a[ 12 ] * a[ 9 ] * a[ 20 ] * a[ 20 ];
      tmp[ 2023 ] += a[ 12 ] * a[ 9 ] * a[ 20 ] * a[ 21 ];
      tmp[ 2024 ] += a[ 12 ] * a[ 9 ] * a[ 20 ] * a[ 22 ];
      tmp[ 2025 ] += a[ 12 ] * a[ 9 ] * a[ 20 ] * a[ 23 ];
      tmp[ 2026 ] += a[ 12 ] * a[ 9 ] * a[ 20 ] * a[ 24 ];
      tmp[ 2027 ] += a[ 12 ] * a[ 9 ] * a[ 21 ] * a[ 21 ];
      tmp[ 2028 ] += a[ 12 ] * a[ 9 ] * a[ 21 ] * a[ 22 ];
      tmp[ 2029 ] += a[ 12 ] * a[ 9 ] * a[ 21 ] * a[ 23 ];
      tmp[ 2030 ] += a[ 12 ] * a[ 9 ] * a[ 21 ] * a[ 24 ];
      tmp[ 2031 ] += a[ 12 ] * a[ 9 ] * a[ 22 ] * a[ 22 ];
      tmp[ 2032 ] += a[ 12 ] * a[ 9 ] * a[ 22 ] * a[ 23 ];
      tmp[ 2033 ] += a[ 12 ] * a[ 9 ] * a[ 22 ] * a[ 24 ];
      tmp[ 2034 ] += a[ 12 ] * a[ 9 ] * a[ 23 ] * a[ 23 ];
      tmp[ 2035 ] += a[ 12 ] * a[ 9 ] * a[ 23 ] * a[ 24 ];
      tmp[ 2036 ] += a[ 12 ] * a[ 9 ] * a[ 24 ] * a[ 24 ];
      tmp[ 2037 ] += a[ 12 ] * a[ 10 ] * a[ 10 ] * a[ 10 ];
      tmp[ 2038 ] += a[ 12 ] * a[ 10 ] * a[ 10 ] * a[ 11 ];
      tmp[ 2039 ] += a[ 12 ] * a[ 10 ] * a[ 10 ] * a[ 12 ];
      tmp[ 2040 ] += a[ 12 ] * a[ 10 ] * a[ 10 ] * a[ 13 ];
      tmp[ 2041 ] += a[ 12 ] * a[ 10 ] * a[ 10 ] * a[ 14 ];
      tmp[ 2042 ] += a[ 12 ] * a[ 10 ] * a[ 10 ] * a[ 18 ];
      tmp[ 2043 ] += a[ 12 ] * a[ 10 ] * a[ 10 ] * a[ 19 ];
      tmp[ 2044 ] += a[ 12 ] * a[ 10 ] * a[ 10 ] * a[ 23 ];
      tmp[ 2045 ] += a[ 12 ] * a[ 10 ] * a[ 10 ] * a[ 24 ];
      tmp[ 2046 ] += a[ 12 ] * a[ 10 ] * a[ 11 ] * a[ 11 ];
      tmp[ 2047 ] += a[ 12 ] * a[ 10 ] * a[ 11 ] * a[ 12 ];
      tmp[ 2048 ] += a[ 12 ] * a[ 10 ] * a[ 11 ] * a[ 13 ];
      tmp[ 2049 ] += a[ 12 ] * a[ 10 ] * a[ 11 ] * a[ 14 ];
      tmp[ 2050 ] += a[ 12 ] * a[ 10 ] * a[ 11 ] * a[ 18 ];
      tmp[ 2051 ] += a[ 12 ] * a[ 10 ] * a[ 11 ] * a[ 19 ];
      tmp[ 2052 ] += a[ 12 ] * a[ 10 ] * a[ 11 ] * a[ 23 ];
      tmp[ 2053 ] += a[ 12 ] * a[ 10 ] * a[ 11 ] * a[ 24 ];
      tmp[ 2054 ] += a[ 12 ] * a[ 10 ] * a[ 12 ] * a[ 12 ];
      tmp[ 2055 ] += a[ 12 ] * a[ 10 ] * a[ 12 ] * a[ 13 ];
      tmp[ 2056 ] += a[ 12 ] * a[ 10 ] * a[ 12 ] * a[ 14 ];
      tmp[ 2057 ] += a[ 12 ] * a[ 10 ] * a[ 12 ] * a[ 18 ];
      tmp[ 2058 ] += a[ 12 ] * a[ 10 ] * a[ 12 ] * a[ 19 ];
      tmp[ 2059 ] += a[ 12 ] * a[ 10 ] * a[ 12 ] * a[ 23 ];
      tmp[ 2060 ] += a[ 12 ] * a[ 10 ] * a[ 12 ] * a[ 24 ];
      tmp[ 2061 ] += a[ 12 ] * a[ 10 ] * a[ 13 ] * a[ 13 ];
      tmp[ 2062 ] += a[ 12 ] * a[ 10 ] * a[ 13 ] * a[ 14 ];
      tmp[ 2063 ] += a[ 12 ] * a[ 10 ] * a[ 13 ] * a[ 15 ];
      tmp[ 2064 ] += a[ 12 ] * a[ 10 ] * a[ 13 ] * a[ 18 ];
      tmp[ 2065 ] += a[ 12 ] * a[ 10 ] * a[ 13 ] * a[ 19 ];
      tmp[ 2066 ] += a[ 12 ] * a[ 10 ] * a[ 13 ] * a[ 20 ];
      tmp[ 2067 ] += a[ 12 ] * a[ 10 ] * a[ 13 ] * a[ 23 ];
      tmp[ 2068 ] += a[ 12 ] * a[ 10 ] * a[ 13 ] * a[ 24 ];
      tmp[ 2069 ] += a[ 12 ] * a[ 10 ] * a[ 14 ] * a[ 14 ];
      tmp[ 2070 ] += a[ 12 ] * a[ 10 ] * a[ 14 ] * a[ 15 ];
      tmp[ 2071 ] += a[ 12 ] * a[ 10 ] * a[ 14 ] * a[ 16 ];
      tmp[ 2072 ] += a[ 12 ] * a[ 10 ] * a[ 14 ] * a[ 18 ];
      tmp[ 2073 ] += a[ 12 ] * a[ 10 ] * a[ 14 ] * a[ 19 ];
      tmp[ 2074 ] += a[ 12 ] * a[ 10 ] * a[ 14 ] * a[ 20 ];
      tmp[ 2075 ] += a[ 12 ] * a[ 10 ] * a[ 14 ] * a[ 21 ];
      tmp[ 2076 ] += a[ 12 ] * a[ 10 ] * a[ 14 ] * a[ 23 ];
      tmp[ 2077 ] += a[ 12 ] * a[ 10 ] * a[ 14 ] * a[ 24 ];
      tmp[ 2078 ] += a[ 12 ] * a[ 10 ] * a[ 15 ] * a[ 18 ];
      tmp[ 2079 ] += a[ 12 ] * a[ 10 ] * a[ 15 ] * a[ 19 ];
      tmp[ 2080 ] += a[ 12 ] * a[ 10 ] * a[ 15 ] * a[ 23 ];
      tmp[ 2081 ] += a[ 12 ] * a[ 10 ] * a[ 15 ] * a[ 24 ];
      tmp[ 2082 ] += a[ 12 ] * a[ 10 ] * a[ 16 ] * a[ 19 ];
      tmp[ 2083 ] += a[ 12 ] * a[ 10 ] * a[ 16 ] * a[ 24 ];
      tmp[ 2084 ] += a[ 12 ] * a[ 10 ] * a[ 18 ] * a[ 18 ];
      tmp[ 2085 ] += a[ 12 ] * a[ 10 ] * a[ 18 ] * a[ 19 ];
      tmp[ 2086 ] += a[ 12 ] * a[ 10 ] * a[ 18 ] * a[ 20 ];
      tmp[ 2087 ] += a[ 12 ] * a[ 10 ] * a[ 18 ] * a[ 23 ];
      tmp[ 2088 ] += a[ 12 ] * a[ 10 ] * a[ 18 ] * a[ 24 ];
      tmp[ 2089 ] += a[ 12 ] * a[ 10 ] * a[ 19 ] * a[ 19 ];
      tmp[ 2090 ] += a[ 12 ] * a[ 10 ] * a[ 19 ] * a[ 20 ];
      tmp[ 2091 ] += a[ 12 ] * a[ 10 ] * a[ 19 ] * a[ 21 ];
      tmp[ 2092 ] += a[ 12 ] * a[ 10 ] * a[ 19 ] * a[ 23 ];
      tmp[ 2093 ] += a[ 12 ] * a[ 10 ] * a[ 19 ] * a[ 24 ];
      tmp[ 2094 ] += a[ 12 ] * a[ 10 ] * a[ 20 ] * a[ 23 ];
      tmp[ 2095 ] += a[ 12 ] * a[ 10 ] * a[ 20 ] * a[ 24 ];
      tmp[ 2096 ] += a[ 12 ] * a[ 10 ] * a[ 21 ] * a[ 24 ];
      tmp[ 2097 ] += a[ 12 ] * a[ 10 ] * a[ 23 ] * a[ 23 ];
      tmp[ 2098 ] += a[ 12 ] * a[ 10 ] * a[ 23 ] * a[ 24 ];
      tmp[ 2099 ] += a[ 12 ] * a[ 10 ] * a[ 24 ] * a[ 24 ];
      tmp[ 2100 ] += a[ 12 ] * a[ 11 ] * a[ 11 ] * a[ 11 ];
      tmp[ 2101 ] += a[ 12 ] * a[ 11 ] * a[ 11 ] * a[ 12 ];
      tmp[ 2102 ] += a[ 12 ] * a[ 11 ] * a[ 11 ] * a[ 14 ];
      tmp[ 2103 ] += a[ 12 ] * a[ 11 ] * a[ 11 ] * a[ 19 ];
      tmp[ 2104 ] += a[ 12 ] * a[ 11 ] * a[ 11 ] * a[ 24 ];
      tmp[ 2105 ] += a[ 12 ] * a[ 11 ] * a[ 12 ] * a[ 12 ];
      tmp[ 2106 ] += a[ 12 ] * a[ 11 ] * a[ 12 ] * a[ 14 ];
      tmp[ 2107 ] += a[ 12 ] * a[ 11 ] * a[ 12 ] * a[ 19 ];
      tmp[ 2108 ] += a[ 12 ] * a[ 11 ] * a[ 12 ] * a[ 24 ];
      tmp[ 2109 ] += a[ 12 ] * a[ 11 ] * a[ 13 ] * a[ 15 ];
      tmp[ 2110 ] += a[ 12 ] * a[ 11 ] * a[ 13 ] * a[ 20 ];
      tmp[ 2111 ] += a[ 12 ] * a[ 11 ] * a[ 14 ] * a[ 14 ];
      tmp[ 2112 ] += a[ 12 ] * a[ 11 ] * a[ 14 ] * a[ 15 ];
      tmp[ 2113 ] += a[ 12 ] * a[ 11 ] * a[ 14 ] * a[ 16 ];
      tmp[ 2114 ] += a[ 12 ] * a[ 11 ] * a[ 14 ] * a[ 19 ];
      tmp[ 2115 ] += a[ 12 ] * a[ 11 ] * a[ 14 ] * a[ 20 ];
      tmp[ 2116 ] += a[ 12 ] * a[ 11 ] * a[ 14 ] * a[ 21 ];
      tmp[ 2117 ] += a[ 12 ] * a[ 11 ] * a[ 14 ] * a[ 24 ];
      tmp[ 2118 ] += a[ 12 ] * a[ 11 ] * a[ 15 ] * a[ 18 ];
      tmp[ 2119 ] += a[ 12 ] * a[ 11 ] * a[ 15 ] * a[ 19 ];
      tmp[ 2120 ] += a[ 12 ] * a[ 11 ] * a[ 15 ] * a[ 23 ];
      tmp[ 2121 ] += a[ 12 ] * a[ 11 ] * a[ 15 ] * a[ 24 ];
      tmp[ 2122 ] += a[ 12 ] * a[ 11 ] * a[ 16 ] * a[ 19 ];
      tmp[ 2123 ] += a[ 12 ] * a[ 11 ] * a[ 16 ] * a[ 24 ];
      tmp[ 2124 ] += a[ 12 ] * a[ 11 ] * a[ 18 ] * a[ 20 ];
      tmp[ 2125 ] += a[ 12 ] * a[ 11 ] * a[ 19 ] * a[ 19 ];
      tmp[ 2126 ] += a[ 12 ] * a[ 11 ] * a[ 19 ] * a[ 20 ];
      tmp[ 2127 ] += a[ 12 ] * a[ 11 ] * a[ 19 ] * a[ 21 ];
      tmp[ 2128 ] += a[ 12 ] * a[ 11 ] * a[ 19 ] * a[ 24 ];
      tmp[ 2129 ] += a[ 12 ] * a[ 11 ] * a[ 20 ] * a[ 23 ];
      tmp[ 2130 ] += a[ 12 ] * a[ 11 ] * a[ 20 ] * a[ 24 ];
      tmp[ 2131 ] += a[ 12 ] * a[ 11 ] * a[ 21 ] * a[ 24 ];
      tmp[ 2132 ] += a[ 12 ] * a[ 11 ] * a[ 24 ] * a[ 24 ];
      tmp[ 2133 ] += a[ 12 ] * a[ 12 ] * a[ 12 ] * a[ 12 ];
      tmp[ 2134 ] += a[ 12 ] * a[ 12 ] * a[ 13 ] * a[ 15 ];
      tmp[ 2135 ] += a[ 12 ] * a[ 12 ] * a[ 13 ] * a[ 20 ];
      tmp[ 2136 ] += a[ 12 ] * a[ 12 ] * a[ 14 ] * a[ 15 ];
      tmp[ 2137 ] += a[ 12 ] * a[ 12 ] * a[ 14 ] * a[ 16 ];
      tmp[ 2138 ] += a[ 12 ] * a[ 12 ] * a[ 14 ] * a[ 20 ];
      tmp[ 2139 ] += a[ 12 ] * a[ 12 ] * a[ 14 ] * a[ 21 ];
      tmp[ 2140 ] += a[ 12 ] * a[ 12 ] * a[ 15 ] * a[ 18 ];
      tmp[ 2141 ] += a[ 12 ] * a[ 12 ] * a[ 15 ] * a[ 19 ];
      tmp[ 2142 ] += a[ 12 ] * a[ 12 ] * a[ 15 ] * a[ 23 ];
      tmp[ 2143 ] += a[ 12 ] * a[ 12 ] * a[ 15 ] * a[ 24 ];
      tmp[ 2144 ] += a[ 12 ] * a[ 12 ] * a[ 16 ] * a[ 19 ];
      tmp[ 2145 ] += a[ 12 ] * a[ 12 ] * a[ 16 ] * a[ 24 ];
      tmp[ 2146 ] += a[ 12 ] * a[ 12 ] * a[ 18 ] * a[ 20 ];
      tmp[ 2147 ] += a[ 12 ] * a[ 12 ] * a[ 19 ] * a[ 20 ];
      tmp[ 2148 ] += a[ 12 ] * a[ 12 ] * a[ 19 ] * a[ 21 ];
      tmp[ 2149 ] += a[ 12 ] * a[ 12 ] * a[ 20 ] * a[ 23 ];
      tmp[ 2150 ] += a[ 12 ] * a[ 12 ] * a[ 20 ] * a[ 24 ];
      tmp[ 2151 ] += a[ 12 ] * a[ 12 ] * a[ 21 ] * a[ 24 ];
      tmp[ 2152 ] += a[ 12 ] * a[ 13 ] * a[ 13 ] * a[ 15 ];
      tmp[ 2153 ] += a[ 12 ] * a[ 13 ] * a[ 13 ] * a[ 20 ];
      tmp[ 2154 ] += a[ 12 ] * a[ 13 ] * a[ 14 ] * a[ 15 ];
      tmp[ 2155 ] += a[ 12 ] * a[ 13 ] * a[ 14 ] * a[ 20 ];
      tmp[ 2156 ] += a[ 12 ] * a[ 13 ] * a[ 15 ] * a[ 15 ];
      tmp[ 2157 ] += a[ 12 ] * a[ 13 ] * a[ 15 ] * a[ 18 ];
      tmp[ 2158 ] += a[ 12 ] * a[ 13 ] * a[ 15 ] * a[ 19 ];
      tmp[ 2159 ] += a[ 12 ] * a[ 13 ] * a[ 15 ] * a[ 20 ];
      tmp[ 2160 ] += a[ 12 ] * a[ 13 ] * a[ 15 ] * a[ 23 ];
      tmp[ 2161 ] += a[ 12 ] * a[ 13 ] * a[ 15 ] * a[ 24 ];
      tmp[ 2162 ] += a[ 12 ] * a[ 13 ] * a[ 18 ] * a[ 20 ];
      tmp[ 2163 ] += a[ 12 ] * a[ 13 ] * a[ 19 ] * a[ 20 ];
      tmp[ 2164 ] += a[ 12 ] * a[ 13 ] * a[ 20 ] * a[ 20 ];
      tmp[ 2165 ] += a[ 12 ] * a[ 13 ] * a[ 20 ] * a[ 23 ];
      tmp[ 2166 ] += a[ 12 ] * a[ 13 ] * a[ 20 ] * a[ 24 ];
      tmp[ 2167 ] += a[ 12 ] * a[ 14 ] * a[ 14 ] * a[ 15 ];
      tmp[ 2168 ] += a[ 12 ] * a[ 14 ] * a[ 14 ] * a[ 16 ];
      tmp[ 2169 ] += a[ 12 ] * a[ 14 ] * a[ 14 ] * a[ 20 ];
      tmp[ 2170 ] += a[ 12 ] * a[ 14 ] * a[ 14 ] * a[ 21 ];
      tmp[ 2171 ] += a[ 12 ] * a[ 14 ] * a[ 15 ] * a[ 15 ];
      tmp[ 2172 ] += a[ 12 ] * a[ 14 ] * a[ 15 ] * a[ 16 ];
      tmp[ 2173 ] += a[ 12 ] * a[ 14 ] * a[ 15 ] * a[ 18 ];
      tmp[ 2174 ] += a[ 12 ] * a[ 14 ] * a[ 15 ] * a[ 19 ];
      tmp[ 2175 ] += a[ 12 ] * a[ 14 ] * a[ 15 ] * a[ 20 ];
      tmp[ 2176 ] += a[ 12 ] * a[ 14 ] * a[ 15 ] * a[ 21 ];
      tmp[ 2177 ] += a[ 12 ] * a[ 14 ] * a[ 15 ] * a[ 23 ];
      tmp[ 2178 ] += a[ 12 ] * a[ 14 ] * a[ 15 ] * a[ 24 ];
      tmp[ 2179 ] += a[ 12 ] * a[ 14 ] * a[ 16 ] * a[ 16 ];
      tmp[ 2180 ] += a[ 12 ] * a[ 14 ] * a[ 16 ] * a[ 19 ];
      tmp[ 2181 ] += a[ 12 ] * a[ 14 ] * a[ 16 ] * a[ 20 ];
      tmp[ 2182 ] += a[ 12 ] * a[ 14 ] * a[ 16 ] * a[ 21 ];
      tmp[ 2183 ] += a[ 12 ] * a[ 14 ] * a[ 16 ] * a[ 24 ];
      tmp[ 2184 ] += a[ 12 ] * a[ 14 ] * a[ 18 ] * a[ 20 ];
      tmp[ 2185 ] += a[ 12 ] * a[ 14 ] * a[ 19 ] * a[ 20 ];
      tmp[ 2186 ] += a[ 12 ] * a[ 14 ] * a[ 19 ] * a[ 21 ];
      tmp[ 2187 ] += a[ 12 ] * a[ 14 ] * a[ 20 ] * a[ 20 ];
      tmp[ 2188 ] += a[ 12 ] * a[ 14 ] * a[ 20 ] * a[ 21 ];
      tmp[ 2189 ] += a[ 12 ] * a[ 14 ] * a[ 20 ] * a[ 23 ];
      tmp[ 2190 ] += a[ 12 ] * a[ 14 ] * a[ 20 ] * a[ 24 ];
      tmp[ 2191 ] += a[ 12 ] * a[ 14 ] * a[ 21 ] * a[ 21 ];
      tmp[ 2192 ] += a[ 12 ] * a[ 14 ] * a[ 21 ] * a[ 24 ];
      tmp[ 2193 ] += a[ 12 ] * a[ 15 ] * a[ 15 ] * a[ 18 ];
      tmp[ 2194 ] += a[ 12 ] * a[ 15 ] * a[ 15 ] * a[ 19 ];
      tmp[ 2195 ] += a[ 12 ] * a[ 15 ] * a[ 15 ] * a[ 23 ];
      tmp[ 2196 ] += a[ 12 ] * a[ 15 ] * a[ 15 ] * a[ 24 ];
      tmp[ 2197 ] += a[ 12 ] * a[ 15 ] * a[ 16 ] * a[ 19 ];
      tmp[ 2198 ] += a[ 12 ] * a[ 15 ] * a[ 16 ] * a[ 24 ];
      tmp[ 2199 ] += a[ 12 ] * a[ 15 ] * a[ 18 ] * a[ 18 ];
      tmp[ 2200 ] += a[ 12 ] * a[ 15 ] * a[ 18 ] * a[ 19 ];
      tmp[ 2201 ] += a[ 12 ] * a[ 15 ] * a[ 18 ] * a[ 20 ];
      tmp[ 2202 ] += a[ 12 ] * a[ 15 ] * a[ 18 ] * a[ 23 ];
      tmp[ 2203 ] += a[ 12 ] * a[ 15 ] * a[ 18 ] * a[ 24 ];
      tmp[ 2204 ] += a[ 12 ] * a[ 15 ] * a[ 19 ] * a[ 19 ];
      tmp[ 2205 ] += a[ 12 ] * a[ 15 ] * a[ 19 ] * a[ 20 ];
      tmp[ 2206 ] += a[ 12 ] * a[ 15 ] * a[ 19 ] * a[ 21 ];
      tmp[ 2207 ] += a[ 12 ] * a[ 15 ] * a[ 19 ] * a[ 23 ];
      tmp[ 2208 ] += a[ 12 ] * a[ 15 ] * a[ 19 ] * a[ 24 ];
      tmp[ 2209 ] += a[ 12 ] * a[ 15 ] * a[ 20 ] * a[ 23 ];
      tmp[ 2210 ] += a[ 12 ] * a[ 15 ] * a[ 20 ] * a[ 24 ];
      tmp[ 2211 ] += a[ 12 ] * a[ 15 ] * a[ 21 ] * a[ 24 ];
      tmp[ 2212 ] += a[ 12 ] * a[ 15 ] * a[ 23 ] * a[ 23 ];
      tmp[ 2213 ] += a[ 12 ] * a[ 15 ] * a[ 23 ] * a[ 24 ];
      tmp[ 2214 ] += a[ 12 ] * a[ 15 ] * a[ 24 ] * a[ 24 ];
      tmp[ 2215 ] += a[ 12 ] * a[ 16 ] * a[ 16 ] * a[ 19 ];
      tmp[ 2216 ] += a[ 12 ] * a[ 16 ] * a[ 16 ] * a[ 24 ];
      tmp[ 2217 ] += a[ 12 ] * a[ 16 ] * a[ 19 ] * a[ 19 ];
      tmp[ 2218 ] += a[ 12 ] * a[ 16 ] * a[ 19 ] * a[ 20 ];
      tmp[ 2219 ] += a[ 12 ] * a[ 16 ] * a[ 19 ] * a[ 21 ];
      tmp[ 2220 ] += a[ 12 ] * a[ 16 ] * a[ 19 ] * a[ 24 ];
      tmp[ 2221 ] += a[ 12 ] * a[ 16 ] * a[ 20 ] * a[ 24 ];
      tmp[ 2222 ] += a[ 12 ] * a[ 16 ] * a[ 21 ] * a[ 24 ];
      tmp[ 2223 ] += a[ 12 ] * a[ 16 ] * a[ 24 ] * a[ 24 ];
      tmp[ 2224 ] += a[ 12 ] * a[ 18 ] * a[ 18 ] * a[ 20 ];
      tmp[ 2225 ] += a[ 12 ] * a[ 18 ] * a[ 19 ] * a[ 20 ];
      tmp[ 2226 ] += a[ 12 ] * a[ 18 ] * a[ 20 ] * a[ 20 ];
      tmp[ 2227 ] += a[ 12 ] * a[ 18 ] * a[ 20 ] * a[ 23 ];
      tmp[ 2228 ] += a[ 12 ] * a[ 18 ] * a[ 20 ] * a[ 24 ];
      tmp[ 2229 ] += a[ 12 ] * a[ 19 ] * a[ 19 ] * a[ 20 ];
      tmp[ 2230 ] += a[ 12 ] * a[ 19 ] * a[ 19 ] * a[ 21 ];
      tmp[ 2231 ] += a[ 12 ] * a[ 19 ] * a[ 20 ] * a[ 20 ];
      tmp[ 2232 ] += a[ 12 ] * a[ 19 ] * a[ 20 ] * a[ 21 ];
      tmp[ 2233 ] += a[ 12 ] * a[ 19 ] * a[ 20 ] * a[ 23 ];
      tmp[ 2234 ] += a[ 12 ] * a[ 19 ] * a[ 20 ] * a[ 24 ];
      tmp[ 2235 ] += a[ 12 ] * a[ 19 ] * a[ 21 ] * a[ 21 ];
      tmp[ 2236 ] += a[ 12 ] * a[ 19 ] * a[ 21 ] * a[ 24 ];
      tmp[ 2237 ] += a[ 12 ] * a[ 20 ] * a[ 20 ] * a[ 23 ];
      tmp[ 2238 ] += a[ 12 ] * a[ 20 ] * a[ 20 ] * a[ 24 ];
      tmp[ 2239 ] += a[ 12 ] * a[ 20 ] * a[ 21 ] * a[ 24 ];
      tmp[ 2240 ] += a[ 12 ] * a[ 20 ] * a[ 23 ] * a[ 23 ];
      tmp[ 2241 ] += a[ 12 ] * a[ 20 ] * a[ 23 ] * a[ 24 ];
      tmp[ 2242 ] += a[ 12 ] * a[ 20 ] * a[ 24 ] * a[ 24 ];
      tmp[ 2243 ] += a[ 12 ] * a[ 21 ] * a[ 21 ] * a[ 24 ];
      tmp[ 2244 ] += a[ 12 ] * a[ 21 ] * a[ 24 ] * a[ 24 ];
    }    
  }
  
  /*** normalize ***/  
#ifdef ENABLE_NORMALIZATION
  double normalVal = (double)( 255 * ( right - left ) * ( down - up ) );
  result[ 0 ] = tmp[ 0 ] / normalVal;
  normalVal *= 255.0;
  for(int i=1; i<DIM_OF_GRAY_HLAC1_5; ++i){
    result[ i ] = tmp[ i ] / normalVal;
  }
  normalVal *= 255.0;
  for(int i=DIM_OF_GRAY_HLAC1_5; i<DIM_OF_GRAY_HLAC2_5; ++i){
    result[ i ] = tmp[ i ] / normalVal;
  }
  normalVal *= 255.0;
  for(int i=DIM_OF_GRAY_HLAC2_5; i<DIM_OF_GRAY_HLAC3_5; ++i){
    result[ i ] = tmp[ i ] / normalVal;
  }
#else /* !ENABLE_NORMALIZATION */
  result[ 0 ] = tmp[ 0 ] / 255.0;
  for(int i=1; i<DIM_OF_GRAY_HLAC1_5; ++i){
    result[ i ] = tmp[ i ] / 65025.0;
  }
  for(int i=DIM_OF_GRAY_HLAC1_5; i<DIM_OF_GRAY_HLAC2_5; ++i){
    result[ i ] = tmp[ i ] / 16581375.0;
  }
  for(int i=DIM_OF_GRAY_HLAC2_5; i<DIM_OF_GRAY_HLAC3_5; ++i){
    result[ i ] = tmp[ i ] / 4228250625.0;
  }
#endif /* ENABLE_NORMALIZATION */
}
#endif /* ENABLE_GRAY_HLAC3_5 */


/*-------------------*
 *  for color image  *
 *-------------------*/
/*** 高々1次の color HLAC, square size = 3 ***********************************/
 
void HLAC::_extractColor1_3( std::vector<float> &result, const cv::Mat &img, 
				const int &rx, const int &ry, 
				const int &left, const int &right,
				const int &up, const int &down )
{
  double a[ 27 ];
  const int width_step = img.step;
  
  /*** initialize ***/
  double tmp[ DIM_OF_COLOR_HLAC1_3 ];
  for(int i=0; i<DIM_OF_COLOR_HLAC1_3; ++i) tmp[ i ] = 0.0;
  
  /*** feature extraction ***/
  for(int j=up; j<down; ++j){
    for(int i=left; i<right; ++i){
      a[  0 ] = (double)img.data[ ( i - rx ) + ( j - ry ) * width_step ];
      a[  1 ] = (double)img.data[   i        + ( j - ry ) * width_step ];
      a[  2 ] = (double)img.data[ ( i + rx ) + ( j - ry ) * width_step ];
      a[  3 ] = (double)img.data[ ( i - rx ) +   j        * width_step ];
      a[  4 ] = (double)img.data[   i        +   j        * width_step ];
      a[  5 ] = (double)img.data[ ( i + rx ) +   j        * width_step ];
      a[  6 ] = (double)img.data[ ( i - rx ) + ( j + ry ) * width_step ];
      a[  7 ] = (double)img.data[   i        + ( j + ry ) * width_step ];
      a[  8 ] = (double)img.data[ ( i + rx ) + ( j + ry ) * width_step ];
      a[  9 ] = (double)img.data[ ( i - rx ) + ( j - ry ) * width_step +1 ];
      a[ 10 ] = (double)img.data[   i        + ( j - ry ) * width_step +1 ];
      a[ 11 ] = (double)img.data[ ( i + rx ) + ( j - ry ) * width_step +1 ];
      a[ 12 ] = (double)img.data[ ( i - rx ) +   j        * width_step +1 ];
      a[ 13 ] = (double)img.data[   i        +   j        * width_step +1 ];
      a[ 14 ] = (double)img.data[ ( i + rx ) +   j        * width_step +1 ];
      a[ 15 ] = (double)img.data[ ( i - rx ) + ( j + ry ) * width_step +1 ];
      a[ 16 ] = (double)img.data[   i        + ( j + ry ) * width_step +1 ];
      a[ 17 ] = (double)img.data[ ( i + rx ) + ( j + ry ) * width_step +1 ];
      a[ 18 ] = (double)img.data[ ( i - rx ) + ( j - ry ) * width_step +2 ];
      a[ 19 ] = (double)img.data[   i        + ( j - ry ) * width_step +2 ];
      a[ 20 ] = (double)img.data[ ( i + rx ) + ( j - ry ) * width_step +2 ];
      a[ 21 ] = (double)img.data[ ( i - rx ) +   j        * width_step +2 ];
      a[ 22 ] = (double)img.data[   i        +   j        * width_step +2 ];
      a[ 23 ] = (double)img.data[ ( i + rx ) +   j        * width_step +2 ];
      a[ 24 ] = (double)img.data[ ( i - rx ) + ( j + ry ) * width_step +2 ];
      a[ 25 ] = (double)img.data[   i        + ( j + ry ) * width_step +2 ];
      a[ 26 ] = (double)img.data[ ( i + rx ) + ( j + ry ) * width_step +2 ];
      tmp[ 0 ] += a[ 4 ];
      tmp[ 1 ] += a[ 13 ];
      tmp[ 2 ] += a[ 22 ];
      tmp[ 3 ] += a[ 4 ] * a[ 0 ];
      tmp[ 4 ] += a[ 4 ] * a[ 1 ];
      tmp[ 5 ] += a[ 4 ] * a[ 2 ];
      tmp[ 6 ] += a[ 4 ] * a[ 3 ];
      tmp[ 7 ] += a[ 4 ] * a[ 4 ];
      tmp[ 8 ] += a[ 4 ] * a[ 9 ];
      tmp[ 9 ] += a[ 4 ] * a[ 10 ];
      tmp[ 10 ] += a[ 4 ] * a[ 11 ];
      tmp[ 11 ] += a[ 4 ] * a[ 12 ];
      tmp[ 12 ] += a[ 4 ] * a[ 13 ];
      tmp[ 13 ] += a[ 4 ] * a[ 14 ];
      tmp[ 14 ] += a[ 4 ] * a[ 15 ];
      tmp[ 15 ] += a[ 4 ] * a[ 16 ];
      tmp[ 16 ] += a[ 4 ] * a[ 17 ];
      tmp[ 17 ] += a[ 4 ] * a[ 18 ];
      tmp[ 18 ] += a[ 4 ] * a[ 19 ];
      tmp[ 19 ] += a[ 4 ] * a[ 20 ];
      tmp[ 20 ] += a[ 4 ] * a[ 21 ];
      tmp[ 21 ] += a[ 4 ] * a[ 22 ];
      tmp[ 22 ] += a[ 4 ] * a[ 23 ];
      tmp[ 23 ] += a[ 4 ] * a[ 24 ];
      tmp[ 24 ] += a[ 4 ] * a[ 25 ];
      tmp[ 25 ] += a[ 4 ] * a[ 26 ];
      tmp[ 26 ] += a[ 13 ] * a[ 9 ];
      tmp[ 27 ] += a[ 13 ] * a[ 10 ];
      tmp[ 28 ] += a[ 13 ] * a[ 11 ];
      tmp[ 29 ] += a[ 13 ] * a[ 12 ];
      tmp[ 30 ] += a[ 13 ] * a[ 13 ];
      tmp[ 31 ] += a[ 13 ] * a[ 18 ];
      tmp[ 32 ] += a[ 13 ] * a[ 19 ];
      tmp[ 33 ] += a[ 13 ] * a[ 20 ];
      tmp[ 34 ] += a[ 13 ] * a[ 21 ];
      tmp[ 35 ] += a[ 13 ] * a[ 22 ];
      tmp[ 36 ] += a[ 13 ] * a[ 23 ];
      tmp[ 37 ] += a[ 13 ] * a[ 24 ];
      tmp[ 38 ] += a[ 13 ] * a[ 25 ];
      tmp[ 39 ] += a[ 13 ] * a[ 26 ];
      tmp[ 40 ] += a[ 22 ] * a[ 18 ];
      tmp[ 41 ] += a[ 22 ] * a[ 19 ];
      tmp[ 42 ] += a[ 22 ] * a[ 20 ];
      tmp[ 43 ] += a[ 22 ] * a[ 21 ];
      tmp[ 44 ] += a[ 22 ] * a[ 22 ];
    }  
  }
  
  /*** normalize ***/  
#ifdef ENABLE_NORMALIZATION
  double normalVal = (double)( 255 * ( right - left ) * ( down - up ) );
  result[ 0 ] = tmp[ 0 ] / normalVal;
  result[ 1 ] = tmp[ 1 ] / normalVal;
  result[ 2 ] = tmp[ 2 ] / normalVal;
  normalVal *= 255.0;
  for(int i=3; i<DIM_OF_COLOR_HLAC1_3; ++i) result[ i ] = tmp[ i ] / normalVal;
#else /* !ENABLE_NORMALIZATION */
  result[ 0 ] = tmp[ 0 ] / 255.0;
  result[ 1 ] = tmp[ 1 ] / 255.0;
  result[ 2 ] = tmp[ 2 ] / 255.0;
  for(int i=3; i<DIM_OF_COLOR_HLAC1_3; ++i) result[ i ] = tmp[ i ] / 65025.0;
#endif /* ENABLE_NORMALIZATION */
}

/*** 高々2次の color HLAC, square size = 3 ***********************************/
 
void HLAC::_extractColor2_3( std::vector<float> &result, const cv::Mat &img, 
				const int &rx, const int &ry, 
				const int &left, const int &right,
				const int &up, const int &down )
{
  double a[ 27 ];
  const int width_step = img.step;
  
  /*** initialize ***/
  double tmp[ DIM_OF_COLOR_HLAC2_3 ];
  for(int i=0; i<DIM_OF_COLOR_HLAC2_3; ++i) tmp[ i ] = 0.0;
  
  /*** feature extraction ***/
  for(int j=up; j<down; ++j){
    for(int i=left; i<right; ++i){
      a[  0 ] = (double)img.data[ ( i - rx ) + ( j - ry ) * width_step ];
      a[  1 ] = (double)img.data[   i        + ( j - ry ) * width_step ];
      a[  2 ] = (double)img.data[ ( i + rx ) + ( j - ry ) * width_step ];
      a[  3 ] = (double)img.data[ ( i - rx ) +   j        * width_step ];
      a[  4 ] = (double)img.data[   i        +   j        * width_step ];
      a[  5 ] = (double)img.data[ ( i + rx ) +   j        * width_step ];
      a[  6 ] = (double)img.data[ ( i - rx ) + ( j + ry ) * width_step ];
      a[  7 ] = (double)img.data[   i        + ( j + ry ) * width_step ];
      a[  8 ] = (double)img.data[ ( i + rx ) + ( j + ry ) * width_step ];
      a[  9 ] = (double)img.data[ ( i - rx ) + ( j - ry ) * width_step +1 ];
      a[ 10 ] = (double)img.data[   i        + ( j - ry ) * width_step +1 ];
      a[ 11 ] = (double)img.data[ ( i + rx ) + ( j - ry ) * width_step +1 ];
      a[ 12 ] = (double)img.data[ ( i - rx ) +   j        * width_step +1 ];
      a[ 13 ] = (double)img.data[   i        +   j        * width_step +1 ];
      a[ 14 ] = (double)img.data[ ( i + rx ) +   j        * width_step +1 ];
      a[ 15 ] = (double)img.data[ ( i - rx ) + ( j + ry ) * width_step +1 ];
      a[ 16 ] = (double)img.data[   i        + ( j + ry ) * width_step +1 ];
      a[ 17 ] = (double)img.data[ ( i + rx ) + ( j + ry ) * width_step +1 ];
      a[ 18 ] = (double)img.data[ ( i - rx ) + ( j - ry ) * width_step +2 ];
      a[ 19 ] = (double)img.data[   i        + ( j - ry ) * width_step +2 ];
      a[ 20 ] = (double)img.data[ ( i + rx ) + ( j - ry ) * width_step +2 ];
      a[ 21 ] = (double)img.data[ ( i - rx ) +   j        * width_step +2 ];
      a[ 22 ] = (double)img.data[   i        +   j        * width_step +2 ];
      a[ 23 ] = (double)img.data[ ( i + rx ) +   j        * width_step +2 ];
      a[ 24 ] = (double)img.data[ ( i - rx ) + ( j + ry ) * width_step +2 ];
      a[ 25 ] = (double)img.data[   i        + ( j + ry ) * width_step +2 ];
      a[ 26 ] = (double)img.data[ ( i + rx ) + ( j + ry ) * width_step +2 ];
      tmp[ 0 ] += a[ 4 ];
      tmp[ 1 ] += a[ 13 ];
      tmp[ 2 ] += a[ 22 ];
      tmp[ 3 ] += a[ 4 ] * a[ 0 ];
      tmp[ 4 ] += a[ 4 ] * a[ 1 ];
      tmp[ 5 ] += a[ 4 ] * a[ 2 ];
      tmp[ 6 ] += a[ 4 ] * a[ 3 ];
      tmp[ 7 ] += a[ 4 ] * a[ 4 ];
      tmp[ 8 ] += a[ 4 ] * a[ 9 ];
      tmp[ 9 ] += a[ 4 ] * a[ 10 ];
      tmp[ 10 ] += a[ 4 ] * a[ 11 ];
      tmp[ 11 ] += a[ 4 ] * a[ 12 ];
      tmp[ 12 ] += a[ 4 ] * a[ 13 ];
      tmp[ 13 ] += a[ 4 ] * a[ 14 ];
      tmp[ 14 ] += a[ 4 ] * a[ 15 ];
      tmp[ 15 ] += a[ 4 ] * a[ 16 ];
      tmp[ 16 ] += a[ 4 ] * a[ 17 ];
      tmp[ 17 ] += a[ 4 ] * a[ 18 ];
      tmp[ 18 ] += a[ 4 ] * a[ 19 ];
      tmp[ 19 ] += a[ 4 ] * a[ 20 ];
      tmp[ 20 ] += a[ 4 ] * a[ 21 ];
      tmp[ 21 ] += a[ 4 ] * a[ 22 ];
      tmp[ 22 ] += a[ 4 ] * a[ 23 ];
      tmp[ 23 ] += a[ 4 ] * a[ 24 ];
      tmp[ 24 ] += a[ 4 ] * a[ 25 ];
      tmp[ 25 ] += a[ 4 ] * a[ 26 ];
      tmp[ 26 ] += a[ 13 ] * a[ 9 ];
      tmp[ 27 ] += a[ 13 ] * a[ 10 ];
      tmp[ 28 ] += a[ 13 ] * a[ 11 ];
      tmp[ 29 ] += a[ 13 ] * a[ 12 ];
      tmp[ 30 ] += a[ 13 ] * a[ 13 ];
      tmp[ 31 ] += a[ 13 ] * a[ 18 ];
      tmp[ 32 ] += a[ 13 ] * a[ 19 ];
      tmp[ 33 ] += a[ 13 ] * a[ 20 ];
      tmp[ 34 ] += a[ 13 ] * a[ 21 ];
      tmp[ 35 ] += a[ 13 ] * a[ 22 ];
      tmp[ 36 ] += a[ 13 ] * a[ 23 ];
      tmp[ 37 ] += a[ 13 ] * a[ 24 ];
      tmp[ 38 ] += a[ 13 ] * a[ 25 ];
      tmp[ 39 ] += a[ 13 ] * a[ 26 ];
      tmp[ 40 ] += a[ 22 ] * a[ 18 ];
      tmp[ 41 ] += a[ 22 ] * a[ 19 ];
      tmp[ 42 ] += a[ 22 ] * a[ 20 ];
      tmp[ 43 ] += a[ 22 ] * a[ 21 ];
      tmp[ 44 ] += a[ 22 ] * a[ 22 ];
      tmp[ 45 ] += a[ 4 ] * a[ 0 ] * a[ 0 ];
      tmp[ 46 ] += a[ 4 ] * a[ 0 ] * a[ 1 ];
      tmp[ 47 ] += a[ 4 ] * a[ 0 ] * a[ 2 ];
      tmp[ 48 ] += a[ 4 ] * a[ 0 ] * a[ 3 ];
      tmp[ 49 ] += a[ 4 ] * a[ 0 ] * a[ 4 ];
      tmp[ 50 ] += a[ 4 ] * a[ 0 ] * a[ 5 ];
      tmp[ 51 ] += a[ 4 ] * a[ 0 ] * a[ 6 ];
      tmp[ 52 ] += a[ 4 ] * a[ 0 ] * a[ 7 ];
      tmp[ 53 ] += a[ 4 ] * a[ 0 ] * a[ 8 ];
      tmp[ 54 ] += a[ 4 ] * a[ 0 ] * a[ 9 ];
      tmp[ 55 ] += a[ 4 ] * a[ 0 ] * a[ 10 ];
      tmp[ 56 ] += a[ 4 ] * a[ 0 ] * a[ 11 ];
      tmp[ 57 ] += a[ 4 ] * a[ 0 ] * a[ 12 ];
      tmp[ 58 ] += a[ 4 ] * a[ 0 ] * a[ 13 ];
      tmp[ 59 ] += a[ 4 ] * a[ 0 ] * a[ 14 ];
      tmp[ 60 ] += a[ 4 ] * a[ 0 ] * a[ 15 ];
      tmp[ 61 ] += a[ 4 ] * a[ 0 ] * a[ 16 ];
      tmp[ 62 ] += a[ 4 ] * a[ 0 ] * a[ 17 ];
      tmp[ 63 ] += a[ 4 ] * a[ 0 ] * a[ 18 ];
      tmp[ 64 ] += a[ 4 ] * a[ 0 ] * a[ 19 ];
      tmp[ 65 ] += a[ 4 ] * a[ 0 ] * a[ 20 ];
      tmp[ 66 ] += a[ 4 ] * a[ 0 ] * a[ 21 ];
      tmp[ 67 ] += a[ 4 ] * a[ 0 ] * a[ 22 ];
      tmp[ 68 ] += a[ 4 ] * a[ 0 ] * a[ 23 ];
      tmp[ 69 ] += a[ 4 ] * a[ 0 ] * a[ 24 ];
      tmp[ 70 ] += a[ 4 ] * a[ 0 ] * a[ 25 ];
      tmp[ 71 ] += a[ 4 ] * a[ 0 ] * a[ 26 ];
      tmp[ 72 ] += a[ 4 ] * a[ 1 ] * a[ 1 ];
      tmp[ 73 ] += a[ 4 ] * a[ 1 ] * a[ 2 ];
      tmp[ 74 ] += a[ 4 ] * a[ 1 ] * a[ 3 ];
      tmp[ 75 ] += a[ 4 ] * a[ 1 ] * a[ 4 ];
      tmp[ 76 ] += a[ 4 ] * a[ 1 ] * a[ 6 ];
      tmp[ 77 ] += a[ 4 ] * a[ 1 ] * a[ 7 ];
      tmp[ 78 ] += a[ 4 ] * a[ 1 ] * a[ 8 ];
      tmp[ 79 ] += a[ 4 ] * a[ 1 ] * a[ 9 ];
      tmp[ 80 ] += a[ 4 ] * a[ 1 ] * a[ 10 ];
      tmp[ 81 ] += a[ 4 ] * a[ 1 ] * a[ 11 ];
      tmp[ 82 ] += a[ 4 ] * a[ 1 ] * a[ 12 ];
      tmp[ 83 ] += a[ 4 ] * a[ 1 ] * a[ 13 ];
      tmp[ 84 ] += a[ 4 ] * a[ 1 ] * a[ 14 ];
      tmp[ 85 ] += a[ 4 ] * a[ 1 ] * a[ 15 ];
      tmp[ 86 ] += a[ 4 ] * a[ 1 ] * a[ 16 ];
      tmp[ 87 ] += a[ 4 ] * a[ 1 ] * a[ 17 ];
      tmp[ 88 ] += a[ 4 ] * a[ 1 ] * a[ 18 ];
      tmp[ 89 ] += a[ 4 ] * a[ 1 ] * a[ 19 ];
      tmp[ 90 ] += a[ 4 ] * a[ 1 ] * a[ 20 ];
      tmp[ 91 ] += a[ 4 ] * a[ 1 ] * a[ 21 ];
      tmp[ 92 ] += a[ 4 ] * a[ 1 ] * a[ 22 ];
      tmp[ 93 ] += a[ 4 ] * a[ 1 ] * a[ 23 ];
      tmp[ 94 ] += a[ 4 ] * a[ 1 ] * a[ 24 ];
      tmp[ 95 ] += a[ 4 ] * a[ 1 ] * a[ 25 ];
      tmp[ 96 ] += a[ 4 ] * a[ 1 ] * a[ 26 ];
      tmp[ 97 ] += a[ 4 ] * a[ 2 ] * a[ 2 ];
      tmp[ 98 ] += a[ 4 ] * a[ 2 ] * a[ 3 ];
      tmp[ 99 ] += a[ 4 ] * a[ 2 ] * a[ 4 ];
      tmp[ 100 ] += a[ 4 ] * a[ 2 ] * a[ 6 ];
      tmp[ 101 ] += a[ 4 ] * a[ 2 ] * a[ 7 ];
      tmp[ 102 ] += a[ 4 ] * a[ 2 ] * a[ 8 ];
      tmp[ 103 ] += a[ 4 ] * a[ 2 ] * a[ 9 ];
      tmp[ 104 ] += a[ 4 ] * a[ 2 ] * a[ 10 ];
      tmp[ 105 ] += a[ 4 ] * a[ 2 ] * a[ 11 ];
      tmp[ 106 ] += a[ 4 ] * a[ 2 ] * a[ 12 ];
      tmp[ 107 ] += a[ 4 ] * a[ 2 ] * a[ 13 ];
      tmp[ 108 ] += a[ 4 ] * a[ 2 ] * a[ 14 ];
      tmp[ 109 ] += a[ 4 ] * a[ 2 ] * a[ 15 ];
      tmp[ 110 ] += a[ 4 ] * a[ 2 ] * a[ 16 ];
      tmp[ 111 ] += a[ 4 ] * a[ 2 ] * a[ 17 ];
      tmp[ 112 ] += a[ 4 ] * a[ 2 ] * a[ 18 ];
      tmp[ 113 ] += a[ 4 ] * a[ 2 ] * a[ 19 ];
      tmp[ 114 ] += a[ 4 ] * a[ 2 ] * a[ 20 ];
      tmp[ 115 ] += a[ 4 ] * a[ 2 ] * a[ 21 ];
      tmp[ 116 ] += a[ 4 ] * a[ 2 ] * a[ 22 ];
      tmp[ 117 ] += a[ 4 ] * a[ 2 ] * a[ 23 ];
      tmp[ 118 ] += a[ 4 ] * a[ 2 ] * a[ 24 ];
      tmp[ 119 ] += a[ 4 ] * a[ 2 ] * a[ 25 ];
      tmp[ 120 ] += a[ 4 ] * a[ 2 ] * a[ 26 ];
      tmp[ 121 ] += a[ 4 ] * a[ 3 ] * a[ 3 ];
      tmp[ 122 ] += a[ 4 ] * a[ 3 ] * a[ 4 ];
      tmp[ 123 ] += a[ 4 ] * a[ 3 ] * a[ 5 ];
      tmp[ 124 ] += a[ 4 ] * a[ 3 ] * a[ 8 ];
      tmp[ 125 ] += a[ 4 ] * a[ 3 ] * a[ 9 ];
      tmp[ 126 ] += a[ 4 ] * a[ 3 ] * a[ 10 ];
      tmp[ 127 ] += a[ 4 ] * a[ 3 ] * a[ 11 ];
      tmp[ 128 ] += a[ 4 ] * a[ 3 ] * a[ 12 ];
      tmp[ 129 ] += a[ 4 ] * a[ 3 ] * a[ 13 ];
      tmp[ 130 ] += a[ 4 ] * a[ 3 ] * a[ 14 ];
      tmp[ 131 ] += a[ 4 ] * a[ 3 ] * a[ 15 ];
      tmp[ 132 ] += a[ 4 ] * a[ 3 ] * a[ 16 ];
      tmp[ 133 ] += a[ 4 ] * a[ 3 ] * a[ 17 ];
      tmp[ 134 ] += a[ 4 ] * a[ 3 ] * a[ 18 ];
      tmp[ 135 ] += a[ 4 ] * a[ 3 ] * a[ 19 ];
      tmp[ 136 ] += a[ 4 ] * a[ 3 ] * a[ 20 ];
      tmp[ 137 ] += a[ 4 ] * a[ 3 ] * a[ 21 ];
      tmp[ 138 ] += a[ 4 ] * a[ 3 ] * a[ 22 ];
      tmp[ 139 ] += a[ 4 ] * a[ 3 ] * a[ 23 ];
      tmp[ 140 ] += a[ 4 ] * a[ 3 ] * a[ 24 ];
      tmp[ 141 ] += a[ 4 ] * a[ 3 ] * a[ 25 ];
      tmp[ 142 ] += a[ 4 ] * a[ 3 ] * a[ 26 ];
      tmp[ 143 ] += a[ 4 ] * a[ 4 ] * a[ 4 ];
      tmp[ 144 ] += a[ 4 ] * a[ 4 ] * a[ 9 ];
      tmp[ 145 ] += a[ 4 ] * a[ 4 ] * a[ 10 ];
      tmp[ 146 ] += a[ 4 ] * a[ 4 ] * a[ 11 ];
      tmp[ 147 ] += a[ 4 ] * a[ 4 ] * a[ 12 ];
      tmp[ 148 ] += a[ 4 ] * a[ 4 ] * a[ 13 ];
      tmp[ 149 ] += a[ 4 ] * a[ 4 ] * a[ 14 ];
      tmp[ 150 ] += a[ 4 ] * a[ 4 ] * a[ 15 ];
      tmp[ 151 ] += a[ 4 ] * a[ 4 ] * a[ 16 ];
      tmp[ 152 ] += a[ 4 ] * a[ 4 ] * a[ 17 ];
      tmp[ 153 ] += a[ 4 ] * a[ 4 ] * a[ 18 ];
      tmp[ 154 ] += a[ 4 ] * a[ 4 ] * a[ 19 ];
      tmp[ 155 ] += a[ 4 ] * a[ 4 ] * a[ 20 ];
      tmp[ 156 ] += a[ 4 ] * a[ 4 ] * a[ 21 ];
      tmp[ 157 ] += a[ 4 ] * a[ 4 ] * a[ 22 ];
      tmp[ 158 ] += a[ 4 ] * a[ 4 ] * a[ 23 ];
      tmp[ 159 ] += a[ 4 ] * a[ 4 ] * a[ 24 ];
      tmp[ 160 ] += a[ 4 ] * a[ 4 ] * a[ 25 ];
      tmp[ 161 ] += a[ 4 ] * a[ 4 ] * a[ 26 ];
      tmp[ 162 ] += a[ 4 ] * a[ 5 ] * a[ 6 ];
      tmp[ 163 ] += a[ 4 ] * a[ 5 ] * a[ 9 ];
      tmp[ 164 ] += a[ 4 ] * a[ 5 ] * a[ 12 ];
      tmp[ 165 ] += a[ 4 ] * a[ 5 ] * a[ 15 ];
      tmp[ 166 ] += a[ 4 ] * a[ 5 ] * a[ 18 ];
      tmp[ 167 ] += a[ 4 ] * a[ 5 ] * a[ 21 ];
      tmp[ 168 ] += a[ 4 ] * a[ 5 ] * a[ 24 ];
      tmp[ 169 ] += a[ 4 ] * a[ 6 ] * a[ 8 ];
      tmp[ 170 ] += a[ 4 ] * a[ 6 ] * a[ 9 ];
      tmp[ 171 ] += a[ 4 ] * a[ 6 ] * a[ 10 ];
      tmp[ 172 ] += a[ 4 ] * a[ 6 ] * a[ 11 ];
      tmp[ 173 ] += a[ 4 ] * a[ 6 ] * a[ 14 ];
      tmp[ 174 ] += a[ 4 ] * a[ 6 ] * a[ 17 ];
      tmp[ 175 ] += a[ 4 ] * a[ 6 ] * a[ 18 ];
      tmp[ 176 ] += a[ 4 ] * a[ 6 ] * a[ 19 ];
      tmp[ 177 ] += a[ 4 ] * a[ 6 ] * a[ 20 ];
      tmp[ 178 ] += a[ 4 ] * a[ 6 ] * a[ 23 ];
      tmp[ 179 ] += a[ 4 ] * a[ 6 ] * a[ 26 ];
      tmp[ 180 ] += a[ 4 ] * a[ 7 ] * a[ 9 ];
      tmp[ 181 ] += a[ 4 ] * a[ 7 ] * a[ 10 ];
      tmp[ 182 ] += a[ 4 ] * a[ 7 ] * a[ 11 ];
      tmp[ 183 ] += a[ 4 ] * a[ 7 ] * a[ 18 ];
      tmp[ 184 ] += a[ 4 ] * a[ 7 ] * a[ 19 ];
      tmp[ 185 ] += a[ 4 ] * a[ 7 ] * a[ 20 ];
      tmp[ 186 ] += a[ 4 ] * a[ 8 ] * a[ 9 ];
      tmp[ 187 ] += a[ 4 ] * a[ 8 ] * a[ 10 ];
      tmp[ 188 ] += a[ 4 ] * a[ 8 ] * a[ 11 ];
      tmp[ 189 ] += a[ 4 ] * a[ 8 ] * a[ 12 ];
      tmp[ 190 ] += a[ 4 ] * a[ 8 ] * a[ 15 ];
      tmp[ 191 ] += a[ 4 ] * a[ 8 ] * a[ 18 ];
      tmp[ 192 ] += a[ 4 ] * a[ 8 ] * a[ 19 ];
      tmp[ 193 ] += a[ 4 ] * a[ 8 ] * a[ 20 ];
      tmp[ 194 ] += a[ 4 ] * a[ 8 ] * a[ 21 ];
      tmp[ 195 ] += a[ 4 ] * a[ 8 ] * a[ 24 ];
      tmp[ 196 ] += a[ 4 ] * a[ 9 ] * a[ 9 ];
      tmp[ 197 ] += a[ 4 ] * a[ 9 ] * a[ 10 ];
      tmp[ 198 ] += a[ 4 ] * a[ 9 ] * a[ 11 ];
      tmp[ 199 ] += a[ 4 ] * a[ 9 ] * a[ 12 ];
      tmp[ 200 ] += a[ 4 ] * a[ 9 ] * a[ 13 ];
      tmp[ 201 ] += a[ 4 ] * a[ 9 ] * a[ 14 ];
      tmp[ 202 ] += a[ 4 ] * a[ 9 ] * a[ 15 ];
      tmp[ 203 ] += a[ 4 ] * a[ 9 ] * a[ 16 ];
      tmp[ 204 ] += a[ 4 ] * a[ 9 ] * a[ 17 ];
      tmp[ 205 ] += a[ 4 ] * a[ 9 ] * a[ 18 ];
      tmp[ 206 ] += a[ 4 ] * a[ 9 ] * a[ 19 ];
      tmp[ 207 ] += a[ 4 ] * a[ 9 ] * a[ 20 ];
      tmp[ 208 ] += a[ 4 ] * a[ 9 ] * a[ 21 ];
      tmp[ 209 ] += a[ 4 ] * a[ 9 ] * a[ 22 ];
      tmp[ 210 ] += a[ 4 ] * a[ 9 ] * a[ 23 ];
      tmp[ 211 ] += a[ 4 ] * a[ 9 ] * a[ 24 ];
      tmp[ 212 ] += a[ 4 ] * a[ 9 ] * a[ 25 ];
      tmp[ 213 ] += a[ 4 ] * a[ 9 ] * a[ 26 ];
      tmp[ 214 ] += a[ 4 ] * a[ 10 ] * a[ 10 ];
      tmp[ 215 ] += a[ 4 ] * a[ 10 ] * a[ 11 ];
      tmp[ 216 ] += a[ 4 ] * a[ 10 ] * a[ 12 ];
      tmp[ 217 ] += a[ 4 ] * a[ 10 ] * a[ 13 ];
      tmp[ 218 ] += a[ 4 ] * a[ 10 ] * a[ 14 ];
      tmp[ 219 ] += a[ 4 ] * a[ 10 ] * a[ 15 ];
      tmp[ 220 ] += a[ 4 ] * a[ 10 ] * a[ 16 ];
      tmp[ 221 ] += a[ 4 ] * a[ 10 ] * a[ 17 ];
      tmp[ 222 ] += a[ 4 ] * a[ 10 ] * a[ 18 ];
      tmp[ 223 ] += a[ 4 ] * a[ 10 ] * a[ 19 ];
      tmp[ 224 ] += a[ 4 ] * a[ 10 ] * a[ 20 ];
      tmp[ 225 ] += a[ 4 ] * a[ 10 ] * a[ 21 ];
      tmp[ 226 ] += a[ 4 ] * a[ 10 ] * a[ 22 ];
      tmp[ 227 ] += a[ 4 ] * a[ 10 ] * a[ 23 ];
      tmp[ 228 ] += a[ 4 ] * a[ 10 ] * a[ 24 ];
      tmp[ 229 ] += a[ 4 ] * a[ 10 ] * a[ 25 ];
      tmp[ 230 ] += a[ 4 ] * a[ 10 ] * a[ 26 ];
      tmp[ 231 ] += a[ 4 ] * a[ 11 ] * a[ 11 ];
      tmp[ 232 ] += a[ 4 ] * a[ 11 ] * a[ 12 ];
      tmp[ 233 ] += a[ 4 ] * a[ 11 ] * a[ 13 ];
      tmp[ 234 ] += a[ 4 ] * a[ 11 ] * a[ 14 ];
      tmp[ 235 ] += a[ 4 ] * a[ 11 ] * a[ 15 ];
      tmp[ 236 ] += a[ 4 ] * a[ 11 ] * a[ 16 ];
      tmp[ 237 ] += a[ 4 ] * a[ 11 ] * a[ 17 ];
      tmp[ 238 ] += a[ 4 ] * a[ 11 ] * a[ 18 ];
      tmp[ 239 ] += a[ 4 ] * a[ 11 ] * a[ 19 ];
      tmp[ 240 ] += a[ 4 ] * a[ 11 ] * a[ 20 ];
      tmp[ 241 ] += a[ 4 ] * a[ 11 ] * a[ 21 ];
      tmp[ 242 ] += a[ 4 ] * a[ 11 ] * a[ 22 ];
      tmp[ 243 ] += a[ 4 ] * a[ 11 ] * a[ 23 ];
      tmp[ 244 ] += a[ 4 ] * a[ 11 ] * a[ 24 ];
      tmp[ 245 ] += a[ 4 ] * a[ 11 ] * a[ 25 ];
      tmp[ 246 ] += a[ 4 ] * a[ 11 ] * a[ 26 ];
      tmp[ 247 ] += a[ 4 ] * a[ 12 ] * a[ 12 ];
      tmp[ 248 ] += a[ 4 ] * a[ 12 ] * a[ 13 ];
      tmp[ 249 ] += a[ 4 ] * a[ 12 ] * a[ 14 ];
      tmp[ 250 ] += a[ 4 ] * a[ 12 ] * a[ 15 ];
      tmp[ 251 ] += a[ 4 ] * a[ 12 ] * a[ 16 ];
      tmp[ 252 ] += a[ 4 ] * a[ 12 ] * a[ 17 ];
      tmp[ 253 ] += a[ 4 ] * a[ 12 ] * a[ 18 ];
      tmp[ 254 ] += a[ 4 ] * a[ 12 ] * a[ 19 ];
      tmp[ 255 ] += a[ 4 ] * a[ 12 ] * a[ 20 ];
      tmp[ 256 ] += a[ 4 ] * a[ 12 ] * a[ 21 ];
      tmp[ 257 ] += a[ 4 ] * a[ 12 ] * a[ 22 ];
      tmp[ 258 ] += a[ 4 ] * a[ 12 ] * a[ 23 ];
      tmp[ 259 ] += a[ 4 ] * a[ 12 ] * a[ 24 ];
      tmp[ 260 ] += a[ 4 ] * a[ 12 ] * a[ 25 ];
      tmp[ 261 ] += a[ 4 ] * a[ 12 ] * a[ 26 ];
      tmp[ 262 ] += a[ 4 ] * a[ 13 ] * a[ 13 ];
      tmp[ 263 ] += a[ 4 ] * a[ 13 ] * a[ 14 ];
      tmp[ 264 ] += a[ 4 ] * a[ 13 ] * a[ 15 ];
      tmp[ 265 ] += a[ 4 ] * a[ 13 ] * a[ 16 ];
      tmp[ 266 ] += a[ 4 ] * a[ 13 ] * a[ 17 ];
      tmp[ 267 ] += a[ 4 ] * a[ 13 ] * a[ 18 ];
      tmp[ 268 ] += a[ 4 ] * a[ 13 ] * a[ 19 ];
      tmp[ 269 ] += a[ 4 ] * a[ 13 ] * a[ 20 ];
      tmp[ 270 ] += a[ 4 ] * a[ 13 ] * a[ 21 ];
      tmp[ 271 ] += a[ 4 ] * a[ 13 ] * a[ 22 ];
      tmp[ 272 ] += a[ 4 ] * a[ 13 ] * a[ 23 ];
      tmp[ 273 ] += a[ 4 ] * a[ 13 ] * a[ 24 ];
      tmp[ 274 ] += a[ 4 ] * a[ 13 ] * a[ 25 ];
      tmp[ 275 ] += a[ 4 ] * a[ 13 ] * a[ 26 ];
      tmp[ 276 ] += a[ 4 ] * a[ 14 ] * a[ 14 ];
      tmp[ 277 ] += a[ 4 ] * a[ 14 ] * a[ 15 ];
      tmp[ 278 ] += a[ 4 ] * a[ 14 ] * a[ 16 ];
      tmp[ 279 ] += a[ 4 ] * a[ 14 ] * a[ 17 ];
      tmp[ 280 ] += a[ 4 ] * a[ 14 ] * a[ 18 ];
      tmp[ 281 ] += a[ 4 ] * a[ 14 ] * a[ 19 ];
      tmp[ 282 ] += a[ 4 ] * a[ 14 ] * a[ 20 ];
      tmp[ 283 ] += a[ 4 ] * a[ 14 ] * a[ 21 ];
      tmp[ 284 ] += a[ 4 ] * a[ 14 ] * a[ 22 ];
      tmp[ 285 ] += a[ 4 ] * a[ 14 ] * a[ 23 ];
      tmp[ 286 ] += a[ 4 ] * a[ 14 ] * a[ 24 ];
      tmp[ 287 ] += a[ 4 ] * a[ 14 ] * a[ 25 ];
      tmp[ 288 ] += a[ 4 ] * a[ 14 ] * a[ 26 ];
      tmp[ 289 ] += a[ 4 ] * a[ 15 ] * a[ 15 ];
      tmp[ 290 ] += a[ 4 ] * a[ 15 ] * a[ 16 ];
      tmp[ 291 ] += a[ 4 ] * a[ 15 ] * a[ 17 ];
      tmp[ 292 ] += a[ 4 ] * a[ 15 ] * a[ 18 ];
      tmp[ 293 ] += a[ 4 ] * a[ 15 ] * a[ 19 ];
      tmp[ 294 ] += a[ 4 ] * a[ 15 ] * a[ 20 ];
      tmp[ 295 ] += a[ 4 ] * a[ 15 ] * a[ 21 ];
      tmp[ 296 ] += a[ 4 ] * a[ 15 ] * a[ 22 ];
      tmp[ 297 ] += a[ 4 ] * a[ 15 ] * a[ 23 ];
      tmp[ 298 ] += a[ 4 ] * a[ 15 ] * a[ 24 ];
      tmp[ 299 ] += a[ 4 ] * a[ 15 ] * a[ 25 ];
      tmp[ 300 ] += a[ 4 ] * a[ 15 ] * a[ 26 ];
      tmp[ 301 ] += a[ 4 ] * a[ 16 ] * a[ 16 ];
      tmp[ 302 ] += a[ 4 ] * a[ 16 ] * a[ 17 ];
      tmp[ 303 ] += a[ 4 ] * a[ 16 ] * a[ 18 ];
      tmp[ 304 ] += a[ 4 ] * a[ 16 ] * a[ 19 ];
      tmp[ 305 ] += a[ 4 ] * a[ 16 ] * a[ 20 ];
      tmp[ 306 ] += a[ 4 ] * a[ 16 ] * a[ 21 ];
      tmp[ 307 ] += a[ 4 ] * a[ 16 ] * a[ 22 ];
      tmp[ 308 ] += a[ 4 ] * a[ 16 ] * a[ 23 ];
      tmp[ 309 ] += a[ 4 ] * a[ 16 ] * a[ 24 ];
      tmp[ 310 ] += a[ 4 ] * a[ 16 ] * a[ 25 ];
      tmp[ 311 ] += a[ 4 ] * a[ 16 ] * a[ 26 ];
      tmp[ 312 ] += a[ 4 ] * a[ 17 ] * a[ 17 ];
      tmp[ 313 ] += a[ 4 ] * a[ 17 ] * a[ 18 ];
      tmp[ 314 ] += a[ 4 ] * a[ 17 ] * a[ 19 ];
      tmp[ 315 ] += a[ 4 ] * a[ 17 ] * a[ 20 ];
      tmp[ 316 ] += a[ 4 ] * a[ 17 ] * a[ 21 ];
      tmp[ 317 ] += a[ 4 ] * a[ 17 ] * a[ 22 ];
      tmp[ 318 ] += a[ 4 ] * a[ 17 ] * a[ 23 ];
      tmp[ 319 ] += a[ 4 ] * a[ 17 ] * a[ 24 ];
      tmp[ 320 ] += a[ 4 ] * a[ 17 ] * a[ 25 ];
      tmp[ 321 ] += a[ 4 ] * a[ 17 ] * a[ 26 ];
      tmp[ 322 ] += a[ 4 ] * a[ 18 ] * a[ 18 ];
      tmp[ 323 ] += a[ 4 ] * a[ 18 ] * a[ 19 ];
      tmp[ 324 ] += a[ 4 ] * a[ 18 ] * a[ 20 ];
      tmp[ 325 ] += a[ 4 ] * a[ 18 ] * a[ 21 ];
      tmp[ 326 ] += a[ 4 ] * a[ 18 ] * a[ 22 ];
      tmp[ 327 ] += a[ 4 ] * a[ 18 ] * a[ 23 ];
      tmp[ 328 ] += a[ 4 ] * a[ 18 ] * a[ 24 ];
      tmp[ 329 ] += a[ 4 ] * a[ 18 ] * a[ 25 ];
      tmp[ 330 ] += a[ 4 ] * a[ 18 ] * a[ 26 ];
      tmp[ 331 ] += a[ 4 ] * a[ 19 ] * a[ 19 ];
      tmp[ 332 ] += a[ 4 ] * a[ 19 ] * a[ 20 ];
      tmp[ 333 ] += a[ 4 ] * a[ 19 ] * a[ 21 ];
      tmp[ 334 ] += a[ 4 ] * a[ 19 ] * a[ 22 ];
      tmp[ 335 ] += a[ 4 ] * a[ 19 ] * a[ 23 ];
      tmp[ 336 ] += a[ 4 ] * a[ 19 ] * a[ 24 ];
      tmp[ 337 ] += a[ 4 ] * a[ 19 ] * a[ 25 ];
      tmp[ 338 ] += a[ 4 ] * a[ 19 ] * a[ 26 ];
      tmp[ 339 ] += a[ 4 ] * a[ 20 ] * a[ 20 ];
      tmp[ 340 ] += a[ 4 ] * a[ 20 ] * a[ 21 ];
      tmp[ 341 ] += a[ 4 ] * a[ 20 ] * a[ 22 ];
      tmp[ 342 ] += a[ 4 ] * a[ 20 ] * a[ 23 ];
      tmp[ 343 ] += a[ 4 ] * a[ 20 ] * a[ 24 ];
      tmp[ 344 ] += a[ 4 ] * a[ 20 ] * a[ 25 ];
      tmp[ 345 ] += a[ 4 ] * a[ 20 ] * a[ 26 ];
      tmp[ 346 ] += a[ 4 ] * a[ 21 ] * a[ 21 ];
      tmp[ 347 ] += a[ 4 ] * a[ 21 ] * a[ 22 ];
      tmp[ 348 ] += a[ 4 ] * a[ 21 ] * a[ 23 ];
      tmp[ 349 ] += a[ 4 ] * a[ 21 ] * a[ 24 ];
      tmp[ 350 ] += a[ 4 ] * a[ 21 ] * a[ 25 ];
      tmp[ 351 ] += a[ 4 ] * a[ 21 ] * a[ 26 ];
      tmp[ 352 ] += a[ 4 ] * a[ 22 ] * a[ 22 ];
      tmp[ 353 ] += a[ 4 ] * a[ 22 ] * a[ 23 ];
      tmp[ 354 ] += a[ 4 ] * a[ 22 ] * a[ 24 ];
      tmp[ 355 ] += a[ 4 ] * a[ 22 ] * a[ 25 ];
      tmp[ 356 ] += a[ 4 ] * a[ 22 ] * a[ 26 ];
      tmp[ 357 ] += a[ 4 ] * a[ 23 ] * a[ 23 ];
      tmp[ 358 ] += a[ 4 ] * a[ 23 ] * a[ 24 ];
      tmp[ 359 ] += a[ 4 ] * a[ 23 ] * a[ 25 ];
      tmp[ 360 ] += a[ 4 ] * a[ 23 ] * a[ 26 ];
      tmp[ 361 ] += a[ 4 ] * a[ 24 ] * a[ 24 ];
      tmp[ 362 ] += a[ 4 ] * a[ 24 ] * a[ 25 ];
      tmp[ 363 ] += a[ 4 ] * a[ 24 ] * a[ 26 ];
      tmp[ 364 ] += a[ 4 ] * a[ 25 ] * a[ 25 ];
      tmp[ 365 ] += a[ 4 ] * a[ 25 ] * a[ 26 ];
      tmp[ 366 ] += a[ 4 ] * a[ 26 ] * a[ 26 ];
      tmp[ 367 ] += a[ 13 ] * a[ 1 ] * a[ 6 ];
      tmp[ 368 ] += a[ 13 ] * a[ 1 ] * a[ 7 ];
      tmp[ 369 ] += a[ 13 ] * a[ 1 ] * a[ 8 ];
      tmp[ 370 ] += a[ 13 ] * a[ 1 ] * a[ 15 ];
      tmp[ 371 ] += a[ 13 ] * a[ 1 ] * a[ 16 ];
      tmp[ 372 ] += a[ 13 ] * a[ 1 ] * a[ 17 ];
      tmp[ 373 ] += a[ 13 ] * a[ 1 ] * a[ 24 ];
      tmp[ 374 ] += a[ 13 ] * a[ 1 ] * a[ 25 ];
      tmp[ 375 ] += a[ 13 ] * a[ 1 ] * a[ 26 ];
      tmp[ 376 ] += a[ 13 ] * a[ 2 ] * a[ 3 ];
      tmp[ 377 ] += a[ 13 ] * a[ 2 ] * a[ 6 ];
      tmp[ 378 ] += a[ 13 ] * a[ 2 ] * a[ 7 ];
      tmp[ 379 ] += a[ 13 ] * a[ 2 ] * a[ 8 ];
      tmp[ 380 ] += a[ 13 ] * a[ 2 ] * a[ 9 ];
      tmp[ 381 ] += a[ 13 ] * a[ 2 ] * a[ 12 ];
      tmp[ 382 ] += a[ 13 ] * a[ 2 ] * a[ 15 ];
      tmp[ 383 ] += a[ 13 ] * a[ 2 ] * a[ 16 ];
      tmp[ 384 ] += a[ 13 ] * a[ 2 ] * a[ 17 ];
      tmp[ 385 ] += a[ 13 ] * a[ 2 ] * a[ 18 ];
      tmp[ 386 ] += a[ 13 ] * a[ 2 ] * a[ 21 ];
      tmp[ 387 ] += a[ 13 ] * a[ 2 ] * a[ 24 ];
      tmp[ 388 ] += a[ 13 ] * a[ 2 ] * a[ 25 ];
      tmp[ 389 ] += a[ 13 ] * a[ 2 ] * a[ 26 ];
      tmp[ 390 ] += a[ 13 ] * a[ 3 ] * a[ 5 ];
      tmp[ 391 ] += a[ 13 ] * a[ 3 ] * a[ 8 ];
      tmp[ 392 ] += a[ 13 ] * a[ 3 ] * a[ 11 ];
      tmp[ 393 ] += a[ 13 ] * a[ 3 ] * a[ 14 ];
      tmp[ 394 ] += a[ 13 ] * a[ 3 ] * a[ 17 ];
      tmp[ 395 ] += a[ 13 ] * a[ 3 ] * a[ 20 ];
      tmp[ 396 ] += a[ 13 ] * a[ 3 ] * a[ 23 ];
      tmp[ 397 ] += a[ 13 ] * a[ 3 ] * a[ 26 ];
      tmp[ 398 ] += a[ 13 ] * a[ 5 ] * a[ 6 ];
      tmp[ 399 ] += a[ 13 ] * a[ 5 ] * a[ 9 ];
      tmp[ 400 ] += a[ 13 ] * a[ 5 ] * a[ 12 ];
      tmp[ 401 ] += a[ 13 ] * a[ 5 ] * a[ 15 ];
      tmp[ 402 ] += a[ 13 ] * a[ 5 ] * a[ 18 ];
      tmp[ 403 ] += a[ 13 ] * a[ 5 ] * a[ 21 ];
      tmp[ 404 ] += a[ 13 ] * a[ 5 ] * a[ 24 ];
      tmp[ 405 ] += a[ 13 ] * a[ 6 ] * a[ 8 ];
      tmp[ 406 ] += a[ 13 ] * a[ 6 ] * a[ 9 ];
      tmp[ 407 ] += a[ 13 ] * a[ 6 ] * a[ 10 ];
      tmp[ 408 ] += a[ 13 ] * a[ 6 ] * a[ 11 ];
      tmp[ 409 ] += a[ 13 ] * a[ 6 ] * a[ 14 ];
      tmp[ 410 ] += a[ 13 ] * a[ 6 ] * a[ 17 ];
      tmp[ 411 ] += a[ 13 ] * a[ 6 ] * a[ 18 ];
      tmp[ 412 ] += a[ 13 ] * a[ 6 ] * a[ 19 ];
      tmp[ 413 ] += a[ 13 ] * a[ 6 ] * a[ 20 ];
      tmp[ 414 ] += a[ 13 ] * a[ 6 ] * a[ 23 ];
      tmp[ 415 ] += a[ 13 ] * a[ 6 ] * a[ 26 ];
      tmp[ 416 ] += a[ 13 ] * a[ 7 ] * a[ 9 ];
      tmp[ 417 ] += a[ 13 ] * a[ 7 ] * a[ 10 ];
      tmp[ 418 ] += a[ 13 ] * a[ 7 ] * a[ 11 ];
      tmp[ 419 ] += a[ 13 ] * a[ 7 ] * a[ 18 ];
      tmp[ 420 ] += a[ 13 ] * a[ 7 ] * a[ 19 ];
      tmp[ 421 ] += a[ 13 ] * a[ 7 ] * a[ 20 ];
      tmp[ 422 ] += a[ 13 ] * a[ 8 ] * a[ 9 ];
      tmp[ 423 ] += a[ 13 ] * a[ 8 ] * a[ 10 ];
      tmp[ 424 ] += a[ 13 ] * a[ 8 ] * a[ 11 ];
      tmp[ 425 ] += a[ 13 ] * a[ 8 ] * a[ 12 ];
      tmp[ 426 ] += a[ 13 ] * a[ 8 ] * a[ 15 ];
      tmp[ 427 ] += a[ 13 ] * a[ 8 ] * a[ 18 ];
      tmp[ 428 ] += a[ 13 ] * a[ 8 ] * a[ 19 ];
      tmp[ 429 ] += a[ 13 ] * a[ 8 ] * a[ 20 ];
      tmp[ 430 ] += a[ 13 ] * a[ 8 ] * a[ 21 ];
      tmp[ 431 ] += a[ 13 ] * a[ 8 ] * a[ 24 ];
      tmp[ 432 ] += a[ 13 ] * a[ 9 ] * a[ 9 ];
      tmp[ 433 ] += a[ 13 ] * a[ 9 ] * a[ 10 ];
      tmp[ 434 ] += a[ 13 ] * a[ 9 ] * a[ 11 ];
      tmp[ 435 ] += a[ 13 ] * a[ 9 ] * a[ 12 ];
      tmp[ 436 ] += a[ 13 ] * a[ 9 ] * a[ 13 ];
      tmp[ 437 ] += a[ 13 ] * a[ 9 ] * a[ 14 ];
      tmp[ 438 ] += a[ 13 ] * a[ 9 ] * a[ 15 ];
      tmp[ 439 ] += a[ 13 ] * a[ 9 ] * a[ 16 ];
      tmp[ 440 ] += a[ 13 ] * a[ 9 ] * a[ 17 ];
      tmp[ 441 ] += a[ 13 ] * a[ 9 ] * a[ 18 ];
      tmp[ 442 ] += a[ 13 ] * a[ 9 ] * a[ 19 ];
      tmp[ 443 ] += a[ 13 ] * a[ 9 ] * a[ 20 ];
      tmp[ 444 ] += a[ 13 ] * a[ 9 ] * a[ 21 ];
      tmp[ 445 ] += a[ 13 ] * a[ 9 ] * a[ 22 ];
      tmp[ 446 ] += a[ 13 ] * a[ 9 ] * a[ 23 ];
      tmp[ 447 ] += a[ 13 ] * a[ 9 ] * a[ 24 ];
      tmp[ 448 ] += a[ 13 ] * a[ 9 ] * a[ 25 ];
      tmp[ 449 ] += a[ 13 ] * a[ 9 ] * a[ 26 ];
      tmp[ 450 ] += a[ 13 ] * a[ 10 ] * a[ 10 ];
      tmp[ 451 ] += a[ 13 ] * a[ 10 ] * a[ 11 ];
      tmp[ 452 ] += a[ 13 ] * a[ 10 ] * a[ 12 ];
      tmp[ 453 ] += a[ 13 ] * a[ 10 ] * a[ 13 ];
      tmp[ 454 ] += a[ 13 ] * a[ 10 ] * a[ 15 ];
      tmp[ 455 ] += a[ 13 ] * a[ 10 ] * a[ 16 ];
      tmp[ 456 ] += a[ 13 ] * a[ 10 ] * a[ 17 ];
      tmp[ 457 ] += a[ 13 ] * a[ 10 ] * a[ 18 ];
      tmp[ 458 ] += a[ 13 ] * a[ 10 ] * a[ 19 ];
      tmp[ 459 ] += a[ 13 ] * a[ 10 ] * a[ 20 ];
      tmp[ 460 ] += a[ 13 ] * a[ 10 ] * a[ 21 ];
      tmp[ 461 ] += a[ 13 ] * a[ 10 ] * a[ 22 ];
      tmp[ 462 ] += a[ 13 ] * a[ 10 ] * a[ 23 ];
      tmp[ 463 ] += a[ 13 ] * a[ 10 ] * a[ 24 ];
      tmp[ 464 ] += a[ 13 ] * a[ 10 ] * a[ 25 ];
      tmp[ 465 ] += a[ 13 ] * a[ 10 ] * a[ 26 ];
      tmp[ 466 ] += a[ 13 ] * a[ 11 ] * a[ 11 ];
      tmp[ 467 ] += a[ 13 ] * a[ 11 ] * a[ 12 ];
      tmp[ 468 ] += a[ 13 ] * a[ 11 ] * a[ 13 ];
      tmp[ 469 ] += a[ 13 ] * a[ 11 ] * a[ 15 ];
      tmp[ 470 ] += a[ 13 ] * a[ 11 ] * a[ 16 ];
      tmp[ 471 ] += a[ 13 ] * a[ 11 ] * a[ 17 ];
      tmp[ 472 ] += a[ 13 ] * a[ 11 ] * a[ 18 ];
      tmp[ 473 ] += a[ 13 ] * a[ 11 ] * a[ 19 ];
      tmp[ 474 ] += a[ 13 ] * a[ 11 ] * a[ 20 ];
      tmp[ 475 ] += a[ 13 ] * a[ 11 ] * a[ 21 ];
      tmp[ 476 ] += a[ 13 ] * a[ 11 ] * a[ 22 ];
      tmp[ 477 ] += a[ 13 ] * a[ 11 ] * a[ 23 ];
      tmp[ 478 ] += a[ 13 ] * a[ 11 ] * a[ 24 ];
      tmp[ 479 ] += a[ 13 ] * a[ 11 ] * a[ 25 ];
      tmp[ 480 ] += a[ 13 ] * a[ 11 ] * a[ 26 ];
      tmp[ 481 ] += a[ 13 ] * a[ 12 ] * a[ 12 ];
      tmp[ 482 ] += a[ 13 ] * a[ 12 ] * a[ 13 ];
      tmp[ 483 ] += a[ 13 ] * a[ 12 ] * a[ 14 ];
      tmp[ 484 ] += a[ 13 ] * a[ 12 ] * a[ 17 ];
      tmp[ 485 ] += a[ 13 ] * a[ 12 ] * a[ 18 ];
      tmp[ 486 ] += a[ 13 ] * a[ 12 ] * a[ 19 ];
      tmp[ 487 ] += a[ 13 ] * a[ 12 ] * a[ 20 ];
      tmp[ 488 ] += a[ 13 ] * a[ 12 ] * a[ 21 ];
      tmp[ 489 ] += a[ 13 ] * a[ 12 ] * a[ 22 ];
      tmp[ 490 ] += a[ 13 ] * a[ 12 ] * a[ 23 ];
      tmp[ 491 ] += a[ 13 ] * a[ 12 ] * a[ 24 ];
      tmp[ 492 ] += a[ 13 ] * a[ 12 ] * a[ 25 ];
      tmp[ 493 ] += a[ 13 ] * a[ 12 ] * a[ 26 ];
      tmp[ 494 ] += a[ 13 ] * a[ 13 ] * a[ 13 ];
      tmp[ 495 ] += a[ 13 ] * a[ 13 ] * a[ 18 ];
      tmp[ 496 ] += a[ 13 ] * a[ 13 ] * a[ 19 ];
      tmp[ 497 ] += a[ 13 ] * a[ 13 ] * a[ 20 ];
      tmp[ 498 ] += a[ 13 ] * a[ 13 ] * a[ 21 ];
      tmp[ 499 ] += a[ 13 ] * a[ 13 ] * a[ 22 ];
      tmp[ 500 ] += a[ 13 ] * a[ 13 ] * a[ 23 ];
      tmp[ 501 ] += a[ 13 ] * a[ 13 ] * a[ 24 ];
      tmp[ 502 ] += a[ 13 ] * a[ 13 ] * a[ 25 ];
      tmp[ 503 ] += a[ 13 ] * a[ 13 ] * a[ 26 ];
      tmp[ 504 ] += a[ 13 ] * a[ 14 ] * a[ 15 ];
      tmp[ 505 ] += a[ 13 ] * a[ 14 ] * a[ 18 ];
      tmp[ 506 ] += a[ 13 ] * a[ 14 ] * a[ 21 ];
      tmp[ 507 ] += a[ 13 ] * a[ 14 ] * a[ 24 ];
      tmp[ 508 ] += a[ 13 ] * a[ 15 ] * a[ 17 ];
      tmp[ 509 ] += a[ 13 ] * a[ 15 ] * a[ 18 ];
      tmp[ 510 ] += a[ 13 ] * a[ 15 ] * a[ 19 ];
      tmp[ 511 ] += a[ 13 ] * a[ 15 ] * a[ 20 ];
      tmp[ 512 ] += a[ 13 ] * a[ 15 ] * a[ 23 ];
      tmp[ 513 ] += a[ 13 ] * a[ 15 ] * a[ 26 ];
      tmp[ 514 ] += a[ 13 ] * a[ 16 ] * a[ 18 ];
      tmp[ 515 ] += a[ 13 ] * a[ 16 ] * a[ 19 ];
      tmp[ 516 ] += a[ 13 ] * a[ 16 ] * a[ 20 ];
      tmp[ 517 ] += a[ 13 ] * a[ 17 ] * a[ 18 ];
      tmp[ 518 ] += a[ 13 ] * a[ 17 ] * a[ 19 ];
      tmp[ 519 ] += a[ 13 ] * a[ 17 ] * a[ 20 ];
      tmp[ 520 ] += a[ 13 ] * a[ 17 ] * a[ 21 ];
      tmp[ 521 ] += a[ 13 ] * a[ 17 ] * a[ 24 ];
      tmp[ 522 ] += a[ 13 ] * a[ 18 ] * a[ 18 ];
      tmp[ 523 ] += a[ 13 ] * a[ 18 ] * a[ 19 ];
      tmp[ 524 ] += a[ 13 ] * a[ 18 ] * a[ 20 ];
      tmp[ 525 ] += a[ 13 ] * a[ 18 ] * a[ 21 ];
      tmp[ 526 ] += a[ 13 ] * a[ 18 ] * a[ 22 ];
      tmp[ 527 ] += a[ 13 ] * a[ 18 ] * a[ 23 ];
      tmp[ 528 ] += a[ 13 ] * a[ 18 ] * a[ 24 ];
      tmp[ 529 ] += a[ 13 ] * a[ 18 ] * a[ 25 ];
      tmp[ 530 ] += a[ 13 ] * a[ 18 ] * a[ 26 ];
      tmp[ 531 ] += a[ 13 ] * a[ 19 ] * a[ 19 ];
      tmp[ 532 ] += a[ 13 ] * a[ 19 ] * a[ 20 ];
      tmp[ 533 ] += a[ 13 ] * a[ 19 ] * a[ 21 ];
      tmp[ 534 ] += a[ 13 ] * a[ 19 ] * a[ 22 ];
      tmp[ 535 ] += a[ 13 ] * a[ 19 ] * a[ 23 ];
      tmp[ 536 ] += a[ 13 ] * a[ 19 ] * a[ 24 ];
      tmp[ 537 ] += a[ 13 ] * a[ 19 ] * a[ 25 ];
      tmp[ 538 ] += a[ 13 ] * a[ 19 ] * a[ 26 ];
      tmp[ 539 ] += a[ 13 ] * a[ 20 ] * a[ 20 ];
      tmp[ 540 ] += a[ 13 ] * a[ 20 ] * a[ 21 ];
      tmp[ 541 ] += a[ 13 ] * a[ 20 ] * a[ 22 ];
      tmp[ 542 ] += a[ 13 ] * a[ 20 ] * a[ 23 ];
      tmp[ 543 ] += a[ 13 ] * a[ 20 ] * a[ 24 ];
      tmp[ 544 ] += a[ 13 ] * a[ 20 ] * a[ 25 ];
      tmp[ 545 ] += a[ 13 ] * a[ 20 ] * a[ 26 ];
      tmp[ 546 ] += a[ 13 ] * a[ 21 ] * a[ 21 ];
      tmp[ 547 ] += a[ 13 ] * a[ 21 ] * a[ 22 ];
      tmp[ 548 ] += a[ 13 ] * a[ 21 ] * a[ 23 ];
      tmp[ 549 ] += a[ 13 ] * a[ 21 ] * a[ 24 ];
      tmp[ 550 ] += a[ 13 ] * a[ 21 ] * a[ 25 ];
      tmp[ 551 ] += a[ 13 ] * a[ 21 ] * a[ 26 ];
      tmp[ 552 ] += a[ 13 ] * a[ 22 ] * a[ 22 ];
      tmp[ 553 ] += a[ 13 ] * a[ 22 ] * a[ 23 ];
      tmp[ 554 ] += a[ 13 ] * a[ 22 ] * a[ 24 ];
      tmp[ 555 ] += a[ 13 ] * a[ 22 ] * a[ 25 ];
      tmp[ 556 ] += a[ 13 ] * a[ 22 ] * a[ 26 ];
      tmp[ 557 ] += a[ 13 ] * a[ 23 ] * a[ 23 ];
      tmp[ 558 ] += a[ 13 ] * a[ 23 ] * a[ 24 ];
      tmp[ 559 ] += a[ 13 ] * a[ 23 ] * a[ 25 ];
      tmp[ 560 ] += a[ 13 ] * a[ 23 ] * a[ 26 ];
      tmp[ 561 ] += a[ 13 ] * a[ 24 ] * a[ 24 ];
      tmp[ 562 ] += a[ 13 ] * a[ 24 ] * a[ 25 ];
      tmp[ 563 ] += a[ 13 ] * a[ 24 ] * a[ 26 ];
      tmp[ 564 ] += a[ 13 ] * a[ 25 ] * a[ 25 ];
      tmp[ 565 ] += a[ 13 ] * a[ 25 ] * a[ 26 ];
      tmp[ 566 ] += a[ 13 ] * a[ 26 ] * a[ 26 ];
      tmp[ 567 ] += a[ 13 ] * a[ 0 ] * a[ 2 ];
      tmp[ 568 ] += a[ 13 ] * a[ 0 ] * a[ 5 ];
      tmp[ 569 ] += a[ 13 ] * a[ 0 ] * a[ 6 ];
      tmp[ 570 ] += a[ 13 ] * a[ 0 ] * a[ 7 ];
      tmp[ 571 ] += a[ 13 ] * a[ 0 ] * a[ 8 ];
      tmp[ 572 ] += a[ 13 ] * a[ 0 ] * a[ 11 ];
      tmp[ 573 ] += a[ 13 ] * a[ 0 ] * a[ 14 ];
      tmp[ 574 ] += a[ 13 ] * a[ 0 ] * a[ 15 ];
      tmp[ 575 ] += a[ 13 ] * a[ 0 ] * a[ 16 ];
      tmp[ 576 ] += a[ 13 ] * a[ 0 ] * a[ 17 ];
      tmp[ 577 ] += a[ 13 ] * a[ 0 ] * a[ 20 ];
      tmp[ 578 ] += a[ 13 ] * a[ 0 ] * a[ 23 ];
      tmp[ 579 ] += a[ 13 ] * a[ 0 ] * a[ 24 ];
      tmp[ 580 ] += a[ 13 ] * a[ 0 ] * a[ 25 ];
      tmp[ 581 ] += a[ 13 ] * a[ 0 ] * a[ 26 ];
      tmp[ 582 ] += a[ 22 ] * a[ 2 ] * a[ 6 ];
      tmp[ 583 ] += a[ 22 ] * a[ 2 ] * a[ 7 ];
      tmp[ 584 ] += a[ 22 ] * a[ 2 ] * a[ 8 ];
      tmp[ 585 ] += a[ 22 ] * a[ 2 ] * a[ 9 ];
      tmp[ 586 ] += a[ 22 ] * a[ 2 ] * a[ 12 ];
      tmp[ 587 ] += a[ 22 ] * a[ 2 ] * a[ 15 ];
      tmp[ 588 ] += a[ 22 ] * a[ 2 ] * a[ 16 ];
      tmp[ 589 ] += a[ 22 ] * a[ 2 ] * a[ 17 ];
      tmp[ 590 ] += a[ 22 ] * a[ 2 ] * a[ 18 ];
      tmp[ 591 ] += a[ 22 ] * a[ 2 ] * a[ 21 ];
      tmp[ 592 ] += a[ 22 ] * a[ 2 ] * a[ 24 ];
      tmp[ 593 ] += a[ 22 ] * a[ 2 ] * a[ 25 ];
      tmp[ 594 ] += a[ 22 ] * a[ 2 ] * a[ 26 ];
      tmp[ 595 ] += a[ 22 ] * a[ 3 ] * a[ 5 ];
      tmp[ 596 ] += a[ 22 ] * a[ 3 ] * a[ 8 ];
      tmp[ 597 ] += a[ 22 ] * a[ 3 ] * a[ 11 ];
      tmp[ 598 ] += a[ 22 ] * a[ 3 ] * a[ 14 ];
      tmp[ 599 ] += a[ 22 ] * a[ 3 ] * a[ 17 ];
      tmp[ 600 ] += a[ 22 ] * a[ 3 ] * a[ 20 ];
      tmp[ 601 ] += a[ 22 ] * a[ 3 ] * a[ 23 ];
      tmp[ 602 ] += a[ 22 ] * a[ 3 ] * a[ 26 ];
      tmp[ 603 ] += a[ 22 ] * a[ 5 ] * a[ 6 ];
      tmp[ 604 ] += a[ 22 ] * a[ 5 ] * a[ 9 ];
      tmp[ 605 ] += a[ 22 ] * a[ 5 ] * a[ 12 ];
      tmp[ 606 ] += a[ 22 ] * a[ 5 ] * a[ 15 ];
      tmp[ 607 ] += a[ 22 ] * a[ 5 ] * a[ 18 ];
      tmp[ 608 ] += a[ 22 ] * a[ 5 ] * a[ 21 ];
      tmp[ 609 ] += a[ 22 ] * a[ 5 ] * a[ 24 ];
      tmp[ 610 ] += a[ 22 ] * a[ 6 ] * a[ 8 ];
      tmp[ 611 ] += a[ 22 ] * a[ 6 ] * a[ 9 ];
      tmp[ 612 ] += a[ 22 ] * a[ 6 ] * a[ 10 ];
      tmp[ 613 ] += a[ 22 ] * a[ 6 ] * a[ 11 ];
      tmp[ 614 ] += a[ 22 ] * a[ 6 ] * a[ 14 ];
      tmp[ 615 ] += a[ 22 ] * a[ 6 ] * a[ 17 ];
      tmp[ 616 ] += a[ 22 ] * a[ 6 ] * a[ 18 ];
      tmp[ 617 ] += a[ 22 ] * a[ 6 ] * a[ 19 ];
      tmp[ 618 ] += a[ 22 ] * a[ 6 ] * a[ 20 ];
      tmp[ 619 ] += a[ 22 ] * a[ 6 ] * a[ 23 ];
      tmp[ 620 ] += a[ 22 ] * a[ 6 ] * a[ 26 ];
      tmp[ 621 ] += a[ 22 ] * a[ 7 ] * a[ 9 ];
      tmp[ 622 ] += a[ 22 ] * a[ 7 ] * a[ 10 ];
      tmp[ 623 ] += a[ 22 ] * a[ 7 ] * a[ 11 ];
      tmp[ 624 ] += a[ 22 ] * a[ 7 ] * a[ 18 ];
      tmp[ 625 ] += a[ 22 ] * a[ 7 ] * a[ 19 ];
      tmp[ 626 ] += a[ 22 ] * a[ 7 ] * a[ 20 ];
      tmp[ 627 ] += a[ 22 ] * a[ 8 ] * a[ 9 ];
      tmp[ 628 ] += a[ 22 ] * a[ 8 ] * a[ 10 ];
      tmp[ 629 ] += a[ 22 ] * a[ 8 ] * a[ 11 ];
      tmp[ 630 ] += a[ 22 ] * a[ 8 ] * a[ 12 ];
      tmp[ 631 ] += a[ 22 ] * a[ 8 ] * a[ 15 ];
      tmp[ 632 ] += a[ 22 ] * a[ 8 ] * a[ 18 ];
      tmp[ 633 ] += a[ 22 ] * a[ 8 ] * a[ 19 ];
      tmp[ 634 ] += a[ 22 ] * a[ 8 ] * a[ 20 ];
      tmp[ 635 ] += a[ 22 ] * a[ 8 ] * a[ 21 ];
      tmp[ 636 ] += a[ 22 ] * a[ 8 ] * a[ 24 ];
      tmp[ 637 ] += a[ 22 ] * a[ 9 ] * a[ 11 ];
      tmp[ 638 ] += a[ 22 ] * a[ 9 ] * a[ 14 ];
      tmp[ 639 ] += a[ 22 ] * a[ 9 ] * a[ 15 ];
      tmp[ 640 ] += a[ 22 ] * a[ 9 ] * a[ 16 ];
      tmp[ 641 ] += a[ 22 ] * a[ 9 ] * a[ 17 ];
      tmp[ 642 ] += a[ 22 ] * a[ 9 ] * a[ 20 ];
      tmp[ 643 ] += a[ 22 ] * a[ 9 ] * a[ 23 ];
      tmp[ 644 ] += a[ 22 ] * a[ 9 ] * a[ 24 ];
      tmp[ 645 ] += a[ 22 ] * a[ 9 ] * a[ 25 ];
      tmp[ 646 ] += a[ 22 ] * a[ 9 ] * a[ 26 ];
      tmp[ 647 ] += a[ 22 ] * a[ 10 ] * a[ 15 ];
      tmp[ 648 ] += a[ 22 ] * a[ 10 ] * a[ 16 ];
      tmp[ 649 ] += a[ 22 ] * a[ 10 ] * a[ 17 ];
      tmp[ 650 ] += a[ 22 ] * a[ 10 ] * a[ 24 ];
      tmp[ 651 ] += a[ 22 ] * a[ 10 ] * a[ 25 ];
      tmp[ 652 ] += a[ 22 ] * a[ 10 ] * a[ 26 ];
      tmp[ 653 ] += a[ 22 ] * a[ 11 ] * a[ 12 ];
      tmp[ 654 ] += a[ 22 ] * a[ 11 ] * a[ 15 ];
      tmp[ 655 ] += a[ 22 ] * a[ 11 ] * a[ 16 ];
      tmp[ 656 ] += a[ 22 ] * a[ 11 ] * a[ 17 ];
      tmp[ 657 ] += a[ 22 ] * a[ 11 ] * a[ 18 ];
      tmp[ 658 ] += a[ 22 ] * a[ 11 ] * a[ 21 ];
      tmp[ 659 ] += a[ 22 ] * a[ 11 ] * a[ 24 ];
      tmp[ 660 ] += a[ 22 ] * a[ 11 ] * a[ 25 ];
      tmp[ 661 ] += a[ 22 ] * a[ 11 ] * a[ 26 ];
      tmp[ 662 ] += a[ 22 ] * a[ 12 ] * a[ 14 ];
      tmp[ 663 ] += a[ 22 ] * a[ 12 ] * a[ 17 ];
      tmp[ 664 ] += a[ 22 ] * a[ 12 ] * a[ 20 ];
      tmp[ 665 ] += a[ 22 ] * a[ 12 ] * a[ 23 ];
      tmp[ 666 ] += a[ 22 ] * a[ 12 ] * a[ 26 ];
      tmp[ 667 ] += a[ 22 ] * a[ 14 ] * a[ 15 ];
      tmp[ 668 ] += a[ 22 ] * a[ 14 ] * a[ 18 ];
      tmp[ 669 ] += a[ 22 ] * a[ 14 ] * a[ 21 ];
      tmp[ 670 ] += a[ 22 ] * a[ 14 ] * a[ 24 ];
      tmp[ 671 ] += a[ 22 ] * a[ 15 ] * a[ 17 ];
      tmp[ 672 ] += a[ 22 ] * a[ 15 ] * a[ 18 ];
      tmp[ 673 ] += a[ 22 ] * a[ 15 ] * a[ 19 ];
      tmp[ 674 ] += a[ 22 ] * a[ 15 ] * a[ 20 ];
      tmp[ 675 ] += a[ 22 ] * a[ 15 ] * a[ 23 ];
      tmp[ 676 ] += a[ 22 ] * a[ 15 ] * a[ 26 ];
      tmp[ 677 ] += a[ 22 ] * a[ 16 ] * a[ 18 ];
      tmp[ 678 ] += a[ 22 ] * a[ 16 ] * a[ 19 ];
      tmp[ 679 ] += a[ 22 ] * a[ 16 ] * a[ 20 ];
      tmp[ 680 ] += a[ 22 ] * a[ 17 ] * a[ 18 ];
      tmp[ 681 ] += a[ 22 ] * a[ 17 ] * a[ 19 ];
      tmp[ 682 ] += a[ 22 ] * a[ 17 ] * a[ 20 ];
      tmp[ 683 ] += a[ 22 ] * a[ 17 ] * a[ 21 ];
      tmp[ 684 ] += a[ 22 ] * a[ 17 ] * a[ 24 ];
      tmp[ 685 ] += a[ 22 ] * a[ 18 ] * a[ 18 ];
      tmp[ 686 ] += a[ 22 ] * a[ 18 ] * a[ 19 ];
      tmp[ 687 ] += a[ 22 ] * a[ 18 ] * a[ 20 ];
      tmp[ 688 ] += a[ 22 ] * a[ 18 ] * a[ 21 ];
      tmp[ 689 ] += a[ 22 ] * a[ 18 ] * a[ 22 ];
      tmp[ 690 ] += a[ 22 ] * a[ 18 ] * a[ 23 ];
      tmp[ 691 ] += a[ 22 ] * a[ 18 ] * a[ 24 ];
      tmp[ 692 ] += a[ 22 ] * a[ 18 ] * a[ 25 ];
      tmp[ 693 ] += a[ 22 ] * a[ 18 ] * a[ 26 ];
      tmp[ 694 ] += a[ 22 ] * a[ 19 ] * a[ 19 ];
      tmp[ 695 ] += a[ 22 ] * a[ 19 ] * a[ 20 ];
      tmp[ 696 ] += a[ 22 ] * a[ 19 ] * a[ 21 ];
      tmp[ 697 ] += a[ 22 ] * a[ 19 ] * a[ 22 ];
      tmp[ 698 ] += a[ 22 ] * a[ 19 ] * a[ 24 ];
      tmp[ 699 ] += a[ 22 ] * a[ 19 ] * a[ 25 ];
      tmp[ 700 ] += a[ 22 ] * a[ 19 ] * a[ 26 ];
      tmp[ 701 ] += a[ 22 ] * a[ 20 ] * a[ 20 ];
      tmp[ 702 ] += a[ 22 ] * a[ 20 ] * a[ 21 ];
      tmp[ 703 ] += a[ 22 ] * a[ 20 ] * a[ 22 ];
      tmp[ 704 ] += a[ 22 ] * a[ 20 ] * a[ 24 ];
      tmp[ 705 ] += a[ 22 ] * a[ 20 ] * a[ 25 ];
      tmp[ 706 ] += a[ 22 ] * a[ 20 ] * a[ 26 ];
      tmp[ 707 ] += a[ 22 ] * a[ 21 ] * a[ 21 ];
      tmp[ 708 ] += a[ 22 ] * a[ 21 ] * a[ 22 ];
      tmp[ 709 ] += a[ 22 ] * a[ 21 ] * a[ 23 ];
      tmp[ 710 ] += a[ 22 ] * a[ 21 ] * a[ 26 ];
      tmp[ 711 ] += a[ 22 ] * a[ 22 ] * a[ 22 ];
      tmp[ 712 ] += a[ 22 ] * a[ 23 ] * a[ 24 ];
      tmp[ 713 ] += a[ 22 ] * a[ 24 ] * a[ 26 ];
    }  
  }
  
  /*** normalize ***/  
#ifdef ENABLE_NORMALIZATION
  double normalVal = (double)( 255 * ( right - left ) * ( down - up ) );
  result[ 0 ] = tmp[ 0 ] / normalVal;
  result[ 1 ] = tmp[ 1 ] / normalVal;
  result[ 2 ] = tmp[ 2 ] / normalVal;
  normalVal *= 255.0;
  for(int i=3; i<DIM_OF_COLOR_HLAC1_3; ++i){
    result[ i ] = tmp[ i ] / normalVal;
  }
  normalVal *= 255.0;
  for(int i=DIM_OF_COLOR_HLAC1_3; i<DIM_OF_COLOR_HLAC2_3; ++i){
    result[ i ] = tmp[ i ] / normalVal;
  }
#else /* !ENABLE_NORMALIZATION */
  result[ 0 ] = tmp[ 0 ] / 255.0;
  result[ 1 ] = tmp[ 1 ] / 255.0;
  result[ 2 ] = tmp[ 2 ] / 255.0;
  for(int i=3; i<DIM_OF_COLOR_HLAC1_3; ++i){
    result[ i ] = tmp[ i ] / 65025.0;
  }
  for(int i=DIM_OF_COLOR_HLAC1_3; i<DIM_OF_COLOR_HLAC2_3; ++i){
    result[ i ] = tmp[ i ] / 16581375.0;
  }
#endif /* ENABLE_NORMALIZATION */
}

/*** 高々1次の color HLAC, square size = 5 ***********************************/
 
void HLAC::_extractColor1_5( std::vector<float> &result, const cv::Mat &img, 
				const int &rx, const int &ry, 
				const int &left, const int &right,
				const int &up, const int &down )
{
  double a[ 75 ];
  const int width_step = img.step;
  
  /*** initialize ***/
  double tmp[ DIM_OF_COLOR_HLAC1_5 ];
  for(int i=0; i<DIM_OF_COLOR_HLAC1_5; ++i) tmp[ i ] = 0.0;
  
  /*** feature extraction ***/
  for(int j=up; j<down; ++j){
    for(int i=left; i<right; ++i){
      a[  0 ] = (double)img.data[ ( i - 2*rx ) + ( j - 2*ry ) * width_step ];
      a[  1 ] = (double)img.data[ ( i -   rx ) + ( j - 2*ry ) * width_step ];
      a[  2 ] = (double)img.data[   i          + ( j - 2*ry ) * width_step ];
      a[  3 ] = (double)img.data[ ( i +   rx ) + ( j - 2*ry ) * width_step ];
      a[  4 ] = (double)img.data[ ( i + 2*rx ) + ( j - 2*ry ) * width_step ];
      a[  5 ] = (double)img.data[ ( i - 2*rx ) + ( j -   ry ) * width_step ];
      a[  6 ] = (double)img.data[ ( i -   rx ) + ( j -   ry ) * width_step ];
      a[  7 ] = (double)img.data[   i          + ( j -   ry ) * width_step ];
      a[  8 ] = (double)img.data[ ( i +   rx ) + ( j -   ry ) * width_step ];
      a[  9 ] = (double)img.data[ ( i + 2*rx ) + ( j -   ry ) * width_step ];
      a[ 10 ] = (double)img.data[ ( i - 2*rx ) +   j          * width_step ];
      a[ 11 ] = (double)img.data[ ( i -   rx ) +   j          * width_step ];
      a[ 12 ] = (double)img.data[   i          +   j          * width_step ];
//       a[ 13 ] = (double)img.data[ ( i +   rx ) +   j          * width_step ];
//       a[ 14 ] = (double)img.data[ ( i + 2*rx ) +   j          * width_step ];
//       a[ 15 ] = (double)img.data[ ( i - 2*rx ) + ( j +   ry ) * width_step ];
//       a[ 16 ] = (double)img.data[ ( i -   rx ) + ( j +   ry ) * width_step ];
//       a[ 17 ] = (double)img.data[   i          + ( j +   ry ) * width_step ];
//       a[ 18 ] = (double)img.data[ ( i +   rx ) + ( j +   ry ) * width_step ];
//       a[ 19 ] = (double)img.data[ ( i + 2*rx ) + ( j +   ry ) * width_step ];
//       a[ 20 ] = (double)img.data[ ( i - 2*rx ) + ( j + 2*ry ) * width_step ];
//       a[ 21 ] = (double)img.data[ ( i -   rx ) + ( j + 2*ry ) * width_step ];
//       a[ 22 ] = (double)img.data[   i          + ( j + 2*ry ) * width_step ];
//       a[ 23 ] = (double)img.data[ ( i +   rx ) + ( j + 2*ry ) * width_step ];
//       a[ 24 ] = (double)img.data[ ( i + 2*rx ) + ( j + 2*ry ) * width_step ];
      a[ 25 ] = (double)img.data[ ( i - 2*rx ) + ( j - 2*ry ) * width_step +1 ];
      a[ 26 ] = (double)img.data[ ( i -   rx ) + ( j - 2*ry ) * width_step +1 ];
      a[ 27 ] = (double)img.data[   i          + ( j - 2*ry ) * width_step +1 ];
      a[ 28 ] = (double)img.data[ ( i +   rx ) + ( j - 2*ry ) * width_step +1 ];
      a[ 29 ] = (double)img.data[ ( i + 2*rx ) + ( j - 2*ry ) * width_step +1 ];
      a[ 30 ] = (double)img.data[ ( i - 2*rx ) + ( j -   ry ) * width_step +1 ];
      a[ 31 ] = (double)img.data[ ( i -   rx ) + ( j -   ry ) * width_step +1 ];
      a[ 32 ] = (double)img.data[   i          + ( j -   ry ) * width_step +1 ];
      a[ 33 ] = (double)img.data[ ( i +   rx ) + ( j -   ry ) * width_step +1 ];
      a[ 34 ] = (double)img.data[ ( i + 2*rx ) + ( j -   ry ) * width_step +1 ];
      a[ 35 ] = (double)img.data[ ( i - 2*rx ) +   j          * width_step +1 ];
      a[ 36 ] = (double)img.data[ ( i -   rx ) +   j          * width_step +1 ];
      a[ 37 ] = (double)img.data[   i          +   j          * width_step +1 ];
      a[ 38 ] = (double)img.data[ ( i +   rx ) +   j          * width_step +1 ];
      a[ 39 ] = (double)img.data[ ( i + 2*rx ) +   j          * width_step +1 ];
      a[ 40 ] = (double)img.data[ ( i - 2*rx ) + ( j +   ry ) * width_step +1 ];
      a[ 41 ] = (double)img.data[ ( i -   rx ) + ( j +   ry ) * width_step +1 ];
      a[ 42 ] = (double)img.data[   i          + ( j +   ry ) * width_step +1 ];
      a[ 43 ] = (double)img.data[ ( i +   rx ) + ( j +   ry ) * width_step +1 ];
      a[ 44 ] = (double)img.data[ ( i + 2*rx ) + ( j +   ry ) * width_step +1 ];
      a[ 45 ] = (double)img.data[ ( i - 2*rx ) + ( j + 2*ry ) * width_step +1 ];
      a[ 46 ] = (double)img.data[ ( i -   rx ) + ( j + 2*ry ) * width_step +1 ];
      a[ 47 ] = (double)img.data[   i          + ( j + 2*ry ) * width_step +1 ];
      a[ 48 ] = (double)img.data[ ( i +   rx ) + ( j + 2*ry ) * width_step +1 ];
      a[ 49 ] = (double)img.data[ ( i + 2*rx ) + ( j + 2*ry ) * width_step +1 ];
      a[ 50 ] = (double)img.data[ ( i - 2*rx ) + ( j - 2*ry ) * width_step +2 ];
      a[ 51 ] = (double)img.data[ ( i -   rx ) + ( j - 2*ry ) * width_step +2 ];
      a[ 52 ] = (double)img.data[   i          + ( j - 2*ry ) * width_step +2 ];
      a[ 53 ] = (double)img.data[ ( i +   rx ) + ( j - 2*ry ) * width_step +2 ];
      a[ 54 ] = (double)img.data[ ( i + 2*rx ) + ( j - 2*ry ) * width_step +2 ];
      a[ 55 ] = (double)img.data[ ( i - 2*rx ) + ( j -   ry ) * width_step +2 ];
      a[ 56 ] = (double)img.data[ ( i -   rx ) + ( j -   ry ) * width_step +2 ];
      a[ 57 ] = (double)img.data[   i          + ( j -   ry ) * width_step +2 ];
      a[ 58 ] = (double)img.data[ ( i +   rx ) + ( j -   ry ) * width_step +2 ];
      a[ 59 ] = (double)img.data[ ( i + 2*rx ) + ( j -   ry ) * width_step +2 ];
      a[ 60 ] = (double)img.data[ ( i - 2*rx ) +   j          * width_step +2 ];
      a[ 61 ] = (double)img.data[ ( i -   rx ) +   j          * width_step +2 ];
      a[ 62 ] = (double)img.data[   i          +   j          * width_step +2 ];
      a[ 63 ] = (double)img.data[ ( i +   rx ) +   j          * width_step +2 ];
      a[ 64 ] = (double)img.data[ ( i + 2*rx ) +   j          * width_step +2 ];
      a[ 65 ] = (double)img.data[ ( i - 2*rx ) + ( j +   ry ) * width_step +2 ];
      a[ 66 ] = (double)img.data[ ( i -   rx ) + ( j +   ry ) * width_step +2 ];
      a[ 67 ] = (double)img.data[   i          + ( j +   ry ) * width_step +2 ];
      a[ 68 ] = (double)img.data[ ( i +   rx ) + ( j +   ry ) * width_step +2 ];
      a[ 69 ] = (double)img.data[ ( i + 2*rx ) + ( j +   ry ) * width_step +2 ];
      a[ 70 ] = (double)img.data[ ( i - 2*rx ) + ( j + 2*ry ) * width_step +2 ];
      a[ 71 ] = (double)img.data[ ( i -   rx ) + ( j + 2*ry ) * width_step +2 ];
      a[ 72 ] = (double)img.data[   i          + ( j + 2*ry ) * width_step +2 ];
      a[ 73 ] = (double)img.data[ ( i +   rx ) + ( j + 2*ry ) * width_step +2 ];
      a[ 74 ] = (double)img.data[ ( i + 2*rx ) + ( j + 2*ry ) * width_step +2 ];
      tmp[ 0 ] += a[ 12 ];
      tmp[ 1 ] += a[ 37 ];
      tmp[ 2 ] += a[ 62 ];
      tmp[ 3 ] += a[ 12 ] * a[ 0 ];
      tmp[ 4 ] += a[ 12 ] * a[ 1 ];
      tmp[ 5 ] += a[ 12 ] * a[ 2 ];
      tmp[ 6 ] += a[ 12 ] * a[ 3 ];
      tmp[ 7 ] += a[ 12 ] * a[ 4 ];
      tmp[ 8 ] += a[ 12 ] * a[ 5 ];
      tmp[ 9 ] += a[ 12 ] * a[ 6 ];
      tmp[ 10 ] += a[ 12 ] * a[ 7 ];
      tmp[ 11 ] += a[ 12 ] * a[ 8 ];
      tmp[ 12 ] += a[ 12 ] * a[ 9 ];
      tmp[ 13 ] += a[ 12 ] * a[ 10 ];
      tmp[ 14 ] += a[ 12 ] * a[ 11 ];
      tmp[ 15 ] += a[ 12 ] * a[ 12 ];
      tmp[ 16 ] += a[ 12 ] * a[ 25 ];
      tmp[ 17 ] += a[ 12 ] * a[ 26 ];
      tmp[ 18 ] += a[ 12 ] * a[ 27 ];
      tmp[ 19 ] += a[ 12 ] * a[ 28 ];
      tmp[ 20 ] += a[ 12 ] * a[ 29 ];
      tmp[ 21 ] += a[ 12 ] * a[ 30 ];
      tmp[ 22 ] += a[ 12 ] * a[ 31 ];
      tmp[ 23 ] += a[ 12 ] * a[ 32 ];
      tmp[ 24 ] += a[ 12 ] * a[ 33 ];
      tmp[ 25 ] += a[ 12 ] * a[ 34 ];
      tmp[ 26 ] += a[ 12 ] * a[ 35 ];
      tmp[ 27 ] += a[ 12 ] * a[ 36 ];
      tmp[ 28 ] += a[ 12 ] * a[ 37 ];
      tmp[ 29 ] += a[ 12 ] * a[ 38 ];
      tmp[ 30 ] += a[ 12 ] * a[ 39 ];
      tmp[ 31 ] += a[ 12 ] * a[ 40 ];
      tmp[ 32 ] += a[ 12 ] * a[ 41 ];
      tmp[ 33 ] += a[ 12 ] * a[ 42 ];
      tmp[ 34 ] += a[ 12 ] * a[ 43 ];
      tmp[ 35 ] += a[ 12 ] * a[ 44 ];
      tmp[ 36 ] += a[ 12 ] * a[ 45 ];
      tmp[ 37 ] += a[ 12 ] * a[ 46 ];
      tmp[ 38 ] += a[ 12 ] * a[ 47 ];
      tmp[ 39 ] += a[ 12 ] * a[ 48 ];
      tmp[ 40 ] += a[ 12 ] * a[ 49 ];
      tmp[ 41 ] += a[ 12 ] * a[ 50 ];
      tmp[ 42 ] += a[ 12 ] * a[ 51 ];
      tmp[ 43 ] += a[ 12 ] * a[ 52 ];
      tmp[ 44 ] += a[ 12 ] * a[ 53 ];
      tmp[ 45 ] += a[ 12 ] * a[ 54 ];
      tmp[ 46 ] += a[ 12 ] * a[ 55 ];
      tmp[ 47 ] += a[ 12 ] * a[ 56 ];
      tmp[ 48 ] += a[ 12 ] * a[ 57 ];
      tmp[ 49 ] += a[ 12 ] * a[ 58 ];
      tmp[ 50 ] += a[ 12 ] * a[ 59 ];
      tmp[ 51 ] += a[ 12 ] * a[ 60 ];
      tmp[ 52 ] += a[ 12 ] * a[ 61 ];
      tmp[ 53 ] += a[ 12 ] * a[ 62 ];
      tmp[ 54 ] += a[ 12 ] * a[ 63 ];
      tmp[ 55 ] += a[ 12 ] * a[ 64 ];
      tmp[ 56 ] += a[ 12 ] * a[ 65 ];
      tmp[ 57 ] += a[ 12 ] * a[ 66 ];
      tmp[ 58 ] += a[ 12 ] * a[ 67 ];
      tmp[ 59 ] += a[ 12 ] * a[ 68 ];
      tmp[ 60 ] += a[ 12 ] * a[ 69 ];
      tmp[ 61 ] += a[ 12 ] * a[ 70 ];
      tmp[ 62 ] += a[ 12 ] * a[ 71 ];
      tmp[ 63 ] += a[ 12 ] * a[ 72 ];
      tmp[ 64 ] += a[ 12 ] * a[ 73 ];
      tmp[ 65 ] += a[ 12 ] * a[ 74 ];
      tmp[ 66 ] += a[ 37 ] * a[ 25 ];
      tmp[ 67 ] += a[ 37 ] * a[ 26 ];
      tmp[ 68 ] += a[ 37 ] * a[ 27 ];
      tmp[ 69 ] += a[ 37 ] * a[ 28 ];
      tmp[ 70 ] += a[ 37 ] * a[ 29 ];
      tmp[ 71 ] += a[ 37 ] * a[ 30 ];
      tmp[ 72 ] += a[ 37 ] * a[ 31 ];
      tmp[ 73 ] += a[ 37 ] * a[ 32 ];
      tmp[ 74 ] += a[ 37 ] * a[ 33 ];
      tmp[ 75 ] += a[ 37 ] * a[ 34 ];
      tmp[ 76 ] += a[ 37 ] * a[ 35 ];
      tmp[ 77 ] += a[ 37 ] * a[ 36 ];
      tmp[ 78 ] += a[ 37 ] * a[ 37 ];
      tmp[ 79 ] += a[ 37 ] * a[ 50 ];
      tmp[ 80 ] += a[ 37 ] * a[ 51 ];
      tmp[ 81 ] += a[ 37 ] * a[ 52 ];
      tmp[ 82 ] += a[ 37 ] * a[ 53 ];
      tmp[ 83 ] += a[ 37 ] * a[ 54 ];
      tmp[ 84 ] += a[ 37 ] * a[ 55 ];
      tmp[ 85 ] += a[ 37 ] * a[ 56 ];
      tmp[ 86 ] += a[ 37 ] * a[ 57 ];
      tmp[ 87 ] += a[ 37 ] * a[ 58 ];
      tmp[ 88 ] += a[ 37 ] * a[ 59 ];
      tmp[ 89 ] += a[ 37 ] * a[ 60 ];
      tmp[ 90 ] += a[ 37 ] * a[ 61 ];
      tmp[ 91 ] += a[ 37 ] * a[ 62 ];
      tmp[ 92 ] += a[ 37 ] * a[ 63 ];
      tmp[ 93 ] += a[ 37 ] * a[ 64 ];
      tmp[ 94 ] += a[ 37 ] * a[ 65 ];
      tmp[ 95 ] += a[ 37 ] * a[ 66 ];
      tmp[ 96 ] += a[ 37 ] * a[ 67 ];
      tmp[ 97 ] += a[ 37 ] * a[ 68 ];
      tmp[ 98 ] += a[ 37 ] * a[ 69 ];
      tmp[ 99 ] += a[ 37 ] * a[ 70 ];
      tmp[ 100 ] += a[ 37 ] * a[ 71 ];
      tmp[ 101 ] += a[ 37 ] * a[ 72 ];
      tmp[ 102 ] += a[ 37 ] * a[ 73 ];
      tmp[ 103 ] += a[ 37 ] * a[ 74 ];
      tmp[ 104 ] += a[ 62 ] * a[ 50 ];
      tmp[ 105 ] += a[ 62 ] * a[ 51 ];
      tmp[ 106 ] += a[ 62 ] * a[ 52 ];
      tmp[ 107 ] += a[ 62 ] * a[ 53 ];
      tmp[ 108 ] += a[ 62 ] * a[ 54 ];
      tmp[ 109 ] += a[ 62 ] * a[ 55 ];
      tmp[ 110 ] += a[ 62 ] * a[ 56 ];
      tmp[ 111 ] += a[ 62 ] * a[ 57 ];
      tmp[ 112 ] += a[ 62 ] * a[ 58 ];
      tmp[ 113 ] += a[ 62 ] * a[ 59 ];
      tmp[ 114 ] += a[ 62 ] * a[ 60 ];
      tmp[ 115 ] += a[ 62 ] * a[ 61 ];
      tmp[ 116 ] += a[ 62 ] * a[ 62 ];
    }
  }
  
  /*** normalize ***/  
#ifdef ENABLE_NORMALIZATION
  double normalVal = (double)( 255 * ( right - left ) * ( down - up ) );
  result[ 0 ] = tmp[ 0 ] / normalVal;
  result[ 1 ] = tmp[ 1 ] / normalVal;
  result[ 2 ] = tmp[ 2 ] / normalVal;
  normalVal *= 255.0;
  for(int i=3; i<DIM_OF_COLOR_HLAC1_5; ++i) result[ i ] = tmp[ i ] / normalVal;
#else /* !ENABLE_NORMALIZATION */
  result[ 0 ] = tmp[ 0 ] / 255.0;
  result[ 1 ] = tmp[ 1 ] / 255.0;
  result[ 2 ] = tmp[ 2 ] / 255.0;
  for(int i=3; i<DIM_OF_COLOR_HLAC1_5; ++i) result[ i ] = tmp[ i ] / 65025.0;
#endif /* ENABLE_NORMALIZATION */
}


/*----------------------------*
 *  簡単な計算やエラー処理等  *
 *----------------------------*/
/*** 特徴ベクトルの次元を決める **********************************************/
 
void HLAC::checkDimensionBin( std::vector<float> &result, 
				 const int &order, const int &size )
{
  int dim;
  
  /*** check order ***/
  if( size == 3 ){
    switch( order ){
    case 1:
      dim = DIM_OF_BIN_HLAC1_3;
      break;
    case 2:
      dim = DIM_OF_BIN_HLAC2_3;
      break;
    case 3:
      dim = DIM_OF_BIN_HLAC3_3;
      break;
    default:
      std::cerr << "HLAC:checkDimensionBin(): not proper order of HLAC" 
		<< std::endl
		<< "-----> order = " << order << ", size = " << size 
		<< std::endl;
      exit(1);
    }
  }else if( size == 5 ){
    switch( order ){
    case 1:
      dim = DIM_OF_BIN_HLAC1_5;
      break;
    case 2:
      dim = DIM_OF_BIN_HLAC2_5;
      break;
#ifdef ENABLE_BIN_HLAC3_5
    case 3:
      dim = DIM_OF_BIN_HLAC3_5;
      break;
#endif /* ENABLE_BIN_HLAC3_5 */
    default:
      std::cerr << "HLAC:checkDimensionBin(): not proper order of HLAC" 
		<< std::endl
		<< "-----> order = " << order << ", size = " << size 
		<< std::endl;
      exit(1);
    }
  }else{
    std::cerr << "HLAC:checkDimensionBin(): square size is not proper" 
	      << std::endl
	      << "-----> order = " << order << ", size = " << size 
	      << std::endl;
    exit(1);
  }
  
  /*** check dimension ***/
  if( (int)result.size() != dim ) result.resize( dim );
}
 
void HLAC::checkDimensionGray( std::vector<float> &result, 
				  const int &order, const int &size )
{
  int dim;
  
  /*** check order ***/
  if( size == 3 ){
    switch( order ){
    case 1:
      dim = DIM_OF_GRAY_HLAC1_3;
      break;
    case 2:
      dim = DIM_OF_GRAY_HLAC2_3;
      break;
    case 3:
      dim = DIM_OF_GRAY_HLAC3_3;
      break;
    default:
      std::cerr << "HLAC:checkDimensionGray(): not proper order of HLAC" 
		<< std::endl
		<< "-----> order = " << order << ", size = " << size 
		<< std::endl;
      exit(1);
    }
  }else if( size == 5 ){
    switch( order ){
    case 1:
      dim = DIM_OF_GRAY_HLAC1_5;
      break;
    case 2:
      dim = DIM_OF_GRAY_HLAC2_5;
      break;
#ifdef ENABLE_GRAY_HLAC3_5
    case 3:
      dim = DIM_OF_GRAY_HLAC3_5;
      break;
#endif /* ENABLE_GRAY_HLAC3_5 */
    default:
      std::cerr << "HLAC:checkDimensionGray(): not proper order of HLAC" 
		<< std::endl
		<< "-----> order = " << order << ", size = " << size 
		<< std::endl;
      exit(1);
    }
  }else{
    std::cerr << "HLAC:checkDimensionGray(): square size is not proper" 
	      << std::endl
	      << "-----> order = " << order << ", size = " << size 
	      << std::endl;
    exit(1);
  }
  
  /*** check dimension ***/
  if( (int)result.size() != dim ) result.resize( dim );
}
 
void HLAC::checkDimensionColor( std::vector<float> &result, const int &order, const int &size )
{
  int dim;
  
  /*** check order ***/
  if( size == 3 ){
    switch( order ){
    case 1:
      dim = DIM_OF_COLOR_HLAC1_3;
      break;
    case 2:
      dim = DIM_OF_COLOR_HLAC2_3;
      break;
    default:
      std::cerr << "HLAC:checkDimensionColor(): not proper order of HLAC" 
		<< std::endl
		<< "-----> order = " << order << ", size = " << size 
		<< std::endl;
      exit(1);
    }
  }else if( size == 5 ){
    switch( order ){
    case 1:
      dim = DIM_OF_COLOR_HLAC1_5;
      break;
    default:
      std::cerr << "HLAC:checkDimensionColor(): not proper order of HLAC" 
		<< std::endl
		<< "-----> order = " << order << ", size = " << size 
		<< std::endl;
      exit(1);
    }
  }else{
    std::cerr << "HLAC:checkDimensionColor(): square size is not proper" 
	      << std::endl
	      << "-----> order = " << order << ", size = " << size 
	      << std::endl;
    exit(1);
  }
  
  /*** check dimension ***/
  if( (int)result.size() != dim ) result.resize( dim );
}


/*** 特徴抽出の範囲を決定する関数 ********************************************/
 
void HLAC::decideWindow( int &left, int &right, int &up, int &down,
			    const int &x, const int &dx,
			    const int &y, const int &dy,
			    const cv::Mat &img, 
			    const int &size, const int &rx, const int &ry )
{
  const int tmpX = rx * ( size - 1 ) / 2;
  const int tmpY = ry * ( size - 1 ) / 2;
  
  //デフォルトなら全範囲特徴抽出
  if( x == -1 && dx == -1 && y == -1 && dy == -1 ){
    left  = tmpX;
    up    = tmpY;
    right = img.cols - tmpX;
    down  = img.rows - tmpY;
  }else{
    left  = x;
    right = x + dx;
    up    = y;
    down  = y + dy;
    //画面の外にはみ出ないかどうか
    if( left  < tmpX ) left = tmpX;
    if( up    < tmpY ) up   = tmpY;
    if( right > img.cols - tmpX ) right = img.cols - tmpX;
    if( down  > img.rows - tmpY ) down  = img.rows - tmpY;
  }
  
  //大小関係は正しいか
  if( left >= right || up >= down ){
    std::cerr << "HLAC::decideWindow(): not proper paremeter of window size" 
	      << std::endl;
    exit(1);
  }
}
