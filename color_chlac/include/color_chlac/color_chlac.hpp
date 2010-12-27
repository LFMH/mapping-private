//*********************************************************************************
//* calculate a feature vector when the target voxel data is rotated by 90 degrees.
inline void 
pcl::rotateFeature90( std::vector<float> &output, const std::vector<float> &input, RotateMode mode ){
  int dim = input.size();

  switch( dim ){
  case DIM_COLOR_BIN_1_3+DIM_COLOR_1_3:
    {
      if( output.size() != (std::vector<float>::size_type)dim )
	output.resize( dim );
      
      std::vector<float> tmp_input(DIM_COLOR_1_3);
      std::vector<float> tmp_output;
      for(int i=0;i<DIM_COLOR_1_3;i++)
	tmp_input[i] = input[i];
      rotateFeature90( tmp_output, tmp_input, mode );
      for(int i=0;i<DIM_COLOR_1_3;i++)
	output[i] = tmp_output[i];
      
      tmp_input.resize( DIM_COLOR_BIN_1_3 );
      for(int i=0;i<DIM_COLOR_BIN_1_3;i++)
	tmp_input[i] = input[i+DIM_COLOR_1_3];
      rotateFeature90( tmp_output, tmp_input, mode );
      for(int i=0;i<DIM_COLOR_BIN_1_3;i++)
	output[i+DIM_COLOR_1_3] = tmp_output[i];    
      break;
    }
    
  case DIM_COLOR_BIN_1_3:
  case DIM_COLOR_1_3:
    if( output.size() != (std::vector<float>::size_type)dim )
      output.resize( dim );
    for(int i=0;i<6;i++)
      output[i]=input[i];
    for(int i=474;i<dim;i++)
      output[i]=input[i];
    
    switch( mode ){
    case R_MODE_1:
      for(int i=0;i<6;i++){
	for(int j=0;j<6;j++){
	  output[  8 + i*9 + j*78 ] = input[  6 + i*9 + j*78 ];
	  output[ 11 + i*9 + j*78 ] = input[  7 + i*9 + j*78 ];
	  output[ 14 + i*9 + j*78 ] = input[  8 + i*9 + j*78 ];
	  output[  7 + i*9 + j*78 ] = input[  9 + i*9 + j*78 ];
	  output[ 10 + i*9 + j*78 ] = input[ 10 + i*9 + j*78 ];
	  output[ 13 + i*9 + j*78 ] = input[ 11 + i*9 + j*78 ];
	  output[  6 + i*9 + j*78 ] = input[ 12 + i*9 + j*78 ];
	  output[  9 + i*9 + j*78 ] = input[ 13 + i*9 + j*78 ];
	  output[ 12 + i*9 + j*78 ] = input[ 14 + i*9 + j*78 ];
	  output[ 62 + i*4 + j*78 ] = input[ 60 + i*4 + j*78 ];
	  output[ 63 + j*4 + i*78 ] = input[ 61 + i*4 + j*78 ]; // Swapping j for i
	  output[ 60 + j*4 + i*78 ] = input[ 62 + i*4 + j*78 ]; // Swapping j for i
	  output[ 61 + i*4 + j*78 ] = input[ 63 + i*4 + j*78 ];
	}
      }
      break;
    case R_MODE_2:
      for(int i=0;i<6;i++){
	for(int j=0;j<6;j++){
	  output[  8 + i*9 + j*78 ] = input[  6 + i*9 + j*78 ];
	  output[ 62 + i*4 + j*78 ] = input[  7 + i*9 + j*78 ];
	  output[ 12 + j*9 + i*78 ] = input[  8 + i*9 + j*78 ]; // Swapping j for i
	  output[ 11 + i*9 + j*78 ] = input[  9 + i*9 + j*78 ];
	  output[ 63 + j*4 + i*78 ] = input[ 10 + i*9 + j*78 ]; // Swapping j for i
	  output[  9 + j*9 + i*78 ] = input[ 11 + i*9 + j*78 ]; // Swapping j for i
	  output[ 14 + i*9 + j*78 ] = input[ 12 + i*9 + j*78 ];
	  output[ 60 + j*4 + i*78 ] = input[ 13 + i*9 + j*78 ]; // Swapping j for i
	  output[  6 + j*9 + i*78 ] = input[ 14 + i*9 + j*78 ]; // Swapping j for i
	  output[  7 + i*9 + j*78 ] = input[ 60 + i*4 + j*78 ];
	  output[ 61 + i*4 + j*78 ] = input[ 61 + i*4 + j*78 ];
	  output[ 13 + j*9 + i*78 ] = input[ 62 + i*4 + j*78 ]; // Swapping j for i
	  output[ 10 + i*9 + j*78 ] = input[ 63 + i*4 + j*78 ];
	}
      }
      break;
    case R_MODE_3:
      for(int i=0;i<6;i++){
	for(int j=0;j<6;j++){
	  output[ 12 + i*9 + j*78 ] = input[  6 + i*9 + j*78 ];
	  output[ 13 + i*9 + j*78 ] = input[  7 + i*9 + j*78 ];
	  output[ 14 + i*9 + j*78 ] = input[  8 + i*9 + j*78 ];
	  output[ 62 + j*4 + i*78 ] = input[  9 + i*9 + j*78 ]; // Swapping j for i
	  output[ 61 + j*4 + i*78 ] = input[ 10 + i*9 + j*78 ]; // Swapping j for i
	  output[ 60 + j*4 + i*78 ] = input[ 11 + i*9 + j*78 ]; // Swapping j for i
	  output[  8 + j*9 + i*78 ] = input[ 12 + i*9 + j*78 ]; // Swapping j for i
	  output[  7 + j*9 + i*78 ] = input[ 13 + i*9 + j*78 ]; // Swapping j for i
	  output[  6 + j*9 + i*78 ] = input[ 14 + i*9 + j*78 ]; // Swapping j for i
	  output[  9 + i*9 + j*78 ] = input[ 60 + i*4 + j*78 ];
	  output[ 10 + i*9 + j*78 ] = input[ 61 + i*4 + j*78 ];
	  output[ 11 + i*9 + j*78 ] = input[ 62 + i*4 + j*78 ];
	  output[ 63 + i*4 + j*78 ] = input[ 63 + i*4 + j*78 ];
	}
      }
      break;
    case R_MODE_4:
      for(int i=0;i<6;i++){
	for(int j=0;j<6;j++){
	  output[ 12 + i*9 + j*78 ] = input[  6 + i*9 + j*78 ];
	  output[  9 + i*9 + j*78 ] = input[  7 + i*9 + j*78 ];
	  output[  6 + i*9 + j*78 ] = input[  8 + i*9 + j*78 ];
	  output[ 13 + i*9 + j*78 ] = input[  9 + i*9 + j*78 ];
	  output[ 10 + i*9 + j*78 ] = input[ 10 + i*9 + j*78 ];
	  output[  7 + i*9 + j*78 ] = input[ 11 + i*9 + j*78 ];
	  output[ 14 + i*9 + j*78 ] = input[ 12 + i*9 + j*78 ];
	  output[ 11 + i*9 + j*78 ] = input[ 13 + i*9 + j*78 ];
	  output[  8 + i*9 + j*78 ] = input[ 14 + i*9 + j*78 ];
	  output[ 62 + j*4 + i*78 ] = input[ 60 + i*4 + j*78 ]; // Swapping j for i
	  output[ 63 + i*4 + j*78 ] = input[ 61 + i*4 + j*78 ];
	  output[ 60 + i*4 + j*78 ] = input[ 62 + i*4 + j*78 ];
	  output[ 61 + j*4 + i*78 ] = input[ 63 + i*4 + j*78 ]; // Swapping j for i
	}
      }
      break;
    default:
      std::cerr << "ERR (in ColorCHLAC::rotateFeature90): unknown RotateMode." << std::endl;
      exit( EXIT_FAILURE );
      break;
    }
    break;
  default:
    std::cerr << "ERR (in ColorCHLAC::rotateFeature90): improper dimension: " << dim << std::endl;
    exit( EXIT_FAILURE );
    break;
  }
}

inline int reverse( int val ){
  return 255 - val;
}

template <typename PointT, typename PointOutT> inline int
pcl::ColorCHLACEstimation<PointT, PointOutT>::binarize_r ( int val )
{
  if( val > color_thR ) return 1;
  return 0;
}
template <typename PointT, typename PointOutT> inline int
pcl::ColorCHLACEstimation<PointT, PointOutT>::binarize_g ( int val )
{
  if( val > color_thG ) return 1;
  return 0;
}
template <typename PointT, typename PointOutT> inline int
pcl::ColorCHLACEstimation<PointT, PointOutT>::binarize_b ( int val )
{
  if( val > color_thB ) return 1;
  return 0;
}

template <typename PointT, typename PointOutT> inline void 
pcl::ColorCHLACEstimation<PointT, PointOutT>::setVoxelFilter ( pcl::VoxelGrid<PointT> grid_, const int subdivision_size_ = 0 )
{ 
  grid = grid_;

  if( subdivision_size_ > 0 ){
    inverse_subdivision_size = 1.0 / subdivision_size_;
    div_b_ = grid.getNrDivisions();
    subdiv_b_ = Eigen3::Vector3i ( ceil( div_b_[0]*inverse_subdivision_size ), ceil( div_b_[1]*inverse_subdivision_size ), ceil( div_b_[2]*inverse_subdivision_size ) );
    subdivb_mul_ = Eigen3::Vector3i ( 1, subdiv_b_[0], subdiv_b_[0] * subdiv_b_[1] );
    hist_num = subdiv_b_[0] * subdiv_b_[1] * subdiv_b_[2];
  }
  else if( subdivision_size_ < 0 ){
    std::cerr << "(In setVoxelFilter) Invalid subdivision size: " << subdivision_size_ << std::endl;
    return;
  }
}

//***********************************************************//
//* functions for ColorCHLACSignature981 (rotation-variant) *//
//***********************************************************//
template <typename PointT, typename PointOutT> inline void 
pcl::ColorCHLACEstimation<PointT, PointOutT>::addColorCHLAC_0 ( const int idx, PointCloudOut &output )
{
  output.points[idx].histogram[   0 ] += center_r;
  output.points[idx].histogram[   1 ] += center_r_;
  output.points[idx].histogram[   2 ] += center_g;
  output.points[idx].histogram[   3 ] += center_g_;
  output.points[idx].histogram[   4 ] += center_b;
  output.points[idx].histogram[   5 ] += center_b_;
  output.points[idx].histogram[ 474 ] += center_r * center_r;
  output.points[idx].histogram[ 475 ] += center_r * center_r_;
  output.points[idx].histogram[ 476 ] += center_r * center_g;
  output.points[idx].histogram[ 477 ] += center_r * center_g_;
  output.points[idx].histogram[ 478 ] += center_r * center_b;
  output.points[idx].histogram[ 479 ] += center_r * center_b_;
  output.points[idx].histogram[ 480 ] += center_r_* center_r_;
  output.points[idx].histogram[ 481 ] += center_r_* center_g;
  output.points[idx].histogram[ 482 ] += center_r_* center_g_;
  output.points[idx].histogram[ 483 ] += center_r_* center_b;
  output.points[idx].histogram[ 484 ] += center_r_* center_b_;
  output.points[idx].histogram[ 485 ] += center_g * center_g;
  output.points[idx].histogram[ 486 ] += center_g * center_g_;
  output.points[idx].histogram[ 487 ] += center_g * center_b;
  output.points[idx].histogram[ 488 ] += center_g * center_b_;
  output.points[idx].histogram[ 489 ] += center_g_* center_g_;
  output.points[idx].histogram[ 490 ] += center_g_* center_b;
  output.points[idx].histogram[ 491 ] += center_g_* center_b_;
  output.points[idx].histogram[ 492 ] += center_b * center_b;
  output.points[idx].histogram[ 493 ] += center_b * center_b_;
  output.points[idx].histogram[ 494 ] += center_b_* center_b_;
}

template <typename PointT, typename PointOutT> inline void 
pcl::ColorCHLACEstimation<PointT, PointOutT>::addColorCHLAC_0_bin ( const int idx, PointCloudOut &output )
{
  if( center_bin_r )
    output.points[idx].histogram[ 495 ] ++;
  else
    output.points[idx].histogram[ 496 ] ++;
  if( center_bin_g )
    output.points[idx].histogram[ 497 ] ++;
  else
    output.points[idx].histogram[ 498 ] ++;
  if( center_bin_b )
    output.points[idx].histogram[ 499 ] ++;
  else
    output.points[idx].histogram[ 500 ] ++;

  if( center_bin_r ){
    if( center_bin_g )
      output.points[idx].histogram[ 969 ] ++;
    else
      output.points[idx].histogram[ 970 ] ++;
    if( center_bin_b )
      output.points[idx].histogram[ 971 ] ++;
    else
      output.points[idx].histogram[ 972 ] ++;
  }
  else{
    if( center_bin_g )
      output.points[idx].histogram[ 973 ] ++;
    else
      output.points[idx].histogram[ 974 ] ++;
    if( center_bin_b )
      output.points[idx].histogram[ 975 ] ++;
    else
      output.points[idx].histogram[ 976 ] ++;
  }
  if( center_bin_g ){
    if( center_bin_b )
      output.points[idx].histogram[ 977 ] ++;
    else
      output.points[idx].histogram[ 978 ] ++;
  }
  else{
    if( center_bin_b )
      output.points[idx].histogram[ 979 ] ++;
    else
      output.points[idx].histogram[ 980 ] ++;
  }
}

template <typename PointT, typename PointOutT> inline void 
pcl::ColorCHLACEstimation<PointT, PointOutT>::addColorCHLAC_1 ( const int idx, PointCloudOut &output, int neighbor_idx, int r, int g, int b )
{
  const int r_ = reverse( r );
  const int g_ = reverse( g );
  const int b_ = reverse( b );

  switch( neighbor_idx ){
  case 0:
    output.points[idx].histogram[   6 ] += center_r * r;
    output.points[idx].histogram[  15 ] += center_r * r_;
    output.points[idx].histogram[  24 ] += center_r * g;
    output.points[idx].histogram[  33 ] += center_r * g_;
    output.points[idx].histogram[  42 ] += center_r * b;
    output.points[idx].histogram[  51 ] += center_r * b_;
    output.points[idx].histogram[  84 ] += center_r_* r;
    output.points[idx].histogram[  93 ] += center_r_* r_;
    output.points[idx].histogram[ 102 ] += center_r_* g;
    output.points[idx].histogram[ 111 ] += center_r_* g_;
    output.points[idx].histogram[ 120 ] += center_r_* b;
    output.points[idx].histogram[ 129 ] += center_r_* b_;
    output.points[idx].histogram[ 162 ] += center_g * r;
    output.points[idx].histogram[ 171 ] += center_g * r_;
    output.points[idx].histogram[ 180 ] += center_g * g;
    output.points[idx].histogram[ 189 ] += center_g * g_;
    output.points[idx].histogram[ 198 ] += center_g * b;
    output.points[idx].histogram[ 207 ] += center_g * b_;
    output.points[idx].histogram[ 240 ] += center_g_* r;
    output.points[idx].histogram[ 249 ] += center_g_* r_;
    output.points[idx].histogram[ 258 ] += center_g_* g;
    output.points[idx].histogram[ 267 ] += center_g_* g_;
    output.points[idx].histogram[ 276 ] += center_g_* b;
    output.points[idx].histogram[ 285 ] += center_g_* b_;
    output.points[idx].histogram[ 318 ] += center_b * r;
    output.points[idx].histogram[ 327 ] += center_b * r_;
    output.points[idx].histogram[ 336 ] += center_b * g;
    output.points[idx].histogram[ 345 ] += center_b * g_;
    output.points[idx].histogram[ 354 ] += center_b * b;
    output.points[idx].histogram[ 363 ] += center_b * b_;
    output.points[idx].histogram[ 396 ] += center_b_* r;
    output.points[idx].histogram[ 405 ] += center_b_* r_;
    output.points[idx].histogram[ 414 ] += center_b_* g;
    output.points[idx].histogram[ 423 ] += center_b_* g_;
    output.points[idx].histogram[ 432 ] += center_b_* b;
    output.points[idx].histogram[ 441 ] += center_b_* b_;
    break;
  case 1:
    output.points[idx].histogram[   7 ] += center_r * r;
    output.points[idx].histogram[  16 ] += center_r * r_;
    output.points[idx].histogram[  25 ] += center_r * g;
    output.points[idx].histogram[  34 ] += center_r * g_;
    output.points[idx].histogram[  43 ] += center_r * b;
    output.points[idx].histogram[  52 ] += center_r * b_;
    output.points[idx].histogram[  85 ] += center_r_* r;
    output.points[idx].histogram[  94 ] += center_r_* r_;
    output.points[idx].histogram[ 103 ] += center_r_* g;
    output.points[idx].histogram[ 112 ] += center_r_* g_;
    output.points[idx].histogram[ 121 ] += center_r_* b;
    output.points[idx].histogram[ 130 ] += center_r_* b_;
    output.points[idx].histogram[ 163 ] += center_g * r;
    output.points[idx].histogram[ 172 ] += center_g * r_;
    output.points[idx].histogram[ 181 ] += center_g * g;
    output.points[idx].histogram[ 190 ] += center_g * g_;
    output.points[idx].histogram[ 199 ] += center_g * b;
    output.points[idx].histogram[ 208 ] += center_g * b_;
    output.points[idx].histogram[ 241 ] += center_g_* r;
    output.points[idx].histogram[ 250 ] += center_g_* r_;
    output.points[idx].histogram[ 259 ] += center_g_* g;
    output.points[idx].histogram[ 268 ] += center_g_* g_;
    output.points[idx].histogram[ 277 ] += center_g_* b;
    output.points[idx].histogram[ 286 ] += center_g_* b_;
    output.points[idx].histogram[ 319 ] += center_b * r;
    output.points[idx].histogram[ 328 ] += center_b * r_;
    output.points[idx].histogram[ 337 ] += center_b * g;
    output.points[idx].histogram[ 346 ] += center_b * g_;
    output.points[idx].histogram[ 355 ] += center_b * b;
    output.points[idx].histogram[ 364 ] += center_b * b_;
    output.points[idx].histogram[ 397 ] += center_b_* r;
    output.points[idx].histogram[ 406 ] += center_b_* r_;
    output.points[idx].histogram[ 415 ] += center_b_* g;
    output.points[idx].histogram[ 424 ] += center_b_* g_;
    output.points[idx].histogram[ 433 ] += center_b_* b;
    output.points[idx].histogram[ 442 ] += center_b_* b_;
    break;
  case 2:
    output.points[idx].histogram[   8 ] += center_r * r;
    output.points[idx].histogram[  17 ] += center_r * r_;
    output.points[idx].histogram[  26 ] += center_r * g;
    output.points[idx].histogram[  35 ] += center_r * g_;
    output.points[idx].histogram[  44 ] += center_r * b;
    output.points[idx].histogram[  53 ] += center_r * b_;
    output.points[idx].histogram[  86 ] += center_r_* r;
    output.points[idx].histogram[  95 ] += center_r_* r_;
    output.points[idx].histogram[ 104 ] += center_r_* g;
    output.points[idx].histogram[ 113 ] += center_r_* g_;
    output.points[idx].histogram[ 122 ] += center_r_* b;
    output.points[idx].histogram[ 131 ] += center_r_* b_;
    output.points[idx].histogram[ 164 ] += center_g * r;
    output.points[idx].histogram[ 173 ] += center_g * r_;
    output.points[idx].histogram[ 182 ] += center_g * g;
    output.points[idx].histogram[ 191 ] += center_g * g_;
    output.points[idx].histogram[ 200 ] += center_g * b;
    output.points[idx].histogram[ 209 ] += center_g * b_;
    output.points[idx].histogram[ 242 ] += center_g_* r;
    output.points[idx].histogram[ 251 ] += center_g_* r_;
    output.points[idx].histogram[ 260 ] += center_g_* g;
    output.points[idx].histogram[ 269 ] += center_g_* g_;
    output.points[idx].histogram[ 278 ] += center_g_* b;
    output.points[idx].histogram[ 287 ] += center_g_* b_;
    output.points[idx].histogram[ 320 ] += center_b * r;
    output.points[idx].histogram[ 329 ] += center_b * r_;
    output.points[idx].histogram[ 338 ] += center_b * g;
    output.points[idx].histogram[ 347 ] += center_b * g_;
    output.points[idx].histogram[ 356 ] += center_b * b;
    output.points[idx].histogram[ 365 ] += center_b * b_;
    output.points[idx].histogram[ 398 ] += center_b_* r;
    output.points[idx].histogram[ 407 ] += center_b_* r_;
    output.points[idx].histogram[ 416 ] += center_b_* g;
    output.points[idx].histogram[ 425 ] += center_b_* g_;
    output.points[idx].histogram[ 434 ] += center_b_* b;
    output.points[idx].histogram[ 443 ] += center_b_* b_;
    break;
  case 3:
    output.points[idx].histogram[   9 ] += center_r * r;
    output.points[idx].histogram[  18 ] += center_r * r_;
    output.points[idx].histogram[  27 ] += center_r * g;
    output.points[idx].histogram[  36 ] += center_r * g_;
    output.points[idx].histogram[  45 ] += center_r * b;
    output.points[idx].histogram[  54 ] += center_r * b_;
    output.points[idx].histogram[  87 ] += center_r_* r;
    output.points[idx].histogram[  96 ] += center_r_* r_;
    output.points[idx].histogram[ 105 ] += center_r_* g;
    output.points[idx].histogram[ 114 ] += center_r_* g_;
    output.points[idx].histogram[ 123 ] += center_r_* b;
    output.points[idx].histogram[ 132 ] += center_r_* b_;
    output.points[idx].histogram[ 165 ] += center_g * r;
    output.points[idx].histogram[ 174 ] += center_g * r_;
    output.points[idx].histogram[ 183 ] += center_g * g;
    output.points[idx].histogram[ 192 ] += center_g * g_;
    output.points[idx].histogram[ 201 ] += center_g * b;
    output.points[idx].histogram[ 210 ] += center_g * b_;
    output.points[idx].histogram[ 243 ] += center_g_* r;
    output.points[idx].histogram[ 252 ] += center_g_* r_;
    output.points[idx].histogram[ 261 ] += center_g_* g;
    output.points[idx].histogram[ 270 ] += center_g_* g_;
    output.points[idx].histogram[ 279 ] += center_g_* b;
    output.points[idx].histogram[ 288 ] += center_g_* b_;
    output.points[idx].histogram[ 321 ] += center_b * r;
    output.points[idx].histogram[ 330 ] += center_b * r_;
    output.points[idx].histogram[ 339 ] += center_b * g;
    output.points[idx].histogram[ 348 ] += center_b * g_;
    output.points[idx].histogram[ 357 ] += center_b * b;
    output.points[idx].histogram[ 366 ] += center_b * b_;
    output.points[idx].histogram[ 399 ] += center_b_* r;
    output.points[idx].histogram[ 408 ] += center_b_* r_;
    output.points[idx].histogram[ 417 ] += center_b_* g;
    output.points[idx].histogram[ 426 ] += center_b_* g_;
    output.points[idx].histogram[ 435 ] += center_b_* b;
    output.points[idx].histogram[ 444 ] += center_b_* b_;
    break;
  case 4:
    output.points[idx].histogram[  10 ] += center_r * r;
    output.points[idx].histogram[  19 ] += center_r * r_;
    output.points[idx].histogram[  28 ] += center_r * g;
    output.points[idx].histogram[  37 ] += center_r * g_;
    output.points[idx].histogram[  46 ] += center_r * b;
    output.points[idx].histogram[  55 ] += center_r * b_;
    output.points[idx].histogram[  88 ] += center_r_* r;
    output.points[idx].histogram[  97 ] += center_r_* r_;
    output.points[idx].histogram[ 106 ] += center_r_* g;
    output.points[idx].histogram[ 115 ] += center_r_* g_;
    output.points[idx].histogram[ 124 ] += center_r_* b;
    output.points[idx].histogram[ 133 ] += center_r_* b_;
    output.points[idx].histogram[ 166 ] += center_g * r;
    output.points[idx].histogram[ 175 ] += center_g * r_;
    output.points[idx].histogram[ 184 ] += center_g * g;
    output.points[idx].histogram[ 193 ] += center_g * g_;
    output.points[idx].histogram[ 202 ] += center_g * b;
    output.points[idx].histogram[ 211 ] += center_g * b_;
    output.points[idx].histogram[ 244 ] += center_g_* r;
    output.points[idx].histogram[ 253 ] += center_g_* r_;
    output.points[idx].histogram[ 262 ] += center_g_* g;
    output.points[idx].histogram[ 271 ] += center_g_* g_;
    output.points[idx].histogram[ 280 ] += center_g_* b;
    output.points[idx].histogram[ 289 ] += center_g_* b_;
    output.points[idx].histogram[ 322 ] += center_b * r;
    output.points[idx].histogram[ 331 ] += center_b * r_;
    output.points[idx].histogram[ 340 ] += center_b * g;
    output.points[idx].histogram[ 349 ] += center_b * g_;
    output.points[idx].histogram[ 358 ] += center_b * b;
    output.points[idx].histogram[ 367 ] += center_b * b_;
    output.points[idx].histogram[ 400 ] += center_b_* r;
    output.points[idx].histogram[ 409 ] += center_b_* r_;
    output.points[idx].histogram[ 418 ] += center_b_* g;
    output.points[idx].histogram[ 427 ] += center_b_* g_;
    output.points[idx].histogram[ 436 ] += center_b_* b;
    output.points[idx].histogram[ 445 ] += center_b_* b_;
    break;
  case 5:
    output.points[idx].histogram[  11 ] += center_r * r;
    output.points[idx].histogram[  20 ] += center_r * r_;
    output.points[idx].histogram[  29 ] += center_r * g;
    output.points[idx].histogram[  38 ] += center_r * g_;
    output.points[idx].histogram[  47 ] += center_r * b;
    output.points[idx].histogram[  56 ] += center_r * b_;
    output.points[idx].histogram[  89 ] += center_r_* r;
    output.points[idx].histogram[  98 ] += center_r_* r_;
    output.points[idx].histogram[ 107 ] += center_r_* g;
    output.points[idx].histogram[ 116 ] += center_r_* g_;
    output.points[idx].histogram[ 125 ] += center_r_* b;
    output.points[idx].histogram[ 134 ] += center_r_* b_;
    output.points[idx].histogram[ 167 ] += center_g * r;
    output.points[idx].histogram[ 176 ] += center_g * r_;
    output.points[idx].histogram[ 185 ] += center_g * g;
    output.points[idx].histogram[ 194 ] += center_g * g_;
    output.points[idx].histogram[ 203 ] += center_g * b;
    output.points[idx].histogram[ 212 ] += center_g * b_;
    output.points[idx].histogram[ 245 ] += center_g_* r;
    output.points[idx].histogram[ 254 ] += center_g_* r_;
    output.points[idx].histogram[ 263 ] += center_g_* g;
    output.points[idx].histogram[ 272 ] += center_g_* g_;
    output.points[idx].histogram[ 281 ] += center_g_* b;
    output.points[idx].histogram[ 290 ] += center_g_* b_;
    output.points[idx].histogram[ 323 ] += center_b * r;
    output.points[idx].histogram[ 332 ] += center_b * r_;
    output.points[idx].histogram[ 341 ] += center_b * g;
    output.points[idx].histogram[ 350 ] += center_b * g_;
    output.points[idx].histogram[ 359 ] += center_b * b;
    output.points[idx].histogram[ 368 ] += center_b * b_;
    output.points[idx].histogram[ 401 ] += center_b_* r;
    output.points[idx].histogram[ 410 ] += center_b_* r_;
    output.points[idx].histogram[ 419 ] += center_b_* g;
    output.points[idx].histogram[ 428 ] += center_b_* g_;
    output.points[idx].histogram[ 437 ] += center_b_* b;
    output.points[idx].histogram[ 446 ] += center_b_* b_;
    break;
  case 6:
    output.points[idx].histogram[  12 ] += center_r * r;
    output.points[idx].histogram[  21 ] += center_r * r_;
    output.points[idx].histogram[  30 ] += center_r * g;
    output.points[idx].histogram[  39 ] += center_r * g_;
    output.points[idx].histogram[  48 ] += center_r * b;
    output.points[idx].histogram[  57 ] += center_r * b_;
    output.points[idx].histogram[  90 ] += center_r_* r;
    output.points[idx].histogram[  99 ] += center_r_* r_;
    output.points[idx].histogram[ 108 ] += center_r_* g;
    output.points[idx].histogram[ 117 ] += center_r_* g_;
    output.points[idx].histogram[ 126 ] += center_r_* b;
    output.points[idx].histogram[ 135 ] += center_r_* b_;
    output.points[idx].histogram[ 168 ] += center_g * r;
    output.points[idx].histogram[ 177 ] += center_g * r_;
    output.points[idx].histogram[ 186 ] += center_g * g;
    output.points[idx].histogram[ 195 ] += center_g * g_;
    output.points[idx].histogram[ 204 ] += center_g * b;
    output.points[idx].histogram[ 213 ] += center_g * b_;
    output.points[idx].histogram[ 246 ] += center_g_* r;
    output.points[idx].histogram[ 255 ] += center_g_* r_;
    output.points[idx].histogram[ 264 ] += center_g_* g;
    output.points[idx].histogram[ 273 ] += center_g_* g_;
    output.points[idx].histogram[ 282 ] += center_g_* b;
    output.points[idx].histogram[ 291 ] += center_g_* b_;
    output.points[idx].histogram[ 324 ] += center_b * r;
    output.points[idx].histogram[ 333 ] += center_b * r_;
    output.points[idx].histogram[ 342 ] += center_b * g;
    output.points[idx].histogram[ 351 ] += center_b * g_;
    output.points[idx].histogram[ 360 ] += center_b * b;
    output.points[idx].histogram[ 369 ] += center_b * b_;
    output.points[idx].histogram[ 402 ] += center_b_* r;
    output.points[idx].histogram[ 411 ] += center_b_* r_;
    output.points[idx].histogram[ 420 ] += center_b_* g;
    output.points[idx].histogram[ 429 ] += center_b_* g_;
    output.points[idx].histogram[ 438 ] += center_b_* b;
    output.points[idx].histogram[ 447 ] += center_b_* b_;
    break;
  case 7:
    output.points[idx].histogram[  13 ] += center_r * r;
    output.points[idx].histogram[  22 ] += center_r * r_;
    output.points[idx].histogram[  31 ] += center_r * g;
    output.points[idx].histogram[  40 ] += center_r * g_;
    output.points[idx].histogram[  49 ] += center_r * b;
    output.points[idx].histogram[  58 ] += center_r * b_;
    output.points[idx].histogram[  91 ] += center_r_* r;
    output.points[idx].histogram[ 100 ] += center_r_* r_;
    output.points[idx].histogram[ 109 ] += center_r_* g;
    output.points[idx].histogram[ 118 ] += center_r_* g_;
    output.points[idx].histogram[ 127 ] += center_r_* b;
    output.points[idx].histogram[ 136 ] += center_r_* b_;
    output.points[idx].histogram[ 169 ] += center_g * r;
    output.points[idx].histogram[ 178 ] += center_g * r_;
    output.points[idx].histogram[ 187 ] += center_g * g;
    output.points[idx].histogram[ 196 ] += center_g * g_;
    output.points[idx].histogram[ 205 ] += center_g * b;
    output.points[idx].histogram[ 214 ] += center_g * b_;
    output.points[idx].histogram[ 247 ] += center_g_* r;
    output.points[idx].histogram[ 256 ] += center_g_* r_;
    output.points[idx].histogram[ 265 ] += center_g_* g;
    output.points[idx].histogram[ 274 ] += center_g_* g_;
    output.points[idx].histogram[ 283 ] += center_g_* b;
    output.points[idx].histogram[ 292 ] += center_g_* b_;
    output.points[idx].histogram[ 325 ] += center_b * r;
    output.points[idx].histogram[ 334 ] += center_b * r_;
    output.points[idx].histogram[ 343 ] += center_b * g;
    output.points[idx].histogram[ 352 ] += center_b * g_;
    output.points[idx].histogram[ 361 ] += center_b * b;
    output.points[idx].histogram[ 370 ] += center_b * b_;
    output.points[idx].histogram[ 403 ] += center_b_* r;
    output.points[idx].histogram[ 412 ] += center_b_* r_;
    output.points[idx].histogram[ 421 ] += center_b_* g;
    output.points[idx].histogram[ 430 ] += center_b_* g_;
    output.points[idx].histogram[ 439 ] += center_b_* b;
    output.points[idx].histogram[ 448 ] += center_b_* b_;
    break;
  case 8:
    output.points[idx].histogram[  14 ] += center_r * r;
    output.points[idx].histogram[  23 ] += center_r * r_;
    output.points[idx].histogram[  32 ] += center_r * g;
    output.points[idx].histogram[  41 ] += center_r * g_;
    output.points[idx].histogram[  50 ] += center_r * b;
    output.points[idx].histogram[  59 ] += center_r * b_;
    output.points[idx].histogram[  92 ] += center_r_* r;
    output.points[idx].histogram[ 101 ] += center_r_* r_;
    output.points[idx].histogram[ 110 ] += center_r_* g;
    output.points[idx].histogram[ 119 ] += center_r_* g_;
    output.points[idx].histogram[ 128 ] += center_r_* b;
    output.points[idx].histogram[ 137 ] += center_r_* b_;
    output.points[idx].histogram[ 170 ] += center_g * r;
    output.points[idx].histogram[ 179 ] += center_g * r_;
    output.points[idx].histogram[ 188 ] += center_g * g;
    output.points[idx].histogram[ 197 ] += center_g * g_;
    output.points[idx].histogram[ 206 ] += center_g * b;
    output.points[idx].histogram[ 215 ] += center_g * b_;
    output.points[idx].histogram[ 248 ] += center_g_* r;
    output.points[idx].histogram[ 257 ] += center_g_* r_;
    output.points[idx].histogram[ 266 ] += center_g_* g;
    output.points[idx].histogram[ 275 ] += center_g_* g_;
    output.points[idx].histogram[ 284 ] += center_g_* b;
    output.points[idx].histogram[ 293 ] += center_g_* b_;
    output.points[idx].histogram[ 326 ] += center_b * r;
    output.points[idx].histogram[ 335 ] += center_b * r_;
    output.points[idx].histogram[ 344 ] += center_b * g;
    output.points[idx].histogram[ 353 ] += center_b * g_;
    output.points[idx].histogram[ 362 ] += center_b * b;
    output.points[idx].histogram[ 371 ] += center_b * b_;
    output.points[idx].histogram[ 404 ] += center_b_* r;
    output.points[idx].histogram[ 413 ] += center_b_* r_;
    output.points[idx].histogram[ 422 ] += center_b_* g;
    output.points[idx].histogram[ 431 ] += center_b_* g_;
    output.points[idx].histogram[ 440 ] += center_b_* b;
    output.points[idx].histogram[ 449 ] += center_b_* b_;
    break;
  case 9:
    output.points[idx].histogram[  60 ] += center_r * r;
    output.points[idx].histogram[  64 ] += center_r * r_;
    output.points[idx].histogram[  68 ] += center_r * g;
    output.points[idx].histogram[  72 ] += center_r * g_;
    output.points[idx].histogram[  76 ] += center_r * b;
    output.points[idx].histogram[  80 ] += center_r * b_;
    output.points[idx].histogram[ 138 ] += center_r_* r;
    output.points[idx].histogram[ 142 ] += center_r_* r_;
    output.points[idx].histogram[ 146 ] += center_r_* g;
    output.points[idx].histogram[ 150 ] += center_r_* g_;
    output.points[idx].histogram[ 154 ] += center_r_* b;
    output.points[idx].histogram[ 158 ] += center_r_* b_;
    output.points[idx].histogram[ 216 ] += center_g * r;
    output.points[idx].histogram[ 220 ] += center_g * r_;
    output.points[idx].histogram[ 224 ] += center_g * g;
    output.points[idx].histogram[ 228 ] += center_g * g_;
    output.points[idx].histogram[ 232 ] += center_g * b;
    output.points[idx].histogram[ 236 ] += center_g * b_;
    output.points[idx].histogram[ 294 ] += center_g_* r;
    output.points[idx].histogram[ 298 ] += center_g_* r_;
    output.points[idx].histogram[ 302 ] += center_g_* g;
    output.points[idx].histogram[ 306 ] += center_g_* g_;
    output.points[idx].histogram[ 310 ] += center_g_* b;
    output.points[idx].histogram[ 314 ] += center_g_* b_;
    output.points[idx].histogram[ 372 ] += center_b * r;
    output.points[idx].histogram[ 376 ] += center_b * r_;
    output.points[idx].histogram[ 380 ] += center_b * g;
    output.points[idx].histogram[ 384 ] += center_b * g_;
    output.points[idx].histogram[ 388 ] += center_b * b;
    output.points[idx].histogram[ 392 ] += center_b * b_;
    output.points[idx].histogram[ 450 ] += center_b_* r;
    output.points[idx].histogram[ 454 ] += center_b_* r_;
    output.points[idx].histogram[ 458 ] += center_b_* g;
    output.points[idx].histogram[ 462 ] += center_b_* g_;
    output.points[idx].histogram[ 466 ] += center_b_* b;
    output.points[idx].histogram[ 470 ] += center_b_* b_;
    break;
  case 10:
    output.points[idx].histogram[  61 ] += center_r * r;
    output.points[idx].histogram[  65 ] += center_r * r_;
    output.points[idx].histogram[  69 ] += center_r * g;
    output.points[idx].histogram[  73 ] += center_r * g_;
    output.points[idx].histogram[  77 ] += center_r * b;
    output.points[idx].histogram[  81 ] += center_r * b_;
    output.points[idx].histogram[ 139 ] += center_r_* r;
    output.points[idx].histogram[ 143 ] += center_r_* r_;
    output.points[idx].histogram[ 147 ] += center_r_* g;
    output.points[idx].histogram[ 151 ] += center_r_* g_;
    output.points[idx].histogram[ 155 ] += center_r_* b;
    output.points[idx].histogram[ 159 ] += center_r_* b_;
    output.points[idx].histogram[ 217 ] += center_g * r;
    output.points[idx].histogram[ 221 ] += center_g * r_;
    output.points[idx].histogram[ 225 ] += center_g * g;
    output.points[idx].histogram[ 229 ] += center_g * g_;
    output.points[idx].histogram[ 233 ] += center_g * b;
    output.points[idx].histogram[ 237 ] += center_g * b_;
    output.points[idx].histogram[ 295 ] += center_g_* r;
    output.points[idx].histogram[ 299 ] += center_g_* r_;
    output.points[idx].histogram[ 303 ] += center_g_* g;
    output.points[idx].histogram[ 307 ] += center_g_* g_;
    output.points[idx].histogram[ 311 ] += center_g_* b;
    output.points[idx].histogram[ 315 ] += center_g_* b_;
    output.points[idx].histogram[ 373 ] += center_b * r;
    output.points[idx].histogram[ 377 ] += center_b * r_;
    output.points[idx].histogram[ 381 ] += center_b * g;
    output.points[idx].histogram[ 385 ] += center_b * g_;
    output.points[idx].histogram[ 389 ] += center_b * b;
    output.points[idx].histogram[ 393 ] += center_b * b_;
    output.points[idx].histogram[ 451 ] += center_b_* r;
    output.points[idx].histogram[ 455 ] += center_b_* r_;
    output.points[idx].histogram[ 459 ] += center_b_* g;
    output.points[idx].histogram[ 463 ] += center_b_* g_;
    output.points[idx].histogram[ 467 ] += center_b_* b;
    output.points[idx].histogram[ 471 ] += center_b_* b_;
    break;
  case 11:
    output.points[idx].histogram[  62 ] += center_r * r;
    output.points[idx].histogram[  66 ] += center_r * r_;
    output.points[idx].histogram[  70 ] += center_r * g;
    output.points[idx].histogram[  74 ] += center_r * g_;
    output.points[idx].histogram[  78 ] += center_r * b;
    output.points[idx].histogram[  82 ] += center_r * b_;
    output.points[idx].histogram[ 140 ] += center_r_* r;
    output.points[idx].histogram[ 144 ] += center_r_* r_;
    output.points[idx].histogram[ 148 ] += center_r_* g;
    output.points[idx].histogram[ 152 ] += center_r_* g_;
    output.points[idx].histogram[ 156 ] += center_r_* b;
    output.points[idx].histogram[ 160 ] += center_r_* b_;
    output.points[idx].histogram[ 218 ] += center_g * r;
    output.points[idx].histogram[ 222 ] += center_g * r_;
    output.points[idx].histogram[ 226 ] += center_g * g;
    output.points[idx].histogram[ 230 ] += center_g * g_;
    output.points[idx].histogram[ 234 ] += center_g * b;
    output.points[idx].histogram[ 238 ] += center_g * b_;
    output.points[idx].histogram[ 296 ] += center_g_* r;
    output.points[idx].histogram[ 300 ] += center_g_* r_;
    output.points[idx].histogram[ 304 ] += center_g_* g;
    output.points[idx].histogram[ 308 ] += center_g_* g_;
    output.points[idx].histogram[ 312 ] += center_g_* b;
    output.points[idx].histogram[ 316 ] += center_g_* b_;
    output.points[idx].histogram[ 374 ] += center_b * r;
    output.points[idx].histogram[ 378 ] += center_b * r_;
    output.points[idx].histogram[ 382 ] += center_b * g;
    output.points[idx].histogram[ 386 ] += center_b * g_;
    output.points[idx].histogram[ 390 ] += center_b * b;
    output.points[idx].histogram[ 394 ] += center_b * b_;
    output.points[idx].histogram[ 452 ] += center_b_* r;
    output.points[idx].histogram[ 456 ] += center_b_* r_;
    output.points[idx].histogram[ 460 ] += center_b_* g;
    output.points[idx].histogram[ 464 ] += center_b_* g_;
    output.points[idx].histogram[ 468 ] += center_b_* b;
    output.points[idx].histogram[ 472 ] += center_b_* b_;
    break;
  case 12:
    output.points[idx].histogram[  63 ] += center_r * r;
    output.points[idx].histogram[  67 ] += center_r * r_;
    output.points[idx].histogram[  71 ] += center_r * g;
    output.points[idx].histogram[  75 ] += center_r * g_;
    output.points[idx].histogram[  79 ] += center_r * b;
    output.points[idx].histogram[  83 ] += center_r * b_;
    output.points[idx].histogram[ 141 ] += center_r_* r;
    output.points[idx].histogram[ 145 ] += center_r_* r_;
    output.points[idx].histogram[ 149 ] += center_r_* g;
    output.points[idx].histogram[ 153 ] += center_r_* g_;
    output.points[idx].histogram[ 157 ] += center_r_* b;
    output.points[idx].histogram[ 161 ] += center_r_* b_;
    output.points[idx].histogram[ 219 ] += center_g * r;
    output.points[idx].histogram[ 223 ] += center_g * r_;
    output.points[idx].histogram[ 227 ] += center_g * g;
    output.points[idx].histogram[ 231 ] += center_g * g_;
    output.points[idx].histogram[ 235 ] += center_g * b;
    output.points[idx].histogram[ 239 ] += center_g * b_;
    output.points[idx].histogram[ 297 ] += center_g_* r;
    output.points[idx].histogram[ 301 ] += center_g_* r_;
    output.points[idx].histogram[ 305 ] += center_g_* g;
    output.points[idx].histogram[ 309 ] += center_g_* g_;
    output.points[idx].histogram[ 313 ] += center_g_* b;
    output.points[idx].histogram[ 317 ] += center_g_* b_;
    output.points[idx].histogram[ 375 ] += center_b * r;
    output.points[idx].histogram[ 379 ] += center_b * r_;
    output.points[idx].histogram[ 383 ] += center_b * g;
    output.points[idx].histogram[ 387 ] += center_b * g_;
    output.points[idx].histogram[ 391 ] += center_b * b;
    output.points[idx].histogram[ 395 ] += center_b * b_;
    output.points[idx].histogram[ 453 ] += center_b_* r;
    output.points[idx].histogram[ 457 ] += center_b_* r_;
    output.points[idx].histogram[ 461 ] += center_b_* g;
    output.points[idx].histogram[ 465 ] += center_b_* g_;
    output.points[idx].histogram[ 469 ] += center_b_* b;
    output.points[idx].histogram[ 473 ] += center_b_* b_;
    break;
    //   case 13: // itself
    //     break;
    //   case 14: // 14 - 26: redundant
    //     break;
  default:
    break;
  }
}

template <typename PointT, typename PointOutT> inline void 
pcl::ColorCHLACEstimation<PointT, PointOutT>::addColorCHLAC_1_bin ( const int idx, PointCloudOut &output, int neighbor_idx, int r, int g, int b )
{
  const int r_ = 1 - r;
  const int g_ = 1 - g;
  const int b_ = 1 - b;

  switch( neighbor_idx ){
  case 0:
    if( center_bin_r ){
      output.points[idx].histogram[ DIM_COLOR_1_3 +   6 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  15 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  24 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  33 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  42 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  51 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 +  84 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  93 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 102 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 111 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 120 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 129 ] += b_;
    }
    if( center_bin_g ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 162 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 171 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 180 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 189 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 198 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 207 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 240 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 249 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 258 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 267 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 276 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 285 ] += b_;
    }
    if( center_bin_b ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 318 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 327 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 336 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 345 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 354 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 363 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 396 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 405 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 414 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 423 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 432 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 441 ] += b_;
    }
    break;
  case 1:
    if( center_bin_r ){
      output.points[idx].histogram[ DIM_COLOR_1_3 +   7 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  16 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  25 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  34 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  43 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  52 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 +  85 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  94 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 103 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 112 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 121 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 130 ] += b_;
    }
    if( center_bin_g ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 163 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 172 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 181 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 190 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 199 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 208 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 241 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 250 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 259 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 268 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 277 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 286 ] += b_;
    }
    if( center_bin_b ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 319 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 328 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 337 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 346 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 355 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 364 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 397 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 406 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 415 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 424 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 433 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 442 ] += b_;
    }
    break;
  case 2:
    if( center_bin_r ){
      output.points[idx].histogram[ DIM_COLOR_1_3 +   8 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  17 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  26 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  35 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  44 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  53 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 +  86 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  95 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 104 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 113 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 122 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 131 ] += b_;
    }
    if( center_bin_g ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 164 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 173 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 182 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 191 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 200 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 209 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 242 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 251 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 260 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 269 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 278 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 287 ] += b_;
    }
    if( center_bin_b){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 320 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 329 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 338 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 347 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 356 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 365 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 398 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 407 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 416 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 425 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 434 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 443 ] += b_;
    }

    break;
  case 3:
    if( center_bin_r ){
      output.points[idx].histogram[ DIM_COLOR_1_3 +   9 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  18 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  27 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  36 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  45 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  54 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 +  87 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  96 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 105 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 114 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 123 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 132 ] += b_;
    }
    if( center_bin_g ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 165 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 174 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 183 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 192 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 201 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 210 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 243 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 252 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 261 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 270 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 279 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 288 ] += b_;
    }
    if( center_bin_b ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 321 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 330 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 339 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 348 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 357 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 366 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 399 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 408 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 417 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 426 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 435 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 444 ] += b_;
    }
    break;
  case 4:
    if( center_bin_r ){
      output.points[idx].histogram[ DIM_COLOR_1_3 +  10 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  19 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  28 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  37 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  46 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  55 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 +  88 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  97 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 106 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 115 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 124 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 133 ] += b_;
    }
    if( center_bin_g ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 166 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 175 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 184 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 193 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 202 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 211 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 244 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 253 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 262 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 271 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 280 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 289 ] += b_;
    }
    if( center_bin_b ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 322 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 331 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 340 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 349 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 358 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 367 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 400 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 409 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 418 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 427 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 436 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 445 ] += b_;
    }
    break;
  case 5:
    if( center_bin_r ){
      output.points[idx].histogram[ DIM_COLOR_1_3 +  11 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  20 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  29 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  38 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  47 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  56 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 +  89 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  98 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 107 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 116 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 125 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 134 ] += b_;
    }
    if( center_bin_g ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 167 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 176 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 185 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 194 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 203 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 212 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 245 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 254 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 263 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 272 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 281 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 290 ] += b_;
    }
    if( center_bin_b ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 323 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 332 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 341 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 350 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 359 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 368 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 401 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 410 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 419 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 428 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 437 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 446 ] += b_;
    }
    break;
  case 6:
    if( center_bin_r ){
      output.points[idx].histogram[ DIM_COLOR_1_3 +  12 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  21 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  30 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  39 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  48 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  57 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 +  90 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  99 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 108 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 117 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 126 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 135 ] += b_;
    }
    if( center_bin_g ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 168 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 177 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 186 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 195 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 204 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 213 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 246 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 255 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 264 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 273 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 282 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 291 ] += b_;
    }
    if( center_bin_b ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 324 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 333 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 342 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 351 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 360 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 369 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 402 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 411 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 420 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 429 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 438 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 447 ] += b_;
    }
    break;
  case 7:
    if( center_bin_r ){
      output.points[idx].histogram[ DIM_COLOR_1_3 +  13 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  22 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  31 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  40 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  49 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  58 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 +  91 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 100 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 109 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 118 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 127 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 136 ] += b_;
    }
    if( center_bin_g ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 169 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 178 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 187 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 196 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 205 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 214 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 247 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 256 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 265 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 274 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 283 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 292 ] += b_;
    }
    if( center_bin_b ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 325 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 334 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 343 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 352 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 361 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 370 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 403 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 412 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 421 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 430 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 439 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 448 ] += b_;
    }
    break;
  case 8:
    if( center_bin_r ){
      output.points[idx].histogram[ DIM_COLOR_1_3 +  14 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  23 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  32 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  41 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  50 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  59 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 +  92 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 101 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 110 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 119 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 128 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 137 ] += b_;
    }
    if( center_bin_g ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 170 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 179 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 188 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 197 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 206 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 215 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 248 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 257 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 266 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 275 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 284 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 293 ] += b_;
    }
    if( center_bin_b ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 326 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 335 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 344 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 353 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 362 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 371 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 404 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 413 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 422 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 431 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 440 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 449 ] += b_;
    }
    break;
  case 9:
    if( center_bin_r ){
      output.points[idx].histogram[ DIM_COLOR_1_3 +  60 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  64 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  68 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  72 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  76 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  80 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 138 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 142 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 146 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 150 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 154 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 158 ] += b_;
    }
    if( center_bin_g ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 216 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 220 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 224 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 228 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 232 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 236 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 294 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 298 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 302 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 306 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 310 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 314 ] += b_;
    }
    if( center_bin_b ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 372 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 376 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 380 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 384 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 388 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 392 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 450 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 454 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 458 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 462 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 466 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 470 ] += b_;
    }
    break;
  case 10:
    if( center_bin_r ){
      output.points[idx].histogram[ DIM_COLOR_1_3 +  61 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  65 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  69 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  73 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  77 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  81 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 139 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 143 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 147 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 151 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 155 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 159 ] += b_;
    }
    if( center_bin_g ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 217 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 221 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 225 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 229 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 233 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 237 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 295 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 299 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 303 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 307 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 311 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 315 ] += b_;
    }
    if( center_bin_b ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 373 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 377 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 381 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 385 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 389 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 393 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 451 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 455 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 459 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 463 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 467 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 471 ] += b_;
    }
    break;
  case 11:
    if( center_bin_r ){
      output.points[idx].histogram[ DIM_COLOR_1_3 +  62 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  66 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  70 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  74 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  78 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  82 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 140 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 144 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 148 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 152 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 156 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 160 ] += b_;
    }
    if( center_bin_g ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 218 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 222 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 226 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 230 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 234 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 238 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 296 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 300 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 304 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 308 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 312 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 316 ] += b_;
    }
    if( center_bin_b ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 374 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 378 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 382 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 386 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 390 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 394 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 452 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 456 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 460 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 464 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 468 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 472 ] += b_;
    }
    break;
  case 12:
    if( center_bin_r ){
      output.points[idx].histogram[ DIM_COLOR_1_3 +  63 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  67 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  71 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  75 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  79 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 +  83 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 141 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 145 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 149 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 153 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 157 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 161 ] += b_;
    }
    if( center_bin_g ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 219 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 223 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 227 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 231 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 235 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 239 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 297 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 301 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 305 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 309 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 313 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 317 ] += b_;
    }
    if( center_bin_b ){
      output.points[idx].histogram[ DIM_COLOR_1_3 + 375 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 379 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 383 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 387 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 391 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 395 ] += b_;
    }
    else{
      output.points[idx].histogram[ DIM_COLOR_1_3 + 453 ] += r;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 457 ] += r_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 461 ] += g;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 465 ] += g_;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 469 ] += b;
      output.points[idx].histogram[ DIM_COLOR_1_3 + 473 ] += b_;
    }
    break;
    //   case 13: // itself
    //     break;
    //   case 14: // 14 - 26: redundant
    //     break;
  default:
    break;
  }

}

template <typename PointT, typename PointOutT> inline void 
pcl::ColorCHLACEstimation<PointT, PointOutT>::computeColorCHLAC ( const pcl::PointCloud<PointT> &cloud,
					       PointCloudOut &output, const int center_idx )
{
  // calc hist_idx
  int hist_idx;
  if( hist_num == 1 ) hist_idx = 0;
  else{
    const int x_mul_y = div_b_[0] * div_b_[1];
    const int tmp_z = center_idx / x_mul_y;
    const int tmp_y = ( center_idx % x_mul_y ) / div_b_[0];
    const int tmp_x = center_idx % div_b_[0];
    Eigen3::Vector3i ijk = Eigen3::Vector3i ( floor ( tmp_x * inverse_subdivision_size), floor ( tmp_y * inverse_subdivision_size), floor ( tmp_z * inverse_subdivision_size) );
    hist_idx = ijk.dot (subdivb_mul_);
  }

  color = *reinterpret_cast<const int*>(&(cloud.points[center_idx].rgb));
  center_r = (0xff0000 & color) >> 16;
  center_g = (0x00ff00 & color) >> 8;
  center_b =  0x0000ff & color;
  center_r_ = reverse( center_r );
  center_g_ = reverse( center_g );
  center_b_ = reverse( center_b );
  center_bin_r = binarize_r( center_r );
  center_bin_g = binarize_g( center_g );
  center_bin_b = binarize_b( center_b );
  addColorCHLAC_0( hist_idx, output );
  addColorCHLAC_0_bin( hist_idx, output );

  const std::vector<int> neighbors = grid.getNeighborCentroidIndices (cloud.points[center_idx], relative_coordinates);

  for (int i = 0; i < 13; ++i){
    // Check if the point is invalid
    if ( neighbors[i]!=-1 ){
      color = *reinterpret_cast<const int*>(&(cloud.points[neighbors[i]].rgb));
      const int r = (0xff0000 & color) >> 16;
      const int g = (0x00ff00 & color) >> 8;
      const int b =  0x0000ff & color;

      addColorCHLAC_1 ( hist_idx, output, i, r, g, b );
      addColorCHLAC_1_bin ( hist_idx, output, i, binarize_r(r), binarize_g(g), binarize_b(b) );
    }
  }
}

template <typename PointT, typename PointOutT> inline void
pcl::ColorCHLACEstimation<PointT, PointOutT>::normalizeColorCHLAC ( PointCloudOut &output )
{
  for( int h=0; h<hist_num; h++ ){
    for(int i=0; i<6; ++i)
      output.points[h].histogram[ i ] *= NORMALIZE_0;
    for(int i=6; i<DIM_COLOR_1_3; ++i)
      output.points[h].histogram[ i ] *= NORMALIZE_1;
    // for(int i=DIM_COLOR_1_3; i<501; ++i)
    //   output.points[h].histogram[ i ] *= NORMALIZE_0_BIN;
    // for(int i=501; i<DIM_COLOR_1_3_ALL; ++i)
    //   output.points[h].histogram[ i ] *= NORMALIZE_1_BIN;
  }
}

//*************************************************************//
//* functions for ColorCHLACSignature117 (rotation-invariant) *//
//*************************************************************//

template <typename PointT, typename PointOutT> inline void 
pcl::ColorCHLAC_RI_Estimation<PointT, PointOutT>::addColorCHLAC_0 ( const int idx, PointCloudOut &output )
{
  output.points[idx].histogram[   0 ] += center_r;
  output.points[idx].histogram[   1 ] += center_r_;
  output.points[idx].histogram[   2 ] += center_g;
  output.points[idx].histogram[   3 ] += center_g_;
  output.points[idx].histogram[   4 ] += center_b;
  output.points[idx].histogram[   5 ] += center_b_;
  output.points[idx].histogram[  42 ] += center_r * center_r;
  output.points[idx].histogram[  43 ] += center_r * center_r_;
  output.points[idx].histogram[  44 ] += center_r * center_g;
  output.points[idx].histogram[  45 ] += center_r * center_g_;
  output.points[idx].histogram[  46 ] += center_r * center_b;
  output.points[idx].histogram[  47 ] += center_r * center_b_;
  output.points[idx].histogram[  48 ] += center_r_* center_r_;
  output.points[idx].histogram[  49 ] += center_r_* center_g;
  output.points[idx].histogram[  50 ] += center_r_* center_g_;
  output.points[idx].histogram[  51 ] += center_r_* center_b;
  output.points[idx].histogram[  52 ] += center_r_* center_b_;
  output.points[idx].histogram[  53 ] += center_g * center_g;
  output.points[idx].histogram[  54 ] += center_g * center_g_;
  output.points[idx].histogram[  55 ] += center_g * center_b;
  output.points[idx].histogram[  56 ] += center_g * center_b_;
  output.points[idx].histogram[  57 ] += center_g_* center_g_;
  output.points[idx].histogram[  58 ] += center_g_* center_b;
  output.points[idx].histogram[  59 ] += center_g_* center_b_;
  output.points[idx].histogram[  60 ] += center_b * center_b;
  output.points[idx].histogram[  61 ] += center_b * center_b_;
  output.points[idx].histogram[  62 ] += center_b_* center_b_;
}

template <typename PointT, typename PointOutT> inline void 
pcl::ColorCHLAC_RI_Estimation<PointT, PointOutT>::addColorCHLAC_0_bin ( const int idx, PointCloudOut &output )
{
  if( center_bin_r )
    output.points[idx].histogram[  63 ] ++;
  else
    output.points[idx].histogram[  64 ] ++;
  if( center_bin_g )
    output.points[idx].histogram[  65 ] ++;
  else
    output.points[idx].histogram[  66 ] ++;
  if( center_bin_b )
    output.points[idx].histogram[  67 ] ++;
  else
    output.points[idx].histogram[  68 ] ++;

  if( center_bin_r ){
    if( center_bin_g )
      output.points[idx].histogram[ 105 ] ++;
    else
      output.points[idx].histogram[ 106 ] ++;
    if( center_bin_b )
      output.points[idx].histogram[ 107 ] ++;
    else
      output.points[idx].histogram[ 108 ] ++;
  }
  else{
    if( center_bin_g )
      output.points[idx].histogram[ 109 ] ++;
    else
      output.points[idx].histogram[ 110 ] ++;
    if( center_bin_b )
      output.points[idx].histogram[ 111 ] ++;
    else
      output.points[idx].histogram[ 112 ] ++;
  }
  if( center_bin_g ){
    if( center_bin_b )
      output.points[idx].histogram[ 113 ] ++;
    else
      output.points[idx].histogram[ 114 ] ++;
  }
  else{
    if( center_bin_b )
      output.points[idx].histogram[ 115 ] ++;
    else
      output.points[idx].histogram[ 116 ] ++;
  }
}

template <typename PointT, typename PointOutT> inline void 
pcl::ColorCHLAC_RI_Estimation<PointT, PointOutT>::addColorCHLAC_1 ( const int idx, PointCloudOut &output, int neighbor_idx, int r, int g, int b )
{
  const int r_ = reverse( r );
  const int g_ = reverse( g );
  const int b_ = reverse( b );

  output.points[idx].histogram[   6 ] += center_r * r;
  output.points[idx].histogram[   7 ] += center_r * r_;
  output.points[idx].histogram[   8 ] += center_r * g;
  output.points[idx].histogram[   9 ] += center_r * g_;
  output.points[idx].histogram[  10 ] += center_r * b;
  output.points[idx].histogram[  11 ] += center_r * b_;
  output.points[idx].histogram[  12 ] += center_r_* r;
  output.points[idx].histogram[  13 ] += center_r_* r_;
  output.points[idx].histogram[  14 ] += center_r_* g;
  output.points[idx].histogram[  15 ] += center_r_* g_;
  output.points[idx].histogram[  16 ] += center_r_* b;
  output.points[idx].histogram[  17 ] += center_r_* b_;
  output.points[idx].histogram[  18 ] += center_g * r;
  output.points[idx].histogram[  19 ] += center_g * r_;
  output.points[idx].histogram[  20 ] += center_g * g;
  output.points[idx].histogram[  21 ] += center_g * g_;
  output.points[idx].histogram[  22 ] += center_g * b;
  output.points[idx].histogram[  23 ] += center_g * b_;
  output.points[idx].histogram[  24 ] += center_g_* r;
  output.points[idx].histogram[  25 ] += center_g_* r_;
  output.points[idx].histogram[  26 ] += center_g_* g;
  output.points[idx].histogram[  27 ] += center_g_* g_;
  output.points[idx].histogram[  28 ] += center_g_* b;
  output.points[idx].histogram[  29 ] += center_g_* b_;
  output.points[idx].histogram[  30 ] += center_b * r;
  output.points[idx].histogram[  31 ] += center_b * r_;
  output.points[idx].histogram[  32 ] += center_b * g;
  output.points[idx].histogram[  33 ] += center_b * g_;
  output.points[idx].histogram[  34 ] += center_b * b;
  output.points[idx].histogram[  35 ] += center_b * b_;
  output.points[idx].histogram[  36 ] += center_b_* r;
  output.points[idx].histogram[  37 ] += center_b_* r_;
  output.points[idx].histogram[  38 ] += center_b_* g;
  output.points[idx].histogram[  39 ] += center_b_* g_;
  output.points[idx].histogram[  40 ] += center_b_* b;
  output.points[idx].histogram[  41 ] += center_b_* b_;
}

template <typename PointT, typename PointOutT> inline void 
pcl::ColorCHLAC_RI_Estimation<PointT, PointOutT>::addColorCHLAC_1_bin ( const int idx, PointCloudOut &output, int neighbor_idx, int r, int g, int b )
{
  const int r_ = 1 - r;
  const int g_ = 1 - g;
  const int b_ = 1 - b;

  if( center_bin_r ){
    output.points[idx].histogram[  69 ] += r;
    output.points[idx].histogram[  70 ] += r_;
    output.points[idx].histogram[  71 ] += g;
    output.points[idx].histogram[  72 ] += g_;
    output.points[idx].histogram[  73 ] += b;
    output.points[idx].histogram[  74 ] += b_;
  }
  else{
    output.points[idx].histogram[  75 ] += r;
    output.points[idx].histogram[  76 ] += r_;
    output.points[idx].histogram[  77 ] += g;
    output.points[idx].histogram[  78 ] += g_;
    output.points[idx].histogram[  79 ] += b;
    output.points[idx].histogram[  80 ] += b_;
  }
  if( center_bin_g ){
    output.points[idx].histogram[  81 ] += r;
    output.points[idx].histogram[  82 ] += r_;
    output.points[idx].histogram[  83 ] += g;
    output.points[idx].histogram[  84 ] += g_;
    output.points[idx].histogram[  85 ] += b;
    output.points[idx].histogram[  86 ] += b_;
  }
  else{
    output.points[idx].histogram[  87 ] += r;
    output.points[idx].histogram[  88 ] += r_;
    output.points[idx].histogram[  89 ] += g;
    output.points[idx].histogram[  90 ] += g_;
    output.points[idx].histogram[  91 ] += b;
    output.points[idx].histogram[  92 ] += b_;
  }
  if( center_bin_b ){
    output.points[idx].histogram[  93 ] += r;
    output.points[idx].histogram[  94 ] += r_;
    output.points[idx].histogram[  95 ] += g;
    output.points[idx].histogram[  96 ] += g_;
    output.points[idx].histogram[  97 ] += b;
    output.points[idx].histogram[  98 ] += b_;
  }
  else{
    output.points[idx].histogram[  99 ] += r;
    output.points[idx].histogram[ 100 ] += r_;
    output.points[idx].histogram[ 101 ] += g;
    output.points[idx].histogram[ 102 ] += g_;
    output.points[idx].histogram[ 103 ] += b;
    output.points[idx].histogram[ 104 ] += b_;
  }
}

// template <typename PointT, typename PointOutT> inline void 
// pcl::ColorCHLAC_RI_Estimation<PointT, PointOutT>::computeColorCHLAC ( const pcl::PointCloud<PointT> &cloud,
// 					       PointCloudOut &output, const int center_idx )
// {
//   color = *reinterpret_cast<const int*>(&(cloud.points[center_idx].rgb));
//   center_r = (0xff0000 & color) >> 16;
//   center_g = (0x00ff00 & color) >> 8;
//   center_b =  0x0000ff & color;
//   center_r_ = reverse( center_r );
//   center_g_ = reverse( center_g );
//   center_b_ = reverse( center_b );
//   center_bin_r = binarize_r( center_r );
//   center_bin_g = binarize_g( center_g );
//   center_bin_b = binarize_b( center_b );
//   addColorCHLAC_RI_0( output );
//   addColorCHLAC_RI_0_bin( output );

//   const std::vector<int> neighbors = grid.getNeighborCentroidIndices (cloud.points[center_idx], relative_coordinates);

//   for (int i = 0; i < 13; ++i){
//     // Check if the point is invalid
//     if ( neighbors[i]!=-1 ){
//       color = *reinterpret_cast<const int*>(&(cloud.points[neighbors[i]].rgb));
//       const int r = (0xff0000 & color) >> 16;
//       const int g = (0x00ff00 & color) >> 8;
//       const int b =  0x0000ff & color;
    
//       addColorCHLAC_RI_1 ( output, i, r, g, b );
//       addColorCHLAC_RI_1_bin ( output, i, binarize_r(r), binarize_g(g), binarize_b(b) );
//     }
//   }
// }

template <typename PointT, typename PointOutT> inline void
pcl::ColorCHLAC_RI_Estimation<PointT, PointOutT>::normalizeColorCHLAC ( PointCloudOut &output )
{
  for( int h=0; h<hist_num; h++ ){
    for(int i=0; i<6; ++i)
      output.points[h].histogram[ i ] *= NORMALIZE_RI_0;
    for(int i=6; i<DIM_COLOR_RI_1_3; ++i)
      output.points[h].histogram[ i ] *= NORMALIZE_RI_1;
    // for(int i=DIM_COLOR_RI_1_3; i<501; ++i)
    //   output.points[h].histogram[ i ] *= NORMALIZE_RI_0_BIN;
    for(int i=69; i<DIM_COLOR_RI_1_3_ALL; ++i)
      output.points[h].histogram[ i ] *= NORMALIZE_RI_1_BIN;
  }
}

//***********************//
//* compute() function  *//
//***********************//
template <typename PointT, typename PointOutT> inline void
pcl::ColorCHLACEstimation<PointT, PointOutT>::computeFeature (PointCloudOut &output)
{
  if( (color_thR<0)||(color_thG<0)||(color_thB<0) ){
    std::cerr << "Invalid color_threshold: " << color_thR << " " << color_thG << " " << color_thB << std::endl;
    return;
  }

  output.points.resize (hist_num);
  output.width = hist_num;
  output.height = 1;

  // initialize histogram
  for( int h=0; h<hist_num; h++ )
    for( int t=0; t<DIM_COLOR_1_3_ALL; t++ )
      output.points[h].histogram[t] = 0;
  
  // Iterating over the entire index vector
  for (size_t idx = 0; idx < indices_->size (); ++idx)
    computeColorCHLAC (*surface_, output, (*indices_)[idx] );
  normalizeColorCHLAC( output );
}

template <typename PointT, typename PointOutT> inline void
pcl::ColorCHLAC_RI_Estimation<PointT, PointOutT>::computeFeature (PointCloudOut &output)
{
  if( (color_thR<0)||(color_thG<0)||(color_thB<0) ){
    std::cerr << "Invalid color_threshold: " << color_thR << " " << color_thG << " " << color_thB << std::endl;
    return;
  }

  output.points.resize (hist_num);
  output.width = hist_num;
  output.height = 1;

  // initialize histogram
  for( int h=0; h<hist_num; h++ )
    for( int t=0; t<DIM_COLOR_RI_1_3_ALL; t++ )
      output.points[h].histogram[t] = 0;
  
  // Iterating over the entire index vector
  for (size_t idx = 0; idx < indices_->size (); ++idx)
    computeColorCHLAC (*surface_, output, (*indices_)[idx] );
  normalizeColorCHLAC( output );
}
