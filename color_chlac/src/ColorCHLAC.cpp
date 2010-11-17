#include "color_chlac/ColorCHLAC.hpp"
#include <cmath>
#include <iostream>

using namespace std;

//********************************************
//* feature extraction (without RGB binalize)
void ColorCHLAC::extractColorCHLAC( std::vector<float> &result, ColorVoxel &voxel ){
  extractColorCHLAC( result, voxel.Vr(), voxel._Vr(), voxel.Vg(), voxel._Vg(), voxel.Vb(), voxel._Vb(), 1, 1, 1, voxel.Xsize()-1, voxel.Ysize()-1, voxel.Zsize()-1, 1, 1, 1, voxel.Xsize(), voxel.Ysize(), voxel.Zsize() );
}
void ColorCHLAC::extractColorCHLAC( std::vector<float> &result, ColorVoxel &voxel, int sx, int sy, int sz, int gx, int gy, int gz ){
  extractColorCHLAC( result, voxel.Vr(), voxel._Vr(), voxel.Vg(), voxel._Vg(), voxel.Vb(), voxel._Vb(), sx, sy, sz, gx, gy, gz, 1, 1, 1, voxel.Xsize(), voxel.Ysize(), voxel.Zsize() );
}

//********************************************
//* feature extraction (with RGB binalize)
void ColorCHLAC::extractColorCHLAC_bin( std::vector<float> &result, ColorVoxel &voxel ){
  extractColorCHLAC_bin( result, voxel.Vr(), voxel._Vr(), voxel.Vg(), voxel._Vg(), voxel.Vb(), voxel._Vb(), 1, 1, 1, voxel.Xsize()-1, voxel.Ysize()-1, voxel.Zsize()-1, 1, 1, 1, voxel.Xsize(), voxel.Ysize(), voxel.Zsize() );
}
void ColorCHLAC::extractColorCHLAC_bin( std::vector<float> &result, ColorVoxel &voxel, int sx, int sy, int sz, int gx, int gy, int gz ){
  extractColorCHLAC_bin( result, voxel.Vr(), voxel._Vr(), voxel.Vg(), voxel._Vg(), voxel.Vb(), voxel._Vb(), sx, sy, sz, gx, gy, gz, 1, 1, 1, voxel.Xsize(), voxel.Ysize(), voxel.Zsize() );
}

//*********************************************************************************
//* calculate a feature vector when the target voxel data is rotated by 90 degrees.
void ColorCHLAC::rotateFeature90( std::vector<float> &output, const std::vector<float> &input, RotateMode mode ){
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
      cerr << "ERR (in ColorCHLAC::rotateFeature90): unknown RotateMode." << endl;
      exit( EXIT_FAILURE );
      break;
    }
    break;
  default:
    cerr << "ERR (in ColorCHLAC::rotateFeature90): improper dimension: " << dim << endl;
    exit( EXIT_FAILURE );
    break;
  }
}

//*********************//
//* private functions *//
//*********************//

// r, g, b : 0 or 1 (binaly)
void ColorCHLAC::extractColorCHLAC_bin( std::vector<float> &result, const unsigned char *red, const unsigned char *nred, const unsigned char *green, const unsigned char *ngreen, const unsigned char *blue, const unsigned char *nblue, int sx, int sy, int sz, int gx, int gy, int gz, int rx, int ry, int rz, int xsize, int ysize, int zsize ){
  if( result.size() != (std::vector<float>::size_type)DIM_COLOR_BIN_1_3 ) result.resize( DIM_COLOR_BIN_1_3 );
  const int xysize = xsize * ysize;
  
  /*** initialize ***/
  int tmp[ DIM_COLOR_BIN_1_3 ];
  for(int i=0; i<DIM_COLOR_BIN_1_3; ++i) tmp[ i ] = 0;
  
  /*** feature extraction ***/
  for(int z=sz; z<gz; z++){
    for(int y=sy; y<gy; y++){
      for(int x=sx; x<gx; x++){
	if( red[ x + y*xsize + z*xysize ] || nred[ x + y*xsize + z*xysize ] ){
	  const int idx1  =      x     +     y     * xsize +     z     * xysize ;
	  const int idx2  =  ( x - rx ) + ( y - ry ) * xsize + ( z - rz ) * xysize ;
	  const int idx3  =      x      + ( y - ry ) * xsize + ( z - rz ) * xysize ;
	  const int idx4  =  ( x + rx ) + ( y - ry ) * xsize + ( z - rz ) * xysize ;
	  const int idx5  =  ( x - rx ) +     y      * xsize + ( z - rz ) * xysize ;
	  const int idx6  =      x      +     y      * xsize + ( z - rz ) * xysize ;
	  const int idx7  =  ( x + rx ) +     y      * xsize + ( z - rz ) * xysize ;
	  const int idx8  =  ( x - rx ) + ( y + ry ) * xsize + ( z - rz ) * xysize ;
	  const int idx9  =      x      + ( y + ry ) * xsize + ( z - rz ) * xysize ;
	  const int idx10 =  ( x + rx ) + ( y + ry ) * xsize + ( z - rz ) * xysize ;
	  const int idx11 =  ( x - rx ) + ( y - ry ) * xsize +     z      * xysize ;
	  const int idx12 =      x      + ( y - ry ) * xsize +     z      * xysize ;
	  const int idx13 =  ( x + rx ) + ( y - ry ) * xsize +     z      * xysize ;
	  const int idx14 =  ( x - rx ) +     y      * xsize +     z      * xysize ;

	  if( red[ idx1 ] ){
	    tmp[   0 ] ++;
	    tmp[ 474 ] += green[ idx1 ];
	    tmp[ 475 ] += ngreen[ idx1 ];
	    tmp[ 476 ] += blue[ idx1 ];
	    tmp[ 477 ] += nblue[ idx1 ];
	    tmp[   6 ] += red[ idx2 ];
	    tmp[   7 ] += red[ idx3 ];
	    tmp[   8 ] += red[ idx4 ];
	    tmp[   9 ] += red[ idx5 ];
	    tmp[  10 ] += red[ idx6 ];
	    tmp[  11 ] += red[ idx7 ];
	    tmp[  12 ] += red[ idx8 ];
	    tmp[  13 ] += red[ idx9 ];
	    tmp[  14 ] += red[ idx10 ];
	    tmp[  15 ] += nred[ idx2 ];
	    tmp[  16 ] += nred[ idx3 ];
	    tmp[  17 ] += nred[ idx4 ];
	    tmp[  18 ] += nred[ idx5 ];
	    tmp[  19 ] += nred[ idx6 ];
	    tmp[  20 ] += nred[ idx7 ];
	    tmp[  21 ] += nred[ idx8 ];
	    tmp[  22 ] += nred[ idx9 ];
	    tmp[  23 ] += nred[ idx10 ];
	    tmp[  24 ] += green[ idx2 ];
	    tmp[  25 ] += green[ idx3 ];
	    tmp[  26 ] += green[ idx4 ];
	    tmp[  27 ] += green[ idx5 ];
	    tmp[  28 ] += green[ idx6 ];
	    tmp[  29 ] += green[ idx7 ];
	    tmp[  30 ] += green[ idx8 ];
	    tmp[  31 ] += green[ idx9 ];
	    tmp[  32 ] += green[ idx10 ];
	    tmp[  33 ] += ngreen[ idx2 ];
	    tmp[  34 ] += ngreen[ idx3 ];
	    tmp[  35 ] += ngreen[ idx4 ];
	    tmp[  36 ] += ngreen[ idx5 ];
	    tmp[  37 ] += ngreen[ idx6 ];
	    tmp[  38 ] += ngreen[ idx7 ];
	    tmp[  39 ] += ngreen[ idx8 ];
	    tmp[  40 ] += ngreen[ idx9 ];
	    tmp[  41 ] += ngreen[ idx10 ];
	    tmp[  42 ] += blue[ idx2 ];
	    tmp[  43 ] += blue[ idx3 ];
	    tmp[  44 ] += blue[ idx4 ];
	    tmp[  45 ] += blue[ idx5 ];
	    tmp[  46 ] += blue[ idx6 ];
	    tmp[  47 ] += blue[ idx7 ];
	    tmp[  48 ] += blue[ idx8 ];
	    tmp[  49 ] += blue[ idx9 ];
	    tmp[  50 ] += blue[ idx10 ];
	    tmp[  51 ] += nblue[ idx2 ];
	    tmp[  52 ] += nblue[ idx3 ];
	    tmp[  53 ] += nblue[ idx4 ];
	    tmp[  54 ] += nblue[ idx5 ];
	    tmp[  55 ] += nblue[ idx6 ];
	    tmp[  56 ] += nblue[ idx7 ];
	    tmp[  57 ] += nblue[ idx8 ];
	    tmp[  58 ] += nblue[ idx9 ];
	    tmp[  59 ] += nblue[ idx10 ];

	    tmp[  60 ] += red[ idx11 ];
	    tmp[  61 ] += red[ idx12 ];
	    tmp[  62 ] += red[ idx13 ];
	    tmp[  63 ] += red[ idx14 ];
	    tmp[  64 ] += nred[ idx11 ];
	    tmp[  65 ] += nred[ idx12 ];
	    tmp[  66 ] += nred[ idx13 ];
	    tmp[  67 ] += nred[ idx14 ];
	    tmp[  68 ] += green[ idx11 ];
	    tmp[  69 ] += green[ idx12 ];
	    tmp[  70 ] += green[ idx13 ];
	    tmp[  71 ] += green[ idx14 ];
	    tmp[  72 ] += ngreen[ idx11 ];
	    tmp[  73 ] += ngreen[ idx12 ];
	    tmp[  74 ] += ngreen[ idx13 ];
	    tmp[  75 ] += ngreen[ idx14 ];
	    tmp[  76 ] += blue[ idx11 ];
	    tmp[  77 ] += blue[ idx12 ];
	    tmp[  78 ] += blue[ idx13 ];
	    tmp[  79 ] += blue[ idx14 ];
	    tmp[  80 ] += nblue[ idx11 ];
	    tmp[  81 ] += nblue[ idx12 ];
	    tmp[  82 ] += nblue[ idx13 ];
	    tmp[  83 ] += nblue[ idx14 ];
	  }
	  else if( nred[ idx1 ] ){
	    tmp[   1 ] ++;
	    tmp[ 478 ] += green[ idx1 ];
	    tmp[ 479 ] += ngreen[ idx1 ];
	    tmp[ 480 ] += blue[ idx1 ];
	    tmp[ 481 ] += nblue[ idx1 ];
	    tmp[  84 ] += red[ idx2 ];
	    tmp[  85 ] += red[ idx3 ];
	    tmp[  86 ] += red[ idx4 ];
	    tmp[  87 ] += red[ idx5 ];
	    tmp[  88 ] += red[ idx6 ];
	    tmp[  89 ] += red[ idx7 ];
	    tmp[  90 ] += red[ idx8 ];
	    tmp[  91 ] += red[ idx9 ];
	    tmp[  92 ] += red[ idx10 ];
	    tmp[  93 ] += nred[ idx2 ];
	    tmp[  94 ] += nred[ idx3 ];
	    tmp[  95 ] += nred[ idx4 ];
	    tmp[  96 ] += nred[ idx5 ];
	    tmp[  97 ] += nred[ idx6 ];
	    tmp[  98 ] += nred[ idx7 ];
	    tmp[  99 ] += nred[ idx8 ];
	    tmp[ 100 ] += nred[ idx9 ];
	    tmp[ 101 ] += nred[ idx10 ];
	    tmp[ 102 ] += green[ idx2 ];
	    tmp[ 103 ] += green[ idx3 ];
	    tmp[ 104 ] += green[ idx4 ];
	    tmp[ 105 ] += green[ idx5 ];
	    tmp[ 106 ] += green[ idx6 ];
	    tmp[ 107 ] += green[ idx7 ];
	    tmp[ 108 ] += green[ idx8 ];
	    tmp[ 109 ] += green[ idx9 ];
	    tmp[ 110 ] += green[ idx10 ];
	    tmp[ 111 ] += ngreen[ idx2 ];
	    tmp[ 112 ] += ngreen[ idx3 ];
	    tmp[ 113 ] += ngreen[ idx4 ];
	    tmp[ 114 ] += ngreen[ idx5 ];
	    tmp[ 115 ] += ngreen[ idx6 ];
	    tmp[ 116 ] += ngreen[ idx7 ];
	    tmp[ 117 ] += ngreen[ idx8 ];
	    tmp[ 118 ] += ngreen[ idx9 ];
	    tmp[ 119 ] += ngreen[ idx10 ];
	    tmp[ 120 ] += blue[ idx2 ];
	    tmp[ 121 ] += blue[ idx3 ];
	    tmp[ 122 ] += blue[ idx4 ];
	    tmp[ 123 ] += blue[ idx5 ];
	    tmp[ 124 ] += blue[ idx6 ];
	    tmp[ 125 ] += blue[ idx7 ];
	    tmp[ 126 ] += blue[ idx8 ];
	    tmp[ 127 ] += blue[ idx9 ];
	    tmp[ 128 ] += blue[ idx10 ];
	    tmp[ 129 ] += nblue[ idx2 ];
	    tmp[ 130 ] += nblue[ idx3 ];
	    tmp[ 131 ] += nblue[ idx4 ];
	    tmp[ 132 ] += nblue[ idx5 ];
	    tmp[ 133 ] += nblue[ idx6 ];
	    tmp[ 134 ] += nblue[ idx7 ];
	    tmp[ 135 ] += nblue[ idx8 ];
	    tmp[ 136 ] += nblue[ idx9 ];
	    tmp[ 137 ] += nblue[ idx10 ];

	    tmp[ 138 ] += red[ idx11 ];
	    tmp[ 139 ] += red[ idx12 ];
	    tmp[ 140 ] += red[ idx13 ];
	    tmp[ 141 ] += red[ idx14 ];
	    tmp[ 142 ] += nred[ idx11 ];
	    tmp[ 143 ] += nred[ idx12 ];
	    tmp[ 144 ] += nred[ idx13 ];
	    tmp[ 145 ] += nred[ idx14 ];
	    tmp[ 146 ] += green[ idx11 ];
	    tmp[ 147 ] += green[ idx12 ];
	    tmp[ 148 ] += green[ idx13 ];
	    tmp[ 149 ] += green[ idx14 ];
	    tmp[ 150 ] += ngreen[ idx11 ];
	    tmp[ 151 ] += ngreen[ idx12 ];
	    tmp[ 152 ] += ngreen[ idx13 ];
	    tmp[ 153 ] += ngreen[ idx14 ];
	    tmp[ 154 ] += blue[ idx11 ];
	    tmp[ 155 ] += blue[ idx12 ];
	    tmp[ 156 ] += blue[ idx13 ];
	    tmp[ 157 ] += blue[ idx14 ];
	    tmp[ 158 ] += nblue[ idx11 ];
	    tmp[ 159 ] += nblue[ idx12 ];
	    tmp[ 160 ] += nblue[ idx13 ];
	    tmp[ 161 ] += nblue[ idx14 ];
	  }
	  if( green[ idx1 ] ){
	    tmp[   2 ] ++;
	    tmp[ 482 ] += blue[ idx1 ];
	    tmp[ 483 ] += nblue[ idx1 ];
	    tmp[ 162 ] += red[ idx2 ];
	    tmp[ 163 ] += red[ idx3 ];
	    tmp[ 164 ] += red[ idx4 ];
	    tmp[ 165 ] += red[ idx5 ];
	    tmp[ 166 ] += red[ idx6 ];
	    tmp[ 167 ] += red[ idx7 ];
	    tmp[ 168 ] += red[ idx8 ];
	    tmp[ 169 ] += red[ idx9 ];
	    tmp[ 170 ] += red[ idx10 ];
	    tmp[ 171 ] += nred[ idx2 ];
	    tmp[ 172 ] += nred[ idx3 ];
	    tmp[ 173 ] += nred[ idx4 ];
	    tmp[ 174 ] += nred[ idx5 ];
	    tmp[ 175 ] += nred[ idx6 ];
	    tmp[ 176 ] += nred[ idx7 ];
	    tmp[ 177 ] += nred[ idx8 ];
	    tmp[ 178 ] += nred[ idx9 ];
	    tmp[ 179 ] += nred[ idx10 ];
	    tmp[ 180 ] += green[ idx2 ];
	    tmp[ 181 ] += green[ idx3 ];
	    tmp[ 182 ] += green[ idx4 ];
	    tmp[ 183 ] += green[ idx5 ];
	    tmp[ 184 ] += green[ idx6 ];
	    tmp[ 185 ] += green[ idx7 ];
	    tmp[ 186 ] += green[ idx8 ];
	    tmp[ 187 ] += green[ idx9 ];
	    tmp[ 188 ] += green[ idx10 ];
	    tmp[ 189 ] += ngreen[ idx2 ];
	    tmp[ 190 ] += ngreen[ idx3 ];
	    tmp[ 191 ] += ngreen[ idx4 ];
	    tmp[ 192 ] += ngreen[ idx5 ];
	    tmp[ 193 ] += ngreen[ idx6 ];
	    tmp[ 194 ] += ngreen[ idx7 ];
	    tmp[ 195 ] += ngreen[ idx8 ];
	    tmp[ 196 ] += ngreen[ idx9 ];
	    tmp[ 197 ] += ngreen[ idx10 ];
	    tmp[ 198 ] += blue[ idx2 ];
	    tmp[ 199 ] += blue[ idx3 ];
	    tmp[ 200 ] += blue[ idx4 ];
	    tmp[ 201 ] += blue[ idx5 ];
	    tmp[ 202 ] += blue[ idx6 ];
	    tmp[ 203 ] += blue[ idx7 ];
	    tmp[ 204 ] += blue[ idx8 ];
	    tmp[ 205 ] += blue[ idx9 ];
	    tmp[ 206 ] += blue[ idx10 ];
	    tmp[ 207 ] += nblue[ idx2 ];
	    tmp[ 208 ] += nblue[ idx3 ];
	    tmp[ 209 ] += nblue[ idx4 ];
	    tmp[ 210 ] += nblue[ idx5 ];
	    tmp[ 211 ] += nblue[ idx6 ];
	    tmp[ 212 ] += nblue[ idx7 ];
	    tmp[ 213 ] += nblue[ idx8 ];
	    tmp[ 214 ] += nblue[ idx9 ];
	    tmp[ 215 ] += nblue[ idx10 ];

	    tmp[ 216 ] += red[ idx11 ];
	    tmp[ 217 ] += red[ idx12 ];
	    tmp[ 218 ] += red[ idx13 ];
	    tmp[ 219 ] += red[ idx14 ];
	    tmp[ 220 ] += nred[ idx11 ];
	    tmp[ 221 ] += nred[ idx12 ];
	    tmp[ 222 ] += nred[ idx13 ];
	    tmp[ 223 ] += nred[ idx14 ];
	    tmp[ 224 ] += green[ idx11 ];
	    tmp[ 225 ] += green[ idx12 ];
	    tmp[ 226 ] += green[ idx13 ];
	    tmp[ 227 ] += green[ idx14 ];
	    tmp[ 228 ] += ngreen[ idx11 ];
	    tmp[ 229 ] += ngreen[ idx12 ];
	    tmp[ 230 ] += ngreen[ idx13 ];
	    tmp[ 231 ] += ngreen[ idx14 ];
	    tmp[ 232 ] += blue[ idx11 ];
	    tmp[ 233 ] += blue[ idx12 ];
	    tmp[ 234 ] += blue[ idx13 ];
	    tmp[ 235 ] += blue[ idx14 ];
	    tmp[ 236 ] += nblue[ idx11 ];
	    tmp[ 237 ] += nblue[ idx12 ];
	    tmp[ 238 ] += nblue[ idx13 ];
	    tmp[ 239 ] += nblue[ idx14 ];
	  }
	  else if( ngreen[ idx1 ] ){
	    tmp[   3 ] ++;
	    tmp[ 484 ] += blue[ idx1 ];
	    tmp[ 485 ] += nblue[ idx1 ];
	    tmp[ 240 ] += red[ idx2 ];
	    tmp[ 241 ] += red[ idx3 ];
	    tmp[ 242 ] += red[ idx4 ];
	    tmp[ 243 ] += red[ idx5 ];
	    tmp[ 244 ] += red[ idx6 ];
	    tmp[ 245 ] += red[ idx7 ];
	    tmp[ 246 ] += red[ idx8 ];
	    tmp[ 247 ] += red[ idx9 ];
	    tmp[ 248 ] += red[ idx10 ];
	    tmp[ 249 ] += nred[ idx2 ];
	    tmp[ 250 ] += nred[ idx3 ];
	    tmp[ 251 ] += nred[ idx4 ];
	    tmp[ 252 ] += nred[ idx5 ];
	    tmp[ 253 ] += nred[ idx6 ];
	    tmp[ 254 ] += nred[ idx7 ];
	    tmp[ 255 ] += nred[ idx8 ];
	    tmp[ 256 ] += nred[ idx9 ];
	    tmp[ 257 ] += nred[ idx10 ];
	    tmp[ 258 ] += green[ idx2 ];
	    tmp[ 259 ] += green[ idx3 ];
	    tmp[ 260 ] += green[ idx4 ];
	    tmp[ 261 ] += green[ idx5 ];
	    tmp[ 262 ] += green[ idx6 ];
	    tmp[ 263 ] += green[ idx7 ];
	    tmp[ 264 ] += green[ idx8 ];
	    tmp[ 265 ] += green[ idx9 ];
	    tmp[ 266 ] += green[ idx10 ];
	    tmp[ 267 ] += ngreen[ idx2 ];
	    tmp[ 268 ] += ngreen[ idx3 ];
	    tmp[ 269 ] += ngreen[ idx4 ];
	    tmp[ 270 ] += ngreen[ idx5 ];
	    tmp[ 271 ] += ngreen[ idx6 ];
	    tmp[ 272 ] += ngreen[ idx7 ];
	    tmp[ 273 ] += ngreen[ idx8 ];
	    tmp[ 274 ] += ngreen[ idx9 ];
	    tmp[ 275 ] += ngreen[ idx10 ];
	    tmp[ 276 ] += blue[ idx2 ];
	    tmp[ 277 ] += blue[ idx3 ];
	    tmp[ 278 ] += blue[ idx4 ];
	    tmp[ 279 ] += blue[ idx5 ];
	    tmp[ 280 ] += blue[ idx6 ];
	    tmp[ 281 ] += blue[ idx7 ];
	    tmp[ 282 ] += blue[ idx8 ];
	    tmp[ 283 ] += blue[ idx9 ];
	    tmp[ 284 ] += blue[ idx10 ];
	    tmp[ 285 ] += nblue[ idx2 ];
	    tmp[ 286 ] += nblue[ idx3 ];
	    tmp[ 287 ] += nblue[ idx4 ];
	    tmp[ 288 ] += nblue[ idx5 ];
	    tmp[ 289 ] += nblue[ idx6 ];
	    tmp[ 290 ] += nblue[ idx7 ];
	    tmp[ 291 ] += nblue[ idx8 ];
	    tmp[ 292 ] += nblue[ idx9 ];
	    tmp[ 293 ] += nblue[ idx10 ];

	    tmp[ 294 ] += red[ idx11 ];
	    tmp[ 295 ] += red[ idx12 ];
	    tmp[ 296 ] += red[ idx13 ];
	    tmp[ 297 ] += red[ idx14 ];
	    tmp[ 298 ] += nred[ idx11 ];
	    tmp[ 299 ] += nred[ idx12 ];
	    tmp[ 300 ] += nred[ idx13 ];
	    tmp[ 301 ] += nred[ idx14 ];
	    tmp[ 302 ] += green[ idx11 ];
	    tmp[ 303 ] += green[ idx12 ];
	    tmp[ 304 ] += green[ idx13 ];
	    tmp[ 305 ] += green[ idx14 ];
	    tmp[ 306 ] += ngreen[ idx11 ];
	    tmp[ 307 ] += ngreen[ idx12 ];
	    tmp[ 308 ] += ngreen[ idx13 ];
	    tmp[ 309 ] += ngreen[ idx14 ];
	    tmp[ 310 ] += blue[ idx11 ];
	    tmp[ 311 ] += blue[ idx12 ];
	    tmp[ 312 ] += blue[ idx13 ];
	    tmp[ 313 ] += blue[ idx14 ];
	    tmp[ 314 ] += nblue[ idx11 ];
	    tmp[ 315 ] += nblue[ idx12 ];
	    tmp[ 316 ] += nblue[ idx13 ];
	    tmp[ 317 ] += nblue[ idx14 ];
	  }
	  if( blue[ idx1 ] ){
	    tmp[   4 ] ++;
	    tmp[ 318 ] += red[ idx2 ];
	    tmp[ 319 ] += red[ idx3 ];
	    tmp[ 320 ] += red[ idx4 ];
	    tmp[ 321 ] += red[ idx5 ];
	    tmp[ 322 ] += red[ idx6 ];
	    tmp[ 323 ] += red[ idx7 ];
	    tmp[ 324 ] += red[ idx8 ];
	    tmp[ 325 ] += red[ idx9 ];
	    tmp[ 326 ] += red[ idx10 ];
	    tmp[ 327 ] += nred[ idx2 ];
	    tmp[ 328 ] += nred[ idx3 ];
	    tmp[ 329 ] += nred[ idx4 ];
	    tmp[ 330 ] += nred[ idx5 ];
	    tmp[ 331 ] += nred[ idx6 ];
	    tmp[ 332 ] += nred[ idx7 ];
	    tmp[ 333 ] += nred[ idx8 ];
	    tmp[ 334 ] += nred[ idx9 ];
	    tmp[ 335 ] += nred[ idx10 ];
	    tmp[ 336 ] += green[ idx2 ];
	    tmp[ 337 ] += green[ idx3 ];
	    tmp[ 338 ] += green[ idx4 ];
	    tmp[ 339 ] += green[ idx5 ];
	    tmp[ 340 ] += green[ idx6 ];
	    tmp[ 341 ] += green[ idx7 ];
	    tmp[ 342 ] += green[ idx8 ];
	    tmp[ 343 ] += green[ idx9 ];
	    tmp[ 344 ] += green[ idx10 ];
	    tmp[ 345 ] += ngreen[ idx2 ];
	    tmp[ 346 ] += ngreen[ idx3 ];
	    tmp[ 347 ] += ngreen[ idx4 ];
	    tmp[ 348 ] += ngreen[ idx5 ];
	    tmp[ 349 ] += ngreen[ idx6 ];
	    tmp[ 350 ] += ngreen[ idx7 ];
	    tmp[ 351 ] += ngreen[ idx8 ];
	    tmp[ 352 ] += ngreen[ idx9 ];
	    tmp[ 353 ] += ngreen[ idx10 ];
	    tmp[ 354 ] += blue[ idx2 ];
	    tmp[ 355 ] += blue[ idx3 ];
	    tmp[ 356 ] += blue[ idx4 ];
	    tmp[ 357 ] += blue[ idx5 ];
	    tmp[ 358 ] += blue[ idx6 ];
	    tmp[ 359 ] += blue[ idx7 ];
	    tmp[ 360 ] += blue[ idx8 ];
	    tmp[ 361 ] += blue[ idx9 ];
	    tmp[ 362 ] += blue[ idx10 ];
	    tmp[ 363 ] += nblue[ idx2 ];
	    tmp[ 364 ] += nblue[ idx3 ];
	    tmp[ 365 ] += nblue[ idx4 ];
	    tmp[ 366 ] += nblue[ idx5 ];
	    tmp[ 367 ] += nblue[ idx6 ];
	    tmp[ 368 ] += nblue[ idx7 ];
	    tmp[ 369 ] += nblue[ idx8 ];
	    tmp[ 370 ] += nblue[ idx9 ];
	    tmp[ 371 ] += nblue[ idx10 ];

	    tmp[ 372 ] += red[ idx11 ];
	    tmp[ 373 ] += red[ idx12 ];
	    tmp[ 374 ] += red[ idx13 ];
	    tmp[ 375 ] += red[ idx14 ];
	    tmp[ 376 ] += nred[ idx11 ];
	    tmp[ 377 ] += nred[ idx12 ];
	    tmp[ 378 ] += nred[ idx13 ];
	    tmp[ 379 ] += nred[ idx14 ];
	    tmp[ 380 ] += green[ idx11 ];
	    tmp[ 381 ] += green[ idx12 ];
	    tmp[ 382 ] += green[ idx13 ];
	    tmp[ 383 ] += green[ idx14 ];
	    tmp[ 384 ] += ngreen[ idx11 ];
	    tmp[ 385 ] += ngreen[ idx12 ];
	    tmp[ 386 ] += ngreen[ idx13 ];
	    tmp[ 387 ] += ngreen[ idx14 ];
	    tmp[ 388 ] += blue[ idx11 ];
	    tmp[ 389 ] += blue[ idx12 ];
	    tmp[ 390 ] += blue[ idx13 ];
	    tmp[ 391 ] += blue[ idx14 ];
	    tmp[ 392 ] += nblue[ idx11 ];
	    tmp[ 393 ] += nblue[ idx12 ];
	    tmp[ 394 ] += nblue[ idx13 ];
	    tmp[ 395 ] += nblue[ idx14 ];
	  }
	  else if( nblue[ idx1 ] ){
	    tmp[   5 ] ++;
	    tmp[ 396 ] += red[ idx2 ];
	    tmp[ 397 ] += red[ idx3 ];
	    tmp[ 398 ] += red[ idx4 ];
	    tmp[ 399 ] += red[ idx5 ];
	    tmp[ 400 ] += red[ idx6 ];
	    tmp[ 401 ] += red[ idx7 ];
	    tmp[ 402 ] += red[ idx8 ];
	    tmp[ 403 ] += red[ idx9 ];
	    tmp[ 404 ] += red[ idx10 ];
	    tmp[ 405 ] += nred[ idx2 ];
	    tmp[ 406 ] += nred[ idx3 ];
	    tmp[ 407 ] += nred[ idx4 ];
	    tmp[ 408 ] += nred[ idx5 ];
	    tmp[ 409 ] += nred[ idx6 ];
	    tmp[ 410 ] += nred[ idx7 ];
	    tmp[ 411 ] += nred[ idx8 ];
	    tmp[ 412 ] += nred[ idx9 ];
	    tmp[ 413 ] += nred[ idx10 ];
	    tmp[ 414 ] += green[ idx2 ];
	    tmp[ 415 ] += green[ idx3 ];
	    tmp[ 416 ] += green[ idx4 ];
	    tmp[ 417 ] += green[ idx5 ];
	    tmp[ 418 ] += green[ idx6 ];
	    tmp[ 419 ] += green[ idx7 ];
	    tmp[ 420 ] += green[ idx8 ];
	    tmp[ 421 ] += green[ idx9 ];
	    tmp[ 422 ] += green[ idx10 ];
	    tmp[ 423 ] += ngreen[ idx2 ];
	    tmp[ 424 ] += ngreen[ idx3 ];
	    tmp[ 425 ] += ngreen[ idx4 ];
	    tmp[ 426 ] += ngreen[ idx5 ];
	    tmp[ 427 ] += ngreen[ idx6 ];
	    tmp[ 428 ] += ngreen[ idx7 ];
	    tmp[ 429 ] += ngreen[ idx8 ];
	    tmp[ 430 ] += ngreen[ idx9 ];
	    tmp[ 431 ] += ngreen[ idx10 ];
	    tmp[ 432 ] += blue[ idx2 ];
	    tmp[ 433 ] += blue[ idx3 ];
	    tmp[ 434 ] += blue[ idx4 ];
	    tmp[ 435 ] += blue[ idx5 ];
	    tmp[ 436 ] += blue[ idx6 ];
	    tmp[ 437 ] += blue[ idx7 ];
	    tmp[ 438 ] += blue[ idx8 ];
	    tmp[ 439 ] += blue[ idx9 ];
	    tmp[ 440 ] += blue[ idx10 ];
	    tmp[ 441 ] += nblue[ idx2 ];
	    tmp[ 442 ] += nblue[ idx3 ];
	    tmp[ 443 ] += nblue[ idx4 ];
	    tmp[ 444 ] += nblue[ idx5 ];
	    tmp[ 445 ] += nblue[ idx6 ];
	    tmp[ 446 ] += nblue[ idx7 ];
	    tmp[ 447 ] += nblue[ idx8 ];
	    tmp[ 448 ] += nblue[ idx9 ];
	    tmp[ 449 ] += nblue[ idx10 ];

	    tmp[ 450 ] += red[ idx11 ];
	    tmp[ 451 ] += red[ idx12 ];
	    tmp[ 452 ] += red[ idx13 ];
	    tmp[ 453 ] += red[ idx14 ];
	    tmp[ 454 ] += nred[ idx11 ];
	    tmp[ 455 ] += nred[ idx12 ];
	    tmp[ 456 ] += nred[ idx13 ];
	    tmp[ 457 ] += nred[ idx14 ];
	    tmp[ 458 ] += green[ idx11 ];
	    tmp[ 459 ] += green[ idx12 ];
	    tmp[ 460 ] += green[ idx13 ];
	    tmp[ 461 ] += green[ idx14 ];
	    tmp[ 462 ] += ngreen[ idx11 ];
	    tmp[ 463 ] += ngreen[ idx12 ];
	    tmp[ 464 ] += ngreen[ idx13 ];
	    tmp[ 465 ] += ngreen[ idx14 ];
	    tmp[ 466 ] += blue[ idx11 ];
	    tmp[ 467 ] += blue[ idx12 ];
	    tmp[ 468 ] += blue[ idx13 ];
	    tmp[ 469 ] += blue[ idx14 ];
	    tmp[ 470 ] += nblue[ idx11 ];
	    tmp[ 471 ] += nblue[ idx12 ];
	    tmp[ 472 ] += nblue[ idx13 ];
	    tmp[ 473 ] += nblue[ idx14 ];
	  }
	}
      }
    }
  }

  for(int i=0; i<6; ++i) result[ i ] = (double)tmp[ i ] / 3.0;
  for(int i=6; i<DIM_COLOR_BIN_1_3; ++i) result[ i ] = (double)tmp[ i ] / 9.0;
}

// r, g, b : 0 ~ 255 (continuous)
void ColorCHLAC::extractColorCHLAC( std::vector<float> &result, const unsigned char *red, const unsigned char *nred, const unsigned char *green, const unsigned char *ngreen, const unsigned char *blue, const unsigned char *nblue, int sx, int sy, int sz, int gx, int gy, int gz, int rx, int ry, int rz, int xsize, int ysize, int zsize){
  if( result.size() != (std::vector<float>::size_type)DIM_COLOR_1_3 ) result.resize( DIM_COLOR_1_3 );
  int xysize = xsize * ysize;
  
  /*** initialize ***/
  double tmp[ DIM_COLOR_1_3 ];
  for(int i=0; i<DIM_COLOR_1_3; ++i) tmp[ i ] = 0.0;
  
  /*** feature extraction ***/
  for(int z=sz; z<gz; z++){
    for(int y=sy; y<gy; y++){
      for(int x=sx; x<gx; x++){
	if(red[ x + y*xsize + z*xysize ]||nred[ x + y*xsize + z*xysize ]){
	  const int idx1  =      x     +     y     * xsize +     z     * xysize ;
	  const int idx2  =  ( x - rx ) + ( y - ry ) * xsize + ( z - rz ) * xysize ;
	  const int idx3  =      x      + ( y - ry ) * xsize + ( z - rz ) * xysize ;
	  const int idx4  =  ( x + rx ) + ( y - ry ) * xsize + ( z - rz ) * xysize ;
	  const int idx5  =  ( x - rx ) +     y      * xsize + ( z - rz ) * xysize ;
	  const int idx6  =      x      +     y      * xsize + ( z - rz ) * xysize ;
	  const int idx7  =  ( x + rx ) +     y      * xsize + ( z - rz ) * xysize ;
	  const int idx8  =  ( x - rx ) + ( y + ry ) * xsize + ( z - rz ) * xysize ;
	  const int idx9  =      x      + ( y + ry ) * xsize + ( z - rz ) * xysize ;
	  const int idx10 =  ( x + rx ) + ( y + ry ) * xsize + ( z - rz ) * xysize ;
	  const int idx11 =  ( x - rx ) + ( y - ry ) * xsize +     z      * xysize ;
	  const int idx12 =      x      + ( y - ry ) * xsize +     z      * xysize ;
	  const int idx13 =  ( x + rx ) + ( y - ry ) * xsize +     z      * xysize ;
	  const int idx14 =  ( x - rx ) +     y      * xsize +     z      * xysize ;
	  const double a  =    red[ idx1 ];
	  const double a_ =   nred[ idx1 ];
	  const double b  =  green[ idx1 ];
	  const double b_ = ngreen[ idx1 ];
	  const double c  =   blue[ idx1 ];
	  const double c_ =  nblue[ idx1 ];

	  tmp[   0 ] += a;
	  tmp[   1 ] += a_;
	  tmp[   2 ] += b;
	  tmp[   3 ] += b_;
	  tmp[   4 ] += c;
	  tmp[   5 ] += c_;
	  tmp[   6 ] += a * red[ idx2 ];
	  tmp[   7 ] += a * red[ idx3 ];
	  tmp[   8 ] += a * red[ idx4 ];
	  tmp[   9 ] += a * red[ idx5 ];
	  tmp[  10 ] += a * red[ idx6 ];
	  tmp[  11 ] += a * red[ idx7 ];
	  tmp[  12 ] += a * red[ idx8 ];
	  tmp[  13 ] += a * red[ idx9 ];
	  tmp[  14 ] += a * red[ idx10 ];
	  tmp[  15 ] += a * nred[ idx2 ];
	  tmp[  16 ] += a * nred[ idx3 ];
	  tmp[  17 ] += a * nred[ idx4 ];
	  tmp[  18 ] += a * nred[ idx5 ];
	  tmp[  19 ] += a * nred[ idx6 ];
	  tmp[  20 ] += a * nred[ idx7 ];
	  tmp[  21 ] += a * nred[ idx8 ];
	  tmp[  22 ] += a * nred[ idx9 ];
	  tmp[  23 ] += a * nred[ idx10 ];
	  tmp[  24 ] += a * green[ idx2 ];
	  tmp[  25 ] += a * green[ idx3 ];
	  tmp[  26 ] += a * green[ idx4 ];
	  tmp[  27 ] += a * green[ idx5 ];
	  tmp[  28 ] += a * green[ idx6 ];
	  tmp[  29 ] += a * green[ idx7 ];
	  tmp[  30 ] += a * green[ idx8 ];
	  tmp[  31 ] += a * green[ idx9 ];
	  tmp[  32 ] += a * green[ idx10 ];
	  tmp[  33 ] += a * ngreen[ idx2 ];
	  tmp[  34 ] += a * ngreen[ idx3 ];
	  tmp[  35 ] += a * ngreen[ idx4 ];
	  tmp[  36 ] += a * ngreen[ idx5 ];
	  tmp[  37 ] += a * ngreen[ idx6 ];
	  tmp[  38 ] += a * ngreen[ idx7 ];
	  tmp[  39 ] += a * ngreen[ idx8 ];
	  tmp[  40 ] += a * ngreen[ idx9 ];
	  tmp[  41 ] += a * ngreen[ idx10 ];
	  tmp[  42 ] += a * blue[ idx2 ];
	  tmp[  43 ] += a * blue[ idx3 ];
	  tmp[  44 ] += a * blue[ idx4 ];
	  tmp[  45 ] += a * blue[ idx5 ];
	  tmp[  46 ] += a * blue[ idx6 ];
	  tmp[  47 ] += a * blue[ idx7 ];
	  tmp[  48 ] += a * blue[ idx8 ];
	  tmp[  49 ] += a * blue[ idx9 ];
	  tmp[  50 ] += a * blue[ idx10 ];
	  tmp[  51 ] += a * nblue[ idx2 ];
	  tmp[  52 ] += a * nblue[ idx3 ];
	  tmp[  53 ] += a * nblue[ idx4 ];
	  tmp[  54 ] += a * nblue[ idx5 ];
	  tmp[  55 ] += a * nblue[ idx6 ];
	  tmp[  56 ] += a * nblue[ idx7 ];
	  tmp[  57 ] += a * nblue[ idx8 ];
	  tmp[  58 ] += a * nblue[ idx9 ];
	  tmp[  59 ] += a * nblue[ idx10 ];

	  tmp[  60 ] += a * red[ idx11 ];
	  tmp[  61 ] += a * red[ idx12 ];
	  tmp[  62 ] += a * red[ idx13 ];
	  tmp[  63 ] += a * red[ idx14 ];
	  tmp[  64 ] += a * nred[ idx11 ];
	  tmp[  65 ] += a * nred[ idx12 ];
	  tmp[  66 ] += a * nred[ idx13 ];
	  tmp[  67 ] += a * nred[ idx14 ];
	  tmp[  68 ] += a * green[ idx11 ];
	  tmp[  69 ] += a * green[ idx12 ];
	  tmp[  70 ] += a * green[ idx13 ];
	  tmp[  71 ] += a * green[ idx14 ];
	  tmp[  72 ] += a * ngreen[ idx11 ];
	  tmp[  73 ] += a * ngreen[ idx12 ];
	  tmp[  74 ] += a * ngreen[ idx13 ];
	  tmp[  75 ] += a * ngreen[ idx14 ];
	  tmp[  76 ] += a * blue[ idx11 ];
	  tmp[  77 ] += a * blue[ idx12 ];
	  tmp[  78 ] += a * blue[ idx13 ];
	  tmp[  79 ] += a * blue[ idx14 ];
	  tmp[  80 ] += a * nblue[ idx11 ];
	  tmp[  81 ] += a * nblue[ idx12 ];
	  tmp[  82 ] += a * nblue[ idx13 ];
	  tmp[  83 ] += a * nblue[ idx14 ];

	  tmp[  84 ] += a_ * red[ idx2 ];
	  tmp[  85 ] += a_ * red[ idx3 ];
	  tmp[  86 ] += a_ * red[ idx4 ];
	  tmp[  87 ] += a_ * red[ idx5 ];
	  tmp[  88 ] += a_ * red[ idx6 ];
	  tmp[  89 ] += a_ * red[ idx7 ];
	  tmp[  90 ] += a_ * red[ idx8 ];
	  tmp[  91 ] += a_ * red[ idx9 ];
	  tmp[  92 ] += a_ * red[ idx10 ];
	  tmp[  93 ] += a_ * nred[ idx2 ];
	  tmp[  94 ] += a_ * nred[ idx3 ];
	  tmp[  95 ] += a_ * nred[ idx4 ];
	  tmp[  96 ] += a_ * nred[ idx5 ];
	  tmp[  97 ] += a_ * nred[ idx6 ];
	  tmp[  98 ] += a_ * nred[ idx7 ];
	  tmp[  99 ] += a_ * nred[ idx8 ];
	  tmp[ 100 ] += a_ * nred[ idx9 ];
	  tmp[ 101 ] += a_ * nred[ idx10 ];
	  tmp[ 102 ] += a_ * green[ idx2 ];
	  tmp[ 103 ] += a_ * green[ idx3 ];
	  tmp[ 104 ] += a_ * green[ idx4 ];
	  tmp[ 105 ] += a_ * green[ idx5 ];
	  tmp[ 106 ] += a_ * green[ idx6 ];
	  tmp[ 107 ] += a_ * green[ idx7 ];
	  tmp[ 108 ] += a_ * green[ idx8 ];
	  tmp[ 109 ] += a_ * green[ idx9 ];
	  tmp[ 110 ] += a_ * green[ idx10 ];
	  tmp[ 111 ] += a_ * ngreen[ idx2 ];
	  tmp[ 112 ] += a_ * ngreen[ idx3 ];
	  tmp[ 113 ] += a_ * ngreen[ idx4 ];
	  tmp[ 114 ] += a_ * ngreen[ idx5 ];
	  tmp[ 115 ] += a_ * ngreen[ idx6 ];
	  tmp[ 116 ] += a_ * ngreen[ idx7 ];
	  tmp[ 117 ] += a_ * ngreen[ idx8 ];
	  tmp[ 118 ] += a_ * ngreen[ idx9 ];
	  tmp[ 119 ] += a_ * ngreen[ idx10 ];
	  tmp[ 120 ] += a_ * blue[ idx2 ];
	  tmp[ 121 ] += a_ * blue[ idx3 ];
	  tmp[ 122 ] += a_ * blue[ idx4 ];
	  tmp[ 123 ] += a_ * blue[ idx5 ];
	  tmp[ 124 ] += a_ * blue[ idx6 ];
	  tmp[ 125 ] += a_ * blue[ idx7 ];
	  tmp[ 126 ] += a_ * blue[ idx8 ];
	  tmp[ 127 ] += a_ * blue[ idx9 ];
	  tmp[ 128 ] += a_ * blue[ idx10 ];
	  tmp[ 129 ] += a_ * nblue[ idx2 ];
	  tmp[ 130 ] += a_ * nblue[ idx3 ];
	  tmp[ 131 ] += a_ * nblue[ idx4 ];
	  tmp[ 132 ] += a_ * nblue[ idx5 ];
	  tmp[ 133 ] += a_ * nblue[ idx6 ];
	  tmp[ 134 ] += a_ * nblue[ idx7 ];
	  tmp[ 135 ] += a_ * nblue[ idx8 ];
	  tmp[ 136 ] += a_ * nblue[ idx9 ];
	  tmp[ 137 ] += a_ * nblue[ idx10 ];

	  tmp[ 138 ] += a_ * red[ idx11 ];
	  tmp[ 139 ] += a_ * red[ idx12 ];
	  tmp[ 140 ] += a_ * red[ idx13 ];
	  tmp[ 141 ] += a_ * red[ idx14 ];
	  tmp[ 142 ] += a_ * nred[ idx11 ];
	  tmp[ 143 ] += a_ * nred[ idx12 ];
	  tmp[ 144 ] += a_ * nred[ idx13 ];
	  tmp[ 145 ] += a_ * nred[ idx14 ];
	  tmp[ 146 ] += a_ * green[ idx11 ];
	  tmp[ 147 ] += a_ * green[ idx12 ];
	  tmp[ 148 ] += a_ * green[ idx13 ];
	  tmp[ 149 ] += a_ * green[ idx14 ];
	  tmp[ 150 ] += a_ * ngreen[ idx11 ];
	  tmp[ 151 ] += a_ * ngreen[ idx12 ];
	  tmp[ 152 ] += a_ * ngreen[ idx13 ];
	  tmp[ 153 ] += a_ * ngreen[ idx14 ];
	  tmp[ 154 ] += a_ * blue[ idx11 ];
	  tmp[ 155 ] += a_ * blue[ idx12 ];
	  tmp[ 156 ] += a_ * blue[ idx13 ];
	  tmp[ 157 ] += a_ * blue[ idx14 ];
	  tmp[ 158 ] += a_ * nblue[ idx11 ];
	  tmp[ 159 ] += a_ * nblue[ idx12 ];
	  tmp[ 160 ] += a_ * nblue[ idx13 ];
	  tmp[ 161 ] += a_ * nblue[ idx14 ];

	  tmp[ 162 ] += b * red[ idx2 ];
	  tmp[ 163 ] += b * red[ idx3 ];
	  tmp[ 164 ] += b * red[ idx4 ];
	  tmp[ 165 ] += b * red[ idx5 ];
	  tmp[ 166 ] += b * red[ idx6 ];
	  tmp[ 167 ] += b * red[ idx7 ];
	  tmp[ 168 ] += b * red[ idx8 ];
	  tmp[ 169 ] += b * red[ idx9 ];
	  tmp[ 170 ] += b * red[ idx10 ];
	  tmp[ 171 ] += b * nred[ idx2 ];
	  tmp[ 172 ] += b * nred[ idx3 ];
	  tmp[ 173 ] += b * nred[ idx4 ];
	  tmp[ 174 ] += b * nred[ idx5 ];
	  tmp[ 175 ] += b * nred[ idx6 ];
	  tmp[ 176 ] += b * nred[ idx7 ];
	  tmp[ 177 ] += b * nred[ idx8 ];
	  tmp[ 178 ] += b * nred[ idx9 ];
	  tmp[ 179 ] += b * nred[ idx10 ];
	  tmp[ 180 ] += b * green[ idx2 ];
	  tmp[ 181 ] += b * green[ idx3 ];
	  tmp[ 182 ] += b * green[ idx4 ];
	  tmp[ 183 ] += b * green[ idx5 ];
	  tmp[ 184 ] += b * green[ idx6 ];
	  tmp[ 185 ] += b * green[ idx7 ];
	  tmp[ 186 ] += b * green[ idx8 ];
	  tmp[ 187 ] += b * green[ idx9 ];
	  tmp[ 188 ] += b * green[ idx10 ];
	  tmp[ 189 ] += b * ngreen[ idx2 ];
	  tmp[ 190 ] += b * ngreen[ idx3 ];
	  tmp[ 191 ] += b * ngreen[ idx4 ];
	  tmp[ 192 ] += b * ngreen[ idx5 ];
	  tmp[ 193 ] += b * ngreen[ idx6 ];
	  tmp[ 194 ] += b * ngreen[ idx7 ];
	  tmp[ 195 ] += b * ngreen[ idx8 ];
	  tmp[ 196 ] += b * ngreen[ idx9 ];
	  tmp[ 197 ] += b * ngreen[ idx10 ];
	  tmp[ 198 ] += b * blue[ idx2 ];
	  tmp[ 199 ] += b * blue[ idx3 ];
	  tmp[ 200 ] += b * blue[ idx4 ];
	  tmp[ 201 ] += b * blue[ idx5 ];
	  tmp[ 202 ] += b * blue[ idx6 ];
	  tmp[ 203 ] += b * blue[ idx7 ];
	  tmp[ 204 ] += b * blue[ idx8 ];
	  tmp[ 205 ] += b * blue[ idx9 ];
	  tmp[ 206 ] += b * blue[ idx10 ];
	  tmp[ 207 ] += b * nblue[ idx2 ];
	  tmp[ 208 ] += b * nblue[ idx3 ];
	  tmp[ 209 ] += b * nblue[ idx4 ];
	  tmp[ 210 ] += b * nblue[ idx5 ];
	  tmp[ 211 ] += b * nblue[ idx6 ];
	  tmp[ 212 ] += b * nblue[ idx7 ];
	  tmp[ 213 ] += b * nblue[ idx8 ];
	  tmp[ 214 ] += b * nblue[ idx9 ];
	  tmp[ 215 ] += b * nblue[ idx10 ];

	  tmp[ 216 ] += b * red[ idx11 ];
	  tmp[ 217 ] += b * red[ idx12 ];
	  tmp[ 218 ] += b * red[ idx13 ];
	  tmp[ 219 ] += b * red[ idx14 ];
	  tmp[ 220 ] += b * nred[ idx11 ];
	  tmp[ 221 ] += b * nred[ idx12 ];
	  tmp[ 222 ] += b * nred[ idx13 ];
	  tmp[ 223 ] += b * nred[ idx14 ];
	  tmp[ 224 ] += b * green[ idx11 ];
	  tmp[ 225 ] += b * green[ idx12 ];
	  tmp[ 226 ] += b * green[ idx13 ];
	  tmp[ 227 ] += b * green[ idx14 ];
	  tmp[ 228 ] += b * ngreen[ idx11 ];
	  tmp[ 229 ] += b * ngreen[ idx12 ];
	  tmp[ 230 ] += b * ngreen[ idx13 ];
	  tmp[ 231 ] += b * ngreen[ idx14 ];
	  tmp[ 232 ] += b * blue[ idx11 ];
	  tmp[ 233 ] += b * blue[ idx12 ];
	  tmp[ 234 ] += b * blue[ idx13 ];
	  tmp[ 235 ] += b * blue[ idx14 ];
	  tmp[ 236 ] += b * nblue[ idx11 ];
	  tmp[ 237 ] += b * nblue[ idx12 ];
	  tmp[ 238 ] += b * nblue[ idx13 ];
	  tmp[ 239 ] += b * nblue[ idx14 ];

	  tmp[ 240 ] += b_ * red[ idx2 ];
	  tmp[ 241 ] += b_ * red[ idx3 ];
	  tmp[ 242 ] += b_ * red[ idx4 ];
	  tmp[ 243 ] += b_ * red[ idx5 ];
	  tmp[ 244 ] += b_ * red[ idx6 ];
	  tmp[ 245 ] += b_ * red[ idx7 ];
	  tmp[ 246 ] += b_ * red[ idx8 ];
	  tmp[ 247 ] += b_ * red[ idx9 ];
	  tmp[ 248 ] += b_ * red[ idx10 ];
	  tmp[ 249 ] += b_ * nred[ idx2 ];
	  tmp[ 250 ] += b_ * nred[ idx3 ];
	  tmp[ 251 ] += b_ * nred[ idx4 ];
	  tmp[ 252 ] += b_ * nred[ idx5 ];
	  tmp[ 253 ] += b_ * nred[ idx6 ];
	  tmp[ 254 ] += b_ * nred[ idx7 ];
	  tmp[ 255 ] += b_ * nred[ idx8 ];
	  tmp[ 256 ] += b_ * nred[ idx9 ];
	  tmp[ 257 ] += b_ * nred[ idx10 ];
	  tmp[ 258 ] += b_ * green[ idx2 ];
	  tmp[ 259 ] += b_ * green[ idx3 ];
	  tmp[ 260 ] += b_ * green[ idx4 ];
	  tmp[ 261 ] += b_ * green[ idx5 ];
	  tmp[ 262 ] += b_ * green[ idx6 ];
	  tmp[ 263 ] += b_ * green[ idx7 ];
	  tmp[ 264 ] += b_ * green[ idx8 ];
	  tmp[ 265 ] += b_ * green[ idx9 ];
	  tmp[ 266 ] += b_ * green[ idx10 ];
	  tmp[ 267 ] += b_ * ngreen[ idx2 ];
	  tmp[ 268 ] += b_ * ngreen[ idx3 ];
	  tmp[ 269 ] += b_ * ngreen[ idx4 ];
	  tmp[ 270 ] += b_ * ngreen[ idx5 ];
	  tmp[ 271 ] += b_ * ngreen[ idx6 ];
	  tmp[ 272 ] += b_ * ngreen[ idx7 ];
	  tmp[ 273 ] += b_ * ngreen[ idx8 ];
	  tmp[ 274 ] += b_ * ngreen[ idx9 ];
	  tmp[ 275 ] += b_ * ngreen[ idx10 ];
	  tmp[ 276 ] += b_ * blue[ idx2 ];
	  tmp[ 277 ] += b_ * blue[ idx3 ];
	  tmp[ 278 ] += b_ * blue[ idx4 ];
	  tmp[ 279 ] += b_ * blue[ idx5 ];
	  tmp[ 280 ] += b_ * blue[ idx6 ];
	  tmp[ 281 ] += b_ * blue[ idx7 ];
	  tmp[ 282 ] += b_ * blue[ idx8 ];
	  tmp[ 283 ] += b_ * blue[ idx9 ];
	  tmp[ 284 ] += b_ * blue[ idx10 ];
	  tmp[ 285 ] += b_ * nblue[ idx2 ];
	  tmp[ 286 ] += b_ * nblue[ idx3 ];
	  tmp[ 287 ] += b_ * nblue[ idx4 ];
	  tmp[ 288 ] += b_ * nblue[ idx5 ];
	  tmp[ 289 ] += b_ * nblue[ idx6 ];
	  tmp[ 290 ] += b_ * nblue[ idx7 ];
	  tmp[ 291 ] += b_ * nblue[ idx8 ];
	  tmp[ 292 ] += b_ * nblue[ idx9 ];
	  tmp[ 293 ] += b_ * nblue[ idx10 ];

	  tmp[ 294 ] += b_ * red[ idx11 ];
	  tmp[ 295 ] += b_ * red[ idx12 ];
	  tmp[ 296 ] += b_ * red[ idx13 ];
	  tmp[ 297 ] += b_ * red[ idx14 ];
	  tmp[ 298 ] += b_ * nred[ idx11 ];
	  tmp[ 299 ] += b_ * nred[ idx12 ];
	  tmp[ 300 ] += b_ * nred[ idx13 ];
	  tmp[ 301 ] += b_ * nred[ idx14 ];
	  tmp[ 302 ] += b_ * green[ idx11 ];
	  tmp[ 303 ] += b_ * green[ idx12 ];
	  tmp[ 304 ] += b_ * green[ idx13 ];
	  tmp[ 305 ] += b_ * green[ idx14 ];
	  tmp[ 306 ] += b_ * ngreen[ idx11 ];
	  tmp[ 307 ] += b_ * ngreen[ idx12 ];
	  tmp[ 308 ] += b_ * ngreen[ idx13 ];
	  tmp[ 309 ] += b_ * ngreen[ idx14 ];
	  tmp[ 310 ] += b_ * blue[ idx11 ];
	  tmp[ 311 ] += b_ * blue[ idx12 ];
	  tmp[ 312 ] += b_ * blue[ idx13 ];
	  tmp[ 313 ] += b_ * blue[ idx14 ];
	  tmp[ 314 ] += b_ * nblue[ idx11 ];
	  tmp[ 315 ] += b_ * nblue[ idx12 ];
	  tmp[ 316 ] += b_ * nblue[ idx13 ];
	  tmp[ 317 ] += b_ * nblue[ idx14 ];

	  tmp[ 318 ] += c * red[ idx2 ];
	  tmp[ 319 ] += c * red[ idx3 ];
	  tmp[ 320 ] += c * red[ idx4 ];
	  tmp[ 321 ] += c * red[ idx5 ];
	  tmp[ 322 ] += c * red[ idx6 ];
	  tmp[ 323 ] += c * red[ idx7 ];
	  tmp[ 324 ] += c * red[ idx8 ];
	  tmp[ 325 ] += c * red[ idx9 ];
	  tmp[ 326 ] += c * red[ idx10 ];
	  tmp[ 327 ] += c * nred[ idx2 ];
	  tmp[ 328 ] += c * nred[ idx3 ];
	  tmp[ 329 ] += c * nred[ idx4 ];
	  tmp[ 330 ] += c * nred[ idx5 ];
	  tmp[ 331 ] += c * nred[ idx6 ];
	  tmp[ 332 ] += c * nred[ idx7 ];
	  tmp[ 333 ] += c * nred[ idx8 ];
	  tmp[ 334 ] += c * nred[ idx9 ];
	  tmp[ 335 ] += c * nred[ idx10 ];
	  tmp[ 336 ] += c * green[ idx2 ];
	  tmp[ 337 ] += c * green[ idx3 ];
	  tmp[ 338 ] += c * green[ idx4 ];
	  tmp[ 339 ] += c * green[ idx5 ];
	  tmp[ 340 ] += c * green[ idx6 ];
	  tmp[ 341 ] += c * green[ idx7 ];
	  tmp[ 342 ] += c * green[ idx8 ];
	  tmp[ 343 ] += c * green[ idx9 ];
	  tmp[ 344 ] += c * green[ idx10 ];
	  tmp[ 345 ] += c * ngreen[ idx2 ];
	  tmp[ 346 ] += c * ngreen[ idx3 ];
	  tmp[ 347 ] += c * ngreen[ idx4 ];
	  tmp[ 348 ] += c * ngreen[ idx5 ];
	  tmp[ 349 ] += c * ngreen[ idx6 ];
	  tmp[ 350 ] += c * ngreen[ idx7 ];
	  tmp[ 351 ] += c * ngreen[ idx8 ];
	  tmp[ 352 ] += c * ngreen[ idx9 ];
	  tmp[ 353 ] += c * ngreen[ idx10 ];
	  tmp[ 354 ] += c * blue[ idx2 ];
	  tmp[ 355 ] += c * blue[ idx3 ];
	  tmp[ 356 ] += c * blue[ idx4 ];
	  tmp[ 357 ] += c * blue[ idx5 ];
	  tmp[ 358 ] += c * blue[ idx6 ];
	  tmp[ 359 ] += c * blue[ idx7 ];
	  tmp[ 360 ] += c * blue[ idx8 ];
	  tmp[ 361 ] += c * blue[ idx9 ];
	  tmp[ 362 ] += c * blue[ idx10 ];
	  tmp[ 363 ] += c * nblue[ idx2 ];
	  tmp[ 364 ] += c * nblue[ idx3 ];
	  tmp[ 365 ] += c * nblue[ idx4 ];
	  tmp[ 366 ] += c * nblue[ idx5 ];
	  tmp[ 367 ] += c * nblue[ idx6 ];
	  tmp[ 368 ] += c * nblue[ idx7 ];
	  tmp[ 369 ] += c * nblue[ idx8 ];
	  tmp[ 370 ] += c * nblue[ idx9 ];
	  tmp[ 371 ] += c * nblue[ idx10 ];

	  tmp[ 372 ] += c * red[ idx11 ];
	  tmp[ 373 ] += c * red[ idx12 ];
	  tmp[ 374 ] += c * red[ idx13 ];
	  tmp[ 375 ] += c * red[ idx14 ];
	  tmp[ 376 ] += c * nred[ idx11 ];
	  tmp[ 377 ] += c * nred[ idx12 ];
	  tmp[ 378 ] += c * nred[ idx13 ];
	  tmp[ 379 ] += c * nred[ idx14 ];
	  tmp[ 380 ] += c * green[ idx11 ];
	  tmp[ 381 ] += c * green[ idx12 ];
	  tmp[ 382 ] += c * green[ idx13 ];
	  tmp[ 383 ] += c * green[ idx14 ];
	  tmp[ 384 ] += c * ngreen[ idx11 ];
	  tmp[ 385 ] += c * ngreen[ idx12 ];
	  tmp[ 386 ] += c * ngreen[ idx13 ];
	  tmp[ 387 ] += c * ngreen[ idx14 ];
	  tmp[ 388 ] += c * blue[ idx11 ];
	  tmp[ 389 ] += c * blue[ idx12 ];
	  tmp[ 390 ] += c * blue[ idx13 ];
	  tmp[ 391 ] += c * blue[ idx14 ];
	  tmp[ 392 ] += c * nblue[ idx11 ];
	  tmp[ 393 ] += c * nblue[ idx12 ];
	  tmp[ 394 ] += c * nblue[ idx13 ];
	  tmp[ 395 ] += c * nblue[ idx14 ];

	  tmp[ 396 ] += c_ * red[ idx2 ];
	  tmp[ 397 ] += c_ * red[ idx3 ];
	  tmp[ 398 ] += c_ * red[ idx4 ];
	  tmp[ 399 ] += c_ * red[ idx5 ];
	  tmp[ 400 ] += c_ * red[ idx6 ];
	  tmp[ 401 ] += c_ * red[ idx7 ];
	  tmp[ 402 ] += c_ * red[ idx8 ];
	  tmp[ 403 ] += c_ * red[ idx9 ];
	  tmp[ 404 ] += c_ * red[ idx10 ];
	  tmp[ 405 ] += c_ * nred[ idx2 ];
	  tmp[ 406 ] += c_ * nred[ idx3 ];
	  tmp[ 407 ] += c_ * nred[ idx4 ];
	  tmp[ 408 ] += c_ * nred[ idx5 ];
	  tmp[ 409 ] += c_ * nred[ idx6 ];
	  tmp[ 410 ] += c_ * nred[ idx7 ];
	  tmp[ 411 ] += c_ * nred[ idx8 ];
	  tmp[ 412 ] += c_ * nred[ idx9 ];
	  tmp[ 413 ] += c_ * nred[ idx10 ];
	  tmp[ 414 ] += c_ * green[ idx2 ];
	  tmp[ 415 ] += c_ * green[ idx3 ];
	  tmp[ 416 ] += c_ * green[ idx4 ];
	  tmp[ 417 ] += c_ * green[ idx5 ];
	  tmp[ 418 ] += c_ * green[ idx6 ];
	  tmp[ 419 ] += c_ * green[ idx7 ];
	  tmp[ 420 ] += c_ * green[ idx8 ];
	  tmp[ 421 ] += c_ * green[ idx9 ];
	  tmp[ 422 ] += c_ * green[ idx10 ];
	  tmp[ 423 ] += c_ * ngreen[ idx2 ];
	  tmp[ 424 ] += c_ * ngreen[ idx3 ];
	  tmp[ 425 ] += c_ * ngreen[ idx4 ];
	  tmp[ 426 ] += c_ * ngreen[ idx5 ];
	  tmp[ 427 ] += c_ * ngreen[ idx6 ];
	  tmp[ 428 ] += c_ * ngreen[ idx7 ];
	  tmp[ 429 ] += c_ * ngreen[ idx8 ];
	  tmp[ 430 ] += c_ * ngreen[ idx9 ];
	  tmp[ 431 ] += c_ * ngreen[ idx10 ];
	  tmp[ 432 ] += c_ * blue[ idx2 ];
	  tmp[ 433 ] += c_ * blue[ idx3 ];
	  tmp[ 434 ] += c_ * blue[ idx4 ];
	  tmp[ 435 ] += c_ * blue[ idx5 ];
	  tmp[ 436 ] += c_ * blue[ idx6 ];
	  tmp[ 437 ] += c_ * blue[ idx7 ];
	  tmp[ 438 ] += c_ * blue[ idx8 ];
	  tmp[ 439 ] += c_ * blue[ idx9 ];
	  tmp[ 440 ] += c_ * blue[ idx10 ];
	  tmp[ 441 ] += c_ * nblue[ idx2 ];
	  tmp[ 442 ] += c_ * nblue[ idx3 ];
	  tmp[ 443 ] += c_ * nblue[ idx4 ];
	  tmp[ 444 ] += c_ * nblue[ idx5 ];
	  tmp[ 445 ] += c_ * nblue[ idx6 ];
	  tmp[ 446 ] += c_ * nblue[ idx7 ];
	  tmp[ 447 ] += c_ * nblue[ idx8 ];
	  tmp[ 448 ] += c_ * nblue[ idx9 ];
	  tmp[ 449 ] += c_ * nblue[ idx10 ];

	  tmp[ 450 ] += c_ * red[ idx11 ];
	  tmp[ 451 ] += c_ * red[ idx12 ];
	  tmp[ 452 ] += c_ * red[ idx13 ];
	  tmp[ 453 ] += c_ * red[ idx14 ];
	  tmp[ 454 ] += c_ * nred[ idx11 ];
	  tmp[ 455 ] += c_ * nred[ idx12 ];
	  tmp[ 456 ] += c_ * nred[ idx13 ];
	  tmp[ 457 ] += c_ * nred[ idx14 ];
	  tmp[ 458 ] += c_ * green[ idx11 ];
	  tmp[ 459 ] += c_ * green[ idx12 ];
	  tmp[ 460 ] += c_ * green[ idx13 ];
	  tmp[ 461 ] += c_ * green[ idx14 ];
	  tmp[ 462 ] += c_ * ngreen[ idx11 ];
	  tmp[ 463 ] += c_ * ngreen[ idx12 ];
	  tmp[ 464 ] += c_ * ngreen[ idx13 ];
	  tmp[ 465 ] += c_ * ngreen[ idx14 ];
	  tmp[ 466 ] += c_ * blue[ idx11 ];
	  tmp[ 467 ] += c_ * blue[ idx12 ];
	  tmp[ 468 ] += c_ * blue[ idx13 ];
	  tmp[ 469 ] += c_ * blue[ idx14 ];
	  tmp[ 470 ] += c_ * nblue[ idx11 ];
	  tmp[ 471 ] += c_ * nblue[ idx12 ];
	  tmp[ 472 ] += c_ * nblue[ idx13 ];
	  tmp[ 473 ] += c_ * nblue[ idx14 ];

	  tmp[ 474 ] += a * a;
	  tmp[ 475 ] += 2 * a * a_;
	  tmp[ 476 ] += 2 * a * b;
	  tmp[ 477 ] += 2 * a * b_;
	  tmp[ 478 ] += 2 * a * c;
	  tmp[ 479 ] += 2 * a * c_;
	  tmp[ 480 ] += a_ * a_;
	  tmp[ 481 ] += 2 * a_ * b;
	  tmp[ 482 ] += 2 * a_ * b_;
	  tmp[ 483 ] += 2 * a_ * c;
	  tmp[ 484 ] += 2 * a_ * c_;
	  tmp[ 485 ] += b * b;
	  tmp[ 486 ] += 2 * b * b_;
	  tmp[ 487 ] += 2 * b * c;
	  tmp[ 488 ] += 2 * b * c_;
	  tmp[ 489 ] += b_ * b_;
	  tmp[ 490 ] += 2 * b_ * c;
	  tmp[ 491 ] += 2 * b_ * c_;
	  tmp[ 492 ] += c * c;
	  tmp[ 493 ] += 2 * c * c_;
	  tmp[ 494 ] += c_ * c_;
	}
      }
    }
  }
  
  for(int i=0; i<6; ++i) result[ i ] = tmp[ i ] / 765.0;
  for(int i=6; i<DIM_COLOR_1_3; ++i) result[ i ] = tmp[ i ] / 585225.0;
}

