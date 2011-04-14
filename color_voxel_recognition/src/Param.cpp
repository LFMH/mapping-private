#include <iostream>
#include <stdio.h>
#include <string.h>

#include "color_voxel_recognition/Param.hpp"

using namespace std;

//******************************
// read the length of voxel side
float Param::readVoxelSize( const char* filename ){
  float val;
  if( readParam( filename, "voxel_size:", val ) ){
    if( val <= 0 ){
      cerr << "ERR (in Param::readVoxelSize): voxel_size must be larger than 0." << endl;
      return -1;
    }
    return val;
  }
  cerr << "ERR (in Param::readVoxelSize): fail" << endl;
  return -1;
}

//*************************************************
// read the dimension of compressed feature vectors
int Param::readDim( const char* filename ){
  int val;
  if( readParam( filename, "dim:", val ) ){
    if( val < 1 ){
      cerr << "ERR (in Param::readDim): dim must be larger than 0." << endl;
      return -1;
    }
    return val;
  }
  cerr << "ERR (in Param::readDim): fail" << endl;
  return -1;
}

//**************************************************************
// read the number of voxels in each subdivision's side of scene
int Param::readBoxSize_scene( const char* filename ){
  int val;
  if( readParam( filename, "box_size(scene):", val ) ){
    if( val < 1 ){
      cerr << "ERR (in Param::readBoxSize_scene): box_size must be larger than 0." << endl;
      return -1;
    }
    return val;
  }
  cerr << "ERR (in Param::readBoxSize_scene): fail" << endl;
  return -1;
}

//************************************************************************
// read the number of voxels in each subdivision's side of a target object
int Param::readBoxSize_model( const char* filename ){
  int val;
  if( readParam( filename, "box_size(model):", val ) ){
    if( val < 1 ){
      cerr << "ERR (in Param::readBoxSize_model): box_size must be larger than 0." << endl;
      return -1;
    }
    return val;
  }
  cerr << "ERR (in Param::readBoxSize_model): fail" << endl;
  return -1;
}

// //********************************************
// // read the max number of voxels created once
// int Param::readMaxVoxelNum( const char* filename ){
//   int val;
//   if( readParam( filename, "max_voxel_num:", val ) ){
//     if( val < 1 ){
//       cerr << "ERR (in Param::readMaxVoxelNum): max_voxel_num must be larger than 0." << endl;
//       return -1;
//     }
//     else if( val < 10000000 )
//       cerr << "Warning (in Param::readMaxVoxelNum): max_voxel_num is too small." << endl;
//     return val;
//   }
//   cerr << "ERR (in Param::readMaxVoxelNum): fail" << endl;
//   return -1;
// }

//***********************************************************
// read the number for synthetic rotation in learning objects
int Param::readRotateNum( const char* filename ){
  int val;
  if( readParam( filename, "rotate_num:", val ) ){
    if( val < 1 ){
      cerr << "ERR (in Param::readRotateNum): rotate_num must be larger than 0." << endl;
      return -1;
    }
    return val;
  }
  cerr << "ERR (in Param::readRotateNum): fail" << endl;
  return -1;
}

//*********************************
// read flag for c3_hlac extraction
int Param::readC3HLAC_flag( const char* filename ){
  int val;
  if( readParam( filename, "c3_hlac_flg:", val ) ) return val;
  cerr << "ERR (in Param::readC3HLAC_flag): fail" << endl;
  return -1;
}

//************************************
// read the threshold for RGB binalize
void Param::readColorThreshold( int &r, int &g, int &b, const char* filename ){
  FILE *fp = fopen( filename, "r" );
  if( fscanf(fp,"%d %d %d\n", &r, &g, &b) == EOF ) std::cerr<< "fscanf err" << std::endl;
  fclose(fp);
  if( ( r < 0 ) || ( r > 254 ) || ( g < 0 ) || ( g > 254 ) || ( b < 0 ) || ( b > 254 ) )
    cerr << "ERR (in Param::readColorThreshold): invalid RGB value." << endl;
}

// //***********************************************************
// // read parameter of SR confidence for auto mesh construction
// int Param::readConfTh( const char* filename ){
//   int val;
//   if( readParam( filename, "confidence_th:", val ) ){
//     if( ( val < 0 ) || ( val > 255 ) ){
//       cerr << "ERR (in Param::readConfTh): conf_th must be larger than -1 and less than 256." << endl;
//       return -1;
//     }
//     return val;
//   }
//   cerr << "ERR (in Param::readConfTh): fail" << endl;
//   return -1;
// }

// //********************************************
// // read parameters for auto mesh construction
// void Param::readParamAuto( float& length_max_rate, float& length_th, float& distance_th, int& confidence_th, bool &relative_mode, const char* filename ){
//   char line[ 100 ];
//   FILE *fp = fopen( filename, "r" );
//   int tmp;
//   if( fscanf( fp, "%s %f\n", line, &length_max_rate ) == EOF ) std::cerr<< "fscanf err" << std::endl;
//   if( fscanf( fp, "%s %f\n", line, &length_th ) == EOF ) std::cerr<< "fscanf err" << std::endl;
//   if( fscanf( fp, "%s %f\n", line, &distance_th ) == EOF ) std::cerr<< "fscanf err" << std::endl;
//   if( fscanf( fp, "%s %d\n", line, &confidence_th ) == EOF ) std::cerr<< "fscanf err" << std::endl;
//   if( fscanf( fp, "%s %d\n", line, &tmp ) == EOF ) std::cerr<< "fscanf err" << std::endl;
//   relative_mode = tmp;
//   fclose( fp );
// }

// //********************************************
// // write parameters for auto mesh construction
// void Param::writeParamAuto( float length_max_rate, float length_th, float distance_th, int confidence_th, bool relative_mode, const char* filename ){
//   FILE *fp = fopen( filename, "w" );
//   fprintf( fp, "length_max_rate: %f\n", length_max_rate );
//   fprintf( fp, "length_th:       %f\n", length_th );
//   fprintf( fp, "distance_th:     %f\n", distance_th );
//   fprintf( fp, "confidence_th:   %d\n", confidence_th );
//   fprintf( fp, "relative_mode:   %d\n", relative_mode );
//   fclose( fp );
// }

//*********************//
//* private functions *//
//*********************//

bool Param::readParam( const char* filename, const char *param_string, int &val ){
  char line[100];
  FILE *fp = fopen( filename, "r" );
  while( fscanf(fp,"%s %d",line,&val)!=EOF ){
    if( strcmp(line,param_string) == 0 ){
      fclose(fp);
      return true;
    }
  }
  fclose(fp);
  return false;
}

bool Param::readParam( const char* filename, const char *param_string, float &val ){
  char line[100];
  FILE *fp = fopen( filename, "r" );
  while( fscanf(fp,"%s %f",line,&val)!=EOF ){
    if( strcmp(line,param_string) == 0 ){
      fclose(fp);
      return true;
    }
  }
  fclose(fp);
  return false;
}
