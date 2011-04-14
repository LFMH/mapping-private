#include <iostream>
#include <c3_hlac/c3_hlac_tools.h>
#include <color_voxel_recognition/libPCA.hpp>
#include <color_voxel_recognition/Param.hpp>
#include "color_voxel_recognition/FILE_MODE"

/***********************************************************************************************************/
/* calculate projection axis for feature compression                                                       */
/*   Note that this process is necessary only once when the system sees the environment for the first time */
/***********************************************************************************************************/

using namespace std;

int main(int argc, char** argv){
  if( argc != 3 ){
    cerr << "usage: " << argv[0] << " [path] <registration_num>" << endl;
    exit( EXIT_FAILURE );
  }
  const int file_num = atoi( argv[2] );
  char tmpname[ 1000 ];
  
  PCA pca( false );
  std::vector< std::vector<float> > feature;
  for( int i=0; i<file_num; i++ ){
    sprintf( tmpname, "%s/scene/Features/%05d.pcd", argv[1], i );
    readFeature( tmpname, feature );
    const int hist_num = feature.size();

    for( int h=0; h<hist_num; h++ ){
      //if( !if_zero_vec( feature[ h ] ) ){ 
      pca.addData( feature[ h ] );

      // std::vector<float> feature_rotate;
      // std::vector<float> feature_rotate_pre = feature[ h ];
      // std::vector<float> feature_rotate_pre2;
      
      // for(int t=0;t<3;t++){
      //   rotateFeature90( feature_rotate,feature_rotate_pre,R_MODE_2);
      //   pca.addData( feature_rotate );
      //   feature_rotate_pre = feature_rotate;
      // }
      
      // rotateFeature90( feature_rotate,feature[ h ],R_MODE_3);
      // pca.addData( feature_rotate );
      // feature_rotate_pre  = feature_rotate;
      // feature_rotate_pre2 = feature_rotate;
	
      // for(int t=0;t<3;t++){
      //   rotateFeature90( feature_rotate,feature_rotate_pre,R_MODE_2);
      // pca.addData( feature_rotate );
      // feature_rotate_pre = feature_rotate;
      // }
	
      // rotateFeature90( feature_rotate,feature_rotate_pre2,R_MODE_3);
      // pca.addData( feature_rotate );
      // feature_rotate_pre = feature_rotate;
      // feature_rotate_pre2 = feature_rotate;
	
      // for(int t=0;t<3;t++){
      //   rotateFeature90( feature_rotate,feature_rotate_pre,R_MODE_2);
      //   pca.addData( feature_rotate );
      //   feature_rotate_pre = feature_rotate;
      // }
	
      // rotateFeature90( feature_rotate,feature_rotate_pre2,R_MODE_3);
      // pca.addData( feature_rotate );
      // feature_rotate_pre = feature_rotate;
	
      // for(int t=0;t<3;t++){
      //   rotateFeature90( feature_rotate,feature_rotate_pre,R_MODE_2);
      //   pca.addData( feature_rotate );
      //   feature_rotate_pre = feature_rotate;
      // }
	
      // rotateFeature90( feature_rotate,feature[ h ],R_MODE_1);
      // pca.addData( feature_rotate );
      // feature_rotate_pre = feature_rotate;
	
      // for(int t=0;t<3;t++){
      //   rotateFeature90( feature_rotate,feature_rotate_pre,R_MODE_2);
      //   pca.addData( feature_rotate );
      //   feature_rotate_pre = feature_rotate;
      // }
	
      // rotateFeature90( feature_rotate,feature[ h ],R_MODE_4);
      // pca.addData( feature_rotate );
      // feature_rotate_pre = feature_rotate;
	
      // for(int t=0;t<3;t++){
      //   rotateFeature90( feature_rotate,feature_rotate_pre,R_MODE_2);
      //   pca.addData( feature_rotate );
      //   feature_rotate_pre = feature_rotate;
      // }
      //}
    }
  }

  //* solve PCA and write results
  pca.solve();
  sprintf( tmpname, "%s/scene/pca_result", argv[1] );
  pca.write( tmpname, ASCII_MODE_P );

  return 0;
}
