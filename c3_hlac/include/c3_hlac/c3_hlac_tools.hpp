//-----------
//* time
double my_clock()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + (double)tv.tv_usec*1e-6;
}

//-----------
//* read
void readFeature(const char *name, std::vector< std::vector<float> > &feature ){
  feature.resize( 0 );

  int dim, sample_num; 
  char line[ 100 ];
  std::string line_s;
  std::vector <float> tmp_feature;
  FILE *fp = fopen( name, "r" );
  while( 1 ){
    if( fgets(line,sizeof(line),fp) == NULL) std::cerr<< "fgets err" << std::endl;
    line_s = std::string (line);
    if( line_s.compare (0, 5, "COUNT") == 0 )
      sscanf( line, "COUNT %d", &dim );
    else if( line_s.compare (0, 6, "POINTS") == 0 )
      sscanf( line, "POINTS %d", &sample_num );
    else if( line_s.compare (0, 4, "DATA") == 0 )
      break;
  }
  tmp_feature.resize( dim );
  for(int n=0;n<sample_num;n++){
    for(int t=0;t<dim;t++)
      if( fscanf(fp,"%f ",&(tmp_feature[ t ]) ) == EOF ) std::cerr<< "fscanf err" << std::endl;
    feature.push_back ( tmp_feature );
  }
  fclose(fp);
}

void readFeature(const char *name, std::vector<float> &feature ){
  std::vector< std::vector<float> > features;
  readFeature( name, features );
  if( features.size() != 1 )
    std::cerr << "Warning in readFeature(): the number of features in " << name << " is not 1. (" << features.size() << ")" << std::endl;  
  feature = features[0];
}

//-----------
//* write
bool if_zero_vec( const std::vector<float> vec ){
  const int vec_size = vec.size();
  for( int i=0; i<vec_size; i++ )
    if( vec[ i ] != 0 ) return false;
  return true;
}
void writeFeature(const char *name, const std::vector< std::vector<float> > feature, bool remove_0_flg ){
  const int feature_size = feature[ 0 ].size();
  const int hist_num_init = feature.size();
  int hist_num = hist_num_init;
  if( remove_0_flg )
    for( int h=0; h<hist_num_init; h++ )
      if( if_zero_vec( feature[ h ] ) )
	hist_num --;
  
  FILE *fp = fopen( name, "w" );
  fprintf(fp,"# .PCD v.7 - Point Cloud Data file format\n");
  fprintf(fp,"FIELDS descriptor\n");
  fprintf(fp,"SIZE 4\n");
  fprintf(fp,"TYPE F\n");
  fprintf(fp,"COUNT %d\n",feature_size);
  fprintf(fp,"WIDTH %d\n", hist_num);
  fprintf(fp,"HEIGHT 1\n");
  fprintf(fp,"POINTS %d\n", hist_num);
  fprintf(fp,"DATA ascii\n");
  for( int h=0; h<hist_num_init; h++ ){
    if( remove_0_flg && if_zero_vec( feature[ h ] ) ) continue;
    for(int t=0;t<feature_size;t++)
      fprintf(fp,"%f ",feature[ h ][ t ]);
    fprintf(fp,"\n");
  }
  fclose(fp);
}
void writeFeature(const char *name, const std::vector<float> feature, bool remove_0_flg ){
  std::vector< std::vector<float> > tmp( 1 );
  tmp[ 0 ] = feature;
  writeFeature( name, tmp, remove_0_flg );
}

//-----------
//* voxelize
template <typename T>
void getVoxelGrid( pcl::VoxelGrid<T> &grid, pcl::PointCloud<T> input_cloud, pcl::PointCloud<T>& output_cloud, const float voxel_size ){
  grid.setLeafSize (voxel_size, voxel_size, voxel_size);
  grid.setInputCloud ( boost::make_shared<const pcl::PointCloud<T> > (input_cloud) );
  grid.setSaveLeafLayout(true);
  grid.filter (output_cloud);
}

//------------------------
//* extract - C3HLAC -
template <typename PointT>
Eigen::Vector3i extract_C3_HLAC_Signature981(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, std::vector< std::vector<float> > &feature, int thR, int thG, int thB, const float voxel_size, const int subdivision_size, const int offset_x, const int offset_y, const int offset_z ){
  feature.resize( 0 );
  pcl::PointCloud<pcl::C3_HLAC_Signature981> c3_hlac_signature;
  pcl::C3_HLAC_Estimation<PointT, pcl::C3_HLAC_Signature981> c3_hlac_;

  c3_hlac_.setRadiusSearch (1.8);
  c3_hlac_.setSearchMethod ( boost::make_shared<pcl::KdTreeFLANN<PointT> > () );
  c3_hlac_.setColorThreshold( thR, thG, thB );
  if( c3_hlac_.setVoxelFilter (grid, subdivision_size, offset_x, offset_y, offset_z, voxel_size) ){
    c3_hlac_.setInputCloud ( boost::make_shared<const pcl::PointCloud<PointT> > (cloud) );
    t1 = my_clock();
    c3_hlac_.compute( c3_hlac_signature );
    t2 = my_clock();
#ifndef QUIET
    ROS_INFO (" %d c3_hlac estimated. (%f sec)", (int)c3_hlac_signature.points.size (), t2-t1);
#endif
    const int hist_num = c3_hlac_signature.points.size();
    feature.resize( hist_num );
    for( int h=0; h<hist_num; h++ ){
      feature[ h ].resize( DIM_C3HLAC_1_3_ALL );
      for( int i=0; i<DIM_C3HLAC_1_3_ALL; i++)
	feature[ h ][ i ] = c3_hlac_signature.points[ h ].histogram[ i ];
    }
  }
  return c3_hlac_.getSubdivNum();
}

template <typename PointT>
void extract_C3_HLAC_Signature981(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, std::vector<float> &feature, int thR, int thG, int thB, const float voxel_size ){
  std::vector< std::vector<float> > tmp( 1 );
  extract_C3_HLAC_Signature981( grid, cloud, tmp, thR, thG, thB, voxel_size ); // for one signature
  feature = tmp[ 0 ];
}

template <typename PointT>
Eigen::Vector3i extract_C3_HLAC_Signature117(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, std::vector< std::vector<float> > &feature, int thR, int thG, int thB, const float voxel_size, const int subdivision_size, const int offset_x , const int offset_y, const int offset_z ){
  feature.resize( 0 );
  pcl::PointCloud<pcl::C3_HLAC_Signature117> c3_hlac_signature;
  pcl::C3_HLAC_RI_Estimation<PointT, pcl::C3_HLAC_Signature117> c3_hlac_;

  c3_hlac_.setRadiusSearch (1.8);
  c3_hlac_.setSearchMethod ( boost::make_shared<pcl::KdTreeFLANN<PointT> > () );
  c3_hlac_.setColorThreshold( thR, thG, thB );
  if( c3_hlac_.setVoxelFilter (grid, subdivision_size, offset_x, offset_y, offset_z, voxel_size) ){
    c3_hlac_.setInputCloud ( boost::make_shared<const pcl::PointCloud<PointT> > (cloud) );
    t1 = my_clock();
    c3_hlac_.compute( c3_hlac_signature );
    t2 = my_clock();
#ifndef QUIET
    ROS_INFO (" %d c3_hlac estimated. (%f sec)", (int)c3_hlac_signature.points.size (), t2-t1);
#endif
    const int hist_num = c3_hlac_signature.points.size();
    feature.resize( hist_num );
    for( int h=0; h<hist_num; h++ ){
      feature[ h ].resize( DIM_C3HLAC_RI_1_3_ALL );
      for( int i=0; i<DIM_C3HLAC_RI_1_3_ALL; i++)
	feature[ h ][ i ] = c3_hlac_signature.points[ h ].histogram[ i ];
    }
  }
  return c3_hlac_.getSubdivNum();
}

template <typename PointT>
void extract_C3_HLAC_Signature117(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, std::vector<float> &feature, int thR, int thG, int thB, const float voxel_size ){
  std::vector< std::vector<float> > tmp( 1 );
  extract_C3_HLAC_Signature117( grid, cloud, tmp, thR, thG, thB, voxel_size ); // for one signature
  feature = tmp[ 0 ];
}
