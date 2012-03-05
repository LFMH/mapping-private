
/*
 * Copyright (c) 2012, Lucian Cosmin Goron <goron@cs.tum.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// ---------- Macros ---------- //

#define _sqr(c) ((c)*(c))

#define M_PI 3.14159265358979323846

////////////////////////////////////////////////////////////////////////////////////////
/** \brief Optimized matrix-vector multiplication for coordinate system transformations.
 * \param T
 * \param p
 * \param r
*/
void transform (double T[4][4], double p[4], double r[4])
{
  int i,j;
  double temp;
  for (i = 0; i < 3; i++)
  {
    temp = 0.0;
    for (j = 0; j < 3; j++)
      temp += T[i][j] * p[j];
    r[i] = temp + T[i][3];
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Compute transformation matrix for coordinate system transformations based on translations and roations around XYZ axis.
 * \param tX, tY, tZ
 * \param rX, rY, rZ
 * \param T
*/
void computeTransformationMatrix (double tX, double tY, double tZ, double rX, double rY, double rZ, double T[4][4])
{
  double cx = cos (rX), sx = sin (rX);
  double cy = cos (rY), sy = sin (rY);
  double cz = cos (rZ), sz = sin (rZ);

  T[0][0] = cz*cy; T[0][1] = sx*sy*cz - cx*sz; T[0][2] = cx*sy*cz + sx*sz; T[0][3] = tX;
  T[1][0] = cy*sz; T[1][1] = sx*sy*sz + cx*cz; T[1][2] = cx*sy*sz - sx*cz; T[1][3] = tY;
  T[2][0] = -sy;   T[2][1] = sx*cy;            T[2][2] = cx*cy;            T[2][3] = tZ;

  T[3][0] = T[3][1] = T[3][2] = 0; T[3][3] = 1;
}

///////////////////////////////
/** \Brief Get model of cuboid.
*/

std::vector<pcl::ModelCoefficients> fetchCuboid (pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr working_cloud,
                                                 pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr    line_cloud,
                                                 pcl::ModelCoefficients::Ptr                        line_coefficients,
                                                 double                                             line_threshold,
                                                 double growing_step,
                                                 double growing_height,
                                                 bool   growing_visualization,
                                                 double rX, double rY, double rZ,
                                                 double tX, double tY, double tZ,
                                                 int  mean_k_filter,
                                                 int std_dev_filter,
                                                 pcl::PointXYZRGBNormalRSD abs_min,
                                                 pcl::visualization::PCLVisualizer &viewer, int size,
                                                 bool till_the_end,
                                                 bool shapes_from_table_plane)
{

  double P1[2];
  P1[0] = line_coefficients->values.at (0);
  P1[1] = line_coefficients->values.at (1);

  double P2[2];
  P2[0] = line_coefficients->values.at (3) + line_coefficients->values.at (0);
  P2[1] = line_coefficients->values.at (4) + line_coefficients->values.at (1);

  pcl::ModelCoefficients::Ptr test_coeffs (new pcl::ModelCoefficients ());
  test_coeffs->values.push_back (P1[0]);
  test_coeffs->values.push_back (P1[1]);
  test_coeffs->values.push_back (0.0);
  test_coeffs->values.push_back (P2[0] - P1[0]);
  test_coeffs->values.push_back (P2[1] - P1[1]);
  test_coeffs->values.push_back (0.0);

  if ( growing_visualization )
  {
    viewer.addLine (*test_coeffs, 1.0, 0.0, 0.5, "test_COEFFS");
    viewer.spin ();
    viewer.removeShape ("test_COEFFS");
  }

  // NOVEL WAY OF GROWING BOXES //

  pcl::PointXYZRGBNormalRSD MiN, MaX;
  pcl::getMinMax3D (*line_cloud, MiN, MaX);

  pcl::PointIndices::Ptr novel_box_inliers (new pcl::PointIndices ());
  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr novel_box_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

  double M[2];
  M[0] = (P1[0] + P2[0]) / 2;
  M[1] = (P1[1] + P2[1]) / 2;

  //////////////////
  // From M To P1 //
  //////////////////

  bool em1_bool = true;

  double M1[2];
  M1[0] = M[0];
  M1[1] = M[1];

  do
  {

    double P2M1V[2];
    P2M1V[0] = M1[0] - P2[0];
    P2M1V[1] = M1[1] - P2[1];

    double P2M1N = sqrt (P2M1V[0]*P2M1V[0] + P2M1V[1]*P2M1V[1]);
    P2M1V[0] = P2M1V[0] / P2M1N;
    P2M1V[1] = P2M1V[1] / P2M1N;

    double EM1[2];
    EM1[0] = M1[0] + P2M1V[0] * growing_step;
    EM1[1] = M1[1] + P2M1V[1] * growing_step;

    pcl::ModelCoefficients::Ptr em1_coeffs (new pcl::ModelCoefficients ());
    em1_coeffs->values.push_back (EM1[0]);
    em1_coeffs->values.push_back (EM1[1]);
    em1_coeffs->values.push_back (0.0);
    em1_coeffs->values.push_back (M1[0] - EM1[0]);
    em1_coeffs->values.push_back (M1[1] - EM1[1]);
    em1_coeffs->values.push_back (0.0);

    if ( growing_visualization )
    {
      viewer.addLine (*em1_coeffs, 1.0, 0.0, 0.5, "EM1_COEFFS");
      viewer.spin ();
      viewer.removeShape ("EM1_COEFFS");
    }

    //

    pcl::PointIndices::Ptr em1_inliers (new pcl::PointIndices ());

    for (int idx = 0; idx < working_cloud->points.size (); idx++)
    {
      double P0[2];
      P0[0] = working_cloud->points.at (idx).x;
      P0[1] = working_cloud->points.at (idx).y;

      double num = (P0[0] - EM1[0])*(M1[0] - EM1[0]) + (P0[1] - EM1[1])*(M1[1] - EM1[1]);
      double den = _sqr (M1[0] - EM1[0])  +  _sqr (M1[1] - EM1[1]);
      double u = num / den;

      double I[2];
      I[0] = EM1[0] + u * (M1[0] - EM1[0]);
      I[1] = EM1[1] + u * (M1[1] - EM1[1]);

      double d = sqrt ( _sqr (I[0] - P0[0])  +  _sqr (I[1] - P0[1]) );

      if ( d < line_threshold )
      {
        double IEM1D  = sqrt ( _sqr  (I[0] - EM1[0])  +  _sqr  (I[1] - EM1[1]) );
        double IM1D   = sqrt ( _sqr  (I[0] -  M1[0])  +  _sqr  (I[1] -  M1[1]) );
        double EM1M1D = sqrt ( _sqr (M1[0] - EM1[0])  +  _sqr (M1[1] - EM1[1]) );

        if ( (IEM1D + IM1D) < (EM1M1D + 0.001) )
          em1_inliers->indices.push_back (idx);
      }
    }

    //

    pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr em1_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

    pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> em1_ei;
    em1_ei.setInputCloud (working_cloud);
    em1_ei.setIndices (em1_inliers);
    em1_ei.setNegative (false);
    em1_ei.filter (*em1_cloud);

    if ( growing_visualization )
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> em1_cloud_color (em1_cloud, 255, 0, 127);
      viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (em1_cloud, em1_cloud_color, "EM1_CLOUD");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "EM1_CLOUD");
      viewer.spin ();
      viewer.removePointCloud ("EM1_CLOUD");
    }

    //

    pcl::PointXYZRGBNormalRSD em1_MiN, em1_MaX;
    pcl::getMinMax3D (*em1_cloud, em1_MiN, em1_MaX);

    cerr << endl;
    cerr << "          MaX = " << MaX.z << endl;
    cerr << "      em1_MaX = " << em1_MaX.z << endl;
    cerr << "      em1_dif = " << fabs (MaX.z - em1_MaX.z) << endl;
    cerr << endl;

    double em1_dif = fabs (MaX.z - em1_MaX.z);

    if ( em1_dif < growing_height )
    {
      cerr << "      OK !" << endl ;

      M1[0] = EM1[0];
      M1[1] = EM1[1];

      //pcl::copyPointCloud (*em1_cloud, *novel_box_cloud);
      novel_box_cloud->points.insert (  novel_box_cloud->points.end(),        em1_cloud->points.begin(),         em1_cloud->points.end());
      novel_box_inliers->indices.insert (novel_box_inliers->indices.end(),     em1_inliers->indices.begin(),     em1_inliers->indices.end());
    }
    else
    {
      cerr << "      NOT OK !" << endl ;

      em1_bool = false;
    }

  } while ( em1_bool );

  cerr << endl << "   EXIT !" << endl << endl ;

  if ( growing_visualization )
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> novel_box_cloud_color (novel_box_cloud, 255, 0, 127);
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (novel_box_cloud, novel_box_cloud_color, "NOVEL_BOX_CLOUD");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "NOVEL_BOX_CLOUD");
    viewer.spin ();
    viewer.removePointCloud ("NOVEL_BOX_CLOUD");
  }

  //////////////////
  // From M To P2 //
  //////////////////

  bool em2_bool = true;

  double M2[2];
  M2[0] = M[0];
  M2[1] = M[1];

  do
  {

    double P1M2V[2];
    P1M2V[0] = M2[0] - P1[0];
    P1M2V[1] = M2[1] - P1[1];

    double P1M2N = sqrt (P1M2V[0]*P1M2V[0] + P1M2V[1]*P1M2V[1]);
    P1M2V[0] = P1M2V[0] / P1M2N;
    P1M2V[1] = P1M2V[1] / P1M2N;

    double EM2[2];
    EM2[0] = M2[0] + P1M2V[0] * growing_step;
    EM2[1] = M2[1] + P1M2V[1] * growing_step;

    pcl::ModelCoefficients::Ptr em2_coeffs (new pcl::ModelCoefficients ());
    em2_coeffs->values.push_back (EM2[0]);
    em2_coeffs->values.push_back (EM2[1]);
    em2_coeffs->values.push_back (0.0);
    em2_coeffs->values.push_back (M2[0] - EM2[0]);
    em2_coeffs->values.push_back (M2[1] - EM2[1]);
    em2_coeffs->values.push_back (0.0);

    if ( growing_visualization )
    {
      viewer.addLine (*em2_coeffs, 1.0, 0.0, 0.5, "EM2_COEFFS");
      viewer.spin ();
      viewer.removeShape ("EM2_COEFFS");
    }

    //

    pcl::PointIndices::Ptr em2_inliers (new pcl::PointIndices ());

    for (int idx = 0; idx < working_cloud->points.size (); idx++)
    {
      double P0[2];
      P0[0] = working_cloud->points.at (idx).x;
      P0[1] = working_cloud->points.at (idx).y;

      double num = (P0[0] - EM2[0])*(M2[0] - EM2[0]) + (P0[1] - EM2[1])*(M2[1] - EM2[1]);
      double den = _sqr (M2[0] - EM2[0])  +  _sqr (M2[1] - EM2[1]);
      double u = num / den;

      double I[2];
      I[0] = EM2[0] + u * (M2[0] - EM2[0]);
      I[1] = EM2[1] + u * (M2[1] - EM2[1]);

      double d = sqrt ( _sqr (I[0] - P0[0])  +  _sqr (I[1] - P0[1]) );

      if ( d < line_threshold )
      {
        double IEM2D  = sqrt ( _sqr  (I[0] - EM2[0])  +  _sqr  (I[1] - EM2[1]) );
        double IM2D   = sqrt ( _sqr  (I[0] -  M2[0])  +  _sqr  (I[1] -  M2[1]) );
        double EM2M2D = sqrt ( _sqr (M2[0] - EM2[0])  +  _sqr (M2[1] - EM2[1]) );

        if ( (IEM2D + IM2D) < (EM2M2D + 0.001) )
          em2_inliers->indices.push_back (idx);
      }
    }

    //

    pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr em2_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

    pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> em2_ei;
    em2_ei.setInputCloud (working_cloud);
    em2_ei.setIndices (em2_inliers);
    em2_ei.setNegative (false);
    em2_ei.filter (*em2_cloud);

    if ( growing_visualization )
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> em2_cloud_color (em2_cloud, 255, 0, 127);
      viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (em2_cloud, em2_cloud_color, "EM2_CLOUD");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "EM2_CLOUD");
      viewer.spin ();
      viewer.removePointCloud ("EM2_CLOUD");
    }

    //

    pcl::PointXYZRGBNormalRSD em2_MiN, em2_MaX;
    pcl::getMinMax3D (*em2_cloud, em2_MiN, em2_MaX);

    cerr << endl;
    cerr << "          MaX = " << MaX.z << endl;
    cerr << "      em2_MaX = " << em2_MaX.z << endl;
    cerr << "      em2_dif = " << fabs (MaX.z - em2_MaX.z) << endl;
    cerr << endl;

    double em2_dif = fabs (MaX.z - em2_MaX.z);

    if ( em2_dif < growing_height )
    {
      cerr << "      OK !" << endl ;

      M2[0] = EM2[0];
      M2[1] = EM2[1];

      //pcl::copyPointCloud (*em2_cloud, *novel_box_cloud);
      novel_box_cloud->points.insert (  novel_box_cloud->points.end(),        em2_cloud->points.begin(),         em2_cloud->points.end());
      novel_box_inliers->indices.insert (novel_box_inliers->indices.end(),     em2_inliers->indices.begin(),     em2_inliers->indices.end());
    }
    else
    {
      cerr << "      NOT OK !" << endl ;

      em2_bool = false;
    }

  } while ( em2_bool );

  cerr << endl << "   EXIT !" << endl << endl ;

  if ( growing_visualization )
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> novel_box_cloud_color (novel_box_cloud, 255, 0, 127);
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (novel_box_cloud, novel_box_cloud_color, "NOVEL_BOX_CLOUD");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "NOVEL_BOX_CLOUD");
    viewer.spin ();
    viewer.removePointCloud ("NOVEL_BOX_CLOUD");
  }

  /*
     pcl::PointIndices::Ptr g_vransac_line_inliers (new pcl::PointIndices ());
   *g_vransac_line_inliers = *vransac_line_inliers;

   float R = 0.010;
   std::vector<int> pointIdxRadiusSearch;
   std::vector<float> pointRadiusSquaredDistance;

   pcl::KdTreeFLANN<pcl::PointXYZRGBNormalRSD>::Ptr g_tree (new pcl::KdTreeFLANN<pcl::PointXYZRGBNormalRSD> ());
   g_tree->setInputCloud (working_cloud);

   std::vector<bool> g_visited_points (working_cloud->points.size ());
   for (int vis = 0; vis < g_visited_points.size (); vis++)
   g_visited_points.at (vis) = false;

   size_t gidx = 0;

   do
   {
   pcl::PointXYZRGBNormalRSD searchPoint;
   searchPoint = working_cloud->points.at (g_vransac_line_inliers->indices.at (gidx));

   if ( g_visited_points.at (g_vransac_line_inliers->indices.at (gidx)) == false )
   {
   g_visited_points.at (g_vransac_line_inliers->indices.at (gidx)) = true;

   double P0[2];
   P0[0] = working_cloud->points.at (g_vransac_line_inliers->indices.at (gidx)).x;
   P0[1] = working_cloud->points.at (g_vransac_line_inliers->indices.at (gidx)).y;

   double num = (P0[0] - P1[0])*(P2[0] - P1[0]) + (P0[1] - P1[1])*(P2[1] - P1[1]);
   double den = _sqr (P2[0] - P1[0])  +  _sqr (P2[1] - P1[1]);
   double u = num / den;

   double I[2];
   I[0] = P1[0] + u * (P2[0] - P1[0]);
   I[1] = P1[1] + u * (P2[1] - P1[1]);

   double d = sqrt ( _sqr (I[0] - P0[0])  +  _sqr (I[1] - P0[1]) );

   if ( d < line_threshold )
   {
   if ( g_tree->radiusSearch (searchPoint, R, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
   {
   for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
   g_vransac_line_inliers->indices.push_back (pointIdxRadiusSearch.at (i));
   }
   }
   }

   gidx++;

   } while ( gidx < g_vransac_line_inliers->indices.size () );

   *vransac_line_inliers = *g_vransac_line_inliers;

   cerr << " vransac_line_inliers = " << vransac_line_inliers->indices.size () << endl;

   sort (g_vransac_line_inliers->indices.begin(), g_vransac_line_inliers->indices.end());
   g_vransac_line_inliers->indices.erase (unique (g_vransac_line_inliers->indices.begin(), g_vransac_line_inliers->indices.end()), g_vransac_line_inliers->indices.end());

   pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr g_vransac_line_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

   pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> g_l_ei;
   g_l_ei.setInputCloud (working_cloud);
   g_l_ei.setIndices (g_vransac_line_inliers);
   g_l_ei.setNegative (false);
   g_l_ei.filter (*g_vransac_line_cloud);

  // Adjust Coefficients Of Line Model //
  adjustLineCoefficients (g_vransac_line_cloud, vransac_line_coefficients);

  if ( space_step )
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> g_vransac_line_cloud_color (g_vransac_line_cloud, 0, 255, 255);
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (g_vransac_line_cloud, g_vransac_line_cloud_color, "G_VRANSAC_LINE_CLOUD");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "G_VRANSAC_LINE_CLOUD");
    viewer.addLine (*vransac_line_coefficients, 0.0, 1.0, 1.0, "VRANSAC_LINE_MODEL");
    viewer.spin ();
    viewer.removePointCloud ("G_VRANSAC_LINE_CLOUD");
    viewer.removeShape ("VRANSAC_LINE_MODEL");
  }
  */

    // Obtain The Scanning Viewpoint //

    double O[3];
  O[0] = 0.0;
  O[1] = 0.0;
  O[2] = 0.0;

  double VP[3];
  double T[4][4];

  rX = DEG2RAD (rX); rY = DEG2RAD (rY); rZ = DEG2RAD (rZ);
  computeTransformationMatrix (tX, tY, tZ, rX, rY, rZ, T);
  transform (T, O, VP);

  if ( !till_the_end )
    if ( true )
    {
      pcl::PointXYZ CVP;
      CVP.x = VP[0];
      CVP.y = VP[1];
      CVP.z = VP[2];

      viewer.addSphere<pcl::PointXYZ> (CVP, 0.010, 1.0, 1.0, 0.0, "VIEWPOINT_SPHERE");
      viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "VIEWPOINT_SPHERE");
      viewer.spin ();
      viewer.removeShape ("VIEWPOINT_SPHERE");
    }

  // Determine The Normal To Line //

  double dx = P2[0] - P1[0];
  double dy = P2[1] - P1[1];

  double N1[2];
  N1[0] = -dy;
  N1[1] =  dx;

  double PN1[2];
  PN1[0] =  ((P1[0] + P2[0]) / 2) + N1[0]*0.25;
  PN1[1] =  ((P1[1] + P2[1]) / 2) + N1[1]*0.25;

  double N2[2];
  N2[0] =  dy;
  N2[1] = -dx;

  double PN2[2];
  PN2[0] =  ((P1[0] + P2[0]) / 2) + N2[0]*0.25;
  PN2[1] =  ((P1[1] + P2[1]) / 2) + N2[1]*0.25;

  if ( false )
  {
    pcl::ModelCoefficients::Ptr first_normal_coefficients (new pcl::ModelCoefficients ());
    first_normal_coefficients->values.push_back ((P1[0] + P2[0]) / 2);
    first_normal_coefficients->values.push_back ((P1[1] + P2[1]) / 2);
    first_normal_coefficients->values.push_back (0.0);
    first_normal_coefficients->values.push_back (PN1[0] - ((P1[0] + P2[0]) / 2));
    first_normal_coefficients->values.push_back (PN1[1] - ((P1[1] + P2[1]) / 2));
    first_normal_coefficients->values.push_back (0.0);

    pcl::ModelCoefficients::Ptr second_normal_coefficients (new pcl::ModelCoefficients ());
    second_normal_coefficients->values.push_back ((P1[0] + P2[0]) / 2);
    second_normal_coefficients->values.push_back ((P1[1] + P2[1]) / 2);
    second_normal_coefficients->values.push_back (0.0);
    second_normal_coefficients->values.push_back (PN2[0] - ((P1[0] + P2[0]) / 2));
    second_normal_coefficients->values.push_back (PN2[1] - ((P1[1] + P2[1]) / 2));
    second_normal_coefficients->values.push_back (0.0);

    viewer.addLine (*first_normal_coefficients, 1.0, 0.0, 1.0, "FIRST_NORMAL_COEFFICIENTS");
    viewer.addLine (*second_normal_coefficients, 1.0, 0.0, 1.0, "SECOND_NORMAL_COEFFICIENTS");
    viewer.spin ();
    viewer.removeShape ("FIRST_NORMAL_COEFFICIENTS");
    viewer.removeShape ("SECOND_NORMAL_COEFFICIENTS");
  }

  double N[2];

  double PN1VP = sqrt ( _sqr (PN1[0] - VP[0]) + _sqr (PN1[1] - VP[1]) );
  double PN2VP = sqrt ( _sqr (PN2[0] - VP[0]) + _sqr (PN2[1] - VP[1]) );

  if ( PN1VP < PN2VP )
  {
    N[0] = N2[0];
    N[1] = N2[1];
  }
  else
  {
    N[0] = N1[0];
    N[1] = N1[1];
  }

  if ( !till_the_end )
    if ( true )
    {
      pcl::ModelCoefficients::Ptr normal_coefficients (new pcl::ModelCoefficients ());
      normal_coefficients->values.push_back ((P1[0] + P2[0]) / 2);
      normal_coefficients->values.push_back ((P1[1] + P2[1]) / 2);
      normal_coefficients->values.push_back (0.0);
      normal_coefficients->values.push_back (N[0]);
      normal_coefficients->values.push_back (N[1]);
      normal_coefficients->values.push_back (0.0);

      viewer.addLine (*normal_coefficients, 1.0, 0.0, 1.0, "NORMAL_COEFFICIENTS");
      viewer.spin ();
      viewer.removeShape ("NORMAL_COEFFICIENTS");
    }

  double NN = sqrt ( N[0]*N[0] + N[1]*N[1] );
  N[0] = N[0] / NN;
  N[1] = N[1] / NN;

  // NOVEL WAY OF GROWING BOXES //

  double NP1[2];
  double NP2[2];

  /////////////////
  // To The Back //
  /////////////////

  bool en1_bool = true;

  NP1[0] = M1[0];
  NP1[1] = M1[1];

  NP2[0] = M2[0];
  NP2[1] = M2[1];

  do
  {
    double EN1[2];
    EN1[0] = NP1[0] + N[0] * growing_step /2;
    EN1[1] = NP1[1] + N[1] * growing_step /2;

    double EN2[2];
    EN2[0] = NP2[0] + N[0] * growing_step /2;
    EN2[1] = NP2[1] + N[1] * growing_step /2;

    pcl::ModelCoefficients::Ptr en1_coeffs (new pcl::ModelCoefficients ());
    en1_coeffs->values.push_back (EN1[0]);
    en1_coeffs->values.push_back (EN1[1]);
    en1_coeffs->values.push_back (0.0);
    en1_coeffs->values.push_back (EN2[0] - EN1[0]);
    en1_coeffs->values.push_back (EN2[1] - EN1[1]);
    en1_coeffs->values.push_back (0.0);

    if ( growing_visualization )
    {
      viewer.addLine (*en1_coeffs, 1.0, 0.0, 0.5, "EN_COEFFS");
      viewer.spin ();
      viewer.removeShape ("EN_COEFFS");
    }

    //

    pcl::PointIndices::Ptr en1_inliers (new pcl::PointIndices ());

    for (int idx = 0; idx < working_cloud->points.size (); idx++)
    {
      double P0[2];
      P0[0] = working_cloud->points.at (idx).x;
      P0[1] = working_cloud->points.at (idx).y;

      double num = (P0[0] - EN1[0])*(EN2[0] - EN1[0]) + (P0[1] - EN1[1])*(EN2[1] - EN1[1]);
      double den = _sqr (EN2[0] - EN1[0])  +  _sqr (EN2[1] - EN1[1]);
      double u = num / den;

      double I[2];
      I[0] = EN1[0] + u * (EN2[0] - EN1[0]);
      I[1] = EN1[1] + u * (EN2[1] - EN1[1]);

      double d = sqrt ( _sqr (I[0] - P0[0])  +  _sqr (I[1] - P0[1]) );

      if ( d < growing_step )
      {
        double IEN1D  = sqrt ( _sqr  (I[0] - EN1[0])  +  _sqr  (I[1] - EN1[1]) );
        double IEN2D  = sqrt ( _sqr  (I[0] - EN2[0])  +  _sqr  (I[1] - EN2[1]) );
        double EN1EN2D = sqrt ( _sqr (EN2[0] - EN1[0])  +  _sqr (EN2[1] - EN1[1]) );

        if ( (IEN1D + IEN2D) < (EN1EN2D + 0.001) )
          en1_inliers->indices.push_back (idx);
      }
    }

    //

    pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr en1_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

    pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> en1_ei;
    en1_ei.setInputCloud (working_cloud);
    en1_ei.setIndices (en1_inliers);
    en1_ei.setNegative (false);
    en1_ei.filter (*en1_cloud);

    if ( growing_visualization )
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> en1_cloud_color (en1_cloud, 255, 0, 127);
      viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (en1_cloud, en1_cloud_color, "EN_CLOUD");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "EN_CLOUD");
      viewer.spin ();
      viewer.removePointCloud ("EN_CLOUD");
    }

    //

    pcl::PointXYZRGBNormalRSD en1_MiN, en1_MaX;
    pcl::getMinMax3D (*en1_cloud, en1_MiN, en1_MaX);

    cerr << endl;
    cerr << "          MaX = " << MaX.z << endl;
    cerr << "      en1_MaX = " << en1_MaX.z << endl;
    cerr << "      en1_dif = " << fabs (MaX.z - en1_MaX.z) << endl;
    cerr << endl;

    double en1_dif = fabs (MaX.z - en1_MaX.z);

    if ( en1_dif < growing_height )
    {
      cerr << "      OK !" << endl ;

      NP1[0] = EN1[0];
      NP1[1] = EN1[1];

      NP2[0] = EN2[0];
      NP2[1] = EN2[1];

      //pcl::copyPointCloud (*en1_cloud, *novel_box_cloud);
      novel_box_cloud->points.insert (   novel_box_cloud->points.end(),        en1_cloud->points.begin(),        en1_cloud->points.end());
      novel_box_inliers->indices.insert (novel_box_inliers->indices.end(),     en1_inliers->indices.begin(),     en1_inliers->indices.end());
    }
    else
    {
      cerr << "      NOT OK !" << endl ;

      en1_bool = false;
    }

  } while ( en1_bool );

  cerr << endl << "   EXIT !" << endl << endl ;

  if ( growing_visualization )
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> novel_box_cloud_color (novel_box_cloud, 255, 0, 127);
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (novel_box_cloud, novel_box_cloud_color, "NOVEL_BOX_CLOUD");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "NOVEL_BOX_CLOUD");
    viewer.spin ();
    viewer.removePointCloud ("NOVEL_BOX_CLOUD");
  }

  //////////////////
  // To The Front //
  //////////////////

  bool en2_bool = true;

  NP1[0] = M1[0];
  NP1[1] = M1[1];

  NP2[0] = M2[0];
  NP2[1] = M2[1];

  do
  {
    double EN1[2];
    EN1[0] = NP1[0] - N[0] * growing_step /2;
    EN1[1] = NP1[1] - N[1] * growing_step /2;

    double EN2[2];
    EN2[0] = NP2[0] - N[0] * growing_step /2;
    EN2[1] = NP2[1] - N[1] * growing_step /2;

    pcl::ModelCoefficients::Ptr en2_coeffs (new pcl::ModelCoefficients ());
    en2_coeffs->values.push_back (EN1[0]);
    en2_coeffs->values.push_back (EN1[1]);
    en2_coeffs->values.push_back (0.0);
    en2_coeffs->values.push_back (EN2[0] - EN1[0]);
    en2_coeffs->values.push_back (EN2[1] - EN1[1]);
    en2_coeffs->values.push_back (0.0);

    if ( growing_visualization )
    {
      viewer.addLine (*en2_coeffs, 1.0, 0.0, 0.5, "EN_COEFFS");
      viewer.spin ();
      viewer.removeShape ("EN_COEFFS");
    }

    //

    pcl::PointIndices::Ptr en2_inliers (new pcl::PointIndices ());

    for (int idx = 0; idx < working_cloud->points.size (); idx++)
    {
      double P0[2];
      P0[0] = working_cloud->points.at (idx).x;
      P0[1] = working_cloud->points.at (idx).y;

      double num = (P0[0] - EN1[0])*(EN2[0] - EN1[0]) + (P0[1] - EN1[1])*(EN2[1] - EN1[1]);
      double den = _sqr (EN2[0] - EN1[0])  +  _sqr (EN2[1] - EN1[1]);
      double u = num / den;

      double I[2];
      I[0] = EN1[0] + u * (EN2[0] - EN1[0]);
      I[1] = EN1[1] + u * (EN2[1] - EN1[1]);

      double d = sqrt ( _sqr (I[0] - P0[0])  +  _sqr (I[1] - P0[1]) );

      if ( d < growing_step )
      {
        double IEN1D  = sqrt ( _sqr  (I[0] - EN1[0])  +  _sqr  (I[1] - EN1[1]) );
        double IEN2D  = sqrt ( _sqr  (I[0] - EN2[0])  +  _sqr  (I[1] - EN2[1]) );
        double EN1EN2D = sqrt ( _sqr (EN2[0] - EN1[0])  +  _sqr (EN2[1] - EN1[1]) );

        if ( (IEN1D + IEN2D) < (EN1EN2D + 0.001) )
          en2_inliers->indices.push_back (idx);
      }
    }

    //

    pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr en2_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

    pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> en2_ei;
    en2_ei.setInputCloud (working_cloud);
    en2_ei.setIndices (en2_inliers);
    en2_ei.setNegative (false);
    en2_ei.filter (*en2_cloud);

    if ( growing_visualization )
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> en2_cloud_color (en2_cloud, 255, 0, 127);
      viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (en2_cloud, en2_cloud_color, "EN_CLOUD");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "EN_CLOUD");
      viewer.spin ();
      viewer.removePointCloud ("EN_CLOUD");
    }

    //

    pcl::PointXYZRGBNormalRSD en2_MiN, en2_MaX;
    pcl::getMinMax3D (*en2_cloud, en2_MiN, en2_MaX);

    cerr << endl;
    cerr << "          MaX = " << MaX.z << endl;
    cerr << "      en2_MaX = " << en2_MaX.z << endl;
    cerr << "      en2_dif = " << fabs (MaX.z - en2_MaX.z) << endl;
    cerr << endl;

    double en2_dif = fabs (MaX.z - en2_MaX.z);

    if ( en2_dif < growing_height )
    {
      cerr << "      OK !" << endl ;

      NP1[0] = EN1[0];
      NP1[1] = EN1[1];

      NP2[0] = EN2[0];
      NP2[1] = EN2[1];

      //pcl::copyPointCloud (*en2_cloud, *novel_box_cloud);
      novel_box_cloud->points.insert (   novel_box_cloud->points.end(),        en2_cloud->points.begin(),        en2_cloud->points.end());
      novel_box_inliers->indices.insert (novel_box_inliers->indices.end(),     en2_inliers->indices.begin(),     en2_inliers->indices.end());
    }
    else
    {
      cerr << "      NOT OK !" << endl ;

      en2_bool = false;
    }

  } while ( en2_bool );

  cerr << endl << "   EXIT !" << endl << endl ;

  if ( growing_visualization )
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> novel_box_cloud_color (novel_box_cloud, 255, 0, 127);
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (novel_box_cloud, novel_box_cloud_color, "NOVEL_BOX_CLOUD");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "NOVEL_BOX_CLOUD");
    viewer.spin ();
    viewer.removePointCloud ("NOVEL_BOX_CLOUD");
  }



  /*

  // Obtain Points Of Box //

  pcl::PointIndices::Ptr box_inliers (new pcl::PointIndices ());
   *box_inliers = *vransac_line_inliers;

   int K = 10;
   std::vector<int> pointIdxNKNSearch(K);
   std::vector<float> pointNKNSquaredDistance(K);

   pcl::KdTreeFLANN<pcl::PointXYZRGBNormalRSD>::Ptr box_tree (new pcl::KdTreeFLANN<pcl::PointXYZRGBNormalRSD> ());
   box_tree->setInputCloud (working_cloud);

   std::vector<bool> visited_points (working_cloud->points.size ());
   for (int vis = 0; vis < visited_points.size (); vis++)
   visited_points.at (vis) = false;

   size_t idx = 0;

   pcl::PointXYZRGBNormalRSD min_of_vlc, max_of_vlc;
   pcl::getMinMax3D (*vransac_line_cloud, min_of_vlc, max_of_vlc);

   do
   {
   pcl::PointXYZRGBNormalRSD searchPoint;
   searchPoint = working_cloud->points.at (box_inliers->indices.at (idx));

   if ( visited_points.at (box_inliers->indices.at (idx)) == false )
   {
   visited_points.at (box_inliers->indices.at (idx)) = true;

   double P0[2];
   P0[0] = working_cloud->points.at (box_inliers->indices.at (idx)).x;
   P0[1] = working_cloud->points.at (box_inliers->indices.at (idx)).y;
   P0[2] = working_cloud->points.at (box_inliers->indices.at (idx)).z;

   if ( ( P0[2] < (max_of_vlc.z + 0.005) ) && ( P0[2] > (min_of_vlc.z - 0.005) ) )
   {
   double num = (P0[0] - P1[0])*(P2[0] - P1[0]) + (P0[1] - P1[1])*(P2[1] - P1[1]);
   double den = _sqr (P2[0] - P1[0])  +  _sqr (P2[1] - P1[1]);
   double u = num / den;

   double I[2];
   I[0] = P1[0] + u * (P2[0] - P1[0]);
   I[1] = P1[1] + u * (P2[1] - P1[1]);

   double  IP1D = sqrt ( _sqr  (I[0] - P1[0])  +  _sqr  (I[1] - P1[1]) );
   double  IP2D = sqrt ( _sqr  (I[0] - P2[0])  +  _sqr  (I[1] - P2[1]) );
   double P1P2D = sqrt ( _sqr (P2[0] - P1[0])  +  _sqr (P2[1] - P1[1]) );

   if ( (IP1D + IP2D) < (P1P2D + 0.001) )
   {
   double  IVPD  = sqrt ( _sqr   (I[0] - VP[0])  +  _sqr   (I[1] - VP[1]) );
   double P0VPD  = sqrt ( _sqr  (P0[0] - VP[0])  +  _sqr  (P0[1] - VP[1]) );

   if ( P0VPD > IVPD )
   {
   if ( box_tree->nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
   {
   for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
   box_inliers->indices.push_back (pointIdxNKNSearch.at (i));
   }
   }
   }
   }
   }

   idx++;

   } while ( idx < box_inliers->indices.size() );

  sort (box_inliers->indices.begin(), box_inliers->indices.end());
  box_inliers->indices.erase (unique (box_inliers->indices.begin(), box_inliers->indices.end()), box_inliers->indices.end());

  */

    pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr box_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

  pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> b_ei;
  b_ei.setInputCloud (working_cloud);
  b_ei.setIndices (novel_box_inliers);

  b_ei.setNegative (false);
  b_ei.filter (*box_cloud);

  b_ei.setNegative (true);
  b_ei.filter (*working_cloud);

  //

  if ( working_cloud->points.size () > 0 )
  {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormalRSD> sor;
    sor.setInputCloud (working_cloud);
    sor.setMeanK (mean_k_filter);
    sor.setStddevMulThresh (std_dev_filter *5);
    sor.filter (*working_cloud);
  }

  //

  //////if ( space_step )
  //////{
  //////std::stringstream box_cloud_id;
  //////box_cloud_id << "BOX_CLOUD_" << getTimestamp ();
  //////pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> box_cloud_color (box_cloud, 127, 0, 255);
  //////viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (box_cloud, box_cloud_color, box_cloud_id.str ());
  //////viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, box_cloud_id.str ());
  //////viewer.spin ();
  //////
  //////viewer.removePointCloud ("WORKING_CLOUD");
  //////pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> working_cloud_color (working_cloud, 0, 0, 0);
  //////viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (working_cloud, working_cloud_color, "WORKING_CLOUD");
  //////viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "WORKING_CLOUD");
  //////viewer.spin ();
  //////}

  // ADD BOX SHAPE //

  double p1[3];
  p1[0] = M1[0];
  p1[1] = M1[1];
  p1[2] = 0.0;

  double p2[3];
  p2[0] = M2[0];
  p2[1] = M2[1];
  p2[2] = 0.0;

  /// Vector of Line ///
  double l[3];
  l[0] = p2[0] - p1[0];
  l[1] = p2[1] - p1[1];
  l[2] = 0.0;

  /// Normalize Vector of Line ///
  double nl = sqrt ( _sqr(l[0]) + _sqr(l[1]) + _sqr(l[2]) );
  l[0] = l[0] / nl;
  l[1] = l[1] / nl;
  l[2] = l[2] / nl;

  /// Unit Vector of Z Axis ///
  double n[3];
  n[0] = 0.0;
  n[1] = 0.0;
  n[2] = 1.0;

  /// Dot Product ///
  double m[3];
  m[0] = l[1]*n[2] - l[2]*n[1];
  m[1] = l[2]*n[0] - l[0]*n[2];
  m[2] = l[0]*n[1] - l[1]*n[0];

  // Length- and Cross- Vectors //
  double vectors[2][2];
  vectors[0][0] = l[0];
  vectors[0][1] = l[1];
  vectors[1][0] = m[0];
  vectors[1][1] = m[1];

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid (*box_cloud, centroid);

  double max_u = -DBL_MAX;
  double min_u =  DBL_MAX;
  double max_v = -DBL_MAX;
  double min_v =  DBL_MAX;

  /*

  // Bounding box only for line inliers //

  for ( int inl = 0; inl < (int) shapes_lines_inliers.at (clu)->indices.size(); inl++ )
  {
  int idx = shapes_lines_inliers.at (clu)->indices.at (inl);

  double c2p[2];
  c2p[0] = shapes_lines_clusters.at (clu)->points.at (idx).x - centroid[0];
  c2p[1] = shapes_lines_clusters.at (clu)->points.at (idx).y - centroid[1];

  double width = vectors[0][0]*c2p[0] + vectors[0][1]*c2p[1];
  if (width > max_u) max_u = width;
  if (width < min_u) min_u = width;

  double length = vectors[1][0]*c2p[0] + vectors[1][1]*c2p[1];
  if (length > max_v) max_v = length;
  if (length < min_v) min_v = length;
  }

*/

  // Bounding box for whole cluster //

  for ( int idx = 0; idx < (int) box_cloud->points.size(); idx++ )
  {
    double c2p[2];
    c2p[0] = box_cloud->points.at (idx).x - centroid[0];
    c2p[1] = box_cloud->points.at (idx).y - centroid[1];

    double width = vectors[0][0]*c2p[0] + vectors[0][1]*c2p[1];
    if (width > max_u) max_u = width;
    if (width < min_u) min_u = width;

    double length = vectors[1][0]*c2p[0] + vectors[1][1]*c2p[1];
    if (length > max_v) max_v = length;
    if (length < min_v) min_v = length;
  }

  double minimus = +DBL_MAX;
  double maximus = -DBL_MAX;

  /*

  // Bounding box only for line inliers //

  for ( int inl = 0; inl < (int) shapes_lines_inliers.at (clu)->indices.size(); inl++ )
  {
  int idx = shapes_lines_inliers.at (clu)->indices.at (inl);

  double Z = shapes_lines_clusters.at (clu)->points.at (idx).z;

  if ( minimus > Z ) minimus = Z;
  if ( maximus < Z ) maximus = Z;
  }

*/

  // Bounding box for whole cluster //

  for ( int idx = 0; idx < (int) box_cloud->points.size(); idx++ )
  {
    double Z = box_cloud->points.at (idx).z;

    if ( minimus > Z ) minimus = Z;
    if ( maximus < Z ) maximus = Z;
  }

  // The edges //

  double edges[4][2];

  edges[0][0] = centroid[0] + vectors[0][0]*max_u + vectors[1][0]*max_v;
  edges[0][1] = centroid[1] + vectors[0][1]*max_u + vectors[1][1]*max_v;

  edges[1][0] = centroid[0] + vectors[0][0]*max_u + vectors[1][0]*min_v;
  edges[1][1] = centroid[1] + vectors[0][1]*max_u + vectors[1][1]*min_v;

  edges[2][0] = centroid[0] + vectors[0][0]*min_u + vectors[1][0]*min_v;
  edges[2][1] = centroid[1] + vectors[0][1]*min_u + vectors[1][1]*min_v;

  edges[3][0] = centroid[0] + vectors[0][0]*min_u + vectors[1][0]*max_v;
  edges[3][1] = centroid[1] + vectors[0][1]*min_u + vectors[1][1]*max_v;

  //////    minimus = working_cloud_minimum.z;

  if ( shapes_from_table_plane )
    minimus = abs_min.z;

  pcl::ModelCoefficients e0, e1, e2, e3;

  e0.values.push_back (edges[0][0]);
  e0.values.push_back (edges[0][1]);
  e0.values.push_back (minimus);
  e0.values.push_back (edges[1][0] - edges[0][0]);
  e0.values.push_back (edges[1][1] - edges[0][1]);
  e0.values.push_back (minimus - minimus);

  e1.values.push_back (edges[1][0]);
  e1.values.push_back (edges[1][1]);
  e1.values.push_back (minimus);
  e1.values.push_back (edges[2][0] - edges[1][0]);
  e1.values.push_back (edges[2][1] - edges[1][1]);
  e1.values.push_back (minimus - minimus);

  e2.values.push_back (edges[2][0]);
  e2.values.push_back (edges[2][1]);
  e2.values.push_back (minimus);
  e2.values.push_back (edges[3][0] - edges[2][0]);
  e2.values.push_back (edges[3][1] - edges[2][1]);
  e2.values.push_back (minimus - minimus);

  e3.values.push_back (edges[3][0]);
  e3.values.push_back (edges[3][1]);
  e3.values.push_back (minimus);
  e3.values.push_back (edges[0][0] - edges[3][0]);
  e3.values.push_back (edges[0][1] - edges[3][1]);
  e3.values.push_back (minimus - minimus);

  /*

     std::stringstream line_0;
     line_0 << "LINE_" << ros::Time::now();
  //adjustLine (shapes_lines_clusters.at (clu), e0);
  line_viewer.addLine (e0, line_0.str ());
  line_viewer.spin ();

  std::stringstream line_1;
  line_1 << "LINE_" << ros::Time::now();
  //adjustLine (shapes_lines_clusters.at (clu), e1);
  line_viewer.addLine (e1, line_1.str ());
  line_viewer.spin ();

  std::stringstream line_2;
  line_2 << "LINE_" << ros::Time::now();
  //adjustLine (shapes_lines_clusters.at (clu), e2);
  line_viewer.addLine (e2, line_2.str ());
  line_viewer.spin ();

  std::stringstream line_3;
  line_3 << "LINE_" << ros::Time::now();
  //adjustLine (shapes_lines_clusters.at (clu), e3);
  line_viewer.addLine (e3, line_3.str ());
  line_viewer.spin ();

*/

  pcl::ModelCoefficients e4, e5, e6, e7;

  e4.values.push_back (edges[0][0]);
  e4.values.push_back (edges[0][1]);
  e4.values.push_back (maximus);
  e4.values.push_back (edges[1][0] - edges[0][0]);
  e4.values.push_back (edges[1][1] - edges[0][1]);
  e4.values.push_back (maximus - maximus);

  e5.values.push_back (edges[1][0]);
  e5.values.push_back (edges[1][1]);
  e5.values.push_back (maximus);
  e5.values.push_back (edges[2][0] - edges[1][0]);
  e5.values.push_back (edges[2][1] - edges[1][1]);
  e5.values.push_back (maximus - maximus);

  e6.values.push_back (edges[2][0]);
  e6.values.push_back (edges[2][1]);
  e6.values.push_back (maximus);
  e6.values.push_back (edges[3][0] - edges[2][0]);
  e6.values.push_back (edges[3][1] - edges[2][1]);
  e6.values.push_back (maximus - maximus);

  e7.values.push_back (edges[3][0]);
  e7.values.push_back (edges[3][1]);
  e7.values.push_back (maximus);
  e7.values.push_back (edges[0][0] - edges[3][0]);
  e7.values.push_back (edges[0][1] - edges[3][1]);
  e7.values.push_back (maximus - maximus);

  /*

     std::stringstream line_4;
     line_4 << "LINE_" << ros::Time::now();
  //adjustLine (shapes_lines_clusters.at (clu), e4);
  line_viewer.addLine (e4, line_4.str ());
  line_viewer.spin ();

  std::stringstream line_5;
  line_5 << "LINE_" << ros::Time::now();
  //adjustLine (shapes_lines_clusters.at (clu), e5);
  line_viewer.addLine (e5, line_5.str ());
  line_viewer.spin ();

  std::stringstream line_6;
  line_6 << "LINE_" << ros::Time::now();
  //adjustLine (shapes_lines_clusters.at (clu), e6);
  line_viewer.addLine (e6, line_6.str ());
  line_viewer.spin ();

  std::stringstream line_7;
  line_7 << "LINE_" << ros::Time::now();
  //adjustLine (shapes_lines_clusters.at (clu), e7);
  line_viewer.addLine (e7, line_7.str ());
  line_viewer.spin ();

*/

  std::vector<pcl::ModelCoefficients> cub;

  cub.push_back (e0);
  cub.push_back (e1);
  cub.push_back (e2);
  cub.push_back (e3);
  cub.push_back (e4);
  cub.push_back (e5);
  cub.push_back (e6);
  cub.push_back (e7);

  cerr << e0.values.at(0) << " " << e0.values.at(1) << " " << e0.values.at(2) << " " << e0.values.at(3) << " " << e0.values.at(4) << " " << e0.values.at(5) << endl ;
  cerr << e1.values.at(0) << " " << e1.values.at(1) << " " << e1.values.at(2) << " " << e1.values.at(3) << " " << e1.values.at(4) << " " << e1.values.at(5) << endl ;
  cerr << e2.values.at(0) << " " << e2.values.at(1) << " " << e2.values.at(2) << " " << e2.values.at(3) << " " << e2.values.at(4) << " " << e2.values.at(5) << endl ;
  cerr << e3.values.at(0) << " " << e3.values.at(1) << " " << e3.values.at(2) << " " << e3.values.at(3) << " " << e3.values.at(4) << " " << e3.values.at(5) << endl ;
  cerr << e4.values.at(0) << " " << e4.values.at(1) << " " << e4.values.at(2) << " " << e4.values.at(3) << " " << e4.values.at(4) << " " << e4.values.at(5) << endl ;
  cerr << e5.values.at(0) << " " << e5.values.at(1) << " " << e5.values.at(2) << " " << e5.values.at(3) << " " << e5.values.at(4) << " " << e5.values.at(5) << endl ;
  cerr << e6.values.at(0) << " " << e6.values.at(1) << " " << e6.values.at(2) << " " << e6.values.at(3) << " " << e6.values.at(4) << " " << e6.values.at(5) << endl ;
  cerr << e7.values.at(0) << " " << e7.values.at(1) << " " << e7.values.at(2) << " " << e7.values.at(3) << " " << e7.values.at(4) << " " << e7.values.at(5) << endl ;



















  // NOVEL WAY OF GROWING BOXES //

  return cub;
}

