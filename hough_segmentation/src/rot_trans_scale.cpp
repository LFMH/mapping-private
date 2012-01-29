/*
 * Copyright (c) 2012, Lucian Cosmin Goron <lucian.goron@aut.utcluj.ro>
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

// ---------- Dependencies ---------- //

#include <string>

//

#include "pcl/console/parse.h"
#include "pcl/filters/crop_box.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"

//

#include "pcl/filters/impl/crop_box.hpp"

// ---------- Types ---------- //

typedef pcl::PointXYZ PointT;

// ---------- Parameters ---------- //

// Visualization //
bool verbose = false;
bool step = false;
int size = 1;

// Control //
bool transform = false;
bool merge = false;
int again = 0;

// Transforming //
float x = 0.0;
float y = 0.0;
float z = 0.0;
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

// ---------- Macros ---------- //

#define _sqr(c) ((c)*(c))

////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Print the usage instructions for the code at hand.
 * \param command The command line binary file.
 */
void printUsage (const char* command)
{
  pcl::console::print_info ("\nThe syntax for the vransac segmentation is:\n");
  pcl::console::print_info ("\n  %s [input].pcd [options]\n", command);
  pcl::console::print_info ("\nWhere options are the following for:\n");
  pcl::console::print_info ("\n[visualization]\n");
  pcl::console::print_info ("  -verbose B                                                   = Show all print messages.\n");
  pcl::console::print_info ("  -size D                                                      = Set size of points.\n");
  pcl::console::print_info ("  -step B                                                      = Wait or not wait.\n");
  return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Get the current timestamp as an unique key
*/
std::string getTimestamp ()
{
  time_t raw_time;
  struct tm *time_info;
  struct timeval fine_time;

  raw_time = time (NULL);
  time_info = localtime (&raw_time);
  gettimeofday (&fine_time, NULL);

  std::stringstream ss;
  ss << std::setw (4) << std::setfill ('0') << time_info->tm_year + 1900;
  ss << std::setw (2) << std::setfill ('0') << time_info->tm_mon;
  ss << std::setw (2) << std::setfill ('0') << time_info->tm_mday;
  ss << ".";
  ss << std::setw (2) << std::setfill ('0') << time_info->tm_hour;
  ss << std::setw (2) << std::setfill ('0') << time_info->tm_min;
  ss << std::setw (2) << std::setfill ('0') << time_info->tm_sec;
  ss << ".";
  ss << std::setw (3) << std::setfill ('0') << fine_time.tv_usec / 1000;
  ss << ".";
  ss << std::setw (3) << std::setfill ('0') << fine_time.tv_usec - fine_time.tv_usec / 1000 * 1000;

  return ss.str ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Main routine of method. Segmentation of 3D point clouds by Voting RANSAC fitted models.
*/
int main (int argc, char** argv)
{

  // ---------- Arguments ---------- //

  if (argc < 2)
  {
    printUsage (argv[0]);
    return (-1);
  }

  if (pcl::console::find_argument (argc, argv, "-h") > 0)
  {
    printUsage (argv[0]);
    pcl::console::print_info ("\nContinuing...\n\n");
  }

  std::vector<int> pcd_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

  if (pcd_file_indices.size () == 0)
  {
    pcl::console::print_error ("No .pcd file given as input!\n");
    pcl::console::print_info  ("No .pcd file given as input!\n");
    pcl::console::print_value ("No .pcd file given as input!\n");
    pcl::console::print_warn  ("No .pcd file given as input!\n");
    return (-1);
  }

  // Visualization //
  pcl::console::parse_argument (argc, argv, "-verbose", verbose);
  pcl::console::parse_argument (argc, argv, "-step", step);
  pcl::console::parse_argument (argc, argv, "-size", size);

  // Control //
  pcl::console::parse_argument (argc, argv, "-transform", transform);
  pcl::console::parse_argument (argc, argv, "-merge", merge);

  // Transforming //
  pcl::console::parse_argument (argc, argv, "-x", x);
  pcl::console::parse_argument (argc, argv, "-y", y);
  pcl::console::parse_argument (argc, argv, "-z", z);
  pcl::console::parse_argument (argc, argv, "-roll", roll);
  pcl::console::parse_argument (argc, argv, "-pitch", pitch);
  pcl::console::parse_argument (argc, argv, "-yaw", yaw);

  if ( transform ) cerr << endl << argv [pcd_file_indices [0]] << endl << argv [pcd_file_indices [1]] << endl << endl ;

  if ( merge ) cerr << endl << argv [pcd_file_indices [0]] << endl << argv [pcd_file_indices [1]] << endl << argv [pcd_file_indices [2]] << endl << endl ;

  // ---------- Initializations ---------- //

  srand (time(0));

  pcl::console::TicToc tt;

  tt.tic ();

  if ( verbose ) pcl::console::print_warn ("Timer started !\n");

  // ---------- 3D Viewer ---------- //

  pcl::visualization::PCLVisualizer viewer ("3D VIEWER");
  viewer.setBackgroundColor (1.0, 1.0, 1.0);
  viewer.addCoordinateSystem (1.0f);
  viewer.getCameraParameters (argc, argv);
  viewer.updateCamera ();

  // ---------- Load Input Data ---------- //

  pcl::PointCloud<PointT>::Ptr input_cloud (new pcl::PointCloud<PointT> ());

  if (pcl::io::loadPCDFile (argv [pcd_file_indices [0]], *input_cloud) == -1)
  {
    pcl::console::print_error ("Couldn't read file %s\n", argv [pcd_file_indices [0]]);
    return (-1);
  }

  if ( verbose ) pcl::console::print_info ("Loaded %d points from %s with following fields: %s\n", (int) (input_cloud->points.size ()), argv [pcd_file_indices [0]], pcl::getFieldsList (*input_cloud).c_str ());

  if ( step )
  {
    pcl::visualization::PointCloudColorHandlerCustom<PointT> input_color (input_cloud, 0, 0, 0);
    viewer.addPointCloud<PointT> (input_cloud, input_color, "generic");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "generic");
    // viewer.spin ();
  }

  // ---------- Introduce Working Cloud ---------- //

  pcl::PointCloud<PointT>::Ptr working_cloud (new pcl::PointCloud<PointT> ());

  pcl::copyPointCloud (*input_cloud, *working_cloud);

  // ---------------------------------------------- //
  // ---------- Transforming Point Cloud ---------- //
  // ---------------------------------------------- //

  if ( transform )
  {
    if ( verbose ) pcl::console::print_info ("Transforming point cloud data...\n");

    /*

    // ---------- Add Plane ---------- //

    if ( step )
    {
    pcl::ModelCoefficients coeffs;
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (1.0);
    coeffs.values.push_back (0.0);
    viewer.addPlane (coeffs, 0.0, 1.0, 1.0, "plane");
    //viewer.spin ();
    }

    */

    /*

    // ---------- Add Cuboid ---------- //

    pcl::ModelCoefficients e0, e1, e2, e3;

    e0.values.push_back (-0.900139);
    e0.values.push_back (-1.16862);
    e0.values.push_back (0.443211);
    e0.values.push_back (-0.0269661);
    e0.values.push_back (-0.0386442);
    e0.values.push_back (0);

    e1.values.push_back (-0.927105);
    e1.values.push_back (-1.20727);
    e1.values.push_back (0.443211);
    e1.values.push_back (0.111587);
    e1.values.push_back (-0.0778655);
    e1.values.push_back (0);

    e2.values.push_back (-0.815519);
    e2.values.push_back (-1.28513);
    e2.values.push_back (0.443211);
    e2.values.push_back (0.0269661);
    e2.values.push_back (0.0386442);
    e2.values.push_back (0);

    e3.values.push_back (-0.788553);
    e3.values.push_back (-1.24649);
    e3.values.push_back (0.443211);
    e3.values.push_back (-0.111587);
    e3.values.push_back (0.0778655);
    e3.values.push_back (0);

    pcl::ModelCoefficients e4, e5, e6, e7;

    e4.values.push_back (-0.900139);
    e4.values.push_back (-1.16862);
    e4.values.push_back (0.632181);
    e4.values.push_back (-0.0269661);
    e4.values.push_back (-0.0386442);
    e4.values.push_back (0);

    e5.values.push_back (-0.927105);
    e5.values.push_back (-1.20727);
    e5.values.push_back (0.632181);
    e5.values.push_back (0.111587);
    e5.values.push_back (-0.0778655);
    e5.values.push_back (0);

    e6.values.push_back (-0.815519);
    e6.values.push_back (-1.28513);
    e6.values.push_back (0.632181);
    e6.values.push_back (0.0269661);
    e6.values.push_back (0.0386442);
    e6.values.push_back (0);

    e7.values.push_back (-0.788553);
    e7.values.push_back (-1.24649);
    e7.values.push_back (0.632181);
    e7.values.push_back (-0.111587);
    e7.values.push_back (0.0778655);
    e7.values.push_back (0);

    std::vector<pcl::ModelCoefficients> cub;

    cub.push_back (e0);
    cub.push_back (e1);
    cub.push_back (e2);
    cub.push_back (e3);
    cub.push_back (e4);
    cub.push_back (e5);
    cub.push_back (e6);
    cub.push_back (e7);

    std::stringstream cub_id;
    cub_id << "CUB_" << getTimestamp ();
    viewer.addCuboid (cub, 0.5, 0.0, 1.0, 0.5, cub_id.str ());

    */

    /*

    // ---------- Add Cuboid ---------- //

    pcl::ModelCoefficients e0, e1, e2, e3;

    e0.values.push_back (-0.440659);
    e0.values.push_back (-1.25178);
    e0.values.push_back (0.441576);
    e0.values.push_back (-0.0192634);
    e0.values.push_back (0.0785663);
    e0.values.push_back (0);

    e1.values.push_back (-0.459923);
    e1.values.push_back (-1.17321);
    e1.values.push_back (0.441576);
    e1.values.push_back (-0.137991);
    e1.values.push_back (-0.0338334);
    e1.values.push_back (0);

    e2.values.push_back (-0.597914);
    e2.values.push_back (-1.20704);
    e2.values.push_back (0.441576);
    e2.values.push_back (0.0192634);
    e2.values.push_back (-0.0785663);
    e2.values.push_back (0);

    e3.values.push_back (-0.57865);
    e3.values.push_back (-1.28561);
    e3.values.push_back (0.441576);
    e3.values.push_back (0.137991);
    e3.values.push_back (0.0338334);
    e3.values.push_back (0);

    pcl::ModelCoefficients e4, e5, e6, e7;

    e4.values.push_back (-0.440659);
    e4.values.push_back (-1.25178);
    e4.values.push_back (0.636602);
    e4.values.push_back (-0.0192634);
    e4.values.push_back (0.0785663);
    e4.values.push_back (0);

    e5.values.push_back (-0.459923);
    e5.values.push_back (-1.17321);
    e5.values.push_back (0.636602);
    e5.values.push_back (-0.137991);
    e5.values.push_back (-0.0338334);
    e5.values.push_back (0);

    e6.values.push_back (-0.597914);
    e6.values.push_back (-1.20704);
    e6.values.push_back (0.636602);
    e6.values.push_back (0.0192634);
    e6.values.push_back (-0.0785663);
    e6.values.push_back (0);

    e7.values.push_back (-0.57865);
    e7.values.push_back (-1.28561);
    e7.values.push_back (0.636602);
    e7.values.push_back (0.137991);
    e7.values.push_back (0.0338334);
    e7.values.push_back (0);

    std::vector<pcl::ModelCoefficients> cub;

    cub.push_back (e0);
    cub.push_back (e1);
    cub.push_back (e2);
    cub.push_back (e3);
    cub.push_back (e4);
    cub.push_back (e5);
    cub.push_back (e6);
    cub.push_back (e7);

    std::stringstream cub_id;
    cub_id << "CUB_" << getTimestamp ();
    viewer.addCuboid (cub, 0.5, 0.0, 1.0, 0.5, cub_id.str ());

    */

    do
    {
      Eigen::Affine3f t;
      pcl::getTransformation (x, y, z, roll, pitch, yaw, t);
      pcl::transformPointCloud (*working_cloud, *working_cloud, t);

      if ( step )
      {
        viewer.removePointCloud ("generic");
        pcl::visualization::PointCloudColorHandlerCustom<PointT> working_color (working_cloud, 0, 0, 0);
        viewer.addPointCloud<PointT> (working_cloud, working_color, "generic");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "generic");
        viewer.spin ();
      }

      // printf ("Again? 0/1 = "); scanf ("%d", &again);

      if ( again == 1 )
      {
        printf ("x = "); scanf ("%f", &x);
        printf ("y = "); scanf ("%f", &y);
        printf ("z = "); scanf ("%f", &z);
        printf (" roll = "); scanf ("%f", &roll);
        printf ("pitch = "); scanf ("%f", &pitch);
        printf ("  yaw = "); scanf ("%f", &yaw);
      }

    } while ( again == 1 );

    pcl::io::savePCDFile (argv [pcd_file_indices [1]], *working_cloud, 10);

    if ( verbose ) pcl::console::print_info ("Saved %d points to %s with following fields: %s\n", (int) (working_cloud->points.size ()), argv [pcd_file_indices [1]], pcl::getFieldsList (*working_cloud).c_str ());
  }
  else if ( verbose ) pcl::console::print_info ("No transforming...\n");

  // ------------------------------------------ //
  // ---------- Merging Point Clouds ---------- //
  // ------------------------------------------ //

  if ( merge )
  {
    if ( verbose ) pcl::console::print_info ("Merging point clouds...\n");

    // ---------- Load Second Cloud ---------- //

    pcl::PointCloud<PointT>::Ptr second_cloud (new pcl::PointCloud<PointT> ());

    if (pcl::io::loadPCDFile (argv [pcd_file_indices [1]], *second_cloud) == -1)
    {
      pcl::console::print_error ("Couldn't read file %s\n", argv [pcd_file_indices [1]]);
      return (-1);
    }

    if ( verbose ) pcl::console::print_info ("Loaded %d points from %s with following fields: %s\n", (int) (second_cloud->points.size ()), argv [pcd_file_indices [1]], pcl::getFieldsList (*second_cloud).c_str ());

    if ( step )
    {
      pcl::visualization::PointCloudColorHandlerCustom<PointT> second_color (second_cloud, 255, 0, 0);
      viewer.addPointCloud<PointT> (second_cloud, second_color, "second");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "second");
      // viewer.spin ();
    }

    // ---------- Transform Input Cloud ---------- //

    Eigen::Affine3f t;
    pcl::getTransformation (x, y, z, roll, pitch, yaw, t);
    pcl::transformPointCloud (*working_cloud, *working_cloud, t);

    if ( step )
    {
      viewer.removePointCloud ("generic");
      pcl::visualization::PointCloudColorHandlerCustom<PointT> working_color (working_cloud, 0, 0, 0);
      viewer.addPointCloud<PointT> (working_cloud, working_color, "generic");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "generic");
      viewer.spin ();
    }

    *working_cloud += *second_cloud;

    pcl::io::savePCDFile (argv [pcd_file_indices [2]], *working_cloud, 10);

    if ( verbose ) pcl::console::print_info ("Saved %d points to %s with following fields: %s\n", (int) (working_cloud->points.size ()), argv [pcd_file_indices [2]], pcl::getFieldsList (*working_cloud).c_str ());
  }
  else if ( verbose ) pcl::console::print_info ("No mergingg...\n");

  // ---------- The End ---------- //

  if ( verbose ) pcl::console::print_warn ("Finished in %g [ms] !\n", tt.toc ());

  // if ( step ) viewer.spin ();

  return (0);
}
