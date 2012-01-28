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

typedef pcl::PointXYZ PointT; //

// ---------- Parameters ---------- //

// Visualization //
bool verbose = false;
bool step = false;
int size = 1;

// Control //
bool transform = false;
bool again = false;

// Transforming //
double x = 0.0;
double y = 0.0;
double z = 0.0;
double roll = 0.0;
double pitch = 0.0;
double yaw = 0.0;

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

  cerr << endl << argv [pcd_file_indices [0]] << endl ;
  cerr << argv [pcd_file_indices [1]] << endl << endl ;

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

  // Transforming //
  pcl::console::parse_argument (argc, argv, "-x", x);
  pcl::console::parse_argument (argc, argv, "-y", y);
  pcl::console::parse_argument (argc, argv, "-z", z);
  pcl::console::parse_argument (argc, argv, "-roll", roll);
  pcl::console::parse_argument (argc, argv, "-pitch", pitch);
  pcl::console::parse_argument (argc, argv, "-yaw", yaw);

  // ---------- Initializations ---------- //

  srand (time(0));

  pcl::console::TicToc tt;

  tt.tic ();

  if ( verbose ) pcl::console::print_warn ("Timer started !\n");

  // ---------- 3D Viewer ---------- //

  pcl::visualization::PCLVisualizer viewer ("3D VIEWER");
  viewer.setBackgroundColor (0.0, 0.0, 0.0);
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
    pcl::visualization::PointCloudColorHandlerCustom<PointT> input_color (input_cloud, 255, 255, 255);
    viewer.addPointCloud<PointT> (input_cloud, input_color, "generic");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "generic");
    viewer.spin ();
  }

  // ---------- Add Plane ---------- //

  if ( step )
  {
    pcl::ModelCoefficients coeffs;
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (1.0);
    coeffs.values.push_back (0.0);
    viewer.addPlane (coeffs, 0.0, 1.0, 1.0, "plane");
    viewer.spin ();
  }

  // ---------- Introduce Working Cloud ---------- //

  pcl::PointCloud<PointT>::Ptr working_cloud (new pcl::PointCloud<PointT> ());

  pcl::copyPointCloud (*input_cloud, *working_cloud);

  // ---------- Transforming Point Cloud ---------- //

  if ( transform )
  {
    if ( verbose ) pcl::console::print_info ("Transforming point cloud data...\n");

    do
    {
      Eigen::Affine3f t;
      pcl::getTransformation (x, y, z, roll, pitch, yaw, t);
      pcl::transformPointCloud (*working_cloud, *working_cloud, t);

      if ( step )
      {
        viewer.removePointCloud ("generic");
        pcl::visualization::PointCloudColorHandlerCustom<PointT> working_color (working_cloud, 255, 255, 255);
        viewer.addPointCloud<PointT> (working_cloud, working_color, "generic");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "generic");
        viewer.spin ();
      }

      printf ("Again ? 0/1 = ");
      scanf ("%d", &again);

    } while ( again == 1 );
  }
  else if ( verbose ) pcl::console::print_info ("No transforming...\n");

  // ---------- Save Transfomred Data ---------- //

  pcl::io::savePCDFile (argv [pcd_file_indices [1]], *working_cloud, 10);


  // ---------- The End ---------- //

  if ( verbose ) pcl::console::print_warn ("Finished in %g [ms] !\n", tt.toc ());

  if ( step ) viewer.spin ();

  return (0);
}
