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

// ---------- Dependencies ---------- //

#include "pcl/console/parse.h"
#include "pcl/console/time.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"

// ---------- Variables ---------- //

// Fields //

// Visualization //
bool verbose = false;
bool step = false;
int size = 1;

/////////////////////////////////////////////////////////////
/** \brief Print the usage instructions for the code at hand.
 * \param command The command line binary file.
 */
void printUsage (const char* command)
{
  pcl::console::print_info ("\nThe syntax for the vransac segmentation is:\n");
  pcl::console::print_info ("\n  %s [input].pcd [options]\n", command);
  pcl::console::print_info ("\nWhere options are the following for:\n");
  pcl::console::print_info ("\n[fields]\n");
  pcl::console::print_info ("\n[points]\n");
  pcl::console::print_info ("\n[visualization]\n");
  pcl::console::print_info ("  -verbose B                                                   = Show all print messages.\n");
  pcl::console::print_info ("  -step B                                                      = Wait or not wait.\n");
  pcl::console::print_info ("  -size D                                                      = Size of points from cloud.\n");

  pcl::console::print_info ("\n");
  return;
}

/////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Main routine of method. Segmentation of 3D point clouds by Voting RANSAC fitted models.
*/
int main (int argc, char** argv)
{

  // ---------- Command Line Arguments ---------- //

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

  // Fields //

  // Points //

  // Visualization //
  pcl::console::parse_argument (argc, argv, "-verbose", verbose);
  pcl::console::parse_argument (argc, argv, "-step", step);
  pcl::console::parse_argument (argc, argv, "-size", size);

  // ---------- Initializations ---------- //

  srand (time(0));

  pcl::console::TicToc tt;

  tt.tic ();

  if ( verbose ) pcl::console::print_warn ("Timer started !\n");

  // ---------- 3D Viewer ---------- //

  /*
  pcl::visualization::PCLVisualizer viewer ("3D VIEWER");

  viewer.setBackgroundColor (1.0, 1.0, 1.0);
  viewer.addCoordinateSystem (0.25f);
  viewer.getCameraParameters (argc, argv);
  viewer.updateCamera ();
  */

  // ---------- Load Input Data Sets ---------- //

  pcl::PointCloud<pcl::PointXYZI>::Ptr xyz_i_cloud (new pcl::PointCloud<pcl::PointXYZI> ());

  if (pcl::io::loadPCDFile (argv [pcd_file_indices [0]], *xyz_i_cloud) == -1)
  {
    pcl::console::print_error ("Couldn't read file %s\n", argv [pcd_file_indices [0]]);
    return (-1);
  }

  if ( verbose ) pcl::console::print_info ("Loaded %d points from %s with fields: %s\n", (int) (xyz_i_cloud->points.size ()), argv [pcd_file_indices [0]], pcl::getFieldsList (*xyz_i_cloud).c_str ());

  //

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_rgb_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

  if (pcl::io::loadPCDFile (argv [pcd_file_indices [1]], *xyz_rgb_cloud) == -1)
  {
    pcl::console::print_error ("Couldn't read file %s\n", argv [pcd_file_indices [1]]);
    return (-1);
  }

  if ( verbose ) pcl::console::print_info ("Loaded %d points from %s with fields: %s\n", (int) (xyz_rgb_cloud->points.size ()), argv [pcd_file_indices [1]], pcl::getFieldsList (*xyz_rgb_cloud).c_str ());

  // ---------- Set Output Data File ---------- //

  pcl::PointCloud<pcl::PointXYZRGBI>::Ptr xyz_rgb_l_cloud (new pcl::PointCloud<pcl::PointXYZRGBI> ());

  if (verbose) pcl::console::print_info (" %s \n", argv [pcd_file_indices [2]]);

  std::string path = argv [pcd_file_indices [2]];

  /*
  xyz_rgb_l_cloud->points.resize (xyz_i_cloud->points.size ());
  xyz_rgb_l_cloud->width        = xyz_i_cloud->width;
  xyz_rgb_l_cloud->height       = xyz_i_cloud->height;

  if (xyz_i_cloud->is_dense) xyz_rgb_l_cloud->is_dense = true; else xyz_rgb_l_cloud->is_dense = false;
  */

  pcl::copyPointCloud (*xyz_rgb_cloud, *xyz_rgb_l_cloud);

  for (size_t i=0; i < xyz_rgb_l_cloud->points.size (); ++i)
    xyz_rgb_l_cloud->points.at (i).intensity = xyz_i_cloud->points.at (i).intensity;

  /*
  for (size_t ii = 0; ii < xyz_rgb_l_cloud->points.size (); ii++)
    for (size_t jj = 0; jj < xyz_rgb_cloud->points.size (); jj++)
      if ((xyz_rgb_l_cloud->points.at (ii).x == xyz_rgb_cloud->points.at (jj).x) &&
          (xyz_rgb_l_cloud->points.at (ii).y == xyz_rgb_cloud->points.at (jj).y) &&
          (xyz_rgb_l_cloud->points.at (ii).z == xyz_rgb_cloud->points.at (jj).z))

      {
        xyz_rgb_l_cloud->points.at (ii).rgb = xyz_rgb_cloud->points.at (jj).rgb;

        cerr << "." ;
      }
  */

  pcl::io::savePCDFileASCII (argv [pcd_file_indices [2]], *xyz_rgb_l_cloud);

  return (0);
}
