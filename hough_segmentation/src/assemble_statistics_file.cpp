
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

//#include <stdio.h>

#include "pcl/console/parse.h"
#include "pcl/io/pcd_io.h"

// Visualization //
bool verbose = true;
bool step = false;

// Centroids //
double threshold_between_centroids_of_cuboids = 0.0;
double threshold_between_centroids_of_cylinders = 0.0;

/////////////////////////////////////////////////////////////
/** \brief Print the usage instructions for the code at hand.
 * \param command The command line binary file.
*/
void printUsage (const char* command)
{
  pcl::console::print_info ("\nThe syntax for the vransac segmentation is:\n");
  pcl::console::print_info ("\n  %s [input1].txt [input2].txt [options]\n", command);
  pcl::console::print_info ("\nWhere options are the following for:\n");
  pcl::console::print_info ("\n[visualization]\n");
  pcl::console::print_info ("  -verbose B                                                   = Show all print messages.\n");
  pcl::console::print_info ("  -step B                                                      = Wait or not wait.\n");
  pcl::console::print_info ("\n[centroids]\n");
  pcl::console::print_info ("  -threshold_between_centroids_of_cuboids X                    = \n");
  pcl::console::print_info ("  -threshold_between_centroids_of_cylinders X                  = \n");
  pcl::console::print_info ("\n");

  return;
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

  std::vector<int> txt_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".txt");

  if (txt_file_indices.size () == 0)
  {
    pcl::console::print_error ("No .txt file given as input!\n");
    pcl::console::print_info  ("No .txt file given as input!\n");
    pcl::console::print_value ("No .txt file given as input!\n");
    pcl::console::print_warn  ("No .txt file given as input!\n");
    return (-1);
  }

  // Visualization //
  pcl::console::parse_argument (argc, argv, "-verbose", verbose);
  pcl::console::parse_argument (argc, argv, "-step", step);

  // Centroids //
  pcl::console::parse_argument (argc, argv, "-threshold_between_centroids_of_cuboids", threshold_between_centroids_of_cuboids);
  pcl::console::parse_argument (argc, argv, "-threshold_between_centroids_of_cylinders", threshold_between_centroids_of_cylinders);



  // ---------- Loading Text Files ---------- //

  int v1 = 0;

  FILE * file1;

  file1 = fopen (argv [txt_file_indices [v1]], "r");

  char line1[259];

  while ( fgets (line1, 259, file1) != NULL )
  {
    printf (" %s ", line1);



    int v1_flag;    

    sscanf (line1, " %d ", &v1_flag);

    if ( v1_flag == 0 )
    {

      double v1_d1, v1_d2, v1_d3, v1_vo, v1_cx, v1_cy, v1_cz;

      sscanf (line1, " %d %lf %lf %lf %lf | %lf %lf %lf ", &v1_flag, &v1_d1, &v1_d2, &v1_d3, &v1_vo, &v1_cx, &v1_cy, &v1_cz);

      printf (" %d \n %12.10lf \n %12.10lf \n %12.10lf \n %12.10lf \n %12.10lf \n %12.10lf \n %12.10lf ", v1_flag, v1_d1, v1_d2, v1_d3, v1_vo, v1_cx, v1_cy, v1_cz);

    }

    if ( v1_flag == 1 )
    {

      double r, h, v, cen1, cen2, cen3;

      sscanf (line1, " %d %lf %lf %lf | %lf %lf %lf ", &v1_flag, &r, &h, &v, &cen1, &cen2, &cen3);

      printf (" %d \n %12.10lf \n %12.10lf \n %12.10lf \n %12.10lf \n %12.10lf \n %12.10lf ", v1_flag, r, h, v, cen1, cen2, cen3);

    }





    getchar ();

    //if ( action1 == -1 ) std::cerr << std::endl;
    //
    //
    //
    //printf (" %d ", action1);
    //
    //
    //
    ////a[i] = limit;
    //i++;

    }
















  fclose (file1);



















  //FILE * file2;
  //
  //file2 = fopen (argv [txt_file_indices [1]], "r");
  //
  //
  //
  //fclose (file2);
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //FILE * file;
  //
  //file = fopen ("characteristics-of-hough-voted-ransac-models.txt", "a");
  //
  //
  //
  //fclose (file);
























  return (0);
}
