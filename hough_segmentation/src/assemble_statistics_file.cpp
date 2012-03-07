
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
#include "pcl/io/pcd_io.h"

// ---------- Namespaces ---------- //

using namespace std;

// ---------- Variables ---------- //

// Visualization //
bool verbose = true;
bool step = false;

// Centroids //
double threshold_between_centroids_of_cuboids = 0.0;
double threshold_between_centroids_of_cylinders = 0.0;

// ---------- Macros ---------- //

#define _sqr(c) ((c)*(c))

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

  vector<int> txt_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".txt");

  if (txt_file_indices.size() == 0)
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

  // ---------------------------------------- //
  // ---------- Loading Text Files ---------- //
  // ---------------------------------------- //

  vector<vector<vector<double> > > v1_models;

  {
    int v1 = 0; // IMPORTANT

    FILE * v1_file;
    v1_file = fopen (argv [txt_file_indices [v1]], "r");

    char v1_line[255];

    vector<vector<double> > per_run;

    while ( fgets (v1_line, 255, v1_file) != NULL )
    {
      printf (" %s ", v1_line);

      int v1_flag;
      sscanf (v1_line, " %d ", &v1_flag);

      vector<double> v1_model;

      if ( v1_flag == 1 ) // BOX
      {
        double d1, d2, d3, v, c1, c2, c3;
        sscanf (v1_line, " %d | %lf %lf %lf %lf | %lf %lf %lf ", &v1_flag, &d1, &d2, &d3, &v, &c1, &c2, &c3);
        printf ("     %12.10lf %12.10lf %12.10lf %12.10lf   %12.10lf %12.10lf %12.10lf \n", d1, d2, d3, v, c1, c2, c3);

        v1_model.push_back (d1);
        v1_model.push_back (d2);
        v1_model.push_back (d3);
        v1_model.push_back (v);
        v1_model.push_back (c1);
        v1_model.push_back (c2);
        v1_model.push_back (c3);

        per_run.push_back (v1_model);
      }

      if ( v1_flag == 2 ) // CYLINDER
      {
        double r, h, v, c1, c2, c3;
        sscanf (v1_line, " %d | %lf %lf %lf | %lf %lf %lf ", &v1_flag, &r, &h, &v, &c1, &c2, &c3);
        printf ("     %12.10lf %12.10lf %12.10lf   %12.10lf %12.10lf %12.10lf \n", r, h, v, c1, c2, c3);

        v1_model.push_back (r);
        v1_model.push_back (h);
        v1_model.push_back (v);
        v1_model.push_back (c1);
        v1_model.push_back (c2);
        v1_model.push_back (c3);

        per_run.push_back (v1_model);
      }

      if ( v1_flag == 0 ) // NEW LINE
      {
        v1_models.push_back (per_run);
        //cerr << " per_run.size() " << per_run.size() << endl ;

        per_run.clear ();
        //cerr << " per_run.size() " << per_run.size() << endl ;
      }

      cerr << endl ;

      if ( step ) getchar ();
    }

    fclose (v1_file);

    cerr << " v1_models.size() " << v1_models.size() << endl ;
    for (int r = 0; r < v1_models.size(); r++)
    {
      cerr << " v1_models.at(r).size() " << v1_models.at(r).size() << endl ;
      for (int m = 0; m < v1_models.at (r).size(); m++)
        cerr << " v1_models.at(r).at(m).size() " << v1_models.at(r).at(m).size() << endl ;
    }

    cerr << endl ;
  }

  vector<vector<vector<double> > > v2_models;

  {
    int v2 = 1; // IMPORTANT

    FILE * v2_file;
    v2_file = fopen (argv [txt_file_indices [v2]], "r");

    char v2_line[255];

    vector<vector<double> > per_run;

    while ( fgets (v2_line, 255, v2_file) != NULL )
    {
      printf (" %s ", v2_line);

      int v2_flag;
      sscanf (v2_line, " %d ", &v2_flag);

      vector<double> v2_model;

      if ( v2_flag == 1 ) // BOX
      {
        double d1, d2, d3, v, c1, c2, c3;
        sscanf (v2_line, " %d | %lf %lf %lf %lf | %lf %lf %lf ", &v2_flag, &d1, &d2, &d3, &v, &c1, &c2, &c3);
        printf ("     %12.10lf %12.10lf %12.10lf %12.10lf   %12.10lf %12.10lf %12.10lf \n", d1, d2, d3, v, c1, c2, c3);

        v2_model.push_back (d1);
        v2_model.push_back (d2);
        v2_model.push_back (d3);
        v2_model.push_back (v);
        v2_model.push_back (c1);
        v2_model.push_back (c2);
        v2_model.push_back (c3);

        per_run.push_back (v2_model);
      }

      if ( v2_flag == 2 ) // CYLINDER
      {
        double r, h, v, c1, c2, c3;
        sscanf (v2_line, " %d | %lf %lf %lf | %lf %lf %lf ", &v2_flag, &r, &h, &v, &c1, &c2, &c3);
        printf ("     %12.10lf %12.10lf %12.10lf   %12.10lf %12.10lf %12.10lf \n", r, h, v, c1, c2, c3);

        v2_model.push_back (r);
        v2_model.push_back (h);
        v2_model.push_back (v);
        v2_model.push_back (c1);
        v2_model.push_back (c2);
        v2_model.push_back (c3);

        per_run.push_back (v2_model);
      }

      if ( v2_flag == 0 ) // NEW LINE
      {
        v2_models.push_back (per_run);
        //cerr << " per_run.size() " << per_run.size() << endl ;

        per_run.clear ();
        //cerr << " per_run.size() " << per_run.size() << endl ;
      }

      cerr << endl ;

      if ( step ) getchar ();
    }

    fclose (v2_file);

    cerr << " v2_models.size() " << v2_models.size() << endl ;
    for (int r = 0; r < v2_models.size(); r++)
    {
      cerr << " v2_models.at(r).size() " << v2_models.at(r).size() << endl ;
      for (int m = 0; m < v2_models.at (r).size(); m++)
        cerr << " v2_models.at(r).at(m).size() " << v2_models.at(r).at(m).size() << endl ;
    }

    cerr << endl ;
  }

  // ------------------------------------------------ //
  // ---------- Generating Statistics File ---------- //
  // ------------------------------------------------ //

  FILE * file;

  file = fopen ("hough-voted-ransac-models.txt", "a");

  int run = 0; // IMPORTANT

  for (int model = 0; model < v1_models.at(run).size(); model++)
  {

    // ------------------ //
    // Dealing with boxes //
    // ------------------ //

    if ( v1_models.at(run).at(model).size() == 7 ) // BOX
    {
      double ddd1 = v1_models.at(run).at(model).at(0);
      double ddd2 = v1_models.at(run).at(model).at(1);
      double ddd3 = v1_models.at(run).at(model).at(2);
      double  vvv = v1_models.at(run).at(model).at(3);
      double ccc1 = v1_models.at(run).at(model).at(4);
      double ccc2 = v1_models.at(run).at(model).at(5);
      double ccc3 = v1_models.at(run).at(model).at(6);

      fprintf (file, "\n----------------------------------------------------------------------------------------------------\n\n");
      fprintf (file, "  model %d [box]  \n\n", model);

      bool match_not_found;

      fprintf (file, "    view 0 \n\n");

      for (int r = 0; r < v1_models.size(); r++)
      {
        match_not_found = true;

        for (int m = 0; m < v1_models.at(r).size(); m++)
        {
          if ( v1_models.at(r).at(m).size() == 7 ) // BOX
          {
            double d1 = v1_models.at(r).at(m).at(0);
            double d2 = v1_models.at(r).at(m).at(1);
            double d3 = v1_models.at(r).at(m).at(2);
            double  v = v1_models.at(r).at(m).at(3);
            double c1 = v1_models.at(r).at(m).at(4);
            double c2 = v1_models.at(r).at(m).at(5);
            double c3 = v1_models.at(r).at(m).at(6);

            double ccc_to_c = sqrt( _sqr(ccc1-c1) + _sqr(ccc2-c2) + _sqr(ccc3-c3) );

            printf (" %7.5f", ccc_to_c);

            if ( ccc_to_c < threshold_between_centroids_of_cuboids )
            {
              match_not_found = false;

              printf (" match found \n");

              fprintf (file, "      run %2d | %12.10f x %12.10f x %12.10f = %12.10f | %12.10f %12.10f %12.10f \n", r, d1, d2, d3, v, c1, c2, c3);
            }
            else
              printf ("\n");
          }
        }

        if ( match_not_found )
          fprintf (file, "      run %2d | match not found \n", r);

        if ( step ) getchar ();
      }

      fprintf (file, "\n    view 1 \n\n");

      for (int r = 0; r < v2_models.size(); r++)
      {
        match_not_found = true;

        for (int m = 0; m < v2_models.at(r).size(); m++)
        {
          if ( v2_models.at(r).at(m).size() == 7 ) // BOX
          {
            double d1 = v2_models.at(r).at(m).at(0);
            double d2 = v2_models.at(r).at(m).at(1);
            double d3 = v2_models.at(r).at(m).at(2);
            double  v = v2_models.at(r).at(m).at(3);
            double c1 = v2_models.at(r).at(m).at(4);
            double c2 = v2_models.at(r).at(m).at(5);
            double c3 = v2_models.at(r).at(m).at(6);

            double ccc_to_c = sqrt( _sqr(ccc1-c1) + _sqr(ccc2-c2) + _sqr(ccc3-c3) );

            printf (" %7.5f", ccc_to_c);

            if ( ccc_to_c < threshold_between_centroids_of_cuboids )
            {
              match_not_found = false;

              printf (" match found \n");

              fprintf (file, "      run %2d | %12.10f x %12.10f x %12.10f = %12.10f | %12.10f %12.10f %12.10f \n", r, d1, d2, d3, v, c1, c2, c3);
            }
            else
              printf ("\n");
          }
        }

        if ( match_not_found )
          fprintf (file, "      run %2d | match not found \n", r);

        if ( step ) getchar ();
      }
    }

    // ---------------------- //
    // Dealing with cylinders //
    // ---------------------- //

    if ( v1_models.at(run).at(model).size() == 6 ) // CYLINDER
    {
      double  rrr = v1_models.at(run).at(model).at(0);
      double  hhh = v1_models.at(run).at(model).at(1);
      double  vvv = v1_models.at(run).at(model).at(2);
      double ccc1 = v1_models.at(run).at(model).at(3);
      double ccc2 = v1_models.at(run).at(model).at(4);
      double ccc3 = v1_models.at(run).at(model).at(5);

      fprintf (file, "\n----------------------------------------------------------------------------------------------------\n\n");
      fprintf (file, "  model %d [cylinder]  \n\n", model);

      bool match_not_found;

      fprintf (file, "    view 0 \n\n");

      for (int r = 0; r < v1_models.size(); r++)
      {
        match_not_found = true;

        for (int m = 0; m < v1_models.at(r).size(); m++)
        {
          if ( v1_models.at(r).at(m).size() == 6 ) // CYLINDER
          {
            double rr = v1_models.at(r).at(m).at(0);
            double  h = v1_models.at(r).at(m).at(1);
            double  v = v1_models.at(r).at(m).at(2);
            double c1 = v1_models.at(r).at(m).at(3);
            double c2 = v1_models.at(r).at(m).at(4);
            double c3 = v1_models.at(r).at(m).at(5);

            double ccc_to_c = sqrt( _sqr(ccc1-c1) + _sqr(ccc2-c2) + _sqr(ccc3-c3) );

            printf (" %7.5f", ccc_to_c);

            if ( ccc_to_c < threshold_between_centroids_of_cylinders )
            {
              match_not_found = false;

              printf (" match found \n");

              fprintf (file, "      run %2d | pi x %12.10f ^2 x %12.10f = %12.10f | %12.10f %12.10f %12.10f \n", r, rr, h, v, c1, c2, c3);
            }
            else
              printf ("\n");
          }
        }

        if ( match_not_found )
          fprintf (file, "      run %2d | match not found \n", r);

        if ( step ) getchar ();
      }

      fprintf (file, "\n    view 1 \n\n");

      for (int r = 0; r < v2_models.size(); r++)
      {
        match_not_found = true;

        for (int m = 0; m < v2_models.at(r).size(); m++)
        {
          if ( v2_models.at(r).at(m).size() == 6 ) // CYLINDER
          {
            double rr = v2_models.at(r).at(m).at(0);
            double  h = v2_models.at(r).at(m).at(1);
            double  v = v2_models.at(r).at(m).at(2);
            double c1 = v2_models.at(r).at(m).at(3);
            double c2 = v2_models.at(r).at(m).at(4);
            double c3 = v2_models.at(r).at(m).at(5);

            double ccc_to_c = sqrt( _sqr(ccc1-c1) + _sqr(ccc2-c2) + _sqr(ccc3-c3) );

            printf (" %7.5f", ccc_to_c);

            if ( ccc_to_c < threshold_between_centroids_of_cylinders )
            {
              match_not_found = false;

              printf (" match found \n");

              fprintf (file, "      run %2d | pi x %12.10f ^2 x %12.10f = %12.10f | %12.10f %12.10f %12.10f \n", r, rr, h, v, c1, c2, c3);
            }
            else
              printf ("\n");
          }
        }

        if ( match_not_found )
          fprintf (file, "      run %2d | match not found \n", r);

        if ( step ) getchar ();
      }
    }

    // ----------------- //
    // End of generating //
    // ----------------- //

  }

  fclose (file);



  return (0);
}
