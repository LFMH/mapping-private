/*
./calib test2.pcd output.pcd ../output.ppm -viewpoint -0.248932,0.626674,1.13531 -viewray 5.12,-3.4,-2.0 -viewup 0.216201,-0.22629,0.949763 -focal_dist 0.035 -maxval 255 -background 255,0,0 -resolution 480,640 -pixel_dim 0.0001
*/

#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <iostream>
#include <math.h>

#include <map>


#include "../common/CommonIORoutines.h"
#include "../common/CommonTerminalRoutines.h"

//#define PI 3.14159265 

typedef std::map<std::pair<int,int>, bool> HoleMap;
//typedef std::vector<bool> HoleMap;

typedef struct xyzVector
{
  double x,y,z;
};

typedef struct pixel
{
  double dist;
  int intensity;
};

typedef struct triangle
{
  int a,b,c;
};


#define does_exist(p,s,width,holes) (!holes[std::pair<int,int>((p),(s))])

// inline int dont_exist(int p, int s, int width, HoleMap holes)
// {
//   return (holes[(p * width + s)]);
// }

#define DIST_3D(a, b) (sqrt((points[(a)][0] - points[(b)][0])*(points[(a)][0] - points[(b)][0]) + (points[(a)][1] - points[(b)][1])*(points[(a)][1] - points[(b)][1]) + (points[(a)][2] - points[(b)][2])*(points[(a)][2] - points[(b)][2])))


double dot_product(xyzVector a, xyzVector b)
{
  return (a.x * b.x + a.y * b.y + a.z * b.z);
}

xyzVector cross_product(xyzVector a, xyzVector b)
{
  xyzVector rez;
  rez.x = a.y * b.z - a.z * b.y;
  rez.y = a.z * b.x - a.x * b.z;
  rez.z = a.x * b.y - a.y * b.x;

  return rez;
}

xyzVector vect_diff(xyzVector a, xyzVector b)
{
  xyzVector dif;
  dif.x = a.x - b.x;
  dif.y = a.y - b.y;
  dif.z = a.z - b.z;

  return dif;
}

double distance(xyzVector a, xyzVector b)
{
  return (sqrt( (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z)));
}

xyzVector norm_vect(xyzVector a)
{
  double norm = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
  a.x = double(a.x/norm);
  a.y = double(a.y/norm);
  a.z = double(a.z/norm);

  return a;
}

void ppm(char output[], std::vector< std::vector<pixel> > img, int width, int height, int maxval)
{
  FILE* f;

  f=fopen(output,"w");

  fprintf(f,"P3\n");
  fprintf(f,"%d %d\n", width, height); 
  fprintf(f,"%d\n",maxval);

  for (int i=0; i<height; i++)
  {
    for (int j=0; j<width; j++)
    {
      fprintf(f," ");
      int intensity = img[i][j].intensity;
      if (intensity > maxval)
        intensity = maxval;
      fprintf(f, "%d %d %d", intensity, intensity, intensity);
      fprintf(f," ");
    }
    fprintf(f,"\n");
  }
  fclose(f);
}

int no_sid_pid(int sid[], int pid[], ANNpointArray points, PCD_Header header,  int sidIdx,  int pidIdx, int height, int width, HoleMap &holes)
{
  int sp = 0;
  int nr = 0;

  for (int i=0; i<height; i++)
  {
    for (int j=0; j<width; j++)
    {
      //printf ("i, j: %d/%d, %d/%d - holes, points: %d, %d / %d\n", i, height, j, width, sp, nr, header.nr_points);

      if ((points[nr][pidIdx] == header.minPD[pidIdx] + i) && (points[nr][sidIdx] == header.minPD[sidIdx] + j))
      {
//        holes[(int(header.minPD[pidIdx]) + i) * width + (int(header.minPD[sidIdx]) + j)] = false;
        nr++;
        if (nr == header.nr_points)
          break;
      }
      else
      {
        holes[std::pair<int,int>((int(header.minPD[pidIdx]) + i) , (int(header.minPD[sidIdx]) + j))] = true;
        sp++;
      }
    }
  }
  printf ("holes, points: %i, %i / %ld\n", sp, nr, width*height);
  //  for (HoleMap::iterator it = holes.begin (); it != holes.end (); it++)
//    printf ("<%i, %i> : %i\n", it->first.first, it->first.second, it->second);

  return sp;
}

std::vector<triangle> det_triangles_2(ANNpointArray points, PCD_Header header, int sidIdx, int pidIdx, int height, int width, int *nr_tr)
{
  std::vector<triangle> tr;

  tr.resize(2*width*height);
  //tr.reserve(2*width*height);

  //HoleMap holes;
  //int sp = no_sid_pid(sid, pid, points, header, sidIdx, pidIdx, height, width, holes);

  int nr = 0; //number of triangles
  int nrpd = 0; // for progress bar

  int a=0, b, c, d, e;
  bool skipped = false;

  float max_length = 0.03;

  for (int i = (int)header.minPD[pidIdx]; i < (int)header.minPD[pidIdx] + height; i++)
  {
    nrpd = print_progressbar (i - (int)header.minPD[pidIdx] + 1, height, nrpd);
    //printf("row %d\n", i);

    for (int j = (int)header.minPD[sidIdx]; j < (int)header.minPD[sidIdx] + width; j++)
    {
      //printf("column %d\n", j);

      //printf("looking for row and column of point %d/d: (%d,%d)\n", a, header.nr_points, (int)points[a][pidIdx], (int)points[a][sidIdx]);

      // find top left corner
      if (points[a][pidIdx] == i && points[a][sidIdx] == j)
      {
        //printf("found point a = %d\n", a);

        b = -1;
        c = -1;
        d = -1;
        e = -1;

        // find top right corner
        if (a+1 < header.nr_points && points[a+1][pidIdx] == i && points[a+1][sidIdx] == j+1)
          b = a+1;

        //printf("resolved point b = %d\n", b);

        // go to next line
        int test = a;
        while (test < header.nr_points && points[test][pidIdx] < i+1)
          test++;

        //printf("resolved next line\n");

        // if next line exists
        if (test < header.nr_points && points[test][pidIdx] == i+1)
        {
          // a skipped triangle exists because of missing 'a'
          if (skipped)
          {
            skipped = false; // reset var

            // go to column j-1
            while (test < header.nr_points && points[test][sidIdx] < j-1)
              test++;

            // if not at the end of dataset
            if (test < header.nr_points)
            {
              // if column exists
              if (points[test][pidIdx] == i+1 && points[test][sidIdx] == j-1)
              {
                e = test;
                test++;
              }
            }
          }
          else
          {
            // go to column j
            while (test < header.nr_points && points[test][sidIdx] < j)
              test++;
          }

          // if not at the end of dataset
          if (test < header.nr_points)
          {
            // if column exists
            if (points[test][pidIdx] == i+1 && points[test][sidIdx] == j)
            {
              c = test;
              if (c+1 < header.nr_points && points[c+1][pidIdx] == i+1 && points[c+1][sidIdx] == j+1)
                d = c+1;
            }
            // if next column was found
            else if (points[test][pidIdx] == i+1 && points[test][sidIdx] == j+1)
              d = test;
          }
        }

        //printf ("a = %d, b = %d, c = %d, d = %d\n", a, b, c, d);

/*
        if (b != -1) // if c is checked first a+c+e is not double
        {
          double AB = DIST_3D(a, b);
        //  printf("AB = %g", AB);
          if (c != -1)
          {
            //printf ("a b c\n");
            // a b c
            double BC = DIST_3D(b, c);
            double AC = DIST_3D(a, c);
            if (AB < max_length && BC < max_length && AC < max_length)
            {
              tr[nr].a = a;
              tr[nr].b = b;
              tr[nr].c = c;
              nr++;
            }

            if (d != -1)
            {
              //printf ("b c d\n");
              // b c d
              double BD = DIST_3D (b, d);
              double CD = DIST_3D (c, d);
              if (BD < max_length && BC < max_length && CD < max_length)
              {
                tr[nr].a = b;
                tr[nr].b = c;
                tr[nr].c = d;
                nr++;
              }
            }

            if (e != -1)
            {
              //printf ("a c e\n");
              // a c e
              double AE = DIST_3D (a, e);
              double CE = DIST_3D (c, e);
              if (AC < max_length && CE < max_length && AE < max_length)
              {
                tr[nr].a = a;
                tr[nr].b = c;
                tr[nr].c = e;
              nr++;
              }
            }
          }
          else if (d != -1)
          {
            //printf ("a b d\n");
            // a b d
            double AD = DIST_3D (a, d);
            double BD = DIST_3D (b, d);
            if (AD < max_length && BD < max_length && AB < max_length)
            {
              tr[nr].a = a;
              tr[nr].b = b;
              tr[nr].c = d;
              nr++;
            }
          }
        }
        else if (c != -1)
        {
          double AC = DIST_3D(a, c);
          if (d != -1)
          {
            //printf ("a c d\n");
            // a c d
            double AD = DIST_3D (a, d);
            double CD = DIST_3D (c, d);
            if (AD < max_length && CD < max_length && AC < max_length)
            {
              tr[nr].a = a;
              tr[nr].b = c;
              tr[nr].c = d;
              nr++;
            }
          }

          if (e != -1)
          {
              //printf ("a c e\n");
              // a c e
            double AE = DIST_3D (a, e);
            double CE = DIST_3D (c, e);
            if (AC < max_length && CE < max_length && AE < max_length)
            {
              tr[nr].a = a;
              tr[nr].b = c;
              tr[nr].c = e;
              nr++;
            }
          }
        }
*/

        if (c != -1)
        {
          double AC = DIST_3D(a, c);
          if (e != -1)
          {
              //printf ("a c e\n");
              // a c e
            double AE = DIST_3D (a, e);
            double CE = DIST_3D (c, e);
            if (AC < max_length && CE < max_length && AE < max_length)
            {
              tr[nr].a = a;
              tr[nr].b = c;
              tr[nr].c = e;
              nr++;
            }
          }

          if (b != -1)
          {
             //printf ("a b c\n");
            // a b c
            double AB = DIST_3D(a, b);
            double BC = DIST_3D(b, c);
            double AC = DIST_3D(a, c);
            if (AB < max_length && BC < max_length && AC < max_length)
            {
              tr[nr].a = a;
              tr[nr].b = b;
              tr[nr].c = c;
              nr++;
            }

            if (d != -1)
            {
              //printf ("b c d\n");
              // b c d
              double BD = DIST_3D (b, d);
              double CD = DIST_3D (c, d);
              if (BD < max_length && BC < max_length && CD < max_length)
              {
                tr[nr].a = b;
                tr[nr].b = c;
                tr[nr].c = d;
                nr++;
              }
            }
          }
          else if (d != -1)
               {
                 //printf ("a c d\n");
                 // a c d
                 double AD = DIST_3D (a, d);
                 double CD = DIST_3D (c, d);
                 if (AD < max_length && CD < max_length && AC < max_length)
                 {
                   tr[nr].a = a;
                   tr[nr].b = c;
                   tr[nr].c = d;
                   nr++;
                 }
               }
        }
        else if (b != -1 && d != -1)
             {
               //printf ("a b d\n");
               // a b d
               double AB = DIST_3D(a, b);
               double AD = DIST_3D (a, d);
               double BD = DIST_3D (b, d);
               if (AD < max_length && BD < max_length && AB < max_length)
               {
                 tr[nr].a = a;
                 tr[nr].b = b;
                 tr[nr].c = d;
                 nr++;
               }
             }

        // move to next point
        a++;
        //skipped = false;
        if (a >= header.nr_points)
          break;
      } // END OF: top left corner found
      else
        skipped = true;
    }
    if (a >= header.nr_points)
      break;
  }
  printf("\n");
  printf("nr = %d\n", nr);
  *nr_tr = nr;

  tr.resize(nr);

  //printf("Printing HoleMap:\n");
  //for (HoleMap::iterator it = holes.begin (); it != holes.end (); it++)
  //  printf ("<%i, %i> : %i\n", it->first.first, it->first.second, it->second);

  return tr;
}

std::vector<triangle> det_triangles(ANNpointArray points, PCD_Header header, int sidIdx, int pidIdx, int height, int width, int *nr_tr)
{
  std::vector<triangle> tr;

  tr.resize(2*width*height);
  //tr.reserve(2*width*height);

  int sid[width*height];
  int pid[width*height];
  HoleMap holes;

  int sp = no_sid_pid(sid, pid, points, header, sidIdx, pidIdx, height, width, holes);

  int count_holes = 0;
  int count_holes2 = 0;
  int nr = 0; //number of triangles
  int nrpd = 0; // for progress bar

  for (int i = 0; i < height - 1; i++)
  {
    nrpd = print_progressbar (i + 1, height-1, nrpd);
//    printf("%i\n", i);

    for (int j = 0; j < width - 1; j++)
    {
      if (does_exist((int)header.minPD[pidIdx] + i, (int)header.minPD[sidIdx] + j, width, holes))
      {
        if (does_exist((int)header.minPD[pidIdx] + i + 1, (int)header.minPD[sidIdx] + j + 1, width, holes))
        {
          if (does_exist((int)header.minPD[pidIdx] + i + 1, (int)header.minPD[sidIdx] + j, width, holes))
          {
            tr[nr].a = i*width + j - count_holes;
            tr[nr].b = (i+1)*width + j - count_holes2;
            tr[nr].c = (i+1)*width + (j+1) - count_holes2;
//             triangle trnr;
//             trnr.a = i*width + j - count_holes;
//             trnr.b = (i+1)*width + j - count_holes2;
//             trnr.c = (i+1)*width + (j+1) - count_holes2;
//             tr.push_back(trnr);

//            printf("created triangle - %i - %i : ---> { %i, %i, %i }\n", count_holes, count_holes2, tr[nr].a, tr[nr].b, tr[nr].c);
            nr++;
          }
          else
          {
            if (j == 0)
              count_holes2 ++;
            else if (i == height - 1)
              count_holes ++;
          }

          if (does_exist(int(header.minPD[pidIdx] + i), int(header.minPD[sidIdx] + j + 1), width, holes))
          {
            tr[nr].a = i*width + j - count_holes;
            tr[nr].b = i*width + (j+1) - count_holes;
            tr[nr].c = (i+1)*width + (j+1) - count_holes2;
//             triangle trnr;
//             trnr.a = i*width + j - count_holes;
//             trnr.b = i*width + (j+1) - count_holes;
//             trnr.c = (i+1)*width + (j+1) - count_holes2;
//             tr.push_back(trnr);

//            printf("created triangle - %i - %i : ---> { %i, %i, %i }\n", count_holes, count_holes2, tr[nr].a, tr[nr].b, tr[nr].c);
            nr++;
          }
          else
          {
            if (i == 0)
              count_holes2 ++;
          }
        }
        else
          count_holes2 ++;
      }
      else
      {
        count_holes ++;
        if (i == 0 && j == 0)
          count_holes2 ++;
      }
    }
  }
  printf("\n");
  printf("nr = %d\n", nr);
  *nr_tr = nr;

  tr.resize(nr);

  //printf("Printing HoleMap:\n");
  //for (HoleMap::iterator it = holes.begin (); it != holes.end (); it++)
  //  printf ("<%i, %i> : %i\n", it->first.first, it->first.second, it->second);

  return tr;
}

void write_vtk_file(char output[], std::vector<triangle> triangles, ANNpointArray points, PCD_Header header, int nr_tr, int iIdx)
{
  /* writing VTK file */

  FILE *f;

  f = fopen(output,"w");

  fprintf (f, "# vtk DataFile Version 3.0\nvtk output\nASCII\nDATASET POLYDATA\nPOINTS %d float\n", header.nr_points);
  int i;
  
  for (i=0; i<header.nr_points; i+=3)
  {
    for (int j=0; (j<3) && (i+j<header.nr_points); j++)
      fprintf (f,"%f %f %f ", points[i+j][0], points[i+j][1], points[i+j][2]);
    fprintf (f,"\n");
  }
  
  fprintf(f,"VERTICES %d %d\n", header.nr_points, 2*header.nr_points);
  for (i=0; i<header.nr_points; i++)
    fprintf(f,"1 %d\n", i);
    
  /*
  fprintf(f,"\nPOLYGONS %d %d\n",triangles.size(), 4*triangles.size());
  for (std::vector<triangle>::iterator it = triangles.begin(); it != triangles.end(); it++)
  fprintf(f,"3 %d %d %d\n",(*it).a, (*it).c, (*it).b);
  */

  printf("vector: %d, nr: %d\n", triangles.size(), nr_tr);
  
  fprintf(f,"\nPOLYGONS %d %ld\n", nr_tr, 4*nr_tr);
  for (int i=0; i<nr_tr; i++)
  {
    if (triangles[i].a >= header.nr_points || triangles[i].a < 0 || isnan(triangles[i].a))
      ;//printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, triangles[i].a, triangles[i].b, triangles[i].c, header.nr_points);
    else if (triangles[i].b >= header.nr_points || triangles[i].b < 0 || isnan(triangles[i].b))
      ;//printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, triangles[i].a, triangles[i].b, triangles[i].c, header.nr_points);
    else if (triangles[i].c >= header.nr_points || triangles[i].c < 0 || isnan(triangles[i].c))
      ;//printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, triangles[i].a, triangles[i].b, triangles[i].c, header.nr_points);
    else if (triangles[i].a == triangles[i].b || triangles[i].a == triangles[i].c || triangles[i].b == triangles[i].c)
      ;//printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, triangles[i].a, triangles[i].b, triangles[i].c, header.nr_points);
    else
      fprintf(f,"3 %d %d %d\n",triangles[i].a, triangles[i].c, triangles[i].b);
  }

  fprintf (f, "\nPOINT_DATA %d\nSCALARS scalars double\nLOOKUP_TABLE default\n", header.nr_points);
  for (i=0; i<header.nr_points; i+=9)
  {
    for (int j=0; (j<9) && (i+j<header.nr_points); j++)
      fprintf (f, "%d ", (int)points[i+j][iIdx]);
    fprintf (f,"\n");
  }
}

int main(int argc, char *argv[])
{
  xyzVector vp;
  vp.x = 0; vp.y = 0; vp.z = 0;

  xyzVector vn, vu; 
  vn.x = 0; vn.y = 0; vn.z = 0;
  vu.x = 0; vu.y = 0; vu.z = 0;
  double focal_dist = 0.0;

  int maxval = 255;
  int background_red = 0, background_green = 0, background_blue = 0;

  int height = 480, width = 640;
  double pixel_dim = 0.0001;

  if (argc < 20)
  {
    print_error (stderr, "Syntax is: %s <input>.pcd <output>.pcd <output>.ppm <options>\n", argv[0]);
    fprintf (stderr, "  where options are:\n");
    fprintf (stderr, "      -viewpoint vpx,vpy,vpz = specify the view point coordinates (default ");
    print_value (stderr, "%g %g %g", vp.x, vp.y, vp.z); fprintf (stderr, ")\n");
    fprintf (stderr, "      -viewray vnx,vny,vnz = specify the view ray xyzVector (default ");
    print_value (stderr, "%g %g %g", vn.x, vn.y, vn.z); fprintf (stderr, ")\n");
    fprintf (stderr, "      -viewup vux,vuy,vuz = specify the view up xyzVector (default ");
    print_value (stderr, "%g %g %g", vu.x, vu.y, vu.z); fprintf (stderr, ")\n");
    fprintf (stderr, "      -focal_dist val_dist = specify the focal length (default ");
    print_value (stderr, "%g", focal_dist); fprintf (stderr, ")\n");
    fprintf (stderr, "      -maxval maxval = specify the maximum color value (default ");
    print_value (stderr, "%d",maxval); fprintf (stderr, ")\n");
    fprintf (stderr, "      -background bg_red,bg_green,bg_blue = specify the backround color (default ");
    print_value (stderr, "%d %d %d", background_red, background_green, background_blue); fprintf (stderr, ")\n");
    fprintf (stderr, "      -resolution height,width = specify the resolution of the image (default ");
    print_value (stderr, "%d %d", height, width); fprintf (stderr, ")\n");
    fprintf (stderr, "      -pixel_dim pixel_dim = specify the pixel dimension (default ");
    print_value (stderr, "%g",pixel_dim); fprintf (stderr, ")\n");
  }

  // Get the .pcd files
  std::vector<int> pPCDFileIndices = ParseFileExtensionArgument (argc, argv, ".pcd");

  if (pPCDFileIndices.size () < 2)
  {
    print_error (stderr, "Need an input and an output .PCD file!\n");
    return (-1);
  }

  // Load the points from file
  PCD_Header header;
  print_info (stderr, "Loading ");
  print_value (stderr, "%s ", argv[pPCDFileIndices.at (0)]);
  ANNpointArray points = LoadPCDFile (argv[pPCDFileIndices.at (0)], header);

  if (points == NULL)
    return (-1);
  fprintf (stderr, "[done : "); print_value (stderr, "%d %d", header.nr_points, header.dimID.size ()); fprintf (stderr, "D points]\n");
  print_info (stderr, "Available dimensions: "); print_value (stderr, "%s\n", getAvailableDimensions (header).c_str ());

  // Checking XYZ dimensions
  int xIdx = getIndex (header, "x");
  int yIdx = getIndex (header, "y");
  int zIdx = getIndex (header, "z");

  if (xIdx != 0 || yIdx != 1 || zIdx != 2)
  {
    print_error (stderr, "XYZ coordinates not first dimensions!\n");
    return (-1); 
  }

  // Checking SID and PID dimension
  int sidIdx = getIndex (header, "sid");
  int pidIdx = getIndex (header, "pid");

  if (sidIdx == -1 || pidIdx == -1)
  {
    print_error (stderr, "SID and PID information missing!\n");
    return (-1);
  }

  // Checking intensities
  int iIdx = getIndex (header, "intensities");

  if (iIdx == -1)
  {
    iIdx = getIndex (header, "i");
    if (iIdx == -1)
    {
      print_error (stderr, "Intensity information requested but not available!\n");
      return (-1);
    }
  }

  int dIdx = getIndex (header, "d");

  // Get the .ppm files
  std::vector<int> pPPMFileIndices = ParseFileExtensionArgument (argc, argv, ".ppm");

  if (pPPMFileIndices.size () < 1)
  {
    print_error (stderr, "Need an output .PPM file!\n");
    return (-1);
  }

  char *output = argv[pPPMFileIndices.at (0)]; 


  Parse3xArguments (argc, argv, "-viewpoint", vp.x, vp.y, vp.z);
  Parse3xArguments (argc, argv, "-viewray", vn.x, vn.y, vn.z);
  Parse3xArguments (argc, argv, "-viewup", vu.x, vu.y, vu.z);
  ParseArgument (argc, argv, "-focal_dist", focal_dist);
  ParseArgument (argc, argv, "-maxval", maxval);
  Parse3xArguments (argc, argv, "-background", background_red, background_green, background_blue);

  ParseRangeArguments (argc, argv, "-resolution", height, width);
  ParseArgument (argc, argv, "-pixel_dim", pixel_dim);

  if (header.maxPD[iIdx] > maxval)
    print_warning(stderr,"if the intensity value of a point is higher then maxval,the intensity value of that point will be set to maxval\n");


  std::vector<int> pVTKFileIndices = ParseFileExtensionArgument (argc, argv, ".vtk");

  if (pVTKFileIndices.size () < 1)
  {
    print_error (stderr, "Need an output .VTK file!\n");
    return (-1);
  }

  char *output_vtk = argv[pVTKFileIndices.at (0)]; 

  vn = norm_vect(vn);
  vu = norm_vect(vu);

  ANNpointArray newPoints = annAllocPts (header.nr_points, header.dimID.size ());

  for (int i=0; i<header.nr_points; i++)
  {
    newPoints[i][xIdx] = points[i][xIdx];
    newPoints[i][yIdx] = points[i][yIdx];
    newPoints[i][zIdx] = points[i][zIdx];
    newPoints[i][iIdx] = points[i][iIdx];
    newPoints[i][dIdx] = points[i][dIdx];
    newPoints[i][sidIdx] = points[i][sidIdx];
    newPoints[i][pidIdx] = points[i][pidIdx];
  }


  xyzVector p4;
  p4.x = vp.x + focal_dist * vn.x;
  p4.y = vp.y + focal_dist * vn.y;
  p4.z = vp.z + focal_dist * vn.z;

  //plane equation parameters
  double d;

  d = vn.x * p4.x + vn.y * p4.y + vn.z * p4.z;


  xyzVector p;
  double t, t1, t2;
  xyzVector la, lb;

  la.x = vp.x; la.y = vp.y; la.z = vp.z;
  t1 = d - dot_product(la, vn);

  // for each point from the array of points we compute the projection on the image plane
  for (int i=0; i<header.nr_points; i++)
  {
    p.x = newPoints[i][xIdx];
    p.y = newPoints[i][yIdx];
    p.z = newPoints[i][zIdx];

    lb.x = p.x; 
    lb.y = p.y; 
    lb.z = p.z;

    t2 = dot_product(vect_diff(lb, la), vn);

    t = double(t1/t2);

    newPoints[i][xIdx] = vp.x + (p.x - vp.x) * t;
    newPoints[i][yIdx] = vp.y + (p.y - vp.y) * t;
    newPoints[i][zIdx] = vp.z + (p.z - vp.z) * t;

  }

/*
  pcd_to_ppm(output, newPoints, header, sidIdx, pidIdx, iIdx, maxval, background_red, background_green, background_blue);

  // Save projected points
  print_info (stderr, "Saving ");
  print_value (stderr, "%s ", argv[pPCDFileIndices.at (1)]);
  SavePCDFile (argv[pPCDFileIndices.at (1)], newPoints, header, 5);
*/

  printf("\n");

  xyzVector u,v;

  v.x = -vu.x;
  v.y = -vu.y;
  v.z = -vu.z;

  v = norm_vect(v);
  printf("v = %g %g %g\n",v.x, v.y, v.z);

  u = cross_product(v, vn);

  u = norm_vect(u);
  printf("u = %g %g %g\n",u.x, u.y, u.z);

  xyzVector dir;

  dir.x = -(u.x + v.x);
  dir.y = -(u.y + v.y);
  dir.z = -(u.z + v.z);

  dir = norm_vect(dir);
  printf("dir = %g %g %g\n",dir.x, dir.y, dir.z);

  double dist = sqrt(pixel_dim * (height/2) * pixel_dim * (height/2) + pixel_dim * (width/2) * pixel_dim * (width/2));

  printf("dist = %g\n",dist);

  xyzVector o;

  o.x = p4.x + dist * dir.x;
  o.y = p4.y + dist * dir.y;
  o.z = p4.z + dist * dir.z;

  printf("orig = %g %g %g\n",o.x, o.y, o.z);


  xyzVector pt;
  for (int i=0; i<header.nr_points; i++)
  {
    pt.x = newPoints[i][xIdx];
    pt.y = newPoints[i][yIdx];
    pt.z = newPoints[i][zIdx];

    newPoints[i][xIdx] = dot_product(vect_diff(pt, o), v);
    newPoints[i][yIdx] = dot_product(vect_diff(pt, o), u);
    newPoints[i][zIdx] = 1;
  }

  std::vector< std::vector<pixel> > img;

  img.resize(height);

  for (int i=0; i<img.size(); i++)
    img[i].resize(width);

  for (int i=0; i< height; i++)
    for (int j=0; j< width; j++)
  {
    img[i][j].intensity = 0;
    img[i][j].dist = DBL_MAX;
  }

  int x,y;
//   for (int i=0; i<header.nr_points; i++)
//   {
//     x = int (round(double(newPoints[i][xIdx]/pixel_dim)));
//     y = int (round(double(newPoints[i][yIdx]/pixel_dim)));
// 
//     pt.x = points[i][xIdx];
//     pt.y = points[i][yIdx];
//     pt.z = points[i][zIdx];
// 
//     d = distance(vp,pt);
// 
//     if (d < img[x][y].dist)
//     {
//       img[x][y].intensity = int (points[i][iIdx]);
//       img[x][y].dist = d;
//     }
//   }
// 
//   ppm(output, img, width, height, maxval);


  double alfa = atan2(double(height/2) * pixel_dim, focal_dist);
  alfa =  double(alfa * 180 / M_PI);
  printf("angle = %g\n",alfa);

  int w = int(header.maxPD[sidIdx] - header.minPD[sidIdx]) + 1;
  int h = int(header.maxPD[pidIdx] - header.minPD[pidIdx]) + 1;
  printf("h = %d\n",h);
  printf("w = %d\n", w);


  int nr_tr;
  std::vector<triangle> tr = det_triangles_2(points, header, sidIdx, pidIdx, h, w, &nr_tr);

  printf("w = %d h = %d\n", w, h);
  printf("nr_tr = %d\n", nr_tr);

  write_vtk_file(output_vtk, tr, points, header, nr_tr, iIdx);

  annDeallocPts (points);
  annDeallocPts (newPoints);

  return 0;
}
