/*
 * Copyright (c) 2011, Lucian Cosmin Goron <goron@cs.tum.edu>
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

// #include <time.h> //

////////////////////////////////////////////////////////////
/** \brief Routines for fitting 2D line models using RANSAC.
*/

std::vector<int> getSamplesLine (pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr cloud)
{

  // srand (time(0)); //

  std::vector<int> random_idx(2);
  do
  {
    for (int i = 0; i < 2; i++)
    {
      random_idx[i] = (int) cloud->points.size () * (rand () / (RAND_MAX + 1.0));
    }
  } while (random_idx[0] == random_idx[1]);

  return random_idx;
}

pcl::PointIndices::Ptr getInliersLine (pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr cloud, std::vector<int> &samples, double line_threshold)
{
  double x1 = cloud->points.at (samples[0]).x;
  double y1 = cloud->points.at (samples[0]).y;
  double x2 = cloud->points.at (samples[1]).x;
  double y2 = cloud->points.at (samples[1]).y;

  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  for (int i = 0; i < (int) cloud->points.size(); i++)
  {
    double x0 = cloud->points.at (i).x;
    double y0 = cloud->points.at (i).y;

    double d = fabs( (x2-x1)*(y1-y0) - (x1-x0)*(y2-y1) ) / sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) );

    if ( fabs (d) < line_threshold )
      inliers->indices.push_back (i);
  }

  return inliers;
}

int fitLine (pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr cloud, pcl::PointIndices::Ptr &line_inliers, pcl::ModelCoefficients::Ptr &line_coefficients, double line_threshold, int maximum_line_iterations, bool verbose=false)
{
  int iterations = 0;
  std::vector<int> line_samples;
  double wlan, p = 0.99999, k = maximum_line_iterations;

  do
  {
    iterations++;
    std::vector<int> samples = getSamplesLine (cloud);
    pcl::PointIndices::Ptr inliers = getInliersLine (cloud, samples, line_threshold);
    if (inliers->indices.size() > line_inliers->indices.size())
    {
      line_inliers = inliers;
      line_samples = samples;
      wlan = pow( (double) inliers->indices.size() / (double) cloud->points.size(), (double) samples.size() );
      k = log (1 - p) / log (1 - wlan);
      if (verbose) fprintf(stderr, "          At iteration %4d number of inliers %4d is bigger then the best. Number of interations: %4d \n", iterations, (int) inliers->indices.size(), (int) k);
    }
  } while ( (iterations < maximum_line_iterations) && (iterations < k) );

  line_coefficients->values.push_back (cloud->points.at (line_samples.at (0)).x);
  line_coefficients->values.push_back (cloud->points.at (line_samples.at (0)).y);
  line_coefficients->values.push_back (0.0);
  line_coefficients->values.push_back (cloud->points.at (line_samples.at (1)).x - cloud->points.at (line_samples.at (0)).x);
  line_coefficients->values.push_back (cloud->points.at (line_samples.at (1)).y - cloud->points.at (line_samples.at (0)).y);
  line_coefficients->values.push_back (0.0);

  return iterations;
}

//////////////////////////////////////////////////////////////
/** \brief Routines for fitting 2D circle models using RANSAC.
*/

std::vector<int>  getSamplesCircle (pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr cloud)
{

  // srand (time(0)); //

  std::vector<int> random_idx(3);
  do
  {
    for (int i = 0; i < 3; i++)
    {
      random_idx[i] = (int) cloud->points.size () * (rand () / (RAND_MAX + 1.0));
    }
  } while ( (random_idx[0] == random_idx[1]) || (random_idx[0] == random_idx[2]) || (random_idx[1] == random_idx[2]) );

  return random_idx;
}

pcl::PointIndices::Ptr getInliersCircle (pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr cloud, std::vector<int> &samples, double circle_threshold, double min_r, double max_r, pcl::PointXYZ &c, double &r)
{
  double x1 = cloud->points.at (samples[0]).x;
  double y1 = cloud->points.at (samples[0]).y;
  double x2 = cloud->points.at (samples[1]).x;
  double y2 = cloud->points.at (samples[1]).y;
  double x3 = cloud->points.at (samples[2]).x;
  double y3 = cloud->points.at (samples[2]).y;

  double ma = (y2 - y1) / (x2 - x1);
  double mb = (y3 - y2) / (x3 - x2);

  c.x = (ma*mb*(y1-y3) + mb*(x1+x2) - ma*(x2+x3)) / (2*(mb - ma));
  c.y = -(1/ma)*(c.x - (x1+x2)/2) + (y1+y2)/2;
  r = sqrt((c.x-x1)*(c.x-x1) + (c.y-y1)*(c.y-y1));

  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  if ( (min_r < r) && (r < max_r) )
  {
    for (int i = 0; i < (int) cloud->points.size(); i++)
    {
      double x0 = cloud->points.at (i).x;
      double y0 = cloud->points.at (i).y;
      double d = sqrt((x0-c.x)*(x0-c.x) + (y0-c.y)*(y0-c.y)) - r;

      if ( fabs (d) < circle_threshold)
        inliers->indices.push_back (i);
    }
  }

  return inliers;
}

int fitCircle (pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr cloud, pcl::PointIndices::Ptr &circle_inliers, pcl::ModelCoefficients::Ptr &circle_coefficients, double circle_threshold, double min_r, double max_r, double maximum_circle_iterations, bool verbose=false)
{
  double r, best_r;
  pcl::PointXYZ c, best_c;
  int iterations = 0;
  std::vector<int> circle_samples;
  double wlan, p = 0.99999, k = maximum_circle_iterations;

  do
  {
    iterations++;
    std::vector<int> samples = getSamplesCircle (cloud);
    pcl::PointIndices::Ptr inliers = getInliersCircle (cloud, samples, circle_threshold, min_r, max_r, c, r);
    if ( inliers->indices.size() > circle_inliers->indices.size())
    {
      circle_inliers = inliers;
      circle_samples = samples;
      best_c.x = c.x;
      best_c.y = c.y;
      best_r = r;
      wlan = pow( (double) inliers->indices.size() / (double) cloud->points.size() , (double) samples.size() );
      k = log(1 - p) / log(1 - wlan);
      if (verbose) fprintf(stderr, "         At iteration %4d number of inliers %4d is bigger then the best. Number of interations: %4d \n", iterations, (int) inliers->indices.size(), (int) k);
    }
  } while ( (iterations < maximum_circle_iterations) && (iterations < k) );

  circle_coefficients->values.push_back (best_c.x);
  circle_coefficients->values.push_back (best_c.y);
  circle_coefficients->values.push_back (best_r);

  return iterations;
}

void getInliersDisk (pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr &cloud, pcl::PointIndices::Ptr &inliers, pcl::PointXYZ c, double r)
{
  for (int idx=0 ; idx < (int) cloud->points.size(); idx++)
  {
    double x0 = cloud->points.at (idx).x;
    double y0 = cloud->points.at (idx).y;
    double d = sqrt( (c.x-x0)*(c.x-x0) + (c.y-y0)*(c.y-y0) );

    if ( d < r )
      inliers->indices.push_back (idx);
  }

  sort (inliers->indices.begin(), inliers->indices.end());
  inliers->indices.erase (unique (inliers->indices.begin(), inliers->indices.end()), inliers->indices.end());
}
