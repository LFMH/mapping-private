#include <cloud_algos/cloud_algos.h>

//instead of maintaining a patch to ROS, we provide a copy of this function
int
  cloud_algos::getChannelIndex (const sensor_msgs::PointCloud &points, std::string value)
{
  // Get the index we need
  for (unsigned int d = 0; d < points.channels.size (); d++)
    if (points.channels[d].name == value)
      return (d);

  return (-1);
}

//instead of maintaining a patch to ROS, we provide a copy of this function
int
  cloud_algos::getChannelIndex (const boost::shared_ptr<const sensor_msgs::PointCloud> points, std::string value)
{
  // Get the index we need
  for (unsigned int d = 0; d < points->channels.size (); d++)
    if (points->channels[d].name == value)
      return (d);

  return (-1);
}

