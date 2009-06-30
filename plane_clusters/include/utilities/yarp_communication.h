#ifndef _YARP_COMMUNICATION_H_
#define _YARP_COMMUNICATION_H_

#include <yarp/os/all.h>
#include <string>

using namespace std;
using namespace yarp::os;

#include "ros/console.h"
#include <Eigen/Core>

int get_yarp_hom_matrix(const string port_name, Eigen::Matrix4f &hom_matrix);
#endif
