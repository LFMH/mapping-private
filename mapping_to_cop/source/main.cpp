/***

 Mapping to cop
 klank@in.tum.de
*/


#include <ros/ros.h>
#include <cop/cop_call.h>
#include <cop/cop_answer.h>
#include <mapping_srvs/GetPlaneClusters.h>
#include <jlo/srvjlo.h>


#define max(a, b) (((a) > (b))  ?  (a) : (b))

typedef uint64_t uint64;

 void publishToCop(std::string object, ros::NodeHandle &n, std::vector<uint64> cluster_ids);
/*    "# Message to quiey the lo-service\n"
    "uint64 id		#id of a frame, there should be unique mapping from a tf-name-string to such an id\n"
    "uint64 parent_id        #id of parent frame\n"
    "float64[16] pose	#pose matrix, fully projective 4x4 matrix\n"
    "float64[36] cov         #covariance for 6 dof (xyz, rpy)\n"
    "uint16 type             #fixed connection with the parent (1) or free in space (0 = default)\n"
    */
uint64_t JloRegisterPose(std::vector<double> mat, std::vector<double> uncertainty)
{
  jlo::srvjlo msg;
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<jlo::srvjlo>("/located_object", true);

  msg.request.query.id = 0;
  msg.request.query.parent_id = 1;  /*ID of swissranger TODO! instead of 5 (= left camera)*/
  msg.request.query.type = 0;
  msg.request.query.pose = mat;
  msg.request.query.cov = uncertainty;

  msg.request.command ="update";


  if (!client.call(msg))
  {
    printf("Error in GetPlaneCluster  srv!\n");
    return 1;
  }
    else if (msg.response.error.length() > 0)
  {
    printf("Error from jlo: %s!\n", msg.response.error.c_str());
    return 1;
  }
  int width2 = 4;
  printf("Showing PosId %d with parent %d:\n", (int)msg.response.answer.id, (int)msg.response.answer.parent_id);

  for(int r = 0; r < width2; r++)
  {
    for(int c = 0; c < width2; c++)
    {
        printf( "%f ", msg.response.answer.pose[r * width2 + c]);

    }
    printf("\n");
  }
  width2 = 6;
  for(int r = 0; r < width2; r++)
  {
    for(int c = 0; c < width2; c++)
    {
       printf( "%f ", msg.response.answer.cov[r * width2 + c]);
    }
    printf("\n");
  }
  return msg.response.answer.id;
}

void normalize(double &a,double &b, double &c)
{
  double norm = sqrt(a*a + b*b + c*c);
  a /= norm;
  b /= norm;
  c /= norm;
}

void CrossProduct(double b_x, double b_y,double b_z,double c_x,double c_y,double c_z,double* a_x,double* a_y,double* a_z)
{
    *a_x = b_y*c_z - b_z*c_y;
    *a_y = b_z*c_x - b_x*c_z;
    *a_z = b_x*c_y - b_y*c_x;
}


double scalarproduct(const double &a,const double &b, const double &c, const double &d, const double &e, const double &f)
{
  return a * d + b* e + c*f;
}

/*
float64 a
float64 b
float64 c
float64 d
robot_msgs/Point32 pcenter
robot_msgs/ObjectOnTable[] oclusters
*/
bool GetPlaneClusterCall(std::vector<uint64> &cluster_ids)
{
  mapping_srvs::GetPlaneClusters msg;
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<mapping_srvs::GetPlaneClusters>("/get_plane_clusters_sr", true);
  if (!client.call(msg))
  {
    printf("Error in /get_plane_clusters_sr  srv!\n");
    return false;
  }
  double a = msg.response.a;
  double b = msg.response.b;
  double c = msg.response.c;
  double s = msg.response.d;
  robot_msgs::Point32 &pcenter = msg.response.pcenter;
  std::vector<robot_msgs::ObjectOnTable> &vec = msg.response.oclusters;
  printf("Got plane equation %f x + %f y + %f z + %f = 0\n", a,b,c,s);
   /*Norm v1*/
  normalize(a,b,c);

  /*Init v2*/
  double d,e,f,g,h,i;
  if (a == b && a == c)
  {
     d = 1; e = 0; f = 0;
  }
  else
  {
    d = b; e = a; f = c;
  }
  /*Orthogonalize v2*/
  double tmp = scalarproduct(a,b,c,d,e,f);
  d = d - tmp * a;
  e = e - tmp * b;
  f = f - tmp * c;

  /*Norma v2*/
  normalize(d,e,f);

  /*Create v3*/

  CrossProduct(a,b,c,d,e,f, &g,&h,&i);
  /**  Build Matrix:
  *   d g a p.x
  *   e h b p.y
  *   f i c p.z
  *   0 0 0 1
  *   for every cluster
  */
  if(vec.size() == 0)
  {
     robot_msgs::ObjectOnTable on;
     on.center.x = 0.0;
     on.center.y = 0.0;
     on.center.z = 0.8;
     
     on.min_bound.x = -0.2;
     on.min_bound.y = -0.2;
     on.min_bound.z = 1.0;
     
     on.max_bound.x = 0.2;
     on.max_bound.y = 0.2;
     on.max_bound.z = 1.1;
     vec.push_back(on);
  }
  printf("Creating a pose for every cluster\n");
  for(int i = 0; i < vec.size(); i++)
  {
    const robot_msgs::Point32 &center = vec[i].center;
    const robot_msgs::Point32 &min_bound = vec[i].min_bound;
    const robot_msgs::Point32 &max_bound = vec[i].max_bound;
    std::vector<double> rotmat;
    rotmat.push_back(d ); rotmat.push_back( g ); rotmat.push_back( a ); rotmat.push_back( center.x);
           rotmat.push_back( e ); rotmat.push_back( h ); rotmat.push_back( b ); rotmat.push_back( center.y);
           rotmat.push_back( f ); rotmat.push_back( i ); rotmat.push_back( c ); rotmat.push_back( center.z);
            rotmat.push_back( 0 ); rotmat.push_back( 0 ); rotmat.push_back( 0 ); rotmat.push_back( 1);
    std::vector<double> cov;
    double covx = max(fabs(center.x - max_bound.x), fabs(center.x - min_bound.x));
    double covy = max(fabs(center.y - max_bound.y), fabs(center.y - min_bound.y));
    double covz = max(fabs(center.z - max_bound.z), fabs(center.z - min_bound.z));
    /*Fill covariance with the cluster size and hardcoded full rotation in normal direction */
     cov.push_back(covx); cov.push_back(0    ); cov.push_back( 0    ); cov.push_back( 0   ); cov.push_back( 0   ); cov.push_back( 0);
     cov.push_back( 0    ); cov.push_back( covy ); cov.push_back( 0    ); cov.push_back( 0   ); cov.push_back( 0   ); cov.push_back( 0);
     cov.push_back( 0    ); cov.push_back( 0    ); cov.push_back( covz ); cov.push_back( 0   ); cov.push_back( 0   ); cov.push_back( 0);
     cov.push_back( 0    ); cov.push_back( 0    ); cov.push_back( 0    ); cov.push_back( 0.3 ); cov.push_back( 0   ); cov.push_back( 0);
     cov.push_back( 0    ); cov.push_back( 0    ); cov.push_back( 0    ); cov.push_back( 0   ); cov.push_back( 0.3 ); cov.push_back( 0);
     cov.push_back( 0    ); cov.push_back( 0    ); cov.push_back( 0    ); cov.push_back( 0   ); cov.push_back( 0   ); cov.push_back( 1.5);

     cluster_ids.push_back(JloRegisterPose(rotmat, cov));
     printf("Pushed back a cluster\n");
  }
  return true;
}

std::string _object = "Mug";

 void callback(const boost::shared_ptr<const cop::cop_answer> &msg)
 {
  printf("got answer from cop! (Errors: %s)\n", msg->error.c_str());
  for(int i = 0; i < msg->found_poses.size(); i++)
  {
    const cop::aposteriori_position &pos =  msg->found_poses[i];
    printf("Foub Obj nr %d with prob %f at pos %d\n", (int)pos.objectId, pos.probability, (int)pos.position);
  }
  printf("End!\n");
  printf("Starting enxt round!\n");

  std::vector<uint64> cluster_ids;

  if(GetPlaneClusterCall(cluster_ids))
  {
      ros::NodeHandle n;
      publishToCop(_object, n, cluster_ids);
  }
  else
    exit(0);
 }

  std::string stTopicName = "/tracking/out";

 void publishToCop(std::string object, ros::NodeHandle &n, std::vector<uint64> cluster_ids)
 {
  ros::NodeHandle nh;
   cop::cop_call call;
   ros::Publisher pub = nh.advertise<cop::cop_call>("/tracking/in", 1); //000);
   ros::Rate r(1);
   r.sleep();
    
  /** Create the cop_call msg*/
  call.outputtopic = stTopicName;
  call.object_classes.push_back(object);
  call.action_type = 0;
  call.number_of_objects = 1;
  cop::apriori_position pos;
  int size = (int)cluster_ids.size();
  printf("Number of pos ids? %d\n", size);
  
  for(int i = 0; i < size; i++)
  {
    pos.probability = 1.0 / size;
    pos.positionId = cluster_ids[i];
    call.list_of_poses.push_back(pos);
  }
  printf("Publish a cop_call at pub.nam: %s\n", pub.getTopic().c_str());
  pub.publish(call);

 }

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "/mapping_to_cop") ;
  cop::cop_call call;
  cop::cop_answer answer;
  ros::NodeHandle n;
  /** new topic for cop*/

     /** subscribe to the topic cop should publish the results*/
  ros::Subscriber read = n.subscribe<cop::cop_answer>(stTopicName, 1000, &callback);
  /** Publish */
  std::vector<uint64> cluster_ids;

  if(GetPlaneClusterCall(cluster_ids))
  {
      ros::NodeHandle n;
      publishToCop(_object, n, cluster_ids);
  }
  ros::Rate r(1);
    
  /**  Wait for results*/
  while(n.ok())
  {
    ros::spinOnce();
    r.sleep();
//    pub.publish(call);
  }
  return 0;
}

