/*
* Copyright (C) 2008
* Robert Bosch LLC
* Research and Technology Center North America
* Palo Alto, California
*
* All rights reserved.
*
*------------------------------------------------------------------------------
* project ....: PUMA: Probablistic Unsupervised Model Acquisition
* file .......: ImageBlender.h
* authors ....: Zhexi Wang
* organization: Robert Bosch LLC
* creation ...: 05/15/2008
* modified ...: $Date: 2009-01-22 18:01:07 -0800 (Thu, 22 Jan 2009) $
* changed by .: $Author: benjaminpitzer $
* revision ...: $Revision: 736 $
*/
#ifndef IMAGEBLENDER_H
#define IMAGEBLENDER_H
#include "SuperLUWrapper.h"
//#include "TAUCSWrapper.h"
#include <Image.h>
#include <Array2.h>
#include <map>
using namespace std;
namespace puma{
class ImageBlender
{
public:
  ImageBlender(void);
  ~ImageBlender(void);

  //clustering, find sequence to blend
  void blendImageSequential(Image3uc *img, Array2i *mask);
  void clusterHelper( Array2i &cluster, pair<int,int> seed, int value );
  void clusterHelper2( Array2i &cluster, pair<int,int> seed, int value );

  //create guide vector from m_last, fill m_v, only where m_mask==m_src or m_mask==m_dst beside m_src
  void createGuideVector();
  //blendIterative src to dst, write the new color into dst
  //src and dst must be at the same size
  //it will also merge the srcmask to dstmask
  void blendIterative(Image3uc *const src, int srcindex, Image3uc *dst, int dstindex, Array2i *mask); //iterative solve
  double iterate();
  bool getSum(int i, int j, int i1, int j1, Vec3f &sum);

  //blend direct by using superLU
  void blendDirect(Image3uc *const src, int srcindex, Image3uc *dst, int dstindex, Array2i *mask); //by superLU direct solve
  bool addToMatrixDirect(int i, int j, int i1, int j1,ccs_data &A, vector<double> B[3]);

//////////////////////////////////////////////////////////////////////////
  // solving using SparseSMat
//   void blendDirect(Image3uc *const src, int srcindex, Image3uc *dst, int dstindex, Array2i *mask);
//   bool addToMatrixDirect( int i, int j, int i1, int j1,SparseSMatd &A, vector<VarVecd> B );

  //blend direct by suing superLU while keeping boundary of the image and keep too big difference on boundary
  void blendDirectKeepBoundary( Image3uc *const src, int srcindex, Image3uc *dst, int dstindex, Array2i *mask );
  bool addToMatrixKeepBoundary(int i, int j, int i1, int j1,ccs_data &A, vector<double> B[3]);

  //blend the whole image, all pixels are considered
  void blendAllDirect(Image3uc *img, Array2i *mask);
  bool addToMatrixAllDirect( int i, int j, int i1, int j1,ccs_data &A, vector<double> B[3] );

  //blend this patch with all others as boundary, do not change the mask value
  //will return the amount of average changing
  void prepareGuidedVector(Image3uc *img, Array2i *mask);
  double blendSinglePatch(Image3uc *img, Array2i *mask, int srcindex);
  bool addToMatrixSinglePatch(int i, int j, int i1, int j1,ccs_data &A, vector<double> B[3]);

  //iteratively blend each patch with all others, to get a global optimization
  void blendItrativeGlobal(Image3uc *img, Array2i *mask);

  //variables that used to communicate between functions
  const Image3uc *m_src;
  Image3uc *m_dst;
  Array2i *m_mask;
  int m_srcindex,m_dstindex;
  Array2<Vec3d> m_last;
  Array2<Vec3d> m_this;
  Array2< Vec2<Vec3d> > m_v;
  int m_nrow,m_ncol;
  //for direct solver
  map<pair<int,int>,int> m_idxmap;
};
}
#endif
