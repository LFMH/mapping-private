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
* modified ...: $Date: 2009-09-07 13:54:01 -0700 (Mon, 07 Sep 2009) $
* changed by .: $Author: benjaminpitzer $
* revision ...: $Revision: 901 $
*/
#include "ImageBlender.h"
#include <deque>
#include <algorithm>
#include <puma.h>
//#include <SparseLinearSystem.h>
namespace puma{
bool noboundary=true;

ImageBlender::ImageBlender(void)
{
}

ImageBlender::~ImageBlender(void)
{
}

void ImageBlender::blendIterative( Image3uc *const src, int srcindex, Image3uc *dst, int dstindex, Array2i *mask )
{
  m_src=src;
  m_mask=mask;
  m_dst=dst;
  m_srcindex=srcindex;
  m_dstindex=dstindex;
  m_nrow=mask->size(0);
  m_ncol=mask->size(1);
  m_last.setSize(m_nrow,m_ncol);
  for (int i=0;i<m_nrow;i++) for (int j=0;j<m_ncol;j++)
    if (m_mask->at(i,j)==m_srcindex)
    {
      m_last.at(i,j).x[0]=m_src->at(i,j).x[0];
      m_last.at(i,j).x[1]=m_src->at(i,j).x[1];
      m_last.at(i,j).x[2]=m_src->at(i,j).x[2];
    }
  m_this=m_last;
  createGuideVector();
  iterate();
//  int count=0;
  while(iterate()>0.2) ;
  //write mask & result
  for (int i=0;i<m_nrow;i++) for (int j=0;j<m_ncol;j++)
    if (m_mask->at(i,j)==srcindex)
    {
      m_mask->at(i,j)=dstindex;
      m_dst->at(i,j).x[0]=(unsigned char)puma_clamp<double>(m_last.at(i,j).x[0],0,254.9);
      m_dst->at(i,j).x[1]=(unsigned char)puma_clamp<double>(m_last.at(i,j).x[1],0,254.9);
      m_dst->at(i,j).x[2]=(unsigned char)puma_clamp<double>(m_last.at(i,j).x[2],0,254.9);
    }
}

double ImageBlender::iterate()
{
  double change=0;
  for (int i=0;i<m_nrow;i++) for (int j=0;j<m_ncol;j++)
  {
    if (m_mask->at(i,j)!=m_srcindex) continue;
    Vec3f sum=0.0f;
    int np=0;
    if ( i!=0 && getSum(i,j,i-1,j,sum) ) np++;
    if ( i!=m_nrow-1 && getSum(i,j,i+1,j,sum) ) np++;
    if ( j!=0 && getSum(i,j,i,j-1,sum) ) np++;
    if ( j!=m_ncol-1 && getSum(i,j,i,j+1,sum) ) np++;
    if (sum[0]<0) sum[0]=0;
    if (sum[1]<0) sum[1]=0;
    if (sum[2]<0) sum[2]=0;
    m_this.at(i,j)=sum/(float)np;
    change+=abs(m_this.at(i,j).x[0]-m_last.at(i,j).x[0])+\
      abs(m_this.at(i,j).x[1]-m_last.at(i,j).x[1])+\
      abs(m_this.at(i,j).x[2]-m_last.at(i,j).x[2]);
  }
  int n_src=0;
  for (int i=0;i<m_nrow;i++) for (int j=0;j<m_ncol;j++)
  {
    if (m_mask->at(i,j)==m_srcindex)
    {
      m_last.at(i,j)=m_this.at(i,j);
      n_src++;
    }
  }
  return change/n_src;
}

bool ImageBlender::getSum( int i, int j, int i1, int j1, Vec3f &sum )
{
  int dir=(i==i1?1:0);
  float sign=(float)(dir==0?i-i1:j-j1);
  if (m_mask->at(i1,j1)==m_srcindex)
  {
    sum+=m_last(i1,j1) + (m_v(i,j).x[dir]+m_v(i1,j1).x[dir])/2.0f*sign;
    return true;
  }
  else if (m_mask->at(i1,j1)==m_dstindex)
  {
    sum+= Vec3f(m_dst->at(i1,j1).x[0],m_dst->at(i1,j1).x[1],m_dst->at(i1,j1).x[2]) + m_v(i,j).x[dir]*sign;
    return true;
  }
  return false;
}

void ImageBlender::createGuideVector()
{
  m_v.setSize(m_nrow,m_ncol);

  for (int i=0;i<m_nrow;i++) for (int j=0;j<m_ncol;j++)
  {
    if (m_mask->at(i,j)==m_srcindex)
    {
      if (i!=0 && m_mask->at(i-1,j)==m_srcindex && i!=m_nrow-1 && m_mask->at(i+1,j)==m_srcindex)
        m_v(i,j).x[0]=(m_last.at(i+1,j)-m_last.at(i-1,j))/2.0;
      else if (i!=0 && m_mask->at(i-1,j)==m_srcindex)
        m_v(i,j).x[0]=m_last.at(i,j)-m_last.at(i-1,j);
      else if (i!=m_nrow-1 && m_mask->at(i+1,j)==m_srcindex)
        m_v(i,j).x[0]=m_last.at(i+1,j)-m_last.at(i,j);
      else
        m_v(i,j).x[0]=0.0;

      if (j!=0 && m_mask->at(i,j-1)==m_srcindex && j!=m_ncol-1 && m_mask->at(i,j+1)==m_srcindex)
        m_v(i,j).x[1]=(m_last.at(i,j+1)-m_last.at(i,j-1))/2.0;
      else if (j!=0 && m_mask->at(i,j-1)==m_srcindex)
        m_v(i,j).x[1]=m_last.at(i,j)-m_last.at(i,j-1);
      else if (j!=m_ncol-1 && m_mask->at(i,j+1)==m_srcindex)
        m_v(i,j).x[1]=m_last.at(i,j+1)-m_last.at(i,j);
      else
        m_v(i,j).x[1]=0.0;
    }
    else if (m_mask->at(i,j)==m_dstindex)
    {
      bool besidesrc=false;
      if (i!=0 && m_mask->at(i-1,j)==m_srcindex) besidesrc=true;
      else if (i!=m_nrow-1 && m_mask->at(i+1,j)==m_srcindex) besidesrc=true;
      else if (j!=0 && m_mask->at(i,j-1)==m_srcindex) besidesrc=true;
      else if (j!=m_ncol-1 && m_mask->at(i,j+1)==m_srcindex) besidesrc=true;
      if (!besidesrc) continue;

      if (i!=0 && m_mask->at(i-1,j)==m_dstindex && i!=m_nrow-1 && m_mask->at(i+1,j)==m_dstindex)
        m_v(i,j).x[0]=(m_last.at(i+1,j)-m_last.at(i-1,j))/2.0;
      else if (i!=0 && m_mask->at(i-1,j)==m_dstindex)
        m_v(i,j).x[0]=m_last.at(i,j)-m_last.at(i-1,j);
      else if (i!=m_nrow-1 && m_mask->at(i+1,j)==m_dstindex)
        m_v(i,j).x[0]=m_last.at(i+1,j)-m_last.at(i,j);
      else
        m_v(i,j).x[0]=0.0;

      if (j!=0 && m_mask->at(i,j-1)==m_dstindex && j!=m_ncol-1 && m_mask->at(i,j+1)==m_dstindex)
        m_v(i,j).x[1]=(m_last.at(i,j+1)-m_last.at(i,j-1))/2.0;
      else if (j!=0 && m_mask->at(i,j-1)==m_dstindex)
        m_v(i,j).x[1]=m_last.at(i,j)-m_last.at(i,j-1);
      else if (j!=m_ncol-1 && m_mask->at(i,j+1)==m_dstindex)
        m_v(i,j).x[1]=m_last.at(i,j+1)-m_last.at(i,j);
      else
        m_v(i,j).x[1]=0.0;
    }
  }
}

void ImageBlender::blendDirect( Image3uc *const src, int srcindex, Image3uc *dst, int dstindex, Array2i *mask )
{
  m_src=src;
  m_mask=mask;
  m_dst=dst;
  m_srcindex=srcindex;
  m_dstindex=dstindex;
  m_nrow=mask->size(0);
  m_ncol=mask->size(1);
  m_last.setSize(m_nrow,m_ncol);
  m_idxmap.clear();
  int n=0;
  for (int i=0;i<m_nrow;i++) for (int j=0;j<m_ncol;j++)
    if (m_mask->at(i,j)==m_srcindex)
    {
      m_last.at(i,j).x[0]=m_src->at(i,j).x[0];
      m_last.at(i,j).x[1]=m_src->at(i,j).x[1];
      m_last.at(i,j).x[2]=m_src->at(i,j).x[2];
      m_idxmap[make_pair(i,j)]=n++;
    }
    else if (m_mask->at(i,j)==m_dstindex)
    {
      m_last.at(i,j).x[0]=m_dst->at(i,j).x[0];
      m_last.at(i,j).x[1]=m_dst->at(i,j).x[1];
      m_last.at(i,j).x[2]=m_dst->at(i,j).x[2];
    }
    cout<<"number of pixel to be processed: "<<n<<endl;
    createGuideVector();

    noboundary=true;
    //fill A,B
    ccs_data A(n);
    vector<double> B[3];
    B[0].resize(n,0);
    B[1].resize(n,0);
    B[2].resize(n,0);
    vector<double> start[3];
    start[0].resize(n,0);
    start[1].resize(n,0);
    start[2].resize(n,0);
    for (map<pair<int,int>,int>::iterator mi=m_idxmap.begin();mi!=m_idxmap.end();mi++)
    {
      int idx=mi->second;
      int r=mi->first.first,c=mi->first.second;
      start[0][idx]=m_src->at(r,c)[0];
      start[1][idx]=m_src->at(r,c)[1];
      start[2][idx]=m_src->at(r,c)[2];
      int np=0;
      if (r>0 && addToMatrixDirect(r,c,r-1,c,A,B)) np++;
      if (r<m_nrow-1 && addToMatrixDirect(r,c,r+1,c,A,B)) np++;
      if (c>0 && addToMatrixDirect(r,c,r,c-1,A,B)) np++;
      if (c<m_ncol-1 && addToMatrixDirect(r,c,r,c+1,A,B)) np++;
      addValue(A,idx,idx,np);
      if (np==0)
        cout<<"isolated point!"<<endl;
    }
    for (size_t i=0;i<A.size();i++)
      sort(A[i].begin(),A[i].end());
    if (noboundary)
    {
      cout<<"NO BOUNDARY CONSTRAINTS, this should not happen!\n";
      for (map<pair<int,int>,int>::iterator mi=m_idxmap.begin();mi!=m_idxmap.end();mi++)
      {
        int i=mi->first.first,j=mi->first.second;
        m_mask->at(i,j)=dstindex;
      }
      return;
    }

    mustSolve(A,n,n,B,start,3);
    //write mask & result
    for (map<pair<int,int>,int>::iterator mi=m_idxmap.begin();mi!=m_idxmap.end();mi++)
    {
      int i=mi->first.first,j=mi->first.second;
      int idx=mi->second;
      m_dst->at(i,j).x[0]=(unsigned char)puma_clamp<double>(B[0][idx],0,255);
      m_dst->at(i,j).x[1]=(unsigned char)puma_clamp<double>(B[1][idx],0,255);
      m_dst->at(i,j).x[2]=(unsigned char)puma_clamp<double>(B[2][idx],0,255);
      m_mask->at(i,j)=dstindex;
    }
}

bool ImageBlender::addToMatrixDirect( int i, int j, int i1, int j1,ccs_data &A, vector<double> B[3] )
{
  int dir=(i==i1?1:0);
  double sign=(dir==0?i-i1:j-j1);
  int idx1=m_idxmap[pair<int,int>(i,j)];

  if (m_mask->at(i1,j1)==m_srcindex)
  {
    int idx2=m_idxmap[pair<int,int>(i1,j1)];
    addValue(A,idx1,idx2,-1);
    B[0][idx1]+=(m_v(i,j).x[dir].x[0]+m_v(i1,j1).x[dir].x[0])/2.0*sign;
    B[1][idx1]+=(m_v(i,j).x[dir].x[1]+m_v(i1,j1).x[dir].x[1])/2.0*sign;
    B[2][idx1]+=(m_v(i,j).x[dir].x[2]+m_v(i1,j1).x[dir].x[2])/2.0*sign;
    return true;
  }
  else if (m_mask->at(i1,j1)==m_dstindex)
  {
    noboundary=false;
    B[0][idx1]+= double(m_dst->at(i1,j1).x[0]) + (m_v(i,j).x[dir].x[0]+m_v(i1,j1).x[dir].x[0])/2.0*sign;
    B[1][idx1]+= double(m_dst->at(i1,j1).x[1]) + (m_v(i,j).x[dir].x[1]+m_v(i1,j1).x[dir].x[1])/2.0*sign;
    B[2][idx1]+= double(m_dst->at(i1,j1).x[2]) + (m_v(i,j).x[dir].x[2]+m_v(i1,j1).x[dir].x[2])/2.0*sign;
    return true;
  }
  return false;
}

// void ImageBlender::blendDirect( Image3uc *const src, int srcindex, Image3uc *dst, int dstindex, Array2i *mask )
// {
//   m_src=src;
//   m_mask=mask;
//   m_dst=dst;
//   m_srcindex=srcindex;
//   m_dstindex=dstindex;
//   m_nrow=mask->size(0);
//   m_ncol=mask->size(1);
//   m_last.setSize(m_nrow,m_ncol);
//   m_idxmap.clear();
//   int n=0;
//   for (int i=0;i<m_nrow;i++) for (int j=0;j<m_ncol;j++)
//     if (m_mask->at(i,j)==m_srcindex)
//     {
//       m_last.at(i,j).x[0]=m_src->at(i,j).x[0];
//       m_last.at(i,j).x[1]=m_src->at(i,j).x[1];
//       m_last.at(i,j).x[2]=m_src->at(i,j).x[2];
//       m_idxmap[make_pair(i,j)]=n++;
//     }
//     else if (m_mask->at(i,j)==m_dstindex)
//     {
//       m_last.at(i,j).x[0]=m_dst->at(i,j).x[0];
//       m_last.at(i,j).x[1]=m_dst->at(i,j).x[1];
//       m_last.at(i,j).x[2]=m_dst->at(i,j).x[2];
//     }
//   cout<<"number of pixel to be processed: "<<n<<endl;
//   createGuideVector();
//
//   noboundary=true;
//   //fill A,B
//   SparseSMatd A(n);
//   vector<VarVecd> B(3);
//   B[0].setSize(n);
//   B[1].setSize(n);
//   B[2].setSize(n);
//   B[0].set(0.0);
//   B[1].set(0.0);
//   B[2].set(0.0);
//
//   for (map<pair<int,int>,int>::iterator mi=m_idxmap.begin();mi!=m_idxmap.end();mi++)
//   {
//     int idx=mi->second;
//     int r=mi->first.first,c=mi->first.second;
//     int np=0;
//     if (r>0 && addToMatrixDirect(r,c,r-1,c,A,B)) np++;
//     if (r<m_nrow-1 && addToMatrixDirect(r,c,r+1,c,A,B)) np++;
//     if (c>0 && addToMatrixDirect(r,c,r,c-1,A,B)) np++;
//     if (c<m_ncol-1 && addToMatrixDirect(r,c,r,c+1,A,B)) np++;
//     A.at(idx,idx)=np;
//     if (np==0)
//       cout<<"isolated point!"<<endl;
//   }
//   if (noboundary)
//   {
//     cout<<"NO BOUNDARY CONSTRAINTS, this should not happen!\n";
//     for (map<pair<int,int>,int>::iterator mi=m_idxmap.begin();mi!=m_idxmap.end();mi++)
//     {
//       int i=mi->first.first,j=mi->first.second;
//       m_mask->at(i,j)=dstindex;
//     }
//     return;
//   }
//
//   SparseLinearSystem<double> solver(A);
//   vector<VarVecd> solution;
//   solver.lu(B,solution);
//   //write mask & result
//   for (map<pair<int,int>,int>::iterator mi=m_idxmap.begin();mi!=m_idxmap.end();mi++)
//   {
//     int i=mi->first.first,j=mi->first.second;
//     int idx=mi->second;
//     m_dst->at(i,j).x[0]=(unsigned char)puma_clamp<double>(solution[0][idx],0,255);
//     m_dst->at(i,j).x[1]=(unsigned char)puma_clamp<double>(solution[1][idx],0,255);
//     m_dst->at(i,j).x[2]=(unsigned char)puma_clamp<double>(solution[2][idx],0,255);
//     m_mask->at(i,j)=dstindex;
//   }
// }
//
// bool ImageBlender::addToMatrixDirect( int i, int j, int i1, int j1,SparseSMatd &A, vector<VarVecd> B )
// {
//   int dir=(i==i1?1:0);
//   double sign=(dir==0?i-i1:j-j1);
//   int idx1=m_idxmap[pair<int,int>(i,j)];
//
//   if (m_mask->at(i1,j1)==m_srcindex)
//   {
//     int idx2=m_idxmap[pair<int,int>(i1,j1)];
//     A.at(idx1,idx2)=-1;
//     B[0][idx1]+=(m_v(i,j).x[dir].x[0]+m_v(i1,j1).x[dir].x[0])/2.0*sign;
//     B[1][idx1]+=(m_v(i,j).x[dir].x[1]+m_v(i1,j1).x[dir].x[1])/2.0*sign;
//     B[2][idx1]+=(m_v(i,j).x[dir].x[2]+m_v(i1,j1).x[dir].x[2])/2.0*sign;
//     return true;
//   }
//   else if (m_mask->at(i1,j1)==m_dstindex)
//   {
//     noboundary=false;
//     B[0][idx1]+= double(m_dst->at(i1,j1).x[0]) + (m_v(i,j).x[dir].x[0]+m_v(i1,j1).x[dir].x[0])/2.0*sign;
//     B[1][idx1]+= double(m_dst->at(i1,j1).x[1]) + (m_v(i,j).x[dir].x[1]+m_v(i1,j1).x[dir].x[1])/2.0*sign;
//     B[2][idx1]+= double(m_dst->at(i1,j1).x[2]) + (m_v(i,j).x[dir].x[2]+m_v(i1,j1).x[dir].x[2])/2.0*sign;
//     return true;
//   }
//   return false;
// }
//////////////////////////////////////////////////////////////////////////
void ImageBlender::blendDirectKeepBoundary( Image3uc *const src, int srcindex, Image3uc *dst, int dstindex, Array2i *mask )
{
  m_src=src;
  m_mask=mask;
  m_dst=dst;
  m_srcindex=srcindex;
  m_dstindex=dstindex;
  m_nrow=mask->size(0);
  m_ncol=mask->size(1);
  m_last.setSize(m_nrow,m_ncol);
  m_idxmap.clear();
  int n=0;
  for (int i=0;i<m_nrow;i++) for (int j=0;j<m_ncol;j++)
    if (m_mask->at(i,j)==m_srcindex)
    {
      m_last.at(i,j).x[0]=m_src->at(i,j).x[0];
      m_last.at(i,j).x[1]=m_src->at(i,j).x[1];
      m_last.at(i,j).x[2]=m_src->at(i,j).x[2];
      m_idxmap[make_pair(i,j)]=n++;
    }
    else if (m_mask->at(i,j)==m_dstindex)
    {
      m_last.at(i,j).x[0]=m_dst->at(i,j).x[0];
      m_last.at(i,j).x[1]=m_dst->at(i,j).x[1];
      m_last.at(i,j).x[2]=m_dst->at(i,j).x[2];
    }
    cout<<"number of pixel to be processed: "<<n<<endl;
    createGuideVector();

    noboundary=true;
    //fill A,B
    ccs_data A(n);
    vector<double> B[3];
    B[0].resize(n,0);
    B[1].resize(n,0);
    B[2].resize(n,0);
    vector<double> start[3];
    start[0].resize(n,0);
    start[1].resize(n,0);
    start[2].resize(n,0);
    for (map<pair<int,int>,int>::iterator mi=m_idxmap.begin();mi!=m_idxmap.end();mi++)
    {
      int idx=mi->second;
      int r=mi->first.first,c=mi->first.second;
      start[0][idx]=m_src->at(r,c)[0];
      start[1][idx]=m_src->at(r,c)[1];
      start[2][idx]=m_src->at(r,c)[2];
      //fix boundary
      if (r==0 || c==0 || r==m_nrow-1 || c==m_ncol-1 \
        || m_mask->at(r-1,c)==-1 || m_mask->at(r+1,c)==-1 || m_mask->at(r,c-1)==-1 || m_mask->at(r,c+1)==-1)
      {
        addValue(A,idx,idx,1);
        B[0][idx]=m_src->at(r,c)[0];
        B[1][idx]=m_src->at(r,c)[1];
        B[2][idx]=m_src->at(r,c)[2];
        continue;
      }
      int np=0;
      if (r>0 && addToMatrixKeepBoundary(r,c,r-1,c,A,B)) np++;
      if (r<m_nrow-1 && addToMatrixKeepBoundary(r,c,r+1,c,A,B)) np++;
      if (c>0 && addToMatrixKeepBoundary(r,c,r,c-1,A,B)) np++;
      if (c<m_ncol-1 && addToMatrixKeepBoundary(r,c,r,c+1,A,B)) np++;
      addValue(A,idx,idx,np);
      if (np==0)
        cout<<"isolated point!"<<endl;
    }
    for (size_t i=0;i<A.size();i++)
      sort(A[i].begin(),A[i].end());
    if (noboundary)
    {
      for (map<pair<int,int>,int>::iterator mi=m_idxmap.begin();mi!=m_idxmap.end();mi++)
      {
        int i=mi->first.first,j=mi->first.second;
        m_mask->at(i,j)=dstindex;
      }
      return;
    }

    mustSolve(A,n,n,B,start,3);
    //write mask & result
    for (map<pair<int,int>,int>::iterator mi=m_idxmap.begin();mi!=m_idxmap.end();mi++)
    {
      int i=mi->first.first,j=mi->first.second;
      int idx=mi->second;
      m_dst->at(i,j).x[0]=(unsigned char)puma_clamp<double>(B[0][idx],0,255);
      m_dst->at(i,j).x[1]=(unsigned char)puma_clamp<double>(B[1][idx],0,255);
      m_dst->at(i,j).x[2]=(unsigned char)puma_clamp<double>(B[2][idx],0,255);
      m_mask->at(i,j)=dstindex;
    }
}

bool ImageBlender::addToMatrixKeepBoundary( int i, int j, int i1, int j1,ccs_data &A, vector<double> B[3] )
{
  int dir=(i==i1?1:0);
  double sign=(dir==0?i-i1:j-j1);
  int idx1=m_idxmap[pair<int,int>(i,j)];

  if (m_mask->at(i1,j1)==m_srcindex)
  {
    int idx2=m_idxmap[pair<int,int>(i1,j1)];
    addValue(A,idx1,idx2,-1);
    B[0][idx1]+=(m_v(i,j).x[dir].x[0]+m_v(i1,j1).x[dir].x[0])/2.0*sign;
    B[1][idx1]+=(m_v(i,j).x[dir].x[1]+m_v(i1,j1).x[dir].x[1])/2.0*sign;
    B[2][idx1]+=(m_v(i,j).x[dir].x[2]+m_v(i1,j1).x[dir].x[2])/2.0*sign;
    return true;
  }
  else if (m_mask->at(i1,j1)==m_dstindex)
  {
    //too large difference
    double c1=double(m_src->at(i,j)[0])+double(m_src->at(i,j)[1])+double(m_src->at(i,j)[2]);
    double c2=double(m_dst->at(i1,j1)[0])+double(m_dst->at(i1,j1)[1])+double(m_dst->at(i1,j1)[2]);
    if (abs(c1-c2)>70*3) return false;
    noboundary=false;
    B[0][idx1]+= double(m_dst->at(i1,j1).x[0]) + (m_v(i,j).x[dir].x[0]+m_v(i1,j1).x[dir].x[0])/2.0*sign;
    B[1][idx1]+= double(m_dst->at(i1,j1).x[1]) + (m_v(i,j).x[dir].x[1]+m_v(i1,j1).x[dir].x[1])/2.0*sign;
    B[2][idx1]+= double(m_dst->at(i1,j1).x[2]) + (m_v(i,j).x[dir].x[2]+m_v(i1,j1).x[dir].x[2])/2.0*sign;
    return true;
  }
  return false;
}
//////////////////////////////////////////////////////////////////////////
void ImageBlender::blendAllDirect( Image3uc *img, Array2i *mask )
{
  m_src=img;
  m_mask=mask;
  m_nrow=mask->size(0);
  m_ncol=mask->size(1);
  m_last.setSize(m_nrow,m_ncol);
  m_idxmap.clear();
  int n=0;
  for (int i=0;i<m_nrow;i++) for (int j=0;j<m_ncol;j++)
  {
    if (m_mask->at(i,j)<0) continue;
    m_last.at(i,j).x[0]=img->at(i,j).x[0];
    m_last.at(i,j).x[1]=img->at(i,j).x[1];
    m_last.at(i,j).x[2]=img->at(i,j).x[2];
    m_idxmap[make_pair(i,j)]=n++;
  }
  cout<<"number of pixel to be processed: "<<n<<endl;
  if(n<10) return;

  //////////////////////////////////////////////////////////////////////////
  //guide vector
  m_v.setSize(m_nrow,m_ncol);
  for (int i=0;i<m_nrow;i++) for (int j=0;j<m_ncol;j++)
  {
    int imask=m_mask->at(i,j);
    if (imask<0) continue;

    if (i!=0 && m_mask->at(i-1,j)==imask && i!=m_nrow-1 && m_mask->at(i+1,j)==imask)
      m_v(i,j).x[0]=(m_last.at(i+1,j)-m_last.at(i-1,j))/2.0;
    else if (i!=0 && m_mask->at(i-1,j)==imask)
      m_v(i,j).x[0]=m_last.at(i,j)-m_last.at(i-1,j);
    else if (i!=m_nrow-1 && m_mask->at(i+1,j)==imask)
      m_v(i,j).x[0]=m_last.at(i+1,j)-m_last.at(i,j);
    else
      m_v(i,j).x[0]=0.0;

    if (j!=0 && m_mask->at(i,j-1)==imask && j!=m_ncol-1 && m_mask->at(i,j+1)==imask)
      m_v(i,j).x[1]=(m_last.at(i,j+1)-m_last.at(i,j-1))/2.0;
    else if (j!=0 && m_mask->at(i,j-1)==imask)
      m_v(i,j).x[1]=m_last.at(i,j)-m_last.at(i,j-1);
    else if (j!=m_ncol-1 && m_mask->at(i,j+1)==imask)
      m_v(i,j).x[1]=m_last.at(i,j+1)-m_last.at(i,j);
    else
      m_v(i,j).x[1]=0.0;
  }

  //////////////////////////////////////////////////////////////////////////
  //fill A,B
  ccs_data A(n);
  vector<double> B[3];
  B[0].resize(n,0);
  B[1].resize(n,0);
  B[2].resize(n,0);
  vector<double> start[3];
  start[0].resize(n,0);
  start[1].resize(n,0);
  start[2].resize(n,0);
  for (int r=0;r<m_nrow;r++) for (int c=0;c<m_ncol;c++)
  {
    if (m_mask->at(r,c)<0) continue;
    int idx=m_idxmap[make_pair(r,c)];
    start[0][idx]=m_src->at(r,c)[0];
    start[1][idx]=m_src->at(r,c)[1];
    start[2][idx]=m_src->at(r,c)[2];

    //fix boundary
    if (r==0 || r==m_nrow-1 || c==0 || c==m_ncol-1 \
      || mask->at(r-1,c)==-1 || mask->at(r+1,c)==-1 || mask->at(r,c)==-1 || mask->at(r,c+1)==-1)
    {
      addValue(A,idx,idx,1);
      B[0][idx]=img->at(r,c)[0];
      B[1][idx]=img->at(r,c)[1];
      B[2][idx]=img->at(r,c)[2];
      continue;
    }
    int np=0;
    if (r>0 && addToMatrixAllDirect(r,c,r-1,c,A,B)) np++;
    if (r<m_nrow-1 && addToMatrixAllDirect(r,c,r+1,c,A,B)) np++;
    if (c>0 && addToMatrixAllDirect(r,c,r,c-1,A,B)) np++;
    if (c<m_ncol-1 && addToMatrixAllDirect(r,c,r,c+1,A,B)) np++;
    if (np==0)
    {
      addValue(A,idx,idx,1);
      B[0][idx]=img->at(r,c)[0];
      B[1][idx]=img->at(r,c)[1];
      B[2][idx]=img->at(r,c)[2];
    }
    else
      addValue(A,idx,idx,np);
  }

  cout<<"solving...";
  mustSolve(A,n,n,B,start,3);
  cout<<"done!\n";
  //write mask & result
  for (map<pair<int,int>,int>::iterator mi=m_idxmap.begin();mi!=m_idxmap.end();mi++)
  {
    int i=mi->first.first,j=mi->first.second;
    int idx=mi->second;
    img->at(i,j).x[0]=(unsigned char)puma_clamp<double>(B[0][idx],0,255);
    img->at(i,j).x[1]=(unsigned char)puma_clamp<double>(B[1][idx],0,255);
    img->at(i,j).x[2]=(unsigned char)puma_clamp<double>(B[2][idx],0,255);
    mask->at(i,j)=1;
  }
}
bool ImageBlender::addToMatrixAllDirect( int i, int j, int i1, int j1,ccs_data &A, vector<double> B[3] )
{
//   double c1=double(m_src->at(i,j)[0])+double(m_src->at(i,j)[1])+double(m_src->at(i,j)[2]);
//   double c2=double(m_src->at(i1,j1)[0])+double(m_src->at(i1,j1)[1])+double(m_src->at(i1,j1)[2]);
//   if (abs(c1-c2)>70*3) return false;
  int dir=(i==i1?1:0);
  double sign=(dir==0?i-i1:j-j1);
  int idx1=m_idxmap[pair<int,int>(i,j)];
  int idx2=m_idxmap[pair<int,int>(i1,j1)];
  addValue(A,idx1,idx2,-1);
  B[0][idx1]+=(m_v(i,j).x[dir].x[0]+m_v(i1,j1).x[dir].x[0])/2.0*sign;
  B[1][idx1]+=(m_v(i,j).x[dir].x[1]+m_v(i1,j1).x[dir].x[1])/2.0*sign;
  B[2][idx1]+=(m_v(i,j).x[dir].x[2]+m_v(i1,j1).x[dir].x[2])/2.0*sign;
  return true;
}

//////////////////////////////////////////////////////////////////////////
void ImageBlender::blendImageSequential( Image3uc *img, Array2i *mask )
{
  Image3uc &image=*img;
  cout<<"texture size: "<<image.columns()<<'x'<<image.rows()<<endl;
  Array2i cluster=*mask;
  //make cluster
  for (int r=0;r<cluster.rows();r++) for (int c=0;c<cluster.columns();c++)
    if (cluster.at(r,c)!=-1) cluster.at(r,c)=-2;  //mark the non-empty region
  int cluster_idx=0;
  for (int r=0;r<cluster.rows();r++) for (int c=0;c<cluster.columns();c++)
    if (cluster.at(r,c)==-2) clusterHelper(cluster,make_pair(r,c),cluster_idx++);
  cout<<"subregion number:"<<cluster_idx<<endl;
  //for each cluster, blend them
  Array2i thismask(cluster.rows(),cluster.columns());
  for (int i=0;i<cluster_idx;i++)
  {
    //prepare mask for this cluster
    for (int r=0;r<cluster.rows();r++) for (int c=0;c<cluster.columns();c++)
    {
      if (cluster.at(r,c)==i)  thismask.at(r,c)=mask->at(r,c);
      else thismask.at(r,c)=-1;
    }
    //cluster again, prevent color apart, (0,1,2,,,)->(-2,-3,-4,,,)
    int cluster_idx2=-2;
    for (int r=0;r<thismask.rows();r++) for (int c=0;c<thismask.columns();c++)
      if (thismask.at(r,c)>-1) clusterHelper2(thismask,make_pair(r,c),cluster_idx2--);
    cluster_idx2=-cluster_idx2-2;
    cout<<"color block number:"<<cluster_idx2<<endl;
    //find start point
    Array2uc checked(image.rows(),image.columns());
    checked.set((unsigned char)0);
    int center_index=-1;
    std::deque<std::pair<int,int> > checking;
    for (int r=0;r<cluster.rows();r++) for (int c=0;c<cluster.columns();c++)
      if (thismask.at(r,c)!=-1)
      {
        thismask.at(r,c)=-thismask.at(r,c)-2; //from negative back to normal mask
        if (center_index==-1)
        {
          center_index=thismask.at(r,c);
          checking.push_back(make_pair(r,c));
          checked.at(r,c)=1;
        }
      }
    while(!checking.empty())
    {
      std::pair<int,int> now=checking.front();
      if (thismask(now.first,now.second)!=center_index)
      {
        cout<<"Blending: "<<thismask(now.first,now.second)<<"->"<<center_index<<endl;
        blendDirectKeepBoundary(&image,thismask(now.first,now.second),&image,center_index,&thismask);
      }
      if (now.first>0 && checked(now.first-1,now.second)==0 && thismask(now.first-1,now.second)!=-1)
      {
        checked(now.first-1,now.second)=1;
        checking.push_back(make_pair(now.first-1,now.second));
      }
      if (now.first<image.rows()-1 && checked(now.first+1,now.second)==0 && thismask(now.first+1,now.second)!=-1)
      {
        checked(now.first+1,now.second)=1;
        checking.push_back(make_pair(now.first+1,now.second));
      }
      if (now.second>0 && checked(now.first,now.second-1)==0 && thismask(now.first,now.second-1)!=-1)
      {
        checked(now.first,now.second-1)=1;
        checking.push_back(make_pair(now.first,now.second-1));
      }
      if (now.second<image.columns()-1 && checked(now.first,now.second+1)==0 && thismask(now.first,now.second+1)!=-1)
      {
        checked(now.first,now.second+1)=1;
        checking.push_back(make_pair(now.first,now.second+1));
      }
      checking.pop_front();
    }
  }
}

void ImageBlender::clusterHelper(  Array2i &cluster, pair<int,int> seed, int value )
{
  Array2uc checked(cluster.size());
  checked.set((unsigned char)0);
  deque<pair<int,int> > checking;
  checking.push_back(seed);
  checked.at(seed.first,seed.second)=1;
  while(!checking.empty())
  {
    pair<int,int> cur=checking.front();
    cluster.at(cur.first,cur.second)=value;
    if (cur.first>0 && checked.at(cur.first-1,cur.second)==0 && cluster.at(cur.first-1,cur.second)!=-1)
    {
      checking.push_back(make_pair(cur.first-1,cur.second));
      checked.at(cur.first-1,cur.second)=1;
    }
    if (cur.first<cluster.rows()-1 && checked.at(cur.first+1,cur.second)==0 && cluster.at(cur.first+1,cur.second)!=-1)
    {
      checking.push_back(make_pair(cur.first+1,cur.second));
      checked.at(cur.first+1,cur.second)=1;
    }
    if (cur.second>0 && checked.at(cur.first,cur.second-1)==0 && cluster.at(cur.first,cur.second-1)!=-1)
    {
      checking.push_back(make_pair(cur.first,cur.second-1));
      checked.at(cur.first,cur.second-1)=1;
    }
    if (cur.second<cluster.columns()-1 && checked.at(cur.first,cur.second+1)==0 && cluster.at(cur.first,cur.second+1)!=-1)
    {
      checking.push_back(make_pair(cur.first,cur.second+1));
      checked.at(cur.first,cur.second+1)=1;
    }
    checking.pop_front();
  }
}

void ImageBlender::clusterHelper2(  Array2i &cluster, pair<int,int> seed, int value )
{
  int orivalue=cluster.at(seed.first,seed.second);
  Array2uc checked(cluster.size());
  checked.set((unsigned char)0);
  deque<pair<int,int> > checking;
  checking.push_back(seed);
  checked.at(seed.first,seed.second)=1;
  while(!checking.empty())
  {
    pair<int,int> cur=checking.front();
    cluster.at(cur.first,cur.second)=value;
    if (cur.first>0 && checked.at(cur.first-1,cur.second)==0 && cluster.at(cur.first-1,cur.second)==orivalue)
    {
      checking.push_back(make_pair(cur.first-1,cur.second));
      checked.at(cur.first-1,cur.second)=1;
    }
    if (cur.first<cluster.rows()-1 && checked.at(cur.first+1,cur.second)==0 && cluster.at(cur.first+1,cur.second)==orivalue)
    {
      checking.push_back(make_pair(cur.first+1,cur.second));
      checked.at(cur.first+1,cur.second)=1;
    }
    if (cur.second>0 && checked.at(cur.first,cur.second-1)==0 && cluster.at(cur.first,cur.second-1)==orivalue)
    {
      checking.push_back(make_pair(cur.first,cur.second-1));
      checked.at(cur.first,cur.second-1)=1;
    }
    if (cur.second<cluster.columns()-1 && checked.at(cur.first,cur.second+1)==0 && cluster.at(cur.first,cur.second+1)==orivalue)
    {
      checking.push_back(make_pair(cur.first,cur.second+1));
      checked.at(cur.first,cur.second+1)=1;
    }
    checking.pop_front();
  }
}

//////////////////////////////////////////////////////////////////////////
void ImageBlender::prepareGuidedVector( Image3uc *img, Array2i *mask )
{
  m_nrow=img->rows();
  m_ncol=img->columns();
  m_mask=mask;
  m_last.setSize(m_nrow,m_ncol);
  for (int r=0;r<m_nrow;r++) for (int c=0;c<m_ncol;c++)
    m_last.at(r,c)=img->at(r,c);
  m_v.setSize(m_nrow,m_ncol);
  for (int r=0;r<m_nrow;r++) for (int c=0;c<m_ncol;c++)
  {
    int idx=mask->at(r,c);
    if (idx==-1) continue;

    if (r!=0 && m_mask->at(r-1,c)==idx && r!=m_nrow-1 && m_mask->at(r+1,c)==idx)
      m_v(r,c).x[0]=(m_last.at(r+1,c)-m_last.at(r-1,c))/2.0;
    else if (r!=0 && m_mask->at(r-1,c)==idx)
      m_v(r,c).x[0]=m_last.at(r,c)-m_last.at(r-1,c);
    else if (r!=m_nrow-1 && m_mask->at(r+1,c)==idx)
      m_v(r,c).x[0]=m_last.at(r+1,c)-m_last.at(r,c);
    else
      m_v(r,c).x[0]=0.0;

    if (c!=0 && m_mask->at(r,c-1)==idx && c!=m_ncol-1 && m_mask->at(r,c+1)==idx)
      m_v(r,c).x[1]=(m_last.at(r,c+1)-m_last.at(r,c-1))/2.0;
    else if (c!=0 && m_mask->at(r,c-1)==idx)
      m_v(r,c).x[1]=m_last.at(r,c)-m_last.at(r,c-1);
    else if (c!=m_ncol-1 && m_mask->at(r,c+1)==idx)
      m_v(r,c).x[1]=m_last.at(r,c+1)-m_last.at(r,c);
    else
      m_v(r,c).x[1]=0.0;
  }
}

double ImageBlender::blendSinglePatch( Image3uc *img, Array2i *mask, int srcindex )
{
  double change=0;
  m_srcindex=srcindex;
  m_mask=mask;
  m_src=img;
  m_idxmap.clear();
  int n=0;
  for (int i=0;i<m_nrow;i++) for (int j=0;j<m_ncol;j++)
  {
    if (m_mask->at(i,j)!=m_srcindex) continue;
    m_idxmap[make_pair(i,j)]=n++;
  }
  cout<<"number of pixel to be processed: "<<n<<endl;

  noboundary=true;
  //fill A,B
  ccs_data A(n);
  vector<double> B[3];
  B[0].resize(n,0);
  B[1].resize(n,0);
  B[2].resize(n,0);
  vector<double> start[3];
  start[0].resize(n,0);
  start[1].resize(n,0);
  start[2].resize(n,0);
  for (map<pair<int,int>,int>::iterator mi=m_idxmap.begin();mi!=m_idxmap.end();mi++)
  {
    int idx=mi->second;
    int r=mi->first.first,c=mi->first.second;
    start[0][idx]=m_src->at(r,c)[0];
    start[1][idx]=m_src->at(r,c)[1];
    start[2][idx]=m_src->at(r,c)[2];

    //fix boundary
    if (r==0 || c==0 || r==m_nrow-1 || c==m_ncol-1 \
      || m_mask->at(r-1,c)==-1 || m_mask->at(r+1,c)==-1 || m_mask->at(r,c-1)==-1 || m_mask->at(r,c+1)==-1)
    {
      addValue(A,idx,idx,1);
      B[0][idx]=img->at(r,c)[0];
      B[1][idx]=img->at(r,c)[1];
      B[2][idx]=img->at(r,c)[2];
      continue;
    }
    int np=0;
    if (r>0 && addToMatrixSinglePatch(r,c,r-1,c,A,B)) np++;
    if (r<m_nrow-1 && addToMatrixSinglePatch(r,c,r+1,c,A,B)) np++;
    if (c>0 && addToMatrixSinglePatch(r,c,r,c-1,A,B)) np++;
    if (c<m_ncol-1 && addToMatrixSinglePatch(r,c,r,c+1,A,B)) np++;
    addValue(A,idx,idx,np);
    if (np==0)
      cout<<"isolated point!"<<endl;
    }
  if (noboundary)
    return -1;
  for (size_t i=0;i<A.size();i++)
    sort(A[i].begin(),A[i].end());

  mustSolve(A,n,n,B,start,3);
  //write mask & result
  for (map<pair<int,int>,int>::iterator mi=m_idxmap.begin();mi!=m_idxmap.end();mi++)
  {
    int r=mi->first.first,c=mi->first.second;
    int idx=mi->second;
    Vec3d newpixel=Vec3d(puma_clamp<double>(B[0][idx],0,255),\
                         puma_clamp<double>(B[1][idx],0,255),\
                         puma_clamp<double>(B[2][idx],0,255));
    change+=abs(newpixel[0]-img->at(r,c)[0]);
    change+=abs(newpixel[1]-img->at(r,c)[1]);
    change+=abs(newpixel[2]-img->at(r,c)[2]);
    img->at(r,c)=newpixel;
  }
  return change/n/3.0;
}

bool ImageBlender::addToMatrixSinglePatch( int i, int j, int i1, int j1,ccs_data &A, vector<double> B[3] )
{
  int dir=(i==i1?1:0);
  double sign=(dir==0?i-i1:j-j1);
  int idx1=m_idxmap[pair<int,int>(i,j)];

  if (m_mask->at(i1,j1)==m_srcindex)
  {
    int idx2=m_idxmap[pair<int,int>(i1,j1)];
    addValue(A,idx1,idx2,-1);
    B[0][idx1]+=(m_v(i,j).x[dir].x[0]+m_v(i1,j1).x[dir].x[0])/2.0*sign;
    B[1][idx1]+=(m_v(i,j).x[dir].x[1]+m_v(i1,j1).x[dir].x[1])/2.0*sign;
    B[2][idx1]+=(m_v(i,j).x[dir].x[2]+m_v(i1,j1).x[dir].x[2])/2.0*sign;
    return true;
  }
  else if (m_mask->at(i1,j1)!=-1)
  {
    noboundary=false;
    B[0][idx1]+= double(m_src->at(i1,j1).x[0]) + (m_v(i,j).x[dir].x[0]+m_v(i1,j1).x[dir].x[0])/2.0*sign;
    B[1][idx1]+= double(m_src->at(i1,j1).x[1]) + (m_v(i,j).x[dir].x[1]+m_v(i1,j1).x[dir].x[1])/2.0*sign;
    B[2][idx1]+= double(m_src->at(i1,j1).x[2]) + (m_v(i,j).x[dir].x[2]+m_v(i1,j1).x[dir].x[2])/2.0*sign;
    return true;
  }
  return false;
}

void ImageBlender::blendItrativeGlobal( Image3uc *img, Array2i *mask )
{
  Image3uc &image=*img;
  cout<<"texture size: "<<image.columns()<<'x'<<image.rows()<<endl;
  Array2i cluster=*mask;
  //make cluster
  for (int r=0;r<cluster.rows();r++) for (int c=0;c<cluster.columns();c++)
    if (cluster.at(r,c)!=-1) cluster.at(r,c)=-2;  //mark the non-empty region
  int cluster_idx=0;
  for (int r=0;r<cluster.rows();r++) for (int c=0;c<cluster.columns();c++)
    if (cluster.at(r,c)==-2) clusterHelper(cluster,make_pair(r,c),cluster_idx++);
  cout<<"subregion number:"<<cluster_idx<<endl;
  //for each cluster, blend them
  Array2i thismask(cluster.rows(),cluster.columns());
  for (int i=0;i<cluster_idx;i++)
  {
    //prepare mask for this cluster
    for (int r=0;r<cluster.rows();r++) for (int c=0;c<cluster.columns();c++)
    {
      if (cluster.at(r,c)==i)  thismask.at(r,c)=mask->at(r,c);
      else thismask.at(r,c)=-1;
    }
    //cluster again, prevent color apart, (0,1,2,,,)->(-2,-3,-4,,,)
    int cluster_idx2=-2;
    for (int r=0;r<thismask.rows();r++) for (int c=0;c<thismask.columns();c++)
      if (thismask.at(r,c)>-1) clusterHelper2(thismask,make_pair(r,c),cluster_idx2--);
    cluster_idx2=-cluster_idx2-2;
    cout<<"color block number:"<<cluster_idx2<<endl;
    for (int r=0;r<cluster.rows();r++) for (int c=0;c<cluster.columns();c++)
      if (thismask.at(r,c)!=-1) thismask.at(r,c)=-thismask.at(r,c)-2; //from negative back to normal mask
    prepareGuidedVector(img,&thismask);
    int count=0;
    deque<double> changes(puma_max(cluster_idx2/10,1),0);
    double allchange=0;
    while(true)
    {
      int x=puma_uniform_rand<int>(0,cluster_idx2);
      double change=blendSinglePatch(img,&thismask,x);
      if (change!=-1)
      {
        allchange+=change;
        allchange-=changes.front();
        changes.pop_front();
        changes.push_back(change);
        cout<<"allchanges:"<<allchange<<"=>"<<changes.size()/2.0<<endl;
      }
      count++;
      if (count>cluster_idx2*2 && allchange*2<changes.size()) break;
    }
  }
}
}
