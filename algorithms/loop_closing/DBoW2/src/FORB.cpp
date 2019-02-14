/**
 * File: FORB.cpp
 * Date: June 2012
 * Author: Dorian Galvez-Lopez
 * Description: functions for ORB descriptors
 * License: see the LICENSE.txt file
 *
 */
 
#include <vector>
#include <string>
#include <sstream>
#include <stdint.h>
#include <limits.h>

#include "FORB.h"

using namespace std;

namespace DBoW2 {

// --------------------------------------------------------------------------

void FORB::meanValue(const std::vector<FORB::pDescriptor> &descriptors, 
  FORB::TDescriptor &mean)
{
  if(descriptors.empty())
  {
    mean.release();
    return;
  }
  else if(descriptors.size() == 1)
  {
    mean = descriptors[0]->clone();
  }
  else
  {
    vector<int> sum(FORB::L * 8, 0);
    
    for(size_t i = 0; i < descriptors.size(); ++i)
    {
      const cv::Mat &d = *descriptors[i];
      const unsigned char *p = d.ptr<unsigned char>();
      
      for(int j = 0; j < d.cols; ++j, ++p)
      {
        if(*p & (1 << 7)) ++sum[ j*8     ];
        if(*p & (1 << 6)) ++sum[ j*8 + 1 ];
        if(*p & (1 << 5)) ++sum[ j*8 + 2 ];
        if(*p & (1 << 4)) ++sum[ j*8 + 3 ];
        if(*p & (1 << 3)) ++sum[ j*8 + 4 ];
        if(*p & (1 << 2)) ++sum[ j*8 + 5 ];
        if(*p & (1 << 1)) ++sum[ j*8 + 6 ];
        if(*p & (1))      ++sum[ j*8 + 7 ];
      }
    }
    
    mean = cv::Mat::zeros(1, FORB::L, CV_8U);
    unsigned char *p = mean.ptr<unsigned char>();
    
    const int N2 = (int)descriptors.size() / 2 + descriptors.size() % 2;
    for(size_t i = 0; i < sum.size(); ++i)
    {
      if(sum[i] >= N2)
      {
        // set bit
        *p |= 1 << (7 - (i % 8));
      }
      
      if(i % 8 == 7) ++p;
    }
  }
}

// --------------------------------------------------------------------------
  
double FORB::distance(const FORB::TDescriptor &a, 
  const FORB::TDescriptor &b)
{
  // Bit count function got from:
  // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetKernighan
  // This implementation assumes that a.cols (CV_8U) % sizeof(uint64_t) == 0
  
  const uint64_t *pa, *pb;
  pa = a.ptr<uint64_t>(); // a & b are actually CV_8U
  pb = b.ptr<uint64_t>(); 
  
  uint64_t v, ret = 0;
  for(size_t i = 0; i < a.cols / sizeof(uint64_t); ++i, ++pa, ++pb)
  {
    v = *pa ^ *pb;
    v = v - ((v >> 1) & (uint64_t)~(uint64_t)0/3);
    v = (v & (uint64_t)~(uint64_t)0/15*3) + ((v >> 2) & 
      (uint64_t)~(uint64_t)0/15*3);
    v = (v + (v >> 4)) & (uint64_t)~(uint64_t)0/255*15;
    ret += (uint64_t)(v * ((uint64_t)~(uint64_t)0/255)) >> 
      (sizeof(uint64_t) - 1) * CHAR_BIT;
  }
  
  return ret;
  
  // // If uint64_t is not defined in your system, you can try this 
  // // portable approach (requires DUtils from DLib)
  // const unsigned char *pa, *pb;
  // pa = a.ptr<unsigned char>();
  // pb = b.ptr<unsigned char>();
  // 
  // int ret = 0;
  // for(int i = 0; i < a.cols; ++i, ++pa, ++pb)
  // {
  //   ret += DUtils::LUT::ones8bits[ *pa ^ *pb ];
  // }
  //  
  // return ret;
}

// --------------------------------------------------------------------------
  
std::string FORB::toString(const FORB::TDescriptor &a)
{
  stringstream ss;
  const unsigned char *p = a.ptr<unsigned char>();
  
  for(int i = 0; i < a.cols; ++i, ++p)
  {
    ss << (int)*p << " ";
  }
  
  return ss.str();
}

// --------------------------------------------------------------------------
  
void FORB::fromString(FORB::TDescriptor &a, const std::string &s)
{
  a.create(1, FORB::L, CV_8U);
  unsigned char *p = a.ptr<unsigned char>();
  
  stringstream ss(s);
  for(int i = 0; i < FORB::L; ++i, ++p)
  {
    int n;
    ss >> n;
    
    if(!ss.fail()) 
      *p = (unsigned char)n;
  }
  
}

// --------------------------------------------------------------------------

void FORB::toMat32F(const std::vector<TDescriptor> &descriptors, 
  cv::Mat &mat)
{
  if(descriptors.empty())
  {
    mat.release();
    return;
  }
  
  const size_t N = descriptors.size();
  
  mat.create(N, FORB::L*8, CV_32F);
  float *p = mat.ptr<float>();
  
  for(size_t i = 0; i < N; ++i)
  {
    const int C = descriptors[i].cols;
    const unsigned char *desc = descriptors[i].ptr<unsigned char>();
    
    for(int j = 0; j < C; ++j, p += 8)
    {
      p[0] = (desc[j] & (1 << 7) ? 1 : 0);
      p[1] = (desc[j] & (1 << 6) ? 1 : 0);
      p[2] = (desc[j] & (1 << 5) ? 1 : 0);
      p[3] = (desc[j] & (1 << 4) ? 1 : 0);
      p[4] = (desc[j] & (1 << 3) ? 1 : 0);
      p[5] = (desc[j] & (1 << 2) ? 1 : 0);
      p[6] = (desc[j] & (1 << 1) ? 1 : 0);
      p[7] = desc[j] & (1);
    }
  } 
}

// --------------------------------------------------------------------------

void FORB::toMat32F(const cv::Mat &descriptors, cv::Mat &mat)
{

  descriptors.convertTo(mat, CV_32F);
  return; 

  if(descriptors.empty())
  {
    mat.release();
    return;
  }
  
  const int N = descriptors.rows;
  const int C = descriptors.cols;
  
  mat.create(N, FORB::L*8, CV_32F);
  float *p = mat.ptr<float>(); // p[i] == 1 or 0
  
  const unsigned char *desc = descriptors.ptr<unsigned char>();
  
  for(int i = 0; i < N; ++i, desc += C)
  {
    for(int j = 0; j < C; ++j, p += 8)
    {
      p[0] = (desc[j] & (1 << 7) ? 1 : 0);
      p[1] = (desc[j] & (1 << 6) ? 1 : 0);
      p[2] = (desc[j] & (1 << 5) ? 1 : 0);
      p[3] = (desc[j] & (1 << 4) ? 1 : 0);
      p[4] = (desc[j] & (1 << 3) ? 1 : 0);
      p[5] = (desc[j] & (1 << 2) ? 1 : 0);
      p[6] = (desc[j] & (1 << 1) ? 1 : 0);
      p[7] = desc[j] & (1);
    }
  } 
}

// --------------------------------------------------------------------------

void FORB::toMat8U(const std::vector<TDescriptor> &descriptors, 
  cv::Mat &mat)
{
  mat.create(descriptors.size(), FORB::L, CV_8U);
  
  unsigned char *p = mat.ptr<unsigned char>();
  
  for(size_t i = 0; i < descriptors.size(); ++i, p += FORB::L)
  {
    const unsigned char *d = descriptors[i].ptr<unsigned char>();
    std::copy(d, d + FORB::L, p);
  }
  
}

// --------------------------------------------------------------------------

} // namespace DBoW2

