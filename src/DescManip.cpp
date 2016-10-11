/**
 * File: DescManip.cpp
 * Date: June 2012
 * Author: Dorian Galvez-Lopez
 * Description: functions for ORB descriptors
 * License: see the LICENSE.txt file
 *
 */
 
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdint.h>
#include <limits.h>

#include "DescManip.h"

using namespace std;

namespace DBoW3 {

// --------------------------------------------------------------------------

void DescManip::meanValue(const std::vector<cv::Mat> &descriptors,
                       cv::Mat &mean)
{

    if(descriptors.empty()) return;

    if(descriptors.size() == 1)
    {
        mean = descriptors[0].clone();
        return;
    }
    //binary descriptor
    if (descriptors[0].type()==CV_8U ){
        //determine number of bytes of the binary descriptor
        int L= getnBytes( descriptors[0]);
        vector<int> sum( L * 8, 0);

        for(size_t i = 0; i < descriptors.size(); ++i)
        {
            const cv::Mat &d = descriptors[i];
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

        mean = cv::Mat::zeros(1, L, CV_8U);
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
    //non binary descriptor
    else{
        assert(descriptors[0].type()==CV_32F );//ensure it is float

        mean.create(1, descriptors[0].cols,descriptors[0].type());
        mean.setTo(cv::Scalar::all(0));
        float inv_s =1./double( descriptors.size());
        for(size_t i=0;i<descriptors.size();i++)
            mean +=  descriptors[i] * inv_s;

    }

}

// --------------------------------------------------------------------------

double DescManip::distance(const cv::Mat &a,  const cv::Mat &b)
{

    //binary descriptor
    if (a.type()==CV_8U){

        uint64_t ret=0;
        const uchar *pa = a.ptr<uchar>(); // a & b are actually CV_8U
        const uchar *pb = b.ptr<uchar>();
        for(int i=0;i<a.cols;i++,pa++,pb++){
            uchar v=(*pa)^(*pb);
#ifdef __GNUG__
            ret+=__builtin_popcount(v);//only in g++
#else

            ret+=v& (1<<0);
            ret+=v& (1<<1);
            ret+=v& (1<<2);
            ret+=v& (1<<3);
            ret+=v& (1<<4);
            ret+=v& (1<<5);
            ret+=v& (1<<6);
            ret+=v& (1<<7);
#endif
    }
        return ret;
    }
    else{
        double sqd = 0.;
        assert(a.type()==CV_32F);
        assert(a.rows==1);
        const float *a_ptr=a.ptr<float>(0);
        const float *b_ptr=b.ptr<float>(0);
        for(int i = 0; i < a.cols; i ++)
            sqd += (a_ptr[i  ] - b_ptr[i  ])*(a_ptr[i  ] - b_ptr[i  ]);
        return sqd;
    }
}




// --------------------------------------------------------------------------
  
std::string DescManip::toString(const cv::Mat &a)
{
    stringstream ss;
    //save size and type


    ss <<a.type()<<" "<<a.cols<<" ";

    if (a.type()==CV_8U){
        const unsigned char *p = a.ptr<unsigned char>();
        for(int i = 0; i < a.cols; ++i, ++p)
            ss << (int)*p << " ";
    }else{

        const float *p = a.ptr<float>();
        for(int i = 0; i < a.cols; ++i, ++p)
            ss <<  *p << " ";

    }

    return ss.str();
}

// --------------------------------------------------------------------------
  
void DescManip::fromString(cv::Mat &a, const std::string &s)
{

  int type,cols;
  stringstream ss(s);
  ss >>type>>cols;
  a.create(1,  cols, type);
  if(type==CV_8UC1){
  unsigned char *p = a.ptr<unsigned char>();
  int n;
  for(int i = 0; i <  a.cols; ++i, ++p)
    if ( ss >> n) *p = (unsigned char)n;
  }
  else{
      float *p = a.ptr<float>();
      for(int i = 0; i <  a.cols; ++i, ++p)
        if ( !(ss >> *p))cerr<<"Error reading. Unexpected EOF. DescManip::fromString"<<endl;
  }

  
}

// --------------------------------------------------------------------------

void DescManip::toMat32F(const std::vector<cv::Mat> &descriptors,
                     cv::Mat &mat)
{
    if(descriptors.empty())
    {
        mat.release();
        return;
    }

    if(descriptors[0].type()==CV_8UC1){

        const size_t N = descriptors.size();
        int L=getnBytes(descriptors[0]);
        mat.create(N,  L*8, CV_32F);
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
    else{
        assert(descriptors[0].type()==CV_32F);
        const int N = descriptors.size();
        int L=descriptors[0].cols;
        mat.create(N, L, CV_32F);
        for(int i = 0; i < N; ++i)
            memcpy(mat.ptr<float>(i),descriptors[i].ptr<float>(0),sizeof(float)*L);
    }
}



// --------------------------------------------------------------------------

} // namespace DBoW3

