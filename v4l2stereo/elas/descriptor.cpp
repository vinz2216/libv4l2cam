/*
Copyright 2010. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libelas.
Authors: Andreas Geiger

libelas is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libelas is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libelas; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/


#include "descriptor.h"
#include <emmintrin.h>

using namespace std;

Descriptor::Descriptor(uint8_t* I,const int32_t* dims) {
  int32_t area         = dims[0]*dims[1];
  I_desc               = (uint8_t*)_mm_malloc(16*area*sizeof(uint8_t),16);
  uint8_t* I_sobel3_du = (uint8_t*)malloc(area*sizeof(uint8_t));
  uint8_t* I_sobel3_dv = (uint8_t*)malloc(area*sizeof(uint8_t));
  filterSobel3(I,I_sobel3_dv,I_sobel3_du,dims);
  createDescriptor(I_sobel3_du,I_sobel3_dv,dims);
  free(I_sobel3_du);
  free(I_sobel3_dv);
}

Descriptor::~Descriptor() {
  _mm_free(I_desc);
}

inline uint8_t Descriptor::saturate( int16_t in ) {
  if( in < 0 )   return 0;
  if( in > 255 ) return 255;
  return in;
}

void Descriptor::filterSobel3( const uint8_t* in, uint8_t* out_v, uint8_t* out_h,const int* dims) {
  
  // get image width and height  
  const int32_t w = dims[0];
  const int32_t h = dims[1];
  
  // init sobel pointers
  const uint8_t* tl = in;
  const uint8_t* tc = in + 1;
  const uint8_t* tr = in + 2;
  const uint8_t* cl = in + w;
  const uint8_t* cc = in + 1 + w;
  const uint8_t* cr = in + 2 + w;
  const uint8_t* bl = in + 2*w;
  const uint8_t* bc = in + 1 + 2*w;
  const uint8_t* br = in + 2 + 2*w;

  // init output pointers
  uint8_t* result_v = out_v + w + 1;
  uint8_t* result_h = out_h + w + 1;

  // stop here
  const uint8_t* end_input = in + w*h;

  // compute sobel response, border pixels are invalid
  for( ; br != end_input; tl++, tc++, tr++, cl++, cc++, cr++, bl++, bc++, br++, result_h++, result_v++ ) {
    int16_t temp_v, temp_h;
    temp_v = -*tl - 2 * *tc - *tr + *bl + 2 * *bc + *br;
    temp_h = -*tl - 2 * *cl - *bl + *tr + 2 * *cr + *br;
    *result_v = saturate( ( temp_v / 4 ) + 128 );
    *result_h = saturate( ( temp_h / 4 ) + 128 );
  }
}

void Descriptor::createDescriptor (uint8_t* I_sobel3_du,uint8_t* I_sobel3_dv,const int *dims) {

  // get image width and height  
  const int32_t w = dims[0];
  const int32_t h = dims[1];

  // init descriptor pointers
  const uint8_t* d00 = I_sobel3_du + 0*w + 2;
  const uint8_t* d01 = I_sobel3_du + 1*w + 0;
  const uint8_t* d02 = I_sobel3_du + 1*w + 2;
  const uint8_t* d03 = I_sobel3_du + 1*w + 4;
  const uint8_t* d04 = I_sobel3_du + 2*w + 1;
  const uint8_t* d05 = I_sobel3_du + 2*w + 2;
  const uint8_t* d06 = I_sobel3_du + 2*w + 2;
  const uint8_t* d07 = I_sobel3_du + 2*w + 3;
  const uint8_t* d08 = I_sobel3_du + 3*w + 0;
  const uint8_t* d09 = I_sobel3_du + 3*w + 2;
  const uint8_t* d10 = I_sobel3_du + 3*w + 4;
  const uint8_t* d11 = I_sobel3_du + 4*w + 2;
  const uint8_t* d12 = I_sobel3_dv + 1*w + 2;
  const uint8_t* d13 = I_sobel3_dv + 2*w + 1;
  const uint8_t* d14 = I_sobel3_dv + 2*w + 3;
  const uint8_t* d15 = I_sobel3_dv + 3*w + 2;

  // init output pointer
  uint8_t* result = I_desc + 16 * (2*w + 2);

  // stop here
  const uint8_t* end_input = I_sobel3_du + w*h - 2;

  // compute descriptor
  for( ; d11 != end_input; d00++,d01++,d02++,d03++,d04++,d05++,d06++,d07++,
                           d08++,d09++,d10++,d11++,d12++,d13++,d14++,d15++ ) {

    *(result++) = *d00;
    *(result++) = *d01;
    *(result++) = *d02;
    *(result++) = *d03;
    *(result++) = *d04;
    *(result++) = *d05;
    *(result++) = *d06;
    *(result++) = *d07;
    *(result++) = *d08;
    *(result++) = *d09;
    *(result++) = *d10;
    *(result++) = *d11;
    *(result++) = *d12;
    *(result++) = *d13;
    *(result++) = *d14;
    *(result++) = *d15;
  }
}

/*
void Descriptor::createDescriptor (uint8_t* I_sobel3_du,uint8_t* I_sobel3_dv,const int *dims) {

  // extract image width and height
  const int width      = dims[0];
  const int height     = dims[1];
  uint8_t *I_desc_curr = I_desc;
  
  uint32_t addr_v0 = 0;
  uint32_t addr_v1,addr_v2,addr_v3,addr_v4;
  
  // create filter strip
  for (int32_t v=0; v<height; v++) {

    addr_v1 = addr_v0+width;
    addr_v2 = addr_v1+width;
    addr_v3 = addr_v2+width;
    addr_v4 = addr_v3+width;
    for (int32_t u=0; u<width; u++) {
      
      if (u<2 || u>=width-2 || v<2 || v>=height-2) {
        I_desc_curr+=16;
        continue;
      }
      
      *(I_desc_curr++) = *(I_sobel3_du+addr_v0+u+2);
      *(I_desc_curr++) = *(I_sobel3_du+addr_v1+u+0);
      *(I_desc_curr++) = *(I_sobel3_du+addr_v1+u+2);
      *(I_desc_curr++) = *(I_sobel3_du+addr_v1+u+4);
      *(I_desc_curr++) = *(I_sobel3_du+addr_v2+u+1);
      *(I_desc_curr++) = *(I_sobel3_du+addr_v2+u+2);
      *(I_desc_curr++) = *(I_sobel3_du+addr_v2+u+2);
      *(I_desc_curr++) = *(I_sobel3_du+addr_v2+u+3);
      *(I_desc_curr++) = *(I_sobel3_du+addr_v3+u+0);
      *(I_desc_curr++) = *(I_sobel3_du+addr_v3+u+2);
      *(I_desc_curr++) = *(I_sobel3_du+addr_v3+u+4);
      *(I_desc_curr++) = *(I_sobel3_du+addr_v4+u+2);
      *(I_desc_curr++) = *(I_sobel3_dv+addr_v1+u+2);
      *(I_desc_curr++) = *(I_sobel3_dv+addr_v2+u+1);
      *(I_desc_curr++) = *(I_sobel3_dv+addr_v2+u+3);
      *(I_desc_curr++) = *(I_sobel3_dv+addr_v3+u+2);
    }
    addr_v0 += width;
  }
}
*/
