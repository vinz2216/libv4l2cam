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

// Note: This descripter is a sparse approximation to the 50-dimensional
// descriptor described in the paper. It produces almost the same results
// but is faster to compute.
// Many thanks also to Julius Ziegler for increasing the speed of the
// sobel filtering functions.

#ifndef __DESCRIPTOR_H__
#define __DESCRIPTOR_H__

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// Define fixed-width datatypes for Visual Studio projects
#ifndef _MSC_VER
  #include <stdint.h>
#else
  typedef __int8            int8_t;
  typedef __int16           int16_t;
  typedef __int32           int32_t;
  typedef __int64           int64_t;
  typedef unsigned __int8   uint8_t;
  typedef unsigned __int16  uint16_t;
  typedef unsigned __int32  uint32_t;
  typedef unsigned __int64  uint64_t;
#endif

class Descriptor {
  
public:
  
  // constructor creates filters
  Descriptor(uint8_t* I,const int32_t* dims);
  
  // deconstructor releases memory
  ~Descriptor();
  
  // descriptors accessible from outside
  uint8_t* I_desc;
  
private:

  inline uint8_t saturate( int16_t in );
  void filterSobel3(const uint8_t* in, uint8_t* out_v, uint8_t* out_h,const int *dims);
  void createDescriptor(uint8_t* I_sobel3_du,uint8_t* I_sobel3_dv,const int *dims);

};

#endif
