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

#include "elas.h"

#include <math.h>
#include <omp.h>
#include "descriptor.h"
#include "triangle.h"
#include "elimination.h"

using namespace std;

void Elas::process (uint8_t* I1,uint8_t* I2,float* D1,float* D2,const int32_t* dims){
  
  int32_t width  = dims[0];
  int32_t height = dims[1];
  
  // allocate memory for disparity grid
  int32_t grid_width   = (int32_t)ceil((float)width/(float)param.grid_size);
  int32_t grid_height  = (int32_t)ceil((float)height/(float)param.grid_size);
  int32_t grid_dims[3] = {param.disp_max+2,grid_width,grid_height};
  
#ifdef PROFILE
  timer.start("Descriptor");  
#endif
  Descriptor *desc1,*desc2;
  #pragma omp parallel for
  for (int cam = 0; cam <= 1; cam++) {
      if (cam==0) {
          desc1 = new Descriptor(I1,dims);
      }
      else {
          desc2 = new Descriptor(I2,dims);
      }
  }

#ifdef PROFILE
  timer.start("Support Matches");
#endif
  vector<support_pt> p_support = computeSupportMatches(desc1->I_desc,desc2->I_desc,dims);  

#ifdef PROFILE
  timer.start("Delaunay Triangulation");
#endif
  vector<triangle> tri_1;
  vector<triangle> tri_2;
  #pragma omp parallel for
  for (int cam = 0; cam <= 1; cam++) {
      if (cam==0) {
          tri_1 = computeDelaunayTriangulation(p_support,0);
      }
      else {
          tri_2 = computeDelaunayTriangulation(p_support,1);
      }
  }

#ifdef PROFILE
  timer.start("Disparity Planes");
#endif
  #pragma omp parallel for
  for (int cam = 0; cam <= 1; cam++) {
      if (cam==0) {
          computeDisparityPlanes(p_support,tri_1,0);
      }
      else {
          computeDisparityPlanes(p_support,tri_2,1);
      }
  }

#ifdef PROFILE
  timer.start("Grid");
#endif
  int32_t* disparity_grid_1 = (int32_t*)malloc((param.disp_max+2)*grid_height*grid_width*sizeof(int32_t));
  int32_t* disparity_grid_2 = (int32_t*)malloc((param.disp_max+2)*grid_height*grid_width*sizeof(int32_t));
  #pragma omp parallel for
  for (int cam = 0; cam <= 1; cam++) {
      if (cam==0) {
          createGrid(p_support,disparity_grid_1,grid_dims,0);
      }
      else {
          createGrid(p_support,disparity_grid_2,grid_dims,1);
      }
  }

#ifdef PROFILE
  timer.start("Matching");
#endif
  #pragma omp parallel for
  for (int cam = 0; cam <= 1; cam++) {
      if (cam==0) {
          computeDisparity(p_support,tri_1,disparity_grid_1,grid_dims,desc1->I_desc,desc2->I_desc,dims,0,D1);
      }
      else {
          computeDisparity(p_support,tri_2,disparity_grid_2,grid_dims,desc1->I_desc,desc2->I_desc,dims,1,D2);
      }
  }

#ifdef PROFILE
  timer.start("L/R Consistency Check");
#endif
  leftRightConsistencyCheck(D1,D2,dims);
  
#ifdef PROFILE
  timer.start("Remove Small Segments");
#endif
  removeSmallSegments(D1,dims);
  if (!param.postprocess_only_left)
    removeSmallSegments(D2,dims);

#ifdef PROFILE
  timer.start("Gap Interpolation");
#endif
  gapInterpolation(D1,dims);
  if (!param.postprocess_only_left)
    gapInterpolation(D2,dims);
  
  if (param.filter_adaptive_mean) {
#ifdef PROFILE
    timer.start("Adaptive Mean");
#endif
    adaptiveMean(D1,dims);
    if (!param.postprocess_only_left)
      adaptiveMean(D2,dims);
  }
  
  if (param.filter_median) {
#ifdef PROFILE
    timer.start("Median");
#endif
    median(D1,dims);
    if (!param.postprocess_only_left)
      median(D2,dims);
  }

  delete desc1;
  delete desc2;
  
#ifdef PROFILE
  timer.plot();
#endif

  // release disparity grid memory
  free(disparity_grid_1);
  free(disparity_grid_2);
}

void Elas::removeInconsistentSupportPoints (int16_t* D_can,int32_t D_can_width,int32_t D_can_height) {
  
  // for all valid support points do
  for (int32_t u_can=0; u_can<D_can_width; u_can++) {
    for (int32_t v_can=0; v_can<D_can_height; v_can++) {
      int16_t d_can = *(D_can+getAddressOffsetImage(u_can,v_can,D_can_width));
      if (d_can>=0) {
        
        // compute number of other points supporting the current point
        int32_t support = 0;
        for (int32_t u_can_2=u_can-param.incon_window_size; u_can_2<=u_can+param.incon_window_size; u_can_2++) {
          for (int32_t v_can_2=v_can-param.incon_window_size; v_can_2<=v_can+param.incon_window_size; v_can_2++) {
            if (u_can_2>=0 && v_can_2>=0 && u_can_2<D_can_width && v_can_2<D_can_height) {
              int16_t d_can_2 = *(D_can+getAddressOffsetImage(u_can_2,v_can_2,D_can_width));
              if (d_can_2>=0 && abs(d_can-d_can_2)<=param.incon_threshold)
                support++;
            }
          }
        }
        
        // invalidate support point if number of supporting points is too low
        if (support<param.incon_min_support)
          *(D_can+getAddressOffsetImage(u_can,v_can,D_can_width)) = -1;
      }
    }
  }
}

void Elas::removeRedundantSupportPoints(int16_t* D_can,int32_t D_can_width,int32_t D_can_height,
                                        int32_t redun_max_dist, int32_t redun_threshold, bool vertical) {
  
  // parameters
  int32_t redun_dir_u[2] = {0,0};
  int32_t redun_dir_v[2] = {0,0};
  if (vertical) {
    redun_dir_v[0] = -1;
    redun_dir_v[1] = +1;
  } else {
    redun_dir_u[0] = -1;
    redun_dir_u[1] = +1;
  }
    
  // for all valid support points do
  for (int32_t u_can=0; u_can<D_can_width; u_can++) {
    for (int32_t v_can=0; v_can<D_can_height; v_can++) {
      int16_t d_can = *(D_can+getAddressOffsetImage(u_can,v_can,D_can_width));
      if (d_can>=0) {
        
        // check all directions for redundancy
        bool redundant = true;
        for (int32_t i=0; i<2; i++) {
          
          // search for support
          int32_t u_can_2 = u_can;
          int32_t v_can_2 = v_can;
          int16_t d_can_2;
          bool support = false;
          for (int32_t j=0; j<redun_max_dist; j++) {
            u_can_2 += redun_dir_u[i];
            v_can_2 += redun_dir_v[i];
            if (u_can_2<0 || v_can_2<0 || u_can_2>=D_can_width || v_can_2>=D_can_height)
              break;
            d_can_2 = *(D_can+getAddressOffsetImage(u_can_2,v_can_2,D_can_width));
            if (d_can_2>=0 && abs(d_can-d_can_2)<=redun_threshold) {
              support = true;
              break;
            }
          }
          
          // if we have no support => point is not redundant
          if (!support) {
            redundant = false;
            break;
          }
        }
               
        // invalidate support point if it is redundant
        if (redundant)
          *(D_can+getAddressOffsetImage(u_can,v_can,D_can_width)) = -1;
      }
    }
  }
}

void Elas::addCornerSupportPoints(vector<support_pt> &p_support,int32_t width,int32_t height) {
  
  // list of border points
  vector<support_pt> p_border;
  p_border.push_back(support_pt(0,0,0));
  p_border.push_back(support_pt(0,height-1,0));
  p_border.push_back(support_pt(width-1,0,0));
  p_border.push_back(support_pt(width-1,height-1,0));
  
  // find closest d
  for (int32_t i=0; i<(int32_t)p_border.size(); i++) {
    int32_t best_dist = 10000000;
    for (int32_t j=0; j<(int32_t)p_support.size(); j++) {
      int32_t du = p_border[i].u-p_support[j].u;
      int32_t dv = p_border[i].v-p_support[j].v;
      int32_t curr_dist = du*du+dv*dv;
      if (curr_dist<best_dist) {
        best_dist = curr_dist;
        p_border[i].d = p_support[j].d;
      }
    }
  }
  
  // for right image
  p_border.push_back(support_pt(p_border[2].u+p_border[2].d,p_border[2].v,p_border[2].d));
  p_border.push_back(support_pt(p_border[3].u+p_border[3].d,p_border[3].v,p_border[3].d));
  
  // add border points to support points
  for (int32_t i=0; i<(int32_t)p_border.size(); i++)
    p_support.push_back(p_border[i]);
}

inline int16_t Elas::computeMatchingDisparity (const int32_t &u,const int32_t &v,const int32_t &width,const int32_t &height,
                                               uint8_t* I1_desc,uint8_t* I2_desc,const bool &right_image) {
  const int32_t u_step      = 2;
  const int32_t v_step      = 2;
  const int32_t window_size = 2;
  
  int32_t desc_offset_1 = -16*u_step-16*width*v_step;
  int32_t desc_offset_2 = +16*u_step-16*width*v_step;
  int32_t desc_offset_3 = -16*u_step+16*width*v_step;
  int32_t desc_offset_4 = +16*u_step+16*width*v_step;
  
  __m128i xmm1,xmm2,xmm3,xmm4,xmm5,xmm6;

  // check if we are inside the image region
  if (u>=window_size+u_step && u<=width-window_size-1-u_step && v>=window_size+v_step && v<=height-window_size-1-v_step) {
    
    // compute desc and start addresses
    int32_t  line_offset = 16*width*v;
    uint8_t *I1_line_addr,*I2_line_addr;
    if (!right_image) {
      I1_line_addr = I1_desc+line_offset;
      I2_line_addr = I2_desc+line_offset;
    } else {
      I1_line_addr = I2_desc+line_offset;
      I2_line_addr = I1_desc+line_offset;
    }

    // compute I1 block start addresses
    uint8_t* I1_block_addr = I1_line_addr+16*u;
    uint8_t* I2_block_addr;
    
    // we require at least some texture
    int32_t sum = 0;
    for (int32_t i=0; i<16; i++)
      sum += abs((int32_t)(*(I1_block_addr+i))-128);
    if (sum<param.support_texture)
      return -1;
    
    // load first blocks to xmm registers
    xmm1 = _mm_load_si128((__m128i*)(I1_block_addr+desc_offset_1));
    xmm2 = _mm_load_si128((__m128i*)(I1_block_addr+desc_offset_2));
    xmm3 = _mm_load_si128((__m128i*)(I1_block_addr+desc_offset_3));
    xmm4 = _mm_load_si128((__m128i*)(I1_block_addr+desc_offset_4));
    
    // declare match energy for each disparity
    int32_t u_warp;
    
    // best match
    int16_t min_1_E = 32767;
    int16_t min_1_d = -1;
    int16_t min_2_E = 32767;
    int16_t min_2_d = -1;

    // get valid disparity range
    int32_t disp_min_valid = max(param.disp_min,0);
    int32_t disp_max_valid = param.disp_max;
    if (!right_image) disp_max_valid = min(param.disp_max,u-window_size-u_step);
    else              disp_max_valid = min(param.disp_max,width-u-window_size-u_step);
    
    // assume, that we can compute at least 10 disparities for this pixel
    if (disp_max_valid-disp_min_valid<10)
      return -1;

    // for all disparities do
    for (int16_t d=disp_min_valid; d<=disp_max_valid; d++) {

      // warp u coordinate
      if (!right_image) u_warp = u-d;
      else              u_warp = u+d;

      // compute I2 block start addresses
      I2_block_addr = I2_line_addr+16*u_warp;

      // compute match energy at this disparity
      xmm6 = _mm_load_si128((__m128i*)(I2_block_addr+desc_offset_1));
      xmm6 = _mm_sad_epu8(xmm1,xmm6);
      xmm5 = _mm_load_si128((__m128i*)(I2_block_addr+desc_offset_2));
      xmm6 = _mm_add_epi16(_mm_sad_epu8(xmm2,xmm5),xmm6);
      xmm5 = _mm_load_si128((__m128i*)(I2_block_addr+desc_offset_3));
      xmm6 = _mm_add_epi16(_mm_sad_epu8(xmm3,xmm5),xmm6);
      xmm5 = _mm_load_si128((__m128i*)(I2_block_addr+desc_offset_4));
      xmm6 = _mm_add_epi16(_mm_sad_epu8(xmm4,xmm5),xmm6);
      sum  = _mm_extract_epi16(xmm6,0)+_mm_extract_epi16(xmm6,4);

      // best + second best match
      if (sum<min_1_E) {
        min_1_E = sum;
        min_1_d = d;
      } else if (sum<min_2_E) {
        min_2_E = sum;
        min_2_d = d;
      }
    }

    // check if best and second best match are available and if matching ratio is sufficient
    if (min_1_d>=0 && min_2_d>=0 && (float)min_1_E<param.support_threshold*(float)min_2_E)
      return min_1_d;
    else
      return -1;
    
  } else
    return -1;
}

vector<Elas::support_pt> Elas::computeSupportMatches (uint8_t* I1_desc,uint8_t* I2_desc,const int32_t* dims) {
  
  // get image width and height
  int32_t width  = dims[0];
  int32_t height = dims[1];
 
  // create matrix for saving disparity candidates
  int32_t D_can_width  = 0;
  int32_t D_can_height = 0;
  for (int32_t u=0; u<width;  u+=param.candidate_stepsize) D_can_width++;
  for (int32_t v=0; v<height; v+=param.candidate_stepsize) D_can_height++;
  int16_t* D_can = (int16_t*)calloc(D_can_width*D_can_height,sizeof(int16_t));

  // loop variables
  int32_t u,v;
  int16_t d,d2;
   
  // for all point candidates in image 1 do
  // search descriptor type 0
  for (int32_t u_can=0; u_can<D_can_width; u_can++) {
    u = u_can*param.candidate_stepsize;
    for (int32_t v_can=0; v_can<D_can_height; v_can++) {
      v = v_can*param.candidate_stepsize;
      
      // initialize disparity candidate to invalid
      *(D_can+getAddressOffsetImage(u_can,v_can,D_can_width)) = -1;
      
      // find match
      d = computeMatchingDisparity(u,v,width,height,I1_desc,I2_desc,false);
      if (d>=0) {
        
        // find backwards
        d2 = computeMatchingDisparity(u-d,v,width,height,I1_desc,I2_desc,true);
        if (abs(d-d2)<=param.lr_threshold)
          *(D_can+getAddressOffsetImage(u_can,v_can,D_can_width)) = d;
      }
    }
  }
  
  // remove inconsistent support points
  removeInconsistentSupportPoints(D_can,D_can_width,D_can_height);
  
  // remove support points on straight lines, since they are redundant
  // this reduces the number of triangles a little bit and hence speeds up
  // the triangulation process
  removeRedundantSupportPoints(D_can,D_can_width,D_can_height,5,1,true);
  removeRedundantSupportPoints(D_can,D_can_width,D_can_height,5,1,false);
  
  // move support points from image representation into a vector representation
  vector<support_pt> p_support;
  for (int32_t u_can=0; u_can<D_can_width; u_can++)
    for (int32_t v_can=0; v_can<D_can_height; v_can++)
      if (*(D_can+getAddressOffsetImage(u_can,v_can,D_can_width))>=0)
        p_support.push_back(support_pt(u_can*param.candidate_stepsize,
                                       v_can*param.candidate_stepsize,
                                       *(D_can+getAddressOffsetImage(u_can,v_can,D_can_width))));
  
  // if flag is set, add support points in image corners
  // with the same disparity as the nearest neighbor support point
  if (param.add_corners)
    addCornerSupportPoints(p_support,width,height);

  // free memory
  free(D_can);
  
  // return support point vector
  return p_support; 
}

vector<Elas::triangle> Elas::computeDelaunayTriangulation (vector<support_pt> p_support,int32_t right_image) {

  // input/output structure for triangulation
  struct triangulateio in, out;
  int32_t k;

  // inputs
  in.numberofpoints = p_support.size();
  in.pointlist = (float*)malloc(in.numberofpoints*2*sizeof(float));
  k=0;
  if (!right_image) {
    for (int32_t i=0; i<(int32_t)p_support.size(); i++) {
      in.pointlist[k++] = p_support[i].u;
      in.pointlist[k++] = p_support[i].v;
    }
  } else {
    for (int32_t i=0; i<(int32_t)p_support.size(); i++) {
      in.pointlist[k++] = p_support[i].u-p_support[i].d;
      in.pointlist[k++] = p_support[i].v;
    }
  }
  in.numberofpointattributes = 0;
  in.pointattributelist      = NULL;
  in.pointmarkerlist         = NULL;
  in.numberofsegments        = 0;
  in.numberofholes           = 0;
  in.numberofregions         = 0;
  in.regionlist              = NULL;
  
  // outputs
  out.pointlist              = NULL;
  out.pointattributelist     = NULL;
  out.pointmarkerlist        = NULL;
  out.trianglelist           = NULL;
  out.triangleattributelist  = NULL;
  out.neighborlist           = NULL;
  out.segmentlist            = NULL;
  out.segmentmarkerlist      = NULL;
  out.edgelist               = NULL;
  out.edgemarkerlist         = NULL;

  // do triangulation (z=zero-based, n=neighbors, Q=quiet, B=no boundary markers)
  char parameters[] = "zQB";
  triangulate(parameters, &in, &out, NULL);
  
  // put resulting triangles into vector tri
  vector<triangle> tri;
  k=0;
  for (int32_t i=0; i<out.numberoftriangles; i++) {
    tri.push_back(triangle(out.trianglelist[k],out.trianglelist[k+1],out.trianglelist[k+2]));
    k+=3;
  }
  
  // free memory used for triangulation
  free(in.pointlist);
  free(out.pointlist);
  free(out.trianglelist);
  
  // return triangles
  return tri;
}

void Elas::computeDisparityPlanes (vector<support_pt> p_support,vector<triangle> &tri,int32_t right_image) {

  // init Gauss-Jordan eliminator
  Elimination elim(3,1);
  
  // for all triangles do
  for (int32_t i=0; i<(int32_t)tri.size(); i++) {
    
    // get triangle corner indices
    int32_t c1 = tri[i].c1;
    int32_t c2 = tri[i].c2;
    int32_t c3 = tri[i].c3;
    
    // compute matrix A for linear system of left triangle
    elim.A[0][0] = p_support[c1].u;
    elim.A[1][0] = p_support[c2].u;
    elim.A[2][0] = p_support[c3].u;
    elim.A[0][1] = p_support[c1].v; elim.A[0][2] = 1;
    elim.A[1][1] = p_support[c2].v; elim.A[1][2] = 1;
    elim.A[2][1] = p_support[c3].v; elim.A[2][2] = 1;
    
    // compute vector b for linear system (containing the disparities)
    elim.B[0][0] = p_support[c1].d;
    elim.B[1][0] = p_support[c2].d;
    elim.B[2][0] = p_support[c3].d;
    
    // on success of gauss jordan elimination
    if (elim.gaussJordan()) {
      
      // grab results from b
      tri[i].t1a = elim.B[0][0];
      tri[i].t1b = elim.B[1][0];
      tri[i].t1c = elim.B[2][0];
      
    // otherwise: invalid
    } else {
      tri[i].t1a = 0;
      tri[i].t1b = 0;
      tri[i].t1c = 0;
    }

    // compute matrix A for linear system of right triangle
    elim.A[0][0] = p_support[c1].u-p_support[c1].d;
    elim.A[1][0] = p_support[c2].u-p_support[c2].d;
    elim.A[2][0] = p_support[c3].u-p_support[c3].d;
    elim.A[0][1] = p_support[c1].v; elim.A[0][2] = 1;
    elim.A[1][1] = p_support[c2].v; elim.A[1][2] = 1;
    elim.A[2][1] = p_support[c3].v; elim.A[2][2] = 1;
    
    // compute vector b for linear system (containing the disparities)
    elim.B[0][0] = p_support[c1].d;
    elim.B[1][0] = p_support[c2].d;
    elim.B[2][0] = p_support[c3].d;
    
    // on success of gauss jordan elimination
    if (elim.gaussJordan()) {
      
      // grab results from b
      tri[i].t2a = elim.B[0][0];
      tri[i].t2b = elim.B[1][0];
      tri[i].t2c = elim.B[2][0];
      
    // otherwise: invalid
    } else {
      tri[i].t2a = 0;
      tri[i].t2b = 0;
      tri[i].t2c = 0;
    }
    
    
  }  
}

void Elas::createGrid(vector<support_pt> p_support,int32_t* disparity_grid,int32_t* grid_dims,bool right_image) {
  
  // get grid dimensions
  int32_t grid_width  = grid_dims[1];
  int32_t grid_height = grid_dims[2];
  
  // allocate temporary memory
  int32_t* temp1 = (int32_t*)calloc((param.disp_max+1)*grid_height*grid_width,sizeof(int32_t));
  int32_t* temp2 = (int32_t*)calloc((param.disp_max+1)*grid_height*grid_width,sizeof(int32_t));
  
  // for all support points do
  for (int32_t i=0; i<(int32_t)p_support.size(); i++) {
    
    // compute disparity range to fill for this support point
    int32_t x_curr = p_support[i].u;
    int32_t y_curr = p_support[i].v;
    int32_t d_curr = p_support[i].d;
    int32_t d_min  = max(d_curr-1,0);
    int32_t d_max  = min(d_curr+1,param.disp_max);
    
    // fill disparity grid helper
    for (int32_t d=d_min; d<=d_max; d++) {
      int32_t x;
      if (!right_image)
        x = floor((float)(x_curr/param.grid_size));
      else
        x = floor((float)(x_curr-d_curr)/(float)param.grid_size);
      int32_t y = floor((float)y_curr/(float)param.grid_size);
      
      // point may potentially lay outside (corner points)
      if (x>=0 && x<=grid_width &&y>=0 && y<=grid_height) {
        int32_t addr = getAddressOffsetGrid(x,y,d,grid_width,param.disp_max+1);
        *(temp1+addr) = 1;
      }
    }
  }
  
  // diffusion pointers
  const int32_t* tl = temp1 + (0*grid_width+0)*(param.disp_max+1);
  const int32_t* tc = temp1 + (0*grid_width+1)*(param.disp_max+1);
  const int32_t* tr = temp1 + (0*grid_width+2)*(param.disp_max+1);
  const int32_t* cl = temp1 + (1*grid_width+0)*(param.disp_max+1);
  const int32_t* cc = temp1 + (1*grid_width+1)*(param.disp_max+1);
  const int32_t* cr = temp1 + (1*grid_width+2)*(param.disp_max+1);
  const int32_t* bl = temp1 + (2*grid_width+0)*(param.disp_max+1);
  const int32_t* bc = temp1 + (2*grid_width+1)*(param.disp_max+1);
  const int32_t* br = temp1 + (2*grid_width+2)*(param.disp_max+1);
  
  int32_t* result    = temp2 + (1*grid_width+1)*(param.disp_max+1); 
  int32_t* end_input = temp1 + grid_width*grid_height*(param.disp_max+1);
  
  // diffuse temporary grid
  for( ; br != end_input; tl++, tc++, tr++, cl++, cc++, cr++, bl++, bc++, br++, result++ )
    *result = *tl | *tc | *tr | *cl | *cc | *cr | *bl | *bc | *br;
  
  // for all grid positions create disparity grid
  for (int32_t x=0; x<grid_width; x++) {
    for (int32_t y=0; y<grid_height; y++) {
        
      // start with second value (first is reserved for count)
      int32_t curr_ind = 1;
      
      // for all disparities do
      for (int32_t d=0; d<=param.disp_max; d++) {

        // if yes => add this disparity to current cell
        if (*(temp2+getAddressOffsetGrid(x,y,d,grid_width,param.disp_max+1))>0) {
          *(disparity_grid+getAddressOffsetGrid(x,y,curr_ind,grid_width,param.disp_max+2))=d;
          curr_ind++;
        }
      }
      
      // finally set number of indices
      *(disparity_grid+getAddressOffsetGrid(x,y,0,grid_width,param.disp_max+2))=curr_ind-1;
    }
  }
  
  // release temporary memory
  free(temp1);
  free(temp2);
}

inline void Elas::updatePosteriorMinimum(__m128i* I2_block_addr,const int32_t &d,const int32_t &w,
                                         const __m128i &xmm1,__m128i &xmm2,int32_t &val,int32_t &min_val,int32_t &min_d) {
  xmm2 = _mm_load_si128(I2_block_addr);
  xmm2 = _mm_sad_epu8(xmm1,xmm2);
  val  = _mm_extract_epi16(xmm2,0)+_mm_extract_epi16(xmm2,4)+w;
  if (val<min_val) {
    min_val = val;
    min_d   = d;
  }
}

inline void Elas::updatePosteriorMinimum(__m128i* I2_block_addr,const int32_t &d,
                                         const __m128i &xmm1,__m128i &xmm2,int32_t &val,int32_t &min_val,int32_t &min_d) {
  xmm2 = _mm_load_si128(I2_block_addr);
  xmm2 = _mm_sad_epu8(xmm1,xmm2);
  val  = _mm_extract_epi16(xmm2,0)+_mm_extract_epi16(xmm2,4);
  if (val<min_val) {
    min_val = val;
    min_d   = d;
  }
}

inline void Elas::findMatch(int32_t &u,int32_t &v,float &plane_a,float &plane_b,float &plane_c,
                            int32_t* disparity_grid,int32_t *grid_dims,uint8_t* I1_desc,uint8_t* I2_desc,const int32_t *dims,
                            int32_t *P,int32_t &plane_radius,bool &valid,bool &right_image,float* D){
  
  // get image width and height
  int32_t width             = dims[0];
  int32_t height            = dims[1];
  const int32_t disp_num    = grid_dims[0]-1;
  const int32_t window_size = 2;

  // address of disparity we want to compute
  uint32_t d_addr = getAddressOffsetImage(u,v,width);
  
  // check if u is ok
  if (u<window_size || u>=width-2)
    return;

  // compute line start address
  int32_t  line_offset = 16*width*max(min(v,height-3),2);
  uint8_t *I1_line_addr,*I2_line_addr;
  if (!right_image) {
    I1_line_addr = I1_desc+line_offset;
    I2_line_addr = I2_desc+line_offset;
  } else {
    I1_line_addr = I2_desc+line_offset;
    I2_line_addr = I1_desc+line_offset;
  }

  // compute I1 block start address
  uint8_t* I1_block_addr = I1_line_addr+16*u;
  
  // does this patch have enough texture?
  int32_t sum = 0;
  for (int32_t i=0; i<16; i++)
    sum += abs((int32_t)(*(I1_block_addr+i))-128);
  if (sum<param.match_texture) {
    *(D+d_addr) = -1;
    return;
  }

  // compute disparity, min disparity and max disparity of plane prior
  int32_t d_plane     = (int32_t)(plane_a*(float)u+plane_b*(float)v+plane_c);
  int32_t d_plane_min = max(d_plane-plane_radius,0);
  int32_t d_plane_max = min(d_plane+plane_radius,disp_num-1);

  // get grid pointer
  int32_t  grid_x    = (int32_t)floor((float)u/(float)param.grid_size);
  int32_t  grid_y    = (int32_t)floor((float)v/(float)param.grid_size);
  uint32_t grid_addr = getAddressOffsetGrid(grid_x,grid_y,0,grid_dims[1],grid_dims[0]);  
  int32_t  num_grid  = *(disparity_grid+grid_addr);
  int32_t* d_grid    = disparity_grid+grid_addr+1;
  
  // loop variables
  int32_t d_curr, u_warp, val;
  int32_t min_val = 10000;
  int32_t min_d   = -1;
  __m128i xmm1    = _mm_load_si128((__m128i*)I1_block_addr);
  __m128i xmm2;

  // left image
  if (!right_image) { 
    for (int32_t i=0; i<num_grid; i++) {
      d_curr = d_grid[i];
      if (d_curr<d_plane_min || d_curr>d_plane_max) {
        u_warp = u-d_curr;
        if (u_warp<window_size || u_warp>width-window_size-1)
          continue;
        updatePosteriorMinimum((__m128i*)(I2_line_addr+16*u_warp),d_curr,xmm1,xmm2,val,min_val,min_d);
      }
    }
    for (d_curr=d_plane_min; d_curr<=d_plane_max; d_curr++) {
      u_warp = u-d_curr;
      if (u_warp<window_size || u_warp>width-window_size-1)
        continue;
      updatePosteriorMinimum((__m128i*)(I2_line_addr+16*u_warp),d_curr,valid?*(P+abs(d_curr-d_plane)):0,xmm1,xmm2,val,min_val,min_d);
    }
    
  // right image
  } else {
    for (int32_t i=0; i<num_grid; i++) {
      d_curr = d_grid[i];
      if (d_curr<d_plane_min || d_curr>d_plane_max) {
        u_warp = u+d_curr;
        if (u_warp<window_size || u_warp>width-window_size-1)
          continue;
        updatePosteriorMinimum((__m128i*)(I2_line_addr+16*u_warp),d_curr,xmm1,xmm2,val,min_val,min_d);
      }
    }
    for (d_curr=d_plane_min; d_curr<=d_plane_max; d_curr++) {
      u_warp = u+d_curr;
      if (u_warp<window_size || u_warp>width-window_size-1)
        continue;
      updatePosteriorMinimum((__m128i*)(I2_line_addr+16*u_warp),d_curr,valid?*(P+abs(d_curr-d_plane)):0,xmm1,xmm2,val,min_val,min_d);
    }
  }

  // set disparity value
  if (min_d>=0) *(D+d_addr) = min_d; // MAP value (min neg-Log probability)
  else          *(D+d_addr) = -1;    // invalid disparity
}

void Elas::computeDisparity(vector<support_pt> p_support,vector<triangle> tri,int32_t* disparity_grid,int32_t *grid_dims,
                            uint8_t* I1_desc,uint8_t* I2_desc,const int32_t *dims,bool right_image,float* D) {

  // extract image width and height and number of disparities
  const int32_t width     = dims[0];
  const int32_t height    = dims[1];
  const int32_t disp_num  = grid_dims[0]-1;
  
  // descriptor window_size
  //int32_t window_size = 2;
  
  // init disparity image to -1
  for (int32_t i=0; i<width*height; i++)
    *(D+i) = -1;
  
  // pre-compute prior 
  float two_sigma_squared = 2*param.sigma*param.sigma;
  int32_t* P = new int32_t[disp_num];
  for (int32_t delta_d=0; delta_d<disp_num; delta_d++)
    P[delta_d] = (int32_t)((-log(param.gamma+exp(-delta_d*delta_d/two_sigma_squared))+log(param.gamma))/param.beta);
  int32_t plane_radius = (int32_t)max((float)ceil(param.sigma*param.sradius),(float)2.0);

  // loop variables
  int32_t c1, c2, c3;
  float plane_a,plane_b,plane_c,plane_d;
  
  // for all triangles do
  for (uint32_t i=0; i<tri.size(); i++) {
    
    // get plane parameters
    //uint32_t p_i = i*3;
    if (!right_image) {
      plane_a = tri[i].t1a;
      plane_b = tri[i].t1b;
      plane_c = tri[i].t1c;
      plane_d = tri[i].t2a;
    } else {
      plane_a = tri[i].t2a;
      plane_b = tri[i].t2b;
      plane_c = tri[i].t2c;
      plane_d = tri[i].t1a;
    }
    
    // triangle corners
    c1 = tri[i].c1;
    c2 = tri[i].c2;
    c3 = tri[i].c3;

    // sort triangle corners wrt. u (ascending)    
    float tri_u[3];
    if (!right_image) {
      tri_u[0] = p_support[c1].u;
      tri_u[1] = p_support[c2].u;
      tri_u[2] = p_support[c3].u;
    } else {
      tri_u[0] = p_support[c1].u-p_support[c1].d;
      tri_u[1] = p_support[c2].u-p_support[c2].d;
      tri_u[2] = p_support[c3].u-p_support[c3].d;
    }
    float tri_v[3] = {p_support[c1].v,p_support[c2].v,p_support[c3].v};
    
    for (uint32_t j=0; j<3; j++) {
      for (uint32_t k=0; k<j; k++) {
        if (tri_u[k]>tri_u[j]) {
          float tri_u_temp = tri_u[j]; tri_u[j] = tri_u[k]; tri_u[k] = tri_u_temp;
          float tri_v_temp = tri_v[j]; tri_v[j] = tri_v[k]; tri_v[k] = tri_v_temp;
        }
      }
    }
    
    // rename corners
    float A_u = tri_u[0]; float A_v = tri_v[0];
    float B_u = tri_u[1]; float B_v = tri_v[1];
    float C_u = tri_u[2]; float C_v = tri_v[2];
    
    // compute straight lines connecting triangle corners
    float AB_a = 0; float AC_a = 0; float BC_a = 0;
    if ((int32_t)(A_u)!=(int32_t)(B_u)) AB_a = (A_v-B_v)/(A_u-B_u);
    if ((int32_t)(A_u)!=(int32_t)(C_u)) AC_a = (A_v-C_v)/(A_u-C_u);
    if ((int32_t)(B_u)!=(int32_t)(C_u)) BC_a = (B_v-C_v)/(B_u-C_u);
    float AB_b = A_v-AB_a*A_u;
    float AC_b = A_v-AC_a*A_u;
    float BC_b = B_v-BC_a*B_u;
    
    // a plane is only valid if itself and its projection
    // into the other image is not slanted too much
    bool valid = fabs(plane_a)<0.7 && fabs(plane_d)<0.7;
        
    // first part (triangle corner A->B)
    if ((int32_t)(A_u)!=(int32_t)(B_u)) {
      for (int32_t u=max((int32_t)A_u,0); u<=min((int32_t)B_u,width-1); u++){
        int32_t v_1 = (uint32_t)(AC_a*(float)u+AC_b);
        int32_t v_2 = (uint32_t)(AB_a*(float)u+AB_b);
        for (int32_t v=min(v_1,v_2); v<=max(v_1,v_2); v++)
          findMatch(u,v,plane_a,plane_b,plane_c,disparity_grid,grid_dims,
                    I1_desc,I2_desc,dims,P,plane_radius,valid,right_image,D);
      }
    }

    // second part (triangle corner B->C)
    if ((int32_t)(B_u)!=(int32_t)(C_u)) {
      for (int32_t u=max((int32_t)B_u,0); u<=min((int32_t)C_u,width-1); u++){
        int32_t v_1 = (uint32_t)(AC_a*(float)u+AC_b);
        int32_t v_2 = (uint32_t)(BC_a*(float)u+BC_b);
        for (int32_t v=min(v_1,v_2); v<=max(v_1,v_2); v++)
          findMatch(u,v,plane_a,plane_b,plane_c,disparity_grid,grid_dims,
                    I1_desc,I2_desc,dims,P,plane_radius,valid,right_image,D);
      }
    }
    
  }

  free(P);
}

void Elas::leftRightConsistencyCheck(float* D1,float* D2,const int32_t* dims) {
  
  // extract image width and height
  const int32_t width     = dims[0];
  const int32_t height    = dims[1];
  
  // make a copy of both images
  float* D1_copy = (float*)malloc(width*height*sizeof(float));
  float* D2_copy = (float*)malloc(width*height*sizeof(float));
  memcpy(D1_copy,D1,width*height*sizeof(float));
  memcpy(D2_copy,D2,width*height*sizeof(float));

  // loop variables
  uint32_t addr, addr_warp;
  float    u_warp_1,u_warp_2,d1,d2;
  
  // for all image points do
  for (int32_t u=0; u<width; u++) {
    for (int32_t v=0; v<height; v++) {
      
      // compute address (u,v) and disparity value
      addr     = getAddressOffsetImage(u,v,width);
      d1       = *(D1_copy+addr);
      d2       = *(D2_copy+addr);
      u_warp_1 = (float)u-d1;
      u_warp_2 = (float)u+d2;
      
      // check if left disparity is valid
      if (d1>=0 && u_warp_1>=0 && u_warp_1<width) {       
                  
        // compute warped image address
        addr_warp = getAddressOffsetImage((int32_t)u_warp_1,v,width);

        // if check failed
        if (abs(*(D2_copy+addr_warp)-d1)>param.lr_threshold)
          *(D1+addr) = -1;          
        
      // set invalid
      } else
        *(D1+addr) = -1;
      
      // check if right disparity is valid
      if (d2>=0 && u_warp_2>=0 && u_warp_2<width) {       

        // compute warped image address
        addr_warp = getAddressOffsetImage((int32_t)u_warp_2,v,width);

        // if check failed
        if (abs(*(D1_copy+addr_warp)-d2)>param.lr_threshold)
          *(D2+addr) = -1;
        
      // set invalid
      } else
        *(D2+addr) = -1;
      
    }
  }
  
  // release memory
  free(D1_copy);
  free(D2_copy);
}

void Elas::removeSmallSegments (float* I,const int32_t *dims) {
 
  // get image width and height
  const int32_t width  = dims[0];
  const int32_t height = dims[1];
  
  // allocate memory on heap for dynamic programming arrays
  int32_t *I_done     = (int32_t*)calloc(width*height,sizeof(int32_t));
  int32_t *seg_list_u = (int32_t*)calloc(width*height,sizeof(int32_t));
  int32_t *seg_list_v = (int32_t*)calloc(width*height,sizeof(int32_t));
  int32_t seg_list_count;
  int32_t seg_list_curr;
  int32_t u_neighbor[4];
  int32_t v_neighbor[4];
  int32_t u_seg_curr;
  int32_t v_seg_curr;
  
  // declare loop variables
  int32_t addr_start, addr_curr, addr_neighbor;
  
  // for all pixels do
  for (int32_t u=0; u<width; u++) {
    for (int32_t v=0; v<height; v++) {
      
      // get address of first pixel in this segment
      addr_start = getAddressOffsetImage(u,v,width);
                  
      // if this pixel has not already been processed
      if (*(I_done+addr_start)==0) {
                
        // init segment list (add first element
        // and set it to be the next element to check)
        *(seg_list_u+0) = u;
        *(seg_list_v+0) = v;
        seg_list_count  = 1;
        seg_list_curr   = 0;
        
        // add neighboring segments as long as there
        // are none-processed pixels in the seg_list;
        // none-processed means: seg_list_curr<seg_list_count
        while (seg_list_curr<seg_list_count) {
        
          // get current position from seg_list
          u_seg_curr = *(seg_list_u+seg_list_curr);
          v_seg_curr = *(seg_list_v+seg_list_curr);
          
          // get address of current pixel in this segment
          addr_curr = getAddressOffsetImage(u_seg_curr,v_seg_curr,width);
          
          // fill list with neighbor positions
          u_neighbor[0] = u_seg_curr-1; v_neighbor[0] = v_seg_curr;
          u_neighbor[1] = u_seg_curr+1; v_neighbor[1] = v_seg_curr;
          u_neighbor[2] = u_seg_curr;   v_neighbor[2] = v_seg_curr-1;
          u_neighbor[3] = u_seg_curr;   v_neighbor[3] = v_seg_curr+1;
          
          // for all neighbors do
          for (int32_t i=0; i<4; i++) {
            
            // check if neighbor is inside image
            if (u_neighbor[i]>=0 && v_neighbor[i]>=0 && u_neighbor[i]<width && v_neighbor[i]<height) {
              
              // get neighbor pixel address
              addr_neighbor = getAddressOffsetImage(u_neighbor[i],v_neighbor[i],width);
              
              // check if neighbor has not been added yet and if it is valid
              if (*(I_done+addr_neighbor)==0 && *(I+addr_neighbor)>=0) {

                // is the neighbor similar to the current pixel
                // (=belonging to the current segment)
                if (fabs(*(I+addr_curr)-*(I+addr_neighbor))<=param.speckle_sim_threshold) {
                  
                  // add neighbor coordinates to segment list
                  *(seg_list_u+seg_list_count) = u_neighbor[i];
                  *(seg_list_v+seg_list_count) = v_neighbor[i];
                  seg_list_count++;            
                  
                  // set neighbor pixel in I_done to "done"
                  // (otherwise a pixel may be added 2 times to the list, as
                  //  neighbor of one pixel and as neighbor of another pixel)
                  *(I_done+addr_neighbor) = 1;
                }
              }
              
            } 
          }
          
          // set current pixel in seg_list to "done"
          seg_list_curr++;
          
          // set current pixel in I_done to "done"
          *(I_done+addr_curr) = 1;

        } // end: while (seg_list_curr<seg_list_count)
        
        // if segment NOT large enough => invalidate pixels
        if (seg_list_count<param.speckle_size) {
          
          // for all pixels in current segment invalidate pixels
          for (int32_t i=0; i<seg_list_count; i++) {
            addr_curr = getAddressOffsetImage(*(seg_list_u+i),*(seg_list_v+i),width);
            *(I+addr_curr) = -1;
          }
        }
      } // end: if (*(I_done+addr_start)==0)
      
    }
  }
  
  // free memory
  free(I_done);
  free(seg_list_u);
  free(seg_list_v);
}

void Elas::gapInterpolation(float* D,const int32_t* dims) {
  
  // grab image width and height
  int32_t width  = dims[0];
  int32_t height = dims[1];
  
  // discontinuity threshold
  int32_t discon_threshold = 5;
  
  // declare loop variables
  int32_t count,addr,v_first,v_last,u_first,u_last;
  float   d1,d2,d_ipol;
  
  // 1. Row-wise:
  // for each row do
  for (int32_t v=0; v<height; v++) {
    
    // init counter
    count = 0;
    
    // for each element of the row do
    for (int32_t u=0; u<width; u++) {
      
      // get address of this location
      addr = getAddressOffsetImage(u,v,width);
      
      // if disparity valid
      if (*(D+addr)>=0) {
        
        // check if speckle is small enough
        if (count>=1 && count<=param.ipol_gap_width) {
          
          // first and last value for interpolation
          u_first = u-count;
          u_last  = u-1;
          
          // if value in range
          if (u_first>0 && u_last<width-1) {
            
            // compute mean disparity
            d1 = *(D+getAddressOffsetImage(u_first-1,v,width));
            d2 = *(D+getAddressOffsetImage(u_last+1,v,width));
            if (abs(d1-d2)<discon_threshold) d_ipol = (d1+d2)/2;
            else                             d_ipol = min(d1,d2);
            
            // set all values to d_ipol
            for (int32_t u_curr=u_first; u_curr<=u_last; u_curr++)
              *(D+getAddressOffsetImage(u_curr,v,width)) = d_ipol;
          }
          
        }
        
        // reset counter
        count = 0;
      
      // otherwise increment counter
      } else {
        count++;
      }
    }
    
    // extrapolate to the left
    for (int32_t u=0; u<width; u++) {
      
      // get address of this location
      addr = getAddressOffsetImage(u,v,width);
      
      // if disparity valid
      if (*(D+addr)>=0) {
        for (int32_t u2=max(u-param.ipol_gap_width,0); u2<u; u2++)
          *(D+getAddressOffsetImage(u2,v,width)) = *(D+addr);
        break;
      }
    }
    
    // extrapolate to the right
    for (int32_t u=width-1; u>=0; u--) {
      
      // get address of this location
      addr = getAddressOffsetImage(u,v,width);
      
      // if disparity valid
      if (*(D+addr)>=0) {
        for (int32_t u2=u; u2<=min(u+param.ipol_gap_width,width-1); u2++) {
          *(D+getAddressOffsetImage(u2,v,width)) = *(D+addr);
        }
        break;
      }
    }
  }

  // 2. Column-wise:
  // for each column do
  for (int32_t u=0; u<width; u++) {
    
    // init counter
    count = 0;
    
    // for each element of the column do
    for (int32_t v=0; v<height; v++) {
      
      // get address of this location
      addr = getAddressOffsetImage(u,v,width);
      
      // if disparity valid
      if (*(D+addr)>=0) {
        
        // check if speckle is small enough
        if (count>=1 && count<=param.ipol_gap_width) {
          
          // first and last value for interpolation
          v_first = v-count;
          v_last  = v-1;
          
          // if value in range
          if (v_first>0 && v_last<height-1) {
            
            // compute mean disparity
            d1 = *(D+getAddressOffsetImage(u,v_first-1,width));
            d2 = *(D+getAddressOffsetImage(u,v_last+1,width));
            if (abs(d1-d2)<(float)discon_threshold) d_ipol = (d1+d2)/2;
            else                                    d_ipol = min(d1,d2);
            
            // set all values to d_ipol
            for (int32_t v_curr=v_first; v_curr<=v_last; v_curr++)
              *(D+getAddressOffsetImage(u,v_curr,width)) = d_ipol;
          }
          
        }
        
        // reset counter
        count = 0;
      
      // otherwise increment counter
      } else {
        count++;
      }
    }
  }
}

void Elas::adaptiveMean (float* D,const int32_t *dims) {
 
  // get image width and height
  const int32_t width  = dims[0];
  const int32_t height = dims[1];
  
  // allocate temporary memory
  float* D_copy = (float*)malloc(width*height*sizeof(float));
  memcpy(D_copy,D,width*height*sizeof(float));

  // disparity image pointers
  const float* d[9];
  d[0] = D_copy;
  d[1] = D_copy+5;
  d[2] = D_copy+10;
  d[3] = D_copy+   width;
  d[4] = D_copy+5+ width;
  d[5] = D_copy+10+width;
  d[6] = D_copy+   2*width;
  d[7] = D_copy+5+ 2*width;
  d[8] = D_copy+10+2*width;
  float* result = D+5+width;

  // stop here
  const float* end_input = D_copy+width*height;
  
  // loop variables
  float val_ref,val;
  float mean;
  int32_t k;
  
  // for all valid pixels compute adaptive mean in 11x11 window with step size 5
  for ( ; d[8] != end_input; d[0]++,d[1]++,d[2]++,d[3]++,d[4]++,d[5]++,d[6]++,d[7]++,d[8]++,result++) {
    k=0; mean=0; val_ref = *d[4];
    if (val_ref>=0) {
      for (int32_t i=0; i<9; i++) {
        val = *d[i];
        if (val>=0 && fabs(val-val_ref)<3.0) {
          mean += val;
          k++;
        }
      }
      *result = mean/k;
    }
  }
  
  // free memory
  free(D_copy);
}

void Elas::median (float* D,const int32_t *dims) {
 
  // get image width and height
  const int32_t width  = dims[0];
  const int32_t height = dims[1];

  // temporary memory
  float *D_temp = (float*)calloc(width*height,sizeof(float));
  
  int32_t window_size = 3; 
  
  float *vals = new float[window_size*2+1];
  int32_t i,j;
  float temp;
  
  // first step: horizontal median filter
  for (int32_t u=window_size; u<width-window_size; u++) {
    for (int32_t v=window_size; v<height-window_size; v++) {
      if (*(D+getAddressOffsetImage(u,v,width))>=0) {    
        j = 0;
        for (int32_t u2=u-window_size; u2<=u+window_size; u2++) {
          temp = *(D+getAddressOffsetImage(u2,v,width));
          i = j-1;
          while (i>=0 && *(vals+i)>temp) {
            *(vals+i+1) = *(vals+i);
            i--;
          }
          *(vals+i+1) = temp;
          j++;
        }
        *(D_temp+getAddressOffsetImage(u,v,width)) = *(vals+window_size);
      } else {
        *(D_temp+getAddressOffsetImage(u,v,width)) = *(D+getAddressOffsetImage(u,v,width));
      }
        
    }
  }
  
  // second step: vertical median filter
  for (int32_t u=window_size; u<width-window_size; u++) {
    for (int32_t v=window_size; v<height-window_size; v++) {
      if (*(D+getAddressOffsetImage(u,v,width))>=0) {
        j = 0;
        for (int32_t v2=v-window_size; v2<=v+window_size; v2++) {
          temp = *(D_temp+getAddressOffsetImage(u,v2,width));
          i = j-1;
          while (i>=0 && *(vals+i)>temp) {
            *(vals+i+1) = *(vals+i);
            i--;
          }
          *(vals+i+1) = temp;
          j++;
        }
        *(D+getAddressOffsetImage(u,v,width)) = *(vals+window_size);
      } else {
        *(D+getAddressOffsetImage(u,v,width)) = *(D+getAddressOffsetImage(u,v,width));
      }
    }
  }
  
  free(D_temp);
  free(vals);
}