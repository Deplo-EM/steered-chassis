#pragma once
#include <cstdint>
struct arm_matrix_instance_f32 { uint16_t numRows,numCols; float* pData; };
inline void arm_mat_init_f32(arm_matrix_instance_f32* m,uint16_t r,uint16_t c,float* p) { *m={r,c,p}; }
inline void arm_mat_vec_mult_f32(const arm_matrix_instance_f32* m,const float* v,float* out) {
  for(int r=0;r<m->numRows;++r) { out[r]=0; for(int c=0;c<m->numCols;++c) out[r]+=m->pData[r*m->numCols+c]*v[c]; }
}
