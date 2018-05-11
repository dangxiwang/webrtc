// This file is generated. Do not edit.
#ifndef VP8_RTCD_H_
#define VP8_RTCD_H_

#ifdef RTCD_C
#define RTCD_EXTERN
#else
#define RTCD_EXTERN extern
#endif

/*
 * VP8
 */

struct blockd;
struct macroblockd;
struct loop_filter_info;

/* Encoder forward decls */
struct block;
struct macroblock;
struct variance_vtable;
union int_mv;
struct yv12_buffer_config;

#ifdef __cplusplus
extern "C" {
#endif

void vp8_bilinear_predict16x16_c(unsigned char* src,
                                 int src_pitch,
                                 int xofst,
                                 int yofst,
                                 unsigned char* dst,
                                 int dst_pitch);
#define vp8_bilinear_predict16x16 vp8_bilinear_predict16x16_c

void vp8_bilinear_predict4x4_c(unsigned char* src,
                               int src_pitch,
                               int xofst,
                               int yofst,
                               unsigned char* dst,
                               int dst_pitch);
#define vp8_bilinear_predict4x4 vp8_bilinear_predict4x4_c

void vp8_bilinear_predict8x4_c(unsigned char* src,
                               int src_pitch,
                               int xofst,
                               int yofst,
                               unsigned char* dst,
                               int dst_pitch);
#define vp8_bilinear_predict8x4 vp8_bilinear_predict8x4_c

void vp8_bilinear_predict8x8_c(unsigned char* src,
                               int src_pitch,
                               int xofst,
                               int yofst,
                               unsigned char* dst,
                               int dst_pitch);
#define vp8_bilinear_predict8x8 vp8_bilinear_predict8x8_c

void vp8_blend_b_c(unsigned char* y,
                   unsigned char* u,
                   unsigned char* v,
                   int y1,
                   int u1,
                   int v1,
                   int alpha,
                   int stride);
#define vp8_blend_b vp8_blend_b_c

void vp8_blend_mb_inner_c(unsigned char* y,
                          unsigned char* u,
                          unsigned char* v,
                          int y1,
                          int u1,
                          int v1,
                          int alpha,
                          int stride);
#define vp8_blend_mb_inner vp8_blend_mb_inner_c

void vp8_blend_mb_outer_c(unsigned char* y,
                          unsigned char* u,
                          unsigned char* v,
                          int y1,
                          int u1,
                          int v1,
                          int alpha,
                          int stride);
#define vp8_blend_mb_outer vp8_blend_mb_outer_c

int vp8_block_error_c(short* coeff, short* dqcoeff);
#define vp8_block_error vp8_block_error_c

void vp8_copy_mem16x16_c(unsigned char* src,
                         int src_pitch,
                         unsigned char* dst,
                         int dst_pitch);
#define vp8_copy_mem16x16 vp8_copy_mem16x16_c

void vp8_copy_mem8x4_c(unsigned char* src,
                       int src_pitch,
                       unsigned char* dst,
                       int dst_pitch);
#define vp8_copy_mem8x4 vp8_copy_mem8x4_c

void vp8_copy_mem8x8_c(unsigned char* src,
                       int src_pitch,
                       unsigned char* dst,
                       int dst_pitch);
#define vp8_copy_mem8x8 vp8_copy_mem8x8_c

void vp8_dc_only_idct_add_c(short input,
                            unsigned char* pred,
                            int pred_stride,
                            unsigned char* dst,
                            int dst_stride);
#define vp8_dc_only_idct_add vp8_dc_only_idct_add_c

int vp8_denoiser_filter_c(unsigned char* mc_running_avg_y,
                          int mc_avg_y_stride,
                          unsigned char* running_avg_y,
                          int avg_y_stride,
                          unsigned char* sig,
                          int sig_stride,
                          unsigned int motion_magnitude,
                          int increase_denoising);
#define vp8_denoiser_filter vp8_denoiser_filter_c

int vp8_denoiser_filter_uv_c(unsigned char* mc_running_avg,
                             int mc_avg_stride,
                             unsigned char* running_avg,
                             int avg_stride,
                             unsigned char* sig,
                             int sig_stride,
                             unsigned int motion_magnitude,
                             int increase_denoising);
#define vp8_denoiser_filter_uv vp8_denoiser_filter_uv_c

void vp8_dequant_idct_add_c(short* input,
                            short* dq,
                            unsigned char* output,
                            int stride);
#define vp8_dequant_idct_add vp8_dequant_idct_add_c

void vp8_dequant_idct_add_uv_block_c(short* q,
                                     short* dq,
                                     unsigned char* dst_u,
                                     unsigned char* dst_v,
                                     int stride,
                                     char* eobs);
#define vp8_dequant_idct_add_uv_block vp8_dequant_idct_add_uv_block_c

void vp8_dequant_idct_add_y_block_c(short* q,
                                    short* dq,
                                    unsigned char* dst,
                                    int stride,
                                    char* eobs);
#define vp8_dequant_idct_add_y_block vp8_dequant_idct_add_y_block_c

void vp8_dequantize_b_c(struct blockd*, short* dqc);
#define vp8_dequantize_b vp8_dequantize_b_c

int vp8_diamond_search_sad_c(struct macroblock* x,
                             struct block* b,
                             struct blockd* d,
                             union int_mv* ref_mv,
                             union int_mv* best_mv,
                             int search_param,
                             int sad_per_bit,
                             int* num00,
                             struct variance_vtable* fn_ptr,
                             int* mvcost[2],
                             union int_mv* center_mv);
#define vp8_diamond_search_sad vp8_diamond_search_sad_c

void vp8_fast_quantize_b_c(struct block*, struct blockd*);
#define vp8_fast_quantize_b vp8_fast_quantize_b_c

void vp8_filter_by_weight16x16_c(unsigned char* src,
                                 int src_stride,
                                 unsigned char* dst,
                                 int dst_stride,
                                 int src_weight);
#define vp8_filter_by_weight16x16 vp8_filter_by_weight16x16_c

void vp8_filter_by_weight4x4_c(unsigned char* src,
                               int src_stride,
                               unsigned char* dst,
                               int dst_stride,
                               int src_weight);
#define vp8_filter_by_weight4x4 vp8_filter_by_weight4x4_c

void vp8_filter_by_weight8x8_c(unsigned char* src,
                               int src_stride,
                               unsigned char* dst,
                               int dst_stride,
                               int src_weight);
#define vp8_filter_by_weight8x8 vp8_filter_by_weight8x8_c

int vp8_full_search_sad_c(struct macroblock* x,
                          struct block* b,
                          struct blockd* d,
                          union int_mv* ref_mv,
                          int sad_per_bit,
                          int distance,
                          struct variance_vtable* fn_ptr,
                          int* mvcost[2],
                          union int_mv* center_mv);
#define vp8_full_search_sad vp8_full_search_sad_c

void vp8_loop_filter_bh_c(unsigned char* y,
                          unsigned char* u,
                          unsigned char* v,
                          int ystride,
                          int uv_stride,
                          struct loop_filter_info* lfi);
#define vp8_loop_filter_bh vp8_loop_filter_bh_c

void vp8_loop_filter_bv_c(unsigned char* y,
                          unsigned char* u,
                          unsigned char* v,
                          int ystride,
                          int uv_stride,
                          struct loop_filter_info* lfi);
#define vp8_loop_filter_bv vp8_loop_filter_bv_c

void vp8_loop_filter_mbh_c(unsigned char* y,
                           unsigned char* u,
                           unsigned char* v,
                           int ystride,
                           int uv_stride,
                           struct loop_filter_info* lfi);
#define vp8_loop_filter_mbh vp8_loop_filter_mbh_c

void vp8_loop_filter_mbv_c(unsigned char* y,
                           unsigned char* u,
                           unsigned char* v,
                           int ystride,
                           int uv_stride,
                           struct loop_filter_info* lfi);
#define vp8_loop_filter_mbv vp8_loop_filter_mbv_c

void vp8_loop_filter_bhs_c(unsigned char* y,
                           int ystride,
                           const unsigned char* blimit);
#define vp8_loop_filter_simple_bh vp8_loop_filter_bhs_c

void vp8_loop_filter_bvs_c(unsigned char* y,
                           int ystride,
                           const unsigned char* blimit);
#define vp8_loop_filter_simple_bv vp8_loop_filter_bvs_c

void vp8_loop_filter_simple_horizontal_edge_c(unsigned char* y,
                                              int ystride,
                                              const unsigned char* blimit);
#define vp8_loop_filter_simple_mbh vp8_loop_filter_simple_horizontal_edge_c

void vp8_loop_filter_simple_vertical_edge_c(unsigned char* y,
                                            int ystride,
                                            const unsigned char* blimit);
#define vp8_loop_filter_simple_mbv vp8_loop_filter_simple_vertical_edge_c

int vp8_mbblock_error_c(struct macroblock* mb, int dc);
#define vp8_mbblock_error vp8_mbblock_error_c

int vp8_mbuverror_c(struct macroblock* mb);
#define vp8_mbuverror vp8_mbuverror_c

int vp8_refining_search_sad_c(struct macroblock* x,
                              struct block* b,
                              struct blockd* d,
                              union int_mv* ref_mv,
                              int sad_per_bit,
                              int distance,
                              struct variance_vtable* fn_ptr,
                              int* mvcost[2],
                              union int_mv* center_mv);
#define vp8_refining_search_sad vp8_refining_search_sad_c

void vp8_regular_quantize_b_c(struct block*, struct blockd*);
#define vp8_regular_quantize_b vp8_regular_quantize_b_c

void vp8_short_fdct4x4_c(short* input, short* output, int pitch);
#define vp8_short_fdct4x4 vp8_short_fdct4x4_c

void vp8_short_fdct8x4_c(short* input, short* output, int pitch);
#define vp8_short_fdct8x4 vp8_short_fdct8x4_c

void vp8_short_idct4x4llm_c(short* input,
                            unsigned char* pred,
                            int pitch,
                            unsigned char* dst,
                            int dst_stride);
#define vp8_short_idct4x4llm vp8_short_idct4x4llm_c

void vp8_short_inv_walsh4x4_c(short* input, short* output);
#define vp8_short_inv_walsh4x4 vp8_short_inv_walsh4x4_c

void vp8_short_inv_walsh4x4_1_c(short* input, short* output);
#define vp8_short_inv_walsh4x4_1 vp8_short_inv_walsh4x4_1_c

void vp8_short_walsh4x4_c(short* input, short* output, int pitch);
#define vp8_short_walsh4x4 vp8_short_walsh4x4_c

void vp8_sixtap_predict16x16_c(unsigned char* src,
                               int src_pitch,
                               int xofst,
                               int yofst,
                               unsigned char* dst,
                               int dst_pitch);
#define vp8_sixtap_predict16x16 vp8_sixtap_predict16x16_c

void vp8_sixtap_predict4x4_c(unsigned char* src,
                             int src_pitch,
                             int xofst,
                             int yofst,
                             unsigned char* dst,
                             int dst_pitch);
#define vp8_sixtap_predict4x4 vp8_sixtap_predict4x4_c

void vp8_sixtap_predict8x4_c(unsigned char* src,
                             int src_pitch,
                             int xofst,
                             int yofst,
                             unsigned char* dst,
                             int dst_pitch);
#define vp8_sixtap_predict8x4 vp8_sixtap_predict8x4_c

void vp8_sixtap_predict8x8_c(unsigned char* src,
                             int src_pitch,
                             int xofst,
                             int yofst,
                             unsigned char* dst,
                             int dst_pitch);
#define vp8_sixtap_predict8x8 vp8_sixtap_predict8x8_c

void vp8_rtcd(void);

#include "vpx_config.h"

#ifdef RTCD_C
static void setup_rtcd_internal(void) {}
#endif

#ifdef __cplusplus
}  // extern "C"
#endif

#endif
