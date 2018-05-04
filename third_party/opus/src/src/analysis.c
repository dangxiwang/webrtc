/* Copyright (c) 2011 Xiph.Org Foundation
   Written by Jean-Marc Valin */
/*
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

   - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define ANALYSIS_C

#include <stdio.h>

#include "mathops.h"
#include "kiss_fft.h"
#include "celt.h"
#include "modes.h"
#include "arch.h"
#include "quant_bands.h"
#include "analysis.h"
#include "mlp.h"
#include "stack_alloc.h"
#include "float_cast.h"

#ifndef M_PI
#define M_PI 3.141592653
#endif

#ifndef DISABLE_FLOAT_API

static const float dct_table[128] = {
        0.250000f, 0.250000f, 0.250000f, 0.250000f, 0.250000f, 0.250000f, 0.250000f, 0.250000f,
        0.250000f, 0.250000f, 0.250000f, 0.250000f, 0.250000f, 0.250000f, 0.250000f, 0.250000f,
        0.351851f, 0.338330f, 0.311806f, 0.273300f, 0.224292f, 0.166664f, 0.102631f, 0.034654f,
       -0.034654f,-0.102631f,-0.166664f,-0.224292f,-0.273300f,-0.311806f,-0.338330f,-0.351851f,
        0.346760f, 0.293969f, 0.196424f, 0.068975f,-0.068975f,-0.196424f,-0.293969f,-0.346760f,
       -0.346760f,-0.293969f,-0.196424f,-0.068975f, 0.068975f, 0.196424f, 0.293969f, 0.346760f,
        0.338330f, 0.224292f, 0.034654f,-0.166664f,-0.311806f,-0.351851f,-0.273300f,-0.102631f,
        0.102631f, 0.273300f, 0.351851f, 0.311806f, 0.166664f,-0.034654f,-0.224292f,-0.338330f,
        0.326641f, 0.135299f,-0.135299f,-0.326641f,-0.326641f,-0.135299f, 0.135299f, 0.326641f,
        0.326641f, 0.135299f,-0.135299f,-0.326641f,-0.326641f,-0.135299f, 0.135299f, 0.326641f,
        0.311806f, 0.034654f,-0.273300f,-0.338330f,-0.102631f, 0.224292f, 0.351851f, 0.166664f,
       -0.166664f,-0.351851f,-0.224292f, 0.102631f, 0.338330f, 0.273300f,-0.034654f,-0.311806f,
        0.293969f,-0.068975f,-0.346760f,-0.196424f, 0.196424f, 0.346760f, 0.068975f,-0.293969f,
       -0.293969f, 0.068975f, 0.346760f, 0.196424f,-0.196424f,-0.346760f,-0.068975f, 0.293969f,
        0.273300f,-0.166664f,-0.338330f, 0.034654f, 0.351851f, 0.102631f,-0.311806f,-0.224292f,
        0.224292f, 0.311806f,-0.102631f,-0.351851f,-0.034654f, 0.338330f, 0.166664f,-0.273300f,
};

static const float analysis_window[240] = {
      0.000043f, 0.000171f, 0.000385f, 0.000685f, 0.001071f, 0.001541f, 0.002098f, 0.002739f,
      0.003466f, 0.004278f, 0.005174f, 0.006156f, 0.007222f, 0.008373f, 0.009607f, 0.010926f,
      0.012329f, 0.013815f, 0.015385f, 0.017037f, 0.018772f, 0.020590f, 0.022490f, 0.024472f,
      0.026535f, 0.028679f, 0.030904f, 0.033210f, 0.035595f, 0.038060f, 0.040604f, 0.043227f,
      0.045928f, 0.048707f, 0.051564f, 0.054497f, 0.057506f, 0.060591f, 0.063752f, 0.066987f,
      0.070297f, 0.073680f, 0.077136f, 0.080665f, 0.084265f, 0.087937f, 0.091679f, 0.095492f,
      0.099373f, 0.103323f, 0.107342f, 0.111427f, 0.115579f, 0.119797f, 0.124080f, 0.128428f,
      0.132839f, 0.137313f, 0.141849f, 0.146447f, 0.151105f, 0.155823f, 0.160600f, 0.165435f,
      0.170327f, 0.175276f, 0.180280f, 0.185340f, 0.190453f, 0.195619f, 0.200838f, 0.206107f,
      0.211427f, 0.216797f, 0.222215f, 0.227680f, 0.233193f, 0.238751f, 0.244353f, 0.250000f,
      0.255689f, 0.261421f, 0.267193f, 0.273005f, 0.278856f, 0.284744f, 0.290670f, 0.296632f,
      0.302628f, 0.308658f, 0.314721f, 0.320816f, 0.326941f, 0.333097f, 0.339280f, 0.345492f,
      0.351729f, 0.357992f, 0.364280f, 0.370590f, 0.376923f, 0.383277f, 0.389651f, 0.396044f,
      0.402455f, 0.408882f, 0.415325f, 0.421783f, 0.428254f, 0.434737f, 0.441231f, 0.447736f,
      0.454249f, 0.460770f, 0.467298f, 0.473832f, 0.480370f, 0.486912f, 0.493455f, 0.500000f,
      0.506545f, 0.513088f, 0.519630f, 0.526168f, 0.532702f, 0.539230f, 0.545751f, 0.552264f,
      0.558769f, 0.565263f, 0.571746f, 0.578217f, 0.584675f, 0.591118f, 0.597545f, 0.603956f,
      0.610349f, 0.616723f, 0.623077f, 0.629410f, 0.635720f, 0.642008f, 0.648271f, 0.654508f,
      0.660720f, 0.666903f, 0.673059f, 0.679184f, 0.685279f, 0.691342f, 0.697372f, 0.703368f,
      0.709330f, 0.715256f, 0.721144f, 0.726995f, 0.732807f, 0.738579f, 0.744311f, 0.750000f,
      0.755647f, 0.761249f, 0.766807f, 0.772320f, 0.777785f, 0.783203f, 0.788573f, 0.793893f,
      0.799162f, 0.804381f, 0.809547f, 0.814660f, 0.819720f, 0.824724f, 0.829673f, 0.834565f,
      0.839400f, 0.844177f, 0.848895f, 0.853553f, 0.858151f, 0.862687f, 0.867161f, 0.871572f,
      0.875920f, 0.880203f, 0.884421f, 0.888573f, 0.892658f, 0.896677f, 0.900627f, 0.904508f,
      0.908321f, 0.912063f, 0.915735f, 0.919335f, 0.922864f, 0.926320f, 0.929703f, 0.933013f,
      0.936248f, 0.939409f, 0.942494f, 0.945503f, 0.948436f, 0.951293f, 0.954072f, 0.956773f,
      0.959396f, 0.961940f, 0.964405f, 0.966790f, 0.969096f, 0.971321f, 0.973465f, 0.975528f,
      0.977510f, 0.979410f, 0.981228f, 0.982963f, 0.984615f, 0.986185f, 0.987671f, 0.989074f,
      0.990393f, 0.991627f, 0.992778f, 0.993844f, 0.994826f, 0.995722f, 0.996534f, 0.997261f,
      0.997902f, 0.998459f, 0.998929f, 0.999315f, 0.999615f, 0.999829f, 0.999957f, 1.000000f,
};

static const int tbands[NB_TBANDS+1] = {
      4, 8, 12, 16, 20, 24, 28, 32, 40, 48, 56, 64, 80, 96, 112, 136, 160, 192, 240
};

#define NB_TONAL_SKIP_BANDS 9

static opus_val32 silk_resampler_down2_hp(
    opus_val32                  *S,                 /* I/O  State vector [ 2 ]                                          */
    opus_val32                  *out,               /* O    Output signal [ floor(len/2) ]                              */
    const opus_val32            *in,                /* I    Input signal [ len ]                                        */
    int                         inLen               /* I    Number of input samples                                     */
)
{
    int k, len2 = inLen/2;
    opus_val32 in32, out32, out32_hp, Y, X;
    opus_val64 hp_ener = 0;
    /* Internal variables and state are in Q10 format */
    for( k = 0; k < len2; k++ ) {
        /* Convert to Q10 */
        in32 = in[ 2 * k ];

        /* All-pass section for even input sample */
        Y      = SUB32( in32, S[ 0 ] );
        X      = MULT16_32_Q15(QCONST16(0.6074371f, 15), Y);
        out32  = ADD32( S[ 0 ], X );
        S[ 0 ] = ADD32( in32, X );
        out32_hp = out32;
        /* Convert to Q10 */
        in32 = in[ 2 * k + 1 ];

        /* All-pass section for odd input sample, and add to output of previous section */
        Y      = SUB32( in32, S[ 1 ] );
        X      = MULT16_32_Q15(QCONST16(0.15063f, 15), Y);
        out32  = ADD32( out32, S[ 1 ] );
        out32  = ADD32( out32, X );
        S[ 1 ] = ADD32( in32, X );

        Y      = SUB32( -in32, S[ 2 ] );
        X      = MULT16_32_Q15(QCONST16(0.15063f, 15), Y);
        out32_hp  = ADD32( out32_hp, S[ 2 ] );
        out32_hp  = ADD32( out32_hp, X );
        S[ 2 ] = ADD32( -in32, X );

        hp_ener += out32_hp*(opus_val64)out32_hp;
        /* Add, convert back to int16 and store to output */
        out[ k ] = HALF32(out32);
    }
#ifdef FIXED_POINT
    /* len2 can be up to 480, so we shift by 8 more to make it fit. */
    hp_ener = hp_ener >> (2*SIG_SHIFT + 8);
#endif
    return (opus_val32)hp_ener;
}

static opus_val32 downmix_and_resample(downmix_func downmix, const void *_x, opus_val32 *y, opus_val32 S[3], int subframe, int offset, int c1, int c2, int C, int Fs)
{
   VARDECL(opus_val32, tmp);
   opus_val32 scale;
   int j;
   opus_val32 ret = 0;
   SAVE_STACK;

   if (subframe==0) return 0;
   if (Fs == 48000)
   {
      subframe *= 2;
      offset *= 2;
   } else if (Fs == 16000) {
      subframe = subframe*2/3;
      offset = offset*2/3;
   }
   ALLOC(tmp, subframe, opus_val32);

   downmix(_x, tmp, subframe, offset, c1, c2, C);
#ifdef FIXED_POINT
   scale = (1<<SIG_SHIFT);
#else
   scale = 1.f/32768;
#endif
   if (c2==-2)
      scale /= C;
   else if (c2>-1)
      scale /= 2;
   for (j=0;j<subframe;j++)
      tmp[j] *= scale;
   if (Fs == 48000)
   {
      ret = silk_resampler_down2_hp(S, y, tmp, subframe);
   } else if (Fs == 24000) {
      OPUS_COPY(y, tmp, subframe);
   } else if (Fs == 16000) {
      VARDECL(opus_val32, tmp3x);
      ALLOC(tmp3x, 3*subframe, opus_val32);
      /* Don't do this at home! This resampler is horrible and it's only (barely)
         usable for the purpose of the analysis because we don't care about all
         the aliasing between 8 kHz and 12 kHz. */
      for (j=0;j<subframe;j++)
      {
         tmp3x[3*j] = tmp[j];
         tmp3x[3*j+1] = tmp[j];
         tmp3x[3*j+2] = tmp[j];
      }
      silk_resampler_down2_hp(S, y, tmp3x, 3*subframe);
   }
   RESTORE_STACK;
   return ret;
}

void tonality_analysis_init(TonalityAnalysisState *tonal, opus_int32 Fs)
{
  /* Initialize reusable fields. */
  tonal->arch = opus_select_arch();
  tonal->Fs = Fs;
  /* Clear remaining fields. */
  tonality_analysis_reset(tonal);
}

void tonality_analysis_reset(TonalityAnalysisState *tonal)
{
  /* Clear non-reusable fields. */
  char *start = (char*)&tonal->TONALITY_ANALYSIS_RESET_START;
  OPUS_CLEAR(start, sizeof(TonalityAnalysisState) - (start - (char*)tonal));
  tonal->music_confidence = .9f;
  tonal->speech_confidence = .1f;
}

void tonality_get_info(TonalityAnalysisState *tonal, AnalysisInfo *info_out, int len)
{
   int pos;
   int curr_lookahead;
   float psum;
   float tonality_max;
   float tonality_avg;
   int tonality_count;
   int i;

   pos = tonal->read_pos;
   curr_lookahead = tonal->write_pos-tonal->read_pos;
   if (curr_lookahead<0)
      curr_lookahead += DETECT_SIZE;

   /* On long frames, look at the second analysis window rather than the first. */
   if (len > tonal->Fs/50 && pos != tonal->write_pos)
   {
      pos++;
      if (pos==DETECT_SIZE)
         pos=0;
   }
   if (pos == tonal->write_pos)
      pos--;
   if (pos<0)
      pos = DETECT_SIZE-1;
   OPUS_COPY(info_out, &tonal->info[pos], 1);
   tonality_max = tonality_avg = info_out->tonality;
   tonality_count = 1;
   /* If possible, look ahead for a tone to compensate for the delay in the tone detector. */
   for (i=0;i<3;i++)
   {
      pos++;
      if (pos==DETECT_SIZE)
         pos = 0;
      if (pos == tonal->write_pos)
         break;
      tonality_max = MAX32(tonality_max, tonal->info[pos].tonality);
      tonality_avg += tonal->info[pos].tonality;
      tonality_count++;
   }
   info_out->tonality = MAX32(tonality_avg/tonality_count, tonality_max-.2f);
   tonal->read_subframe += len/(tonal->Fs/400);
   while (tonal->read_subframe>=8)
   {
      tonal->read_subframe -= 8;
      tonal->read_pos++;
   }
   if (tonal->read_pos>=DETECT_SIZE)
      tonal->read_pos-=DETECT_SIZE;

   /* The -1 is to compensate for the delay in the features themselves. */
   curr_lookahead = IMAX(curr_lookahead-1, 0);

   psum=0;
   /* Summing the probability of transition patterns that involve music at
      time (DETECT_SIZE-curr_lookahead-1) */
   for (i=0;i<DETECT_SIZE-curr_lookahead;i++)
      psum += tonal->pmusic[i];
   for (;i<DETECT_SIZE;i++)
      psum += tonal->pspeech[i];
   psum = psum*tonal->music_confidence + (1-psum)*tonal->speech_confidence;
   /*printf("%f %f %f %f %f\n", psum, info_out->music_prob, info_out->vad_prob, info_out->activity_probability, info_out->tonality);*/

   info_out->music_prob = psum;
}

static const float std_feature_bias[9] = {
      5.684947f, 3.475288f, 1.770634f, 1.599784f, 3.773215f,
      2.163313f, 1.260756f, 1.116868f, 1.918795f
};

#define LEAKAGE_OFFSET 2.5f
#define LEAKAGE_SLOPE 2.f

#ifdef FIXED_POINT
/* For fixed-point, the input is +/-2^15 shifted up by SIG_SHIFT, so we need to
   compensate for that in the energy. */
#define SCALE_COMPENS (1.f/((opus_int32)1<<(15+SIG_SHIFT)))
#define SCALE_ENER(e) ((SCALE_COMPENS*SCALE_COMPENS)*(e))
#else
#define SCALE_ENER(e) (e)
#endif

static void tonality_analysis(TonalityAnalysisState *tonal, const CELTMode *celt_mode, const void *x, int len, int offset, int c1, int c2, int C, int lsb_depth, downmix_func downmix)
{
    int i, b;
    const kiss_fft_state *kfft;
    VARDECL(kiss_fft_cpx, in);
    VARDECL(kiss_fft_cpx, out);
    int N = 480, N2=240;
    float * OPUS_RESTRICT A = tonal->angle;
    float * OPUS_RESTRICT dA = tonal->d_angle;
    float * OPUS_RESTRICT d2A = tonal->d2_angle;
    VARDECL(float, tonality);
    VARDECL(float, noisiness);
    float band_tonality[NB_TBANDS];
    float logE[NB_TBANDS];
    float BFCC[8];
    float features[25];
    float frame_tonality;
    float max_frame_tonality;
    /*float tw_sum=0;*/
    float frame_noisiness;
    const float pi4 = (float)(M_PI*M_PI*M_PI*M_PI);
    float slope=0;
    float frame_stationarity;
    float relativeE;
    float frame_probs[2];
    float alpha, alphaE, alphaE2;
    float frame_loudness;
    float bandwidth_mask;
    int bandwidth=0;
    float maxE = 0;
    float noise_floor;
    int remaining;
    AnalysisInfo *info;
    float hp_ener;
    float tonality2[240];
    float midE[8];
    float spec_variability=0;
    float band_log2[NB_TBANDS+1];
    float leakage_from[NB_TBANDS+1];
    float leakage_to[NB_TBANDS+1];
    SAVE_STACK;

    alpha = 1.f/IMIN(10, 1+tonal->count);
    alphaE = 1.f/IMIN(25, 1+tonal->count);
    alphaE2 = 1.f/IMIN(500, 1+tonal->count);

    if (tonal->Fs == 48000)
    {
       /* len and offset are now at 24 kHz. */
       len/= 2;
       offset /= 2;
    } else if (tonal->Fs == 16000) {
       len = 3*len/2;
       offset = 3*offset/2;
    }

    if (tonal->count<4) {
       if (tonal->application == OPUS_APPLICATION_VOIP)
          tonal->music_prob = .1f;
       else
          tonal->music_prob = .625f;
    }
    kfft = celt_mode->mdct.kfft[0];
    if (tonal->count==0)
       tonal->mem_fill = 240;
    tonal->hp_ener_accum += (float)downmix_and_resample(downmix, x,
          &tonal->inmem[tonal->mem_fill], tonal->downmix_state,
          IMIN(len, ANALYSIS_BUF_SIZE-tonal->mem_fill), offset, c1, c2, C, tonal->Fs);
    if (tonal->mem_fill+len < ANALYSIS_BUF_SIZE)
    {
       tonal->mem_fill += len;
       /* Don't have enough to update the analysis */
       RESTORE_STACK;
       return;
    }
    hp_ener = tonal->hp_ener_accum;
    info = &tonal->info[tonal->write_pos++];
    if (tonal->write_pos>=DETECT_SIZE)
       tonal->write_pos-=DETECT_SIZE;

    ALLOC(in, 480, kiss_fft_cpx);
    ALLOC(out, 480, kiss_fft_cpx);
    ALLOC(tonality, 240, float);
    ALLOC(noisiness, 240, float);
    for (i=0;i<N2;i++)
    {
       float w = analysis_window[i];
       in[i].r = (kiss_fft_scalar)(w*tonal->inmem[i]);
       in[i].i = (kiss_fft_scalar)(w*tonal->inmem[N2+i]);
       in[N-i-1].r = (kiss_fft_scalar)(w*tonal->inmem[N-i-1]);
       in[N-i-1].i = (kiss_fft_scalar)(w*tonal->inmem[N+N2-i-1]);
    }
    OPUS_MOVE(tonal->inmem, tonal->inmem+ANALYSIS_BUF_SIZE-240, 240);
    remaining = len - (ANALYSIS_BUF_SIZE-tonal->mem_fill);
    tonal->hp_ener_accum = (float)downmix_and_resample(downmix, x,
          &tonal->inmem[240], tonal->downmix_state, remaining,
          offset+ANALYSIS_BUF_SIZE-tonal->mem_fill, c1, c2, C, tonal->Fs);
    tonal->mem_fill = 240 + remaining;
    opus_fft(kfft, in, out, tonal->arch);
#ifndef FIXED_POINT
    /* If there's any NaN on the input, the entire output will be NaN, so we only need to check one value. */
    if (celt_isnan(out[0].r))
    {
       info->valid = 0;
       RESTORE_STACK;
       return;
    }
#endif

    for (i=1;i<N2;i++)
    {
       float X1r, X2r, X1i, X2i;
       float angle, d_angle, d2_angle;
       float angle2, d_angle2, d2_angle2;
       float mod1, mod2, avg_mod;
       X1r = (float)out[i].r+out[N-i].r;
       X1i = (float)out[i].i-out[N-i].i;
       X2r = (float)out[i].i+out[N-i].i;
       X2i = (float)out[N-i].r-out[i].r;

       angle = (float)(.5f/M_PI)*fast_atan2f(X1i, X1r);
       d_angle = angle - A[i];
       d2_angle = d_angle - dA[i];

       angle2 = (float)(.5f/M_PI)*fast_atan2f(X2i, X2r);
       d_angle2 = angle2 - angle;
       d2_angle2 = d_angle2 - d_angle;

       mod1 = d2_angle - (float)float2int(d2_angle);
       noisiness[i] = ABS16(mod1);
       mod1 *= mod1;
       mod1 *= mod1;

       mod2 = d2_angle2 - (float)float2int(d2_angle2);
       noisiness[i] += ABS16(mod2);
       mod2 *= mod2;
       mod2 *= mod2;

       avg_mod = .25f*(d2A[i]+mod1+2*mod2);
       /* This introduces an extra delay of 2 frames in the detection. */
       tonality[i] = 1.f/(1.f+40.f*16.f*pi4*avg_mod)-.015f;
       /* No delay on this detection, but it's less reliable. */
       tonality2[i] = 1.f/(1.f+40.f*16.f*pi4*mod2)-.015f;

       A[i] = angle2;
       dA[i] = d_angle2;
       d2A[i] = mod2;
    }
    for (i=2;i<N2-1;i++)
    {
       float tt = MIN32(tonality2[i], MAX32(tonality2[i-1], tonality2[i+1]));
       tonality[i] = .9f*MAX32(tonality[i], tt-.1f);
    }
    frame_tonality = 0;
    max_frame_tonality = 0;
    /*tw_sum = 0;*/
    info->activity = 0;
    frame_noisiness = 0;
    frame_stationarity = 0;
    if (!tonal->count)
    {
       for (b=0;b<NB_TBANDS;b++)
       {
          tonal->lowE[b] = 1e10;
          tonal->highE[b] = -1e10;
       }
    }
    relativeE = 0;
    frame_loudness = 0;
    /* The energy of the very first band is special because of DC. */
    {
       float E = 0;
       float X1r, X2r;
       X1r = 2*(float)out[0].r;
       X2r = 2*(float)out[0].i;
       E = X1r*X1r + X2r*X2r;
       for (i=1;i<4;i++)
       {
          float binE = out[i].r*(float)out[i].r + out[N-i].r*(float)out[N-i].r
                     + out[i].i*(float)out[i].i + out[N-i].i*(float)out[N-i].i;
          E += binE;
       }
       E = SCALE_ENER(E);
       band_log2[0] = .5f*1.442695f*(float)log(E+1e-10f);
    }
    for (b=0;b<NB_TBANDS;b++)
    {
       float E=0, tE=0, nE=0;
       float L1, L2;
       float stationarity;
       for (i=tbands[b];i<tbands[b+1];i++)
       {
          float binE = out[i].r*(float)out[i].r + out[N-i].r*(float)out[N-i].r
                     + out[i].i*(float)out[i].i + out[N-i].i*(float)out[N-i].i;
          binE = SCALE_ENER(binE);
          E += binE;
          tE += binE*MAX32(0, tonality[i]);
          nE += binE*2.f*(.5f-noisiness[i]);
       }
#ifndef FIXED_POINT
       /* Check for extreme band energies that could cause NaNs later. */
       if (!(E<1e9f) || celt_isnan(E))
       {
          info->valid = 0;
          RESTORE_STACK;
          return;
       }
#endif

       tonal->E[tonal->E_count][b] = E;
       frame_noisiness += nE/(1e-15f+E);

       frame_loudness += (float)sqrt(E+1e-10f);
       logE[b] = (float)log(E+1e-10f);
       band_log2[b+1] = .5f*1.442695f*(float)log(E+1e-10f);
       tonal->logE[tonal->E_count][b] = logE[b];
       if (tonal->count==0)
          tonal->highE[b] = tonal->lowE[b] = logE[b];
       if (tonal->highE[b] > tonal->lowE[b] + 7.5)
       {
          if (tonal->highE[b] - logE[b] > logE[b] - tonal->lowE[b])
             tonal->highE[b] -= .01f;
          else
             tonal->lowE[b] += .01f;
       }
       if (logE[b] > tonal->highE[b])
       {
          tonal->highE[b] = logE[b];
          tonal->lowE[b] = MAX32(tonal->highE[b]-15, tonal->lowE[b]);
       } else if (logE[b] < tonal->lowE[b])
       {
          tonal->lowE[b] = logE[b];
          tonal->highE[b] = MIN32(tonal->lowE[b]+15, tonal->highE[b]);
       }
       relativeE += (logE[b]-tonal->lowE[b])/(1e-15f + (tonal->highE[b]-tonal->lowE[b]));

       L1=L2=0;
       for (i=0;i<NB_FRAMES;i++)
       {
          L1 += (float)sqrt(tonal->E[i][b]);
          L2 += tonal->E[i][b];
       }

       stationarity = MIN16(0.99f,L1/(float)sqrt(1e-15+NB_FRAMES*L2));
       stationarity *= stationarity;
       stationarity *= stationarity;
       frame_stationarity += stationarity;
       /*band_tonality[b] = tE/(1e-15+E)*/;
       band_tonality[b] = MAX16(tE/(1e-15f+E), stationarity*tonal->prev_band_tonality[b]);
#if 0
       if (b>=NB_TONAL_SKIP_BANDS)
       {
          frame_tonality += tweight[b]*band_tonality[b];
          tw_sum += tweight[b];
       }
#else
       frame_tonality += band_tonality[b];
       if (b>=NB_TBANDS-NB_TONAL_SKIP_BANDS)
          frame_tonality -= band_tonality[b-NB_TBANDS+NB_TONAL_SKIP_BANDS];
#endif
       max_frame_tonality = MAX16(max_frame_tonality, (1.f+.03f*(b-NB_TBANDS))*frame_tonality);
       slope += band_tonality[b]*(b-8);
       /*printf("%f %f ", band_tonality[b], stationarity);*/
       tonal->prev_band_tonality[b] = band_tonality[b];
    }

    leakage_from[0] = band_log2[0];
    leakage_to[0] = band_log2[0] - LEAKAGE_OFFSET;
    for (b=1;b<NB_TBANDS+1;b++)
    {
       float leak_slope = LEAKAGE_SLOPE*(tbands[b]-tbands[b-1])/4;
       leakage_from[b] = MIN16(leakage_from[b-1]+leak_slope, band_log2[b]);
       leakage_to[b] = MAX16(leakage_to[b-1]-leak_slope, band_log2[b]-LEAKAGE_OFFSET);
    }
    for (b=NB_TBANDS-2;b>=0;b--)
    {
       float leak_slope = LEAKAGE_SLOPE*(tbands[b+1]-tbands[b])/4;
       leakage_from[b] = MIN16(leakage_from[b+1]+leak_slope, leakage_from[b]);
       leakage_to[b] = MAX16(leakage_to[b+1]-leak_slope, leakage_to[b]);
    }
    celt_assert(NB_TBANDS+1 <= LEAK_BANDS);
    for (b=0;b<NB_TBANDS+1;b++)
    {
       /* leak_boost[] is made up of two terms. The first, based on leakage_to[],
          represents the boost needed to overcome the amount of analysis leakage
          cause in a weaker band b by louder neighbouring bands.
          The second, based on leakage_from[], applies to a loud band b for
          which the quantization noise causes synthesis leakage to the weaker
          neighbouring bands. */
       float boost = MAX16(0, leakage_to[b] - band_log2[b]) +
             MAX16(0, band_log2[b] - (leakage_from[b]+LEAKAGE_OFFSET));
       info->leak_boost[b] = IMIN(255, (int)floor(.5 + 64.f*boost));
    }
    for (;b<LEAK_BANDS;b++) info->leak_boost[b] = 0;

    for (i=0;i<NB_FRAMES;i++)
    {
       int j;
       float mindist = 1e15f;
       for (j=0;j<NB_FRAMES;j++)
       {
          int k;
          float dist=0;
          for (k=0;k<NB_TBANDS;k++)
          {
             float tmp;
             tmp = tonal->logE[i][k] - tonal->logE[j][k];
             dist += tmp*tmp;
          }
          if (j!=i)
             mindist = MIN32(mindist, dist);
       }
       spec_variability += mindist;
    }
    spec_variability = (float)sqrt(spec_variability/NB_FRAMES/NB_TBANDS);
    bandwidth_mask = 0;
    bandwidth = 0;
    maxE = 0;
    noise_floor = 5.7e-4f/(1<<(IMAX(0,lsb_depth-8)));
    noise_floor *= noise_floor;
    for (b=0;b<NB_TBANDS;b++)
    {
       float E=0;
       int band_start, band_end;
       /* Keep a margin of 300 Hz for aliasing */
       band_start = tbands[b];
       band_end = tbands[b+1];
       for (i=band_start;i<band_end;i++)
       {
          float binE = out[i].r*(float)out[i].r + out[N-i].r*(float)out[N-i].r
                     + out[i].i*(float)out[i].i + out[N-i].i*(float)out[N-i].i;
          E += binE;
       }
       E = SCALE_ENER(E);
       maxE = MAX32(maxE, E);
       tonal->meanE[b] = MAX32((1-alphaE2)*tonal->meanE[b], E);
       E = MAX32(E, tonal->meanE[b]);
       /* Use a simple follower with 13 dB/Bark slope for spreading function */
       bandwidth_mask = MAX32(.05f*bandwidth_mask, E);
       /* Consider the band "active" only if all these conditions are met:
          1) less than 10 dB below the simple follower
          2) less than 90 dB below the peak band (maximal masking possible considering
             both the ATH and the loudness-dependent slope of the spreading function)
          3) above the PCM quantization noise floor
          We use b+1 because the first CELT band isn't included in tbands[]
       */
       if (E>.1*bandwidth_mask && E*1e9f > maxE && E > noise_floor*(band_end-band_start))
          bandwidth = b+1;
    }
    /* Special case for the last two bands, for which we don't have spectrum but only
       the energy above 12 kHz. */
    if (tonal->Fs == 48000) {
       float ratio;
       float E = hp_ener*(1.f/(240*240));
       ratio = tonal->prev_bandwidth==20 ? 0.03f : 0.07f;
#ifdef FIXED_POINT
       /* silk_resampler_down2_hp() shifted right by an extra 8 bits. */
       E *= 256.f*(1.f/Q15ONE)*(1.f/Q15ONE);
#endif
       maxE = MAX32(maxE, E);
       tonal->meanE[b] = MAX32((1-alphaE2)*tonal->meanE[b], E);
       E = MAX32(E, tonal->meanE[b]);
       /* Use a simple follower with 13 dB/Bark slope for spreading function */
       bandwidth_mask = MAX32(.05f*bandwidth_mask, E);
       if (E>ratio*bandwidth_mask && E*1e9f > maxE && E > noise_floor*160)
          bandwidth = 20;
       /* This detector is unreliable, so if the bandwidth is close to SWB, assume it's FB. */
       if (bandwidth >= 17)
          bandwidth = 20;
    }
    if (tonal->count<=2)
       bandwidth = 20;
    frame_loudness = 20*(float)log10(frame_loudness);
    tonal->Etracker = MAX32(tonal->Etracker-.003f, frame_loudness);
    tonal->lowECount *= (1-alphaE);
    if (frame_loudness < tonal->Etracker-30)
       tonal->lowECount += alphaE;

    for (i=0;i<8;i++)
    {
       float sum=0;
       for (b=0;b<16;b++)
          sum += dct_table[i*16+b]*logE[b];
       BFCC[i] = sum;
    }
    for (i=0;i<8;i++)
    {
       float sum=0;
       for (b=0;b<16;b++)
          sum += dct_table[i*16+b]*.5f*(tonal->highE[b]+tonal->lowE[b]);
       midE[i] = sum;
    }

    frame_stationarity /= NB_TBANDS;
    relativeE /= NB_TBANDS;
    if (tonal->count<10)
       relativeE = .5f;
    frame_noisiness /= NB_TBANDS;
#if 1
    info->activity = frame_noisiness + (1-frame_noisiness)*relativeE;
#else
    info->activity = .5*(1+frame_noisiness-frame_stationarity);
#endif
    frame_tonality = (max_frame_tonality/(NB_TBANDS-NB_TONAL_SKIP_BANDS));
    frame_tonality = MAX16(frame_tonality, tonal->prev_tonality*.8f);
    tonal->prev_tonality = frame_tonality;

    slope /= 8*8;
    info->tonality_slope = slope;

    tonal->E_count = (tonal->E_count+1)%NB_FRAMES;
    tonal->count = IMIN(tonal->count+1, ANALYSIS_COUNT_MAX);
    info->tonality = frame_tonality;

    for (i=0;i<4;i++)
       features[i] = -0.12299f*(BFCC[i]+tonal->mem[i+24]) + 0.49195f*(tonal->mem[i]+tonal->mem[i+16]) + 0.69693f*tonal->mem[i+8] - 1.4349f*tonal->cmean[i];

    for (i=0;i<4;i++)
       tonal->cmean[i] = (1-alpha)*tonal->cmean[i] + alpha*BFCC[i];

    for (i=0;i<4;i++)
        features[4+i] = 0.63246f*(BFCC[i]-tonal->mem[i+24]) + 0.31623f*(tonal->mem[i]-tonal->mem[i+16]);
    for (i=0;i<3;i++)
        features[8+i] = 0.53452f*(BFCC[i]+tonal->mem[i+24]) - 0.26726f*(tonal->mem[i]+tonal->mem[i+16]) -0.53452f*tonal->mem[i+8];

    if (tonal->count > 5)
    {
       for (i=0;i<9;i++)
          tonal->std[i] = (1-alpha)*tonal->std[i] + alpha*features[i]*features[i];
    }
    for (i=0;i<4;i++)
       features[i] = BFCC[i]-midE[i];

    for (i=0;i<8;i++)
    {
       tonal->mem[i+24] = tonal->mem[i+16];
       tonal->mem[i+16] = tonal->mem[i+8];
       tonal->mem[i+8] = tonal->mem[i];
       tonal->mem[i] = BFCC[i];
    }
    for (i=0;i<9;i++)
       features[11+i] = (float)sqrt(tonal->std[i]) - std_feature_bias[i];
    features[18] = spec_variability - 0.78f;
    features[20] = info->tonality - 0.154723f;
    features[21] = info->activity - 0.724643f;
    features[22] = frame_stationarity - 0.743717f;
    features[23] = info->tonality_slope + 0.069216f;
    features[24] = tonal->lowECount - 0.067930f;

    mlp_process(&net, features, frame_probs);
    frame_probs[0] = .5f*(frame_probs[0]+1);
    /* Curve fitting between the MLP probability and the actual probability */
    /*frame_probs[0] = .01f + 1.21f*frame_probs[0]*frame_probs[0] - .23f*(float)pow(frame_probs[0], 10);*/
    /* Probability of active audio (as opposed to silence) */
    frame_probs[1] = .5f*frame_probs[1]+.5f;
    frame_probs[1] *= frame_probs[1];

    /* Probability of speech or music vs noise */
    info->activity_probability = frame_probs[1];

    /*printf("%f %f\n", frame_probs[0], frame_probs[1]);*/
    {
       /* Probability of state transition */
       float tau;
       /* Represents independence of the MLP probabilities, where
          beta=1 means fully independent. */
       float beta;
       /* Denormalized probability of speech (p0) and music (p1) after update */
       float p0, p1;
       /* Probabilities for "all speech" and "all music" */
       float s0, m0;
       /* Probability sum for renormalisation */
       float psum;
       /* Instantaneous probability of speech and music, with beta pre-applied. */
       float speech0;
       float music0;
       float p, q;

       /* More silence transitions for speech than for music. */
       tau = .001f*tonal->music_prob + .01f*(1-tonal->music_prob);
       p = MAX16(.05f,MIN16(.95f,frame_probs[1]));
       q = MAX16(.05f,MIN16(.95f,tonal->vad_prob));
       beta = .02f+.05f*ABS16(p-q)/(p*(1-q)+q*(1-p));
       /* p0 and p1 are the probabilities of speech and music at this frame
          using only information from previous frame and applying the
          state transition model */
       p0 = (1-tonal->vad_prob)*(1-tau) +    tonal->vad_prob *tau;
       p1 =    tonal->vad_prob *(1-tau) + (1-tonal->vad_prob)*tau;
       /* We apply the current probability with exponent beta to work around
          the fact that the probability estimates aren't independent. */
       p0 *= (float)pow(1-frame_probs[1], beta);
       p1 *= (float)pow(frame_probs[1], beta);
       /* Normalise the probabilities to get the Marokv probability of music. */
       tonal->vad_prob = p1/(p0+p1);
       info->vad_prob = tonal->vad_prob;
       /* Consider that silence has a 50-50 probability of being speech or music. */
       frame_probs[0] = tonal->vad_prob*frame_probs[0] + (1-tonal->vad_prob)*.5f;

       /* One transition every 3 minutes of active audio */
       tau = .0001f;
       /* Adapt beta based on how "unexpected" the new prob is */
       p = MAX16(.05f,MIN16(.95f,frame_probs[0]));
       q = MAX16(.05f,MIN16(.95f,tonal->music_prob));
       beta = .02f+.05f*ABS16(p-q)/(p*(1-q)+q*(1-p));
       /* p0 and p1 are the probabilities of speech and music at this frame
          using only information from previous frame and applying the
          state transition model */
       p0 = (1-tonal->music_prob)*(1-tau) +    tonal->music_prob *tau;
       p1 =    tonal->music_prob *(1-tau) + (1-tonal->music_prob)*tau;
       /* We apply the current probability with exponent beta to work around
          the fact that the probability estimates aren't independent. */
       p0 *= (float)pow(1-frame_probs[0], beta);
       p1 *= (float)pow(frame_probs[0], beta);
       /* Normalise the probabilities to get the Marokv probability of music. */
       tonal->music_prob = p1/(p0+p1);
       info->music_prob = tonal->music_prob;

       /*printf("%f %f %f %f\n", frame_probs[0], frame_probs[1], tonal->music_prob, tonal->vad_prob);*/
       /* This chunk of code deals with delayed decision. */
       psum=1e-20f;
       /* Instantaneous probability of speech and music, with beta pre-applied. */
       speech0 = (float)pow(1-frame_probs[0], beta);
       music0  = (float)pow(frame_probs[0], beta);
       if (tonal->count==1)
       {
          if (tonal->application == OPUS_APPLICATION_VOIP)
             tonal->pmusic[0] = .1f;
          else
             tonal->pmusic[0] = .625f;
          tonal->pspeech[0] = 1-tonal->pmusic[0];
       }
       /* Updated probability of having only speech (s0) or only music (m0),
          before considering the new observation. */
       s0 = tonal->pspeech[0] + tonal->pspeech[1];
       m0 = tonal->pmusic [0] + tonal->pmusic [1];
       /* Updates s0 and m0 with instantaneous probability. */
       tonal->pspeech[0] = s0*(1-tau)*speech0;
       tonal->pmusic [0] = m0*(1-tau)*music0;
       /* Propagate the transition probabilities */
       for (i=1;i<DETECT_SIZE-1;i++)
       {
          tonal->pspeech[i] = tonal->pspeech[i+1]*speech0;
          tonal->pmusic [i] = tonal->pmusic [i+1]*music0;
       }
       /* Probability that the latest frame is speech, when all the previous ones were music. */
       tonal->pspeech[DETECT_SIZE-1] = m0*tau*speech0;
       /* Probability that the latest frame is music, when all the previous ones were speech. */
       tonal->pmusic [DETECT_SIZE-1] = s0*tau*music0;

       /* Renormalise probabilities to 1 */
       for (i=0;i<DETECT_SIZE;i++)
          psum += tonal->pspeech[i] + tonal->pmusic[i];
       psum = 1.f/psum;
       for (i=0;i<DETECT_SIZE;i++)
       {
          tonal->pspeech[i] *= psum;
          tonal->pmusic [i] *= psum;
       }
       psum = tonal->pmusic[0];
       for (i=1;i<DETECT_SIZE;i++)
          psum += tonal->pspeech[i];

       /* Estimate our confidence in the speech/music decisions */
       if (frame_probs[1]>.75)
       {
          if (tonal->music_prob>.9)
          {
             float adapt;
             adapt = 1.f/(++tonal->music_confidence_count);
             tonal->music_confidence_count = IMIN(tonal->music_confidence_count, 500);
             tonal->music_confidence += adapt*MAX16(-.2f,frame_probs[0]-tonal->music_confidence);
          }
          if (tonal->music_prob<.1)
          {
             float adapt;
             adapt = 1.f/(++tonal->speech_confidence_count);
             tonal->speech_confidence_count = IMIN(tonal->speech_confidence_count, 500);
             tonal->speech_confidence += adapt*MIN16(.2f,frame_probs[0]-tonal->speech_confidence);
          }
       }
    }
    tonal->last_music = tonal->music_prob>.5f;
#ifdef MLP_TRAINING
    for (i=0;i<25;i++)
       printf("%f ", features[i]);
    printf("\n");
#endif

    info->bandwidth = bandwidth;
    tonal->prev_bandwidth = bandwidth;
    /*printf("%d %d\n", info->bandwidth, info->opus_bandwidth);*/
    info->noisiness = frame_noisiness;
    info->valid = 1;
    RESTORE_STACK;
}

void run_analysis(TonalityAnalysisState *analysis, const CELTMode *celt_mode, const void *analysis_pcm,
                 int analysis_frame_size, int frame_size, int c1, int c2, int C, opus_int32 Fs,
                 int lsb_depth, downmix_func downmix, AnalysisInfo *analysis_info)
{
   int offset;
   int pcm_len;

   analysis_frame_size -= analysis_frame_size&1;
   if (analysis_pcm != NULL)
   {
      /* Avoid overflow/wrap-around of the analysis buffer */
      analysis_frame_size = IMIN((DETECT_SIZE-5)*Fs/50, analysis_frame_size);

      pcm_len = analysis_frame_size - analysis->analysis_offset;
      offset = analysis->analysis_offset;
      while (pcm_len>0) {
         tonality_analysis(analysis, celt_mode, analysis_pcm, IMIN(Fs/50, pcm_len), offset, c1, c2, C, lsb_depth, downmix);
         offset += Fs/50;
         pcm_len -= Fs/50;
      }
      analysis->analysis_offset = analysis_frame_size;

      analysis->analysis_offset -= frame_size;
   }

   analysis_info->valid = 0;
   tonality_get_info(analysis, analysis_info, frame_size);
}

#endif /* DISABLE_FLOAT_API */
