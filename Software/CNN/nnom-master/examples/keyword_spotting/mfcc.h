/*
 * Copyright (C) 2018 Arm Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Modified by Jianjia Ma for C implementation
 *
 */

#ifndef __MFCC_H__
#define __MFCC_H__


// in main.c define "PLATFORM_ARM" before including 'mfcc.h' to use ARM optimized FFT
#ifdef PLATFORM_ARM
#include "arm_math.h"
#define MFCC_PLATFORM_ARM
#endif

#include <stdint.h>
#include "string.h"
#include <math.h>

#define SAMP_FREQ 16000
#define MEL_LOW_FREQ 20
#define MEL_HIGH_FREQ 4000

#define M_2PI 6.283185307179586476925286766559005

typedef struct _mfcc_t{
    int num_mfcc_features;
	int num_features_offset;
	int num_fbank;
    int frame_len;
    int frame_len_padded;
	int is_append_energy;
	float preempha;
    float * frame;
    float * buffer;
    float * mel_energies;
    float * window_func;
    int32_t * fbank_filter_first;
    int32_t * fbank_filter_last;
    float ** mel_fbank;
    float * dct_matrix;
	#ifdef PLATFORM_ARM
		arm_rfft_fast_instance_f32* rfft;
	#else
		float* fft_buffer;
	#endif
} mfcc_t;

static inline float InverseMelScale(float mel_freq) {
  return 700.0f * (pow(10,(mel_freq / 2595.0f)) - 1.0f);
}

static inline float MelScale(float freq) {
  return 2595.0f * log10(1.0f + freq / 700.0f);
}

float * create_dct_matrix(int32_t input_length, int32_t coefficient_count);
float ** create_mel_fbank(mfcc_t* mfcc);

mfcc_t *mfcc_create(int num_mfcc_features, int feature_offset, int num_fbank, int frame_len, float preempha, int is_append_energy);
void mfcc_delete(mfcc_t* mfcc);

void mfcc_compute(mfcc_t *mfcc, const int16_t * audio_data, float* mfcc_out);

#endif



