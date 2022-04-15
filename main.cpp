/* ----------------------------------------------------------------------
 * Copyright (C) 2010-2012 ARM Limited. All rights reserved.
 *
* $Date:         17. January 2013
* $Revision:     V1.4.0
*
* Project:       CMSIS DSP Library
 * Title:        arm_fir_example_f32.c
 *
 * Description:  Example code demonstrating how an FIR filter can be used
 *               as a low pass filter.
 *
 * Target Processor: Cortex-M4/Cortex-M3
 *
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------- */
/* ----------------------------------------------------------------------
** Include Files
** ------------------------------------------------------------------- */
#include "mbed.h"
#include "arm_math.h"
#include "math_helper.h"
#include "stm32l475e_iot01_gyro.h"
#include <cstdio>
#include <stdio.h>
/* ----------------------------------------------------------------------
** Macro Defines
** ------------------------------------------------------------------- */
#define TEST_LENGTH_SAMPLES  320
/*
This SNR is a bit small. Need to understand why
this example is not giving better SNR ...
*/
#define BLOCK_SIZE            32
#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
/* Must be a multiple of 16 */
#define NUM_TAPS_ARRAY_SIZE              32
#else
#define NUM_TAPS_ARRAY_SIZE              29
#endif
#define NUM_TAPS              29
/* -------------------------------------------------------------------
 * The input signal and reference output (computed with MATLAB)
 * are defined externally in arm_fir_lpf_data.c.
 * ------------------------------------------------------------------- */
/* -------------------------------------------------------------------
 * Declare Test output buffer
 * ------------------------------------------------------------------- */
/* -------------------------------------------------------------------
 * Declare State buffer of size (numTaps + blockSize - 1)
 * ------------------------------------------------------------------- */
#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
static float32_t firStateF32[2 * BLOCK_SIZE + NUM_TAPS - 1];
#else
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
#endif 
/* ----------------------------------------------------------------------
** FIR Coefficients buffer generated using fir1() MATLAB function.
** fir1(28, 6/24)
** ------------------------------------------------------------------- */
#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f, 0.0f,0.0f,0.0f
};
#else
const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
};
#endif
/* ------------------------------------------------------------------
 * Global variables for FIR LPF Example
 * ------------------------------------------------------------------- */
uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = TEST_LENGTH_SAMPLES/BLOCK_SIZE;
/* ----------------------------------------------------------------------
 * FIR LPF Example
 * ------------------------------------------------------------------- */
int main(void)
{
  BSP_GYRO_Init();
  float pDataXYZ[3] = {0};
  uint32_t i, j = 0;
  arm_fir_instance_f32 S1;
  arm_fir_instance_f32 S2;
  arm_fir_instance_f32 S3;
  arm_status status;
  float32_t inputF32_x[320], inputF32_y[320], inputF32_z[320];
  float32_t outputF32_x[320], outputF32_y[320], outputF32_z[320];
  /* Initialize input and output buffer pointers */
  while(j < 320) {
    BSP_GYRO_GetXYZ(pDataXYZ);
    printf("original gyro x: %f\n", pDataXYZ[0]);
    // printf("original x: %f, original y: %f, original z: %f\n", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);
    inputF32_x[j] = (float32_t)pDataXYZ[0];
    inputF32_y[j] = (float32_t)pDataXYZ[1];
    inputF32_z[j] = (float32_t)pDataXYZ[2];
    j++;
  }
  /* Call FIR init function to initialize the instance structure. */
  
//   /* ----------------------------------------------------------------------
//   ** Call the FIR process function for every blockSize samples
//   ** ------------------------------------------------------------------- */
//   for(i=0; i<320; i++) {
//     iinputF32[0][i] = (float32_t)inputF32[0][i];
//     iinputF32[1][i] = (float32_t)inputF32[1][i];
//     iinputF32[2][i] = (float32_t)inputF32[2][i];
//   }
    float32_t *iinputF32_x, *iinputF32_y, *iinputF32_z, *ooutputF32_x, *ooutputF32_y, *ooutputF32_z;
    iinputF32_x = &inputF32_x[0];
    iinputF32_y = &inputF32_y[0];
    iinputF32_z = &inputF32_z[0];
    ooutputF32_x = &outputF32_x[0];
    ooutputF32_y = &outputF32_y[0];
    ooutputF32_z = &outputF32_z[0];
    arm_fir_init_f32(&S1, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);
    for(i=0; i < numBlocks; i++)
    {
        arm_fir_f32(&S1, iinputF32_x + (i * blockSize), ooutputF32_x + (i * blockSize), blockSize);
        // arm_fir_f32(&S2, iinputF32_y + (i * blockSize), ooutputF32_y + (i * blockSize), blockSize);
        // arm_fir_f32(&S3, iinputF32_z + (i * blockSize), ooutputF32_z + (i * blockSize), blockSize);
    }
    // arm_fir_init_f32(&S2, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);
    // for(i=0; i < numBlocks; i++)
    // {
    //     // arm_fir_f32(&S1, iinputF32_x + (i * blockSize), ooutputF32_x + (i * blockSize), blockSize);
    //     arm_fir_f32(&S2, iinputF32_y + (i * blockSize), ooutputF32_y + (i * blockSize), blockSize);
    //     // arm_fir_f32(&S3, iinputF32_z + (i * blockSize), ooutputF32_z + (i * blockSize), blockSize);
    // }
    // arm_fir_init_f32(&S3, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);
    // for(i=0; i < numBlocks; i++)
    // {
    //     // arm_fir_f32(&S1, iinputF32_x + (i * blockSize), ooutputF32_x + (i * blockSize), blockSize);
    //     // arm_fir_f32(&S2, iinputF32_y + (i * blockSize), ooutputF32_y + (i * blockSize), blockSize);
    //     arm_fir_f32(&S3, iinputF32_z + (i * blockSize), ooutputF32_z + (i * blockSize), blockSize);
    // }

    j = 0;
    while(j < blockSize * numBlocks) {
        printf("filtered gyro x: %.5f\n", ooutputF32_x[j]);
        // printf("filtered x: %.5f, filtered y: %.5f, filtered z: %.5f\n", ooutputF32_x[j], outputF32_y[j], outputF32_z[j]);
        j++;
    }
    while(1);
}
