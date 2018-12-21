/*
 * digital_filter.h
 *
 */

#ifndef DIGITAL_FILTER_H_
#define DIGITAL_FILTER_H_
#include <stdint.h>

// Functions
void filter_fft(int dir, int m, float *real, float *imag);
void filter_dft(int dir, int len, float *real, float *imag);
void filter_fftshift(float *data, int len);
void filter_hamming(float *data, int len);
void filter_zeroPad(float *data, float *result, int dataLen, int resultLen);
void filter_create_fir_lowpass(float *filter_vector, float f_break, int bits, int use_hamming);
float filter_run_fir_iteration(float *vector, float *filter, int bits, uint32_t offset);
void filter_add_sample(float *buffer, float sample, int bits, uint32_t *offset);

#endif /* DIGITAL_FILTER_H_ */
