#ifndef __GEN_RR_H__
#define __GEB_RR_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct _Peak{
    int * index;
    //float32_t * height;
    //float32_t * value;
    float *height;
    float *value;
    int peak_num;
} Peak;
// Peak* findpeaks(float32_t * raw_data, float minPeakDistance, float32_t minPeakProminence, int data_len);
Peak *findpeaks(float *raw_data, float minPeakDistance, float minPeakProminence, int data_len, float maxPeakProminence);
//double * gen_rr(double * raw_data, double rr_period, int length, double sampling_period, double minPeakDistance, double minPeakProminence);

#endif
