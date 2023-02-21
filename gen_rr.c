#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gen_rr.h"
#include <stdbool.h>
//#include "arm_const_structs.h"

//Peak* findpeaks(float32_t * raw_data, float32_t minPeakDistance, float32_t minPeakProminence, int data_len)
Peak *findpeaks(float *raw_data, float minPeakDistance, float minPeakProminence, int data_len, float maxPeakProminence)
{
    // Implementation of findpeaks function of MATLAB
    // Algorithm based on explanation on MATLAB findpeaks page

    int peak_idx_len = 0; // number of peaks 
    int temp_idx = 0; // temporary index for later use
    //float32_t right_min = 0;
    //float32_t left_min = 0;
    float right_min = 0;
    float left_min = 0;
    //int right_cross = 0;
    //int left_cross = 0;
    //float32_t height = 0;
    float height = 0;
    int temp = 0;
    //float32_t temp2 = 0;
    float temp2 = 0;
    int temp3 = 0;
    //int *peak_idx_loc;
    //int peak_idx_loc_idx = 0;
    int *peak_idx;
    //float32_t *peak_height;
    float *peak_height;
    //float *peak_value;
    //int *peak_idx_idx;
    int *peak_idx_ans;
    //float32_t *peak_height_ans;
    float *peak_height_ans;
    //float32_t *peak_value_ans;
    float *peak_value_ans;
    Peak *answer;
    int l = 0;
    int not_peak_flag = 0;
    

    // Step 1 : find all peaks, and store them into peak_idx array
    //int * peak_idx = malloc(data_len * sizeof(int)); // stores indices of peak data of raw_data(or input data)
    //memset(peak_idx, 0, data_len*sizeof(int));

    //printf("Vir_raw_data[0] : %d\n", (int) raw_data[0] * 100);

    answer = malloc(sizeof(Peak));
    for(int i = 1 ; i < data_len-1 ; i++)
    {
        if(raw_data[i] > raw_data[i-1] && raw_data[i] >= raw_data[i+1])
        {
            if(raw_data[i] == raw_data[i+1])
            {
                for(int j = i+2; j < data_len; j++)
                {
                    if(raw_data[i] < raw_data[j])
                    {
                        not_peak_flag = 1;
                        break;
                    }
                    if(raw_data[i] > raw_data[j])
                    {
                        not_peak_flag = 0;
                        break;
                    }
                }
                if(not_peak_flag == 0){peak_idx_len++;}
            }
            else{peak_idx_len++;}
            not_peak_flag = 0;
        }
    }
    peak_idx = malloc(peak_idx_len * sizeof(int));
    peak_height = malloc(peak_idx_len * sizeof(float));
    //peak_value = malloc(peak_idx_len *sizeof(float));
    memset(peak_idx, 0, peak_idx_len *sizeof(int));
    memset(peak_height, 0, peak_idx_len * sizeof(float));
    //memset(peak_value, 0, peak_idx_len*sizeof(float));
    peak_idx_len = 0;

    for(int i = 1; i < data_len -1 ; i++)
    {
        if(raw_data[i] > raw_data[i-1] && raw_data[i] >= raw_data[i+1])
        {
            if(raw_data[i] == raw_data[i+1])
            {
                for(int j = i+2 ; j < data_len ;j++)
                {
                    if(raw_data[i] < raw_data[j])
                    {
                        not_peak_flag = 1;
                        break;
                    }
                    if(raw_data[i] > raw_data[j])
                    {
                        not_peak_flag = 0;
                        break;
                    }
                }
                if(not_peak_flag == 0)
                {
                    peak_idx[peak_idx_len] = i;
                    peak_idx_len++;
                }
            }
            else{
                peak_idx[peak_idx_len] = i;
                peak_idx_len++;
            }
            not_peak_flag = 0;
        }
    }

    temp3 = peak_idx_len;

    // Step 2 : Prominence check, leave peaks with height larger then minPeakProminence
    for(int i = 0 ; i < temp3 ; i++)
    {
        // Step 2-1 : Find right,left crossing point
        right_min = raw_data[peak_idx[i]];
        left_min = raw_data[peak_idx[i]];
        for(int j = peak_idx[i] ; j < data_len ; j++)
        {
            // keep track of minimum value for later use
            if(right_min > raw_data[j])
            {
                right_min = raw_data[j];
            }

            if(raw_data[peak_idx[i]] < raw_data[j])
            {
                //right_cross = j;
                break;
            }
            //right_cross = data_len - 1; 
        }

        for(int j = peak_idx[i] ; j >= 0 ; j--)
        {
            // keep track of minimum value for later use
            if(left_min > raw_data[j])
            {
                left_min = raw_data[j];
            }

            if(raw_data[peak_idx[i]] < raw_data[j])
            {
                //left_cross = j;
                break;
            }
            //left_cross = 0;
        }

        // Step 2-2 : Determine base point for height calculation, and calculate height
        height = raw_data[peak_idx[i]] - left_min;
        if(height > raw_data[peak_idx[i]]-right_min)
        {
            height = raw_data[peak_idx[i]]-right_min;
        }

        //printf("%dth height value is : %f\n",i, height);

        // Step 2-3 : Check if height is taller than prominence value
        // if this point does not reach minPeakProminence, reduce peak_idx_len, and let the next point overlap this value
        if(height < minPeakProminence || height > maxPeakProminence) 
        {
            peak_idx_len--;
        }
        else
        {
            peak_idx[temp_idx] = peak_idx[i];
            peak_height[temp_idx] = height;
            temp_idx++;
        }        
    }



    // Step 3 : minimum peak distance check
    // Step 3-1 : sort the index from largest to smallest
    
    if(minPeakDistance != 0){
        //peak_idx_idx = malloc(peak_idx_len * sizeof(int));
        //memset(peak_idx_idx, 0, peak_idx_len * sizeof(int));

        for(int i = 1 ; i < peak_idx_len ; i++)
        {
            temp = peak_idx[i];
            temp2 = peak_height[i];
            //temp2 = raw_data[i];
            for(l = i ; l > 0 && (raw_data[peak_idx[l-1]] < raw_data[temp]) ; l--)
            {
                //if(temp2 > peak_height[j]) // large values at the front
                peak_idx[l] = peak_idx[l-1];
                peak_height[l] = peak_height[l-1];
            }
            peak_idx[l] = temp;
            peak_height[l] = temp2;
            
        }
        /*
        for(int i = 0 ; i < peak_idx_len ; i++)
        {
            printf("peak_idx[%d] after sort : %d\n", i, 1700+peak_idx[i]);
            printf("peak_height[%d] after sort : %f\n", i, peak_height[i]);
            printf("peak_value[%d] after sort : %f\n", i, raw_data[peak_idx[i]]);
        }
        */
    
        temp_idx = peak_idx_len;
        
        // Step 3-2 : starting from 1st value, if rest of the values lie inside the range of minPeakDistance, remove it by setting the index -1 or 0
        for(int i = 0 ; i < peak_idx_len-1 ; i++)
        {
            //if(peak_idx[i] == 0) {continue;} // if already removed, skip the idx
            //for(int j = i+1 ; j < peak_idx_len ; j++)
            l = i+1;
            while(l < peak_idx_len)
            {
                //if(peak_idx[j] == 0) {continue;}
                if(abs(peak_idx[i] - peak_idx[l]) <= (int) (minPeakDistance))
                {
                    
                    for(int k = l ; k < peak_idx_len-1 ; k++) // overlap the data
                    {
                        peak_idx[k] = peak_idx[k+1];
                        peak_height[k] = peak_height[k+1];
                    }
                    peak_idx_len--;

                }
                else
                {
                    l++;
                }
            }
        }

        // Step 3-3 : sort in time order again
        //peak_idx_loc = (int *) malloc(peak_idx_len * sizeof(int));
        for(int i = 1; i < peak_idx_len ; i++)
        {
            temp = peak_idx[i];
            temp2 = peak_height[i];
            for(int j = i-1 ; j >= 0 ; j--){
                if(temp < peak_idx[j])
                {
                    peak_idx[j+1] = peak_idx[j];
                    peak_idx[j] = temp;
                    peak_height[j+1] = peak_height[j];
                    peak_height[j] = temp2;
                }
            }
        }
        
        /*
        for(int i = 0 ; i < temp_idx ; i++)
        {
            printf("peak_idx[%d] array after minpeakdistance : %d\n",i,peak_idx[i]);
        }
        */
    }

    // Step 4 : Copy the answer to new area
    // Step 4-1 : allocate new area for answer
    peak_idx_ans = malloc(peak_idx_len * sizeof(int));
    peak_height_ans = malloc(peak_idx_len * sizeof(float));
    peak_value_ans = malloc(peak_idx_len * sizeof(float));
    memset(peak_idx_ans, 0, peak_idx_len *sizeof(int));
    memset(peak_height_ans, 0, peak_idx_len * sizeof(float));
    memset(peak_value_ans, 0, peak_idx_len * sizeof(float));

    for(int i = 0 ; i < peak_idx_len ; i++)
    {
        peak_idx_ans[i] = peak_idx[i];
        peak_height_ans[i] = peak_height[i];
        peak_value_ans[i] = raw_data[peak_idx[i]];
    }

    // Step 4-2 : free unnecessary area
    free(peak_idx);
    free(peak_height);

    // Step 5 : peak_idx_len contains the number of remaining peaks.
    //return peak_idx_len;
    answer->index = peak_idx_ans;
    answer->height = peak_height_ans;
    answer->value = peak_value_ans;
    answer->peak_num = peak_idx_len;

    return answer;
}

// function for generating rr value
// raw_data : input data, rr_period : observation time to calculate rr, length : length of input data, sampling_period : sampling time of input data
// minPeakDistance : peak distance, minPeakProminence : peak prominence for peak detection
/*
double * gen_rr(double * raw_data, double rr_period, int length, double sampling_period, double minPeakDistance, double minPeakProminence)
{
    int data_num_per_rr = 0; // number of data to calculate single rr value
    int rr_num = 0; // number of calculated rr data
    int cnt = 0;
    double * rr;
    double * rr_temp;

    data_num_per_rr = rr_period/sampling_period; // numbers of sample data used to calculate rr
    rr_num = length/data_num_per_rr; // number of calculated rrs per given raw data
    
    rr = malloc(rr_num * sizeof(double)); //
    memset(rr,0,rr_num*sizeof(double));

    rr_temp = malloc(data_num_per_rr * sizeof(double));
    memset(rr_temp, 0, data_num_per_rr * sizeof(double));
    for(int i = 0 ; i < rr_num ; i++)
    {
        // load data from raw to rr_temp
        for(int j = 0 ; j < data_num_per_rr ; j++)
        {
            rr_temp[j] = raw_data[data_num_per_rr * i + j];
        }

        rr[i] = findpeaks(rr_temp, minPeakDistance, minPeakProminence, data_num_per_rr) * 1.0 / rr_period;
    }

    free(rr_temp);
    return rr;
}
*/