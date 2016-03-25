#include "Filter.h"
#include "math.h"
#include "global.h"
void IIR_set_cutoff_freq(IIRFilter *filter, float cutoff_freq, float smpl_freq)
{
	float fr = smpl_freq/cutoff_freq;
    float ohm = tan(PI/fr);
    float c = 1.0f+2.0f*cos(PI/4.0f)*ohm + ohm*ohm;
    filter->b0 = ohm*ohm/c;
    filter->b1 = 2.0f*filter->b0;
    filter->b2 = filter->b0;
    filter->a1 = 2.0f*(ohm*ohm-1.0f)/c;
    filter->a2 = (1.0f-2.0f*cos(PI/4.0f)*ohm+ohm*ohm)/c;
}
//for 25Hz
//a1=-1.561, a2=0.6414,
//b0=0.0201, b1=0.0402, b2=0.0201, 
//fr=20,ohm=0.1584
float IIR_apply(IIRFilter *filter, float sample)
{
	float delay_element_0 = sample - filter->delay_element_1 * filter->a1 - filter->delay_element_2 * filter->a2;
    float output = delay_element_0 * filter->b0 + filter->delay_element_1 * filter->b1 + filter->delay_element_2 * filter->b2;   
    filter->delay_element_2 = filter->delay_element_1;
    filter->delay_element_1 = delay_element_0;
    return output;
}
float IIR_reset(IIRFilter *filter, float sample) 
{
	float dval = sample / (filter->b0 + filter->b1 + filter->b2);
    filter->delay_element_1 = dval;
    filter->delay_element_2 = dval;
    return IIR_apply(filter, sample);
}

