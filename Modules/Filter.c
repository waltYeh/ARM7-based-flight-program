#include "Filter.h"
#include "math.h"
#include "../Main/global.h"
void IIR_set_cutoff_freq(IIRFilter *f, float cutoff_freq, float smpl_freq)
{
	float fr = smpl_freq/cutoff_freq;
    float ohm = tan(PI/fr);
    float c = 1.0f+2.0f*cos(PI/4.0f)*ohm + ohm*ohm;
    f->b0 = ohm*ohm/c;
    f->b1 = 2.0f*f->b0;
    f->b2 = f->b0;
    f->a1 = 2.0f*(ohm*ohm-1.0f)/c;
    f->a2 = (1.0f-2.0f*cos(PI/4.0f)*ohm+ohm*ohm)/c;
}
//for 25Hz
//a1=-1.561, a2=0.6414,
//b0=0.0201, b1=0.0402, b2=0.0201, 
//fr=20,ohm=0.1584
float IIR_apply(IIRFilter *f, float sample)
{
	float delay_element_0 = sample - f->delay_element_1 * f->a1 - f->delay_element_2 * f->a2;
    float output = delay_element_0 * f->b0 + f->delay_element_1 * f->b1 + f->delay_element_2 * f->b2;   
    f->delay_element_2 = f->delay_element_1;
    f->delay_element_1 = delay_element_0;
    return output;
}
float IIR_reset(IIRFilter *f, float sample) 
{
	float dval = sample / (f->b0 + f->b1 + f->b2);
    f->delay_element_1 = dval;
    f->delay_element_2 = dval;
    return IIR_apply(f, sample);
}
void FIR_set_cutoff_freq(FIRFilter *filter, float cutoff_freq, float smpl_freq)
{

}
float FIR_apply(FIRFilter *filter, float sample)
{
    return 1;
}
float FIR_reset(FIRFilter *filter, float sample) 
{
    return FIR_apply(filter, sample);
}
