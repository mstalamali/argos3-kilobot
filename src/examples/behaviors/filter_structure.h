#ifndef FILTER_STRUCT_H
#define FILTER_STRUCT_H
#include <math.h>

typedef struct filter_structure
{
    float utility, distance;
    int count_1, count_2, data_switch, im_leaf;
    float *data_1, *data_2, gain;
}filter_a;

void set_filter(filter_a *myfilter,const float Gain,const int Im_leaf)
{
    myfilter->utility=0;
    myfilter->distance=-1;
    myfilter->im_leaf=Im_leaf;
    myfilter->count_1=0;
    myfilter->count_2=0;
    myfilter->data_switch=0;
    myfilter->gain=Gain;
    myfilter->data_1=(float*)malloc(2*sizeof(float));
    myfilter->data_2=(float*)malloc(2*sizeof(float));
    // printf("filter created with gain:%f\n",myfilter->gain);
}

void update_filter(filter_a *myfilter,const float Sensed_utility, const float Ref_distance)
{
    if(myfilter->utility==0 && myfilter->distance==-1)
    {
        myfilter->utility = Sensed_utility;
        myfilter->distance = 1;
    }
    else
    {
        myfilter->utility = myfilter->utility*myfilter->gain + (1-myfilter->gain)*Sensed_utility;
    }
    if(myfilter->im_leaf)
    {
        if(myfilter->data_switch)
        {
            myfilter->count_1++;
            myfilter->data_1[0] = myfilter->data_1[0] + (Sensed_utility-myfilter->data_1[0])/myfilter->count_1;
            if(myfilter->count_1 > 1)
            {
                myfilter->data_1[1] = myfilter->data_1[1]*((myfilter->count_1-2)/(myfilter->count_1-1)) + pow(Sensed_utility-myfilter->data_1[0],2)/myfilter->count_1;
            }
            myfilter->data_switch = 0;
        }
        else
        {
            myfilter->count_2++;
            myfilter->data_2[0] = myfilter->data_2[0] + (Sensed_utility-myfilter->data_2[0])/myfilter->count_2;
            if(myfilter->count_2 > 1)
            {
                myfilter->data_2[1] = myfilter->data_2[1]*((myfilter->count_2-2)/(myfilter->count_2-1)) + pow(Sensed_utility-myfilter->data_2[0],2)/myfilter->count_2;
            }
            myfilter->data_switch = 1;
        }
        if(myfilter->count_1>1 && myfilter->count_2>1)
        {
            myfilter->distance = 1 - sqrt(2*sqrt(myfilter->data_1[1])*sqrt(myfilter->data_2[1])/(.0000000000000001 + myfilter->data_1[1] + myfilter->data_2[1])) * exp(-.25*((pow(myfilter->data_1[0]-myfilter->data_2[0],2))/(.0000000000000001 + myfilter->data_1[1] + myfilter->data_2[1])));
        }
    }
    else
    {
        myfilter->distance=Ref_distance;
    }
}

void erase_filter(filter_a *myfilter)
{
    free(myfilter->data_1);
    free(myfilter->data_2);
}

float get_utility(filter_a *myfilter)
{
    return myfilter->utility;
}

#endif
