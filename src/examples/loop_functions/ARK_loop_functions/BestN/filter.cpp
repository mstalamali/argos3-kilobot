#include "filter.h"

Filter::Filter(){}

Filter::Filter(const float Alpha,const int Leaf)
{
    alpha = Alpha;
    leaf = Leaf;
}

void Filter::update_filter(const float Sensed_utility,const float Ref_distance)
{
    if(distance == -1.0 && utility == 0.0)
    {
        utility = Sensed_utility;
        distance = 1.0;
    }
    else
    {
        utility = utility*alpha + (1-alpha)*Sensed_utility;
    }
    if(leaf == 1)
    {
        if(data_switch == 0)
        {
            count1 += 1;
            data1[0] = data1[0] + (Sensed_utility-data1[0])/count1;
            if(count1 > 1)
            {
                data1[1] = data1[1]*((count1-2)/(count1-1)) + pow(Sensed_utility-data1[0],2)/count1;
            }
            data_switch = 1;
        }
        else
        {
            count2 += 1;
            data2[0] = data2[0] + (Sensed_utility-data2[0])/count2;
            if(count2 > 1)
            {
                data2[1] = data2[1]*((count2-2)/(count2-1)) + pow(Sensed_utility-data2[0],2)/count2;
            }
            data_switch = 0;
        }
        if(count1 > 1 && count2 >1)
        {
            distance = 1 - sqrt(2*sqrt(data1[1])*sqrt(data2[1])/(.0000000000000001 + data1[1] + data2[1])) * exp(-.25*((pow(data1[0]-data2[0],2))/(.0000000000000001 + data1[1] + data2[1])));
        }
    }    
    else
    {
        distance = Ref_distance;
    }
}

