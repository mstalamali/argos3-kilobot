/**
 * @file <filter.h>
 *
 * @author Fabio Oddi <fabio.oddi@uniroma1.it>
 */

#ifndef FILTER_H
#define FILTER_H

#include <math.h>
#include <vector>

class Filter
{
    private:
        int leaf = 0; // 0 not a leaf, 1 a leaf
        float utility = 0.0;
        float distance = -1.0;
        float alpha = 0.7;
        std::vector<float> data1 = {0.0,0.0}; // {CURRENTmean,CURRENTvariance}
        std::vector<float> data2 = {0.0,0.0};
        int count1 = 0;
        int count2 = 0;
        int data_switch = 0;
        
    public:
        Filter();

        Filter(const float Alpha,const int Leaf);

        void update_filter(const float Sensed_utility,const float Ref_distance);

        friend class Node;
};
#endif