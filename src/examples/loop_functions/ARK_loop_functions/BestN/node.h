/**
 * @file <node.h>
 *
 * @author Fabio Oddi <fabio.oddi@uniroma1.it>
 */
#ifndef NODE_H
#define NODE_H
#include <string>
#include <random>
#include "filter.h"
#include <argos3/core/utility/math/vector2.h>

namespace argos
{
    class CSpace;
}

using namespace argos;

class Node
{
    private:
        int id;
        int distance_from_opt=-1;
        float utility=-1;
        float noise=-1;
        Filter *filter=NULL;
        Node *parent=NULL;
        std::vector<Node *> children;
        std::vector<int> committed_agents;
        float **kernel=NULL;
        CVector2 kernel_size=CVector2();
        struct Vertices
        {
            CVector2 tl=CVector2(),br=CVector2(),tl_offset=CVector2(),br_offset=CVector2();
        }tl_br;

    public:
        Node();

        Node(const std::string Env,const int SwarmSize,const int Depth,const int Id,const float Alpha,const float Utility,const float Noise);

        ~Node();
                
        void set_parent(Node *Parent);

        void add_child(Node *Child);
        
        void set_distance_from_opt(const int Distance);

        void set_vertices(CVector2 Tl,CVector2 Br);
        
        void set_vertices_offset(CVector2 Tl,CVector2 Br);

        void init_kernel(const float Unit);

        void update_utility(const float Utility);
        
        void update_noise(const float Noise);

        void update_filter(const float Sensed_utility,const float Ref_distance);

        int get_distance_from_opt();

        int get_id();
        
        CVector2 get_top_left_angle();
        
        CVector2 get_bottom_right_angle();
        
        Node* get_parent();
        
        std::vector<Node *> get_children();
        
        std::vector<Node *> get_siblings();
        
        float** get_kernel();
        
        CVector2 get_kernel_size();

        friend class hierarchicFloor;
};
#endif