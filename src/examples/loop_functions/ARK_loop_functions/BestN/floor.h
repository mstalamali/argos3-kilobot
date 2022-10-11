/**
 * @file <floor.h>
 *
 * @author Fabio Oddi <fabio.oddi@uniroma1.it>
 */

#ifndef HIERARCHICFLOOR_H
#define HIERARCHICFLOOR_H
#include "node.h"

class hierarchicFloor
{
    private:
        Node *root=NULL;
        struct XYoffset
        {
            float x=-1,y=-1;
        }v_offset;
        float kernel_unit=0.01;
        std::vector<Node *> leafs;
        int num_nodes=0;
        int depth=-1;
        int branches=-1;
        int swarm_size=-1;
        std::string env="arena";
        float filter_gain=0.7;
        float max_utility=-1;
        float k=-1;
        float noise=-1;

    public:
        hierarchicFloor();

        hierarchicFloor(const std::string Env,const CVector2 Tl,const CVector2 Br,const int SwarmSize,const int Depth,const int Branches,const float Alpha,const float Utility,const float K,const float Noise,const float Offsetx,const float Offset);

        ~hierarchicFloor();
        
        void complete_tree();

        void complete_tree(Node *ToComplete,const int Deep);

        void assign_random_MAXutility();

        void bottom_up_utility_update(Node *Start_node);

        void set_distances_from_opt_node(Node *Start_node,const int Distance);

        void set_vertices();
        
        void loop_set_vertices(Node *Start_node,const int Index,const int Ref);

        void set_vertices_and_kernel();

        void adjust_vertices(Node *Start_node);

        std::vector<Node *> get_leafs_from_node(Node *Start_node);
        
        std::vector<Node *> get_leafs();

        Node* get_random_leaf();

        Node* get_best_leaf();

        Node* get_node(const int Id);

        Node* get_node(Node *Start_node,const int Id);

};
#endif