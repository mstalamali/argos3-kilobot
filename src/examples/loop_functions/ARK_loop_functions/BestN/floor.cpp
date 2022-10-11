// @author Fabio Oddi <fabio.oddi@uniroma1.it>

#include "floor.h"

hierarchicFloor::hierarchicFloor(){}

hierarchicFloor::hierarchicFloor(const std::string Env,const CVector2 Tl,const CVector2 Br,const int SwarmSize,const int Depth,const int Branches,const float Alpha,const float Utility,const float K,const float Noise,const float Offsetx,const float Offsety)
{
    env = Env;
    if(env=="arena")
    {
        swarm_size = SwarmSize;
        depth = Depth;
        branches = Branches;
        max_utility = Utility;
        k = K;
        noise = Noise;
        v_offset.x = Offsetx;
        v_offset.y = Offsety;
    }
    filter_gain = Alpha;

    root = new Node(env,swarm_size,depth,0,filter_gain,max_utility*k,noise);
    // std::cout <<"root id: "<<root->id<<"\n";
    num_nodes++;
    root->set_vertices(Tl,Br);
}

hierarchicFloor::~hierarchicFloor()
{
    delete [] root,leafs,env;
}

void hierarchicFloor::complete_tree()
{
    int deep = depth-1;
    for(int i=0;i<branches;i++)
    {
        Node *temp = new Node(env,swarm_size,deep,num_nodes++,filter_gain,max_utility*k,noise);
        root->add_child(temp);
        temp->set_parent(root);
        if(deep == 0)
        {
            leafs.push_back(temp);
            // std::cout <<"leaf id: "<<temp->id<<"\n";
        }
        // else
        // {
        //     std::cout <<"node id: "<<temp->id<<"\n";
        // }
        this->complete_tree(temp,deep);
    }
}

void hierarchicFloor::complete_tree(Node *ToComplete,const int Deep)
{
    if(Deep>0)
    {
        int deep = Deep-1;
        for(int i=0;i<branches;i++)
        {
            Node *temp = new Node(env,swarm_size,deep,num_nodes++,filter_gain,max_utility*k,noise);
            ToComplete->add_child(temp);
            temp->set_parent(ToComplete);
            if(deep == 0)
            {
                leafs.push_back(temp);
            //     std::cout <<"leaf id: "<<temp->id<<"\n";
            }
            // else
            // {
            //     std::cout <<"node id: "<<temp->id<<"\n";
            // }
            this->complete_tree(temp,deep);
        }
    }
}

void hierarchicFloor::assign_random_MAXutility()
{
    Node *random_leaf = leafs[rand()%leafs.size()];
    random_leaf->update_utility(max_utility);
    random_leaf->set_distance_from_opt(0);
    if(random_leaf->parent!=NULL)
    {
        this->bottom_up_utility_update(random_leaf->parent);
        this->set_distances_from_opt_node(random_leaf,0);
    }
}

void hierarchicFloor::bottom_up_utility_update(Node *Start_node)
{
    if(env=="arena")
    {
        int utility_temp  = 0;
        for(int i=0;i<branches;i++)
        {
            if(Start_node->children[i]->utility > utility_temp)
            {
                utility_temp = Start_node->children[i]->utility;
            }
        }
        Start_node->update_utility(utility_temp);

        if(Start_node->parent!=NULL)
        {
            this->bottom_up_utility_update(Start_node->parent);
        }
    }
}

void hierarchicFloor::set_distances_from_opt_node(Node *Start_node,const int Distance)
{
    if(Start_node->parent!=NULL)
    {
        for(int i=0;i<branches;i++)
        {
            Node *temp = Start_node->parent->children[i];
            if(temp->id!=Start_node->id)
            {
                if(temp->children.size()==branches)
                {
                    std::vector<Node *> temp_leafs=this->get_leafs_from_node(temp);
                    for(int j=0;j<temp_leafs.size();j++)
                    {
                        temp_leafs[j]->set_distance_from_opt(Distance+1);
                    }
                }
                else
                {
                    temp->set_distance_from_opt(Distance+1);
                }
            }
        }
        this->set_distances_from_opt_node(Start_node->parent,Distance+1);
    }
}

void hierarchicFloor::set_vertices_and_kernel()
{
    this->set_vertices();
    for(int i=0;i<this->leafs.size();i++)
    {
        this->leafs[i]->init_kernel(kernel_unit);
    }
}

void hierarchicFloor::set_vertices()
{
    int indx=0;
    for(int i=0;i<12;i++)
    {
        if(pow(4,i)==branches)
        {
            indx = i;
            break;
        }
    }
    this->loop_set_vertices(root,indx,-1);
    root->set_vertices_offset(CVector2(root->get_top_left_angle().GetX(),root->get_top_left_angle().GetY()),CVector2(root->get_bottom_right_angle().GetX(),root->get_bottom_right_angle().GetY()));
    root->set_vertices(CVector2(root->get_top_left_angle().GetX()-v_offset.x,root->get_top_left_angle().GetY()-v_offset.y),CVector2(root->get_bottom_right_angle().GetX()-v_offset.x,root->get_bottom_right_angle().GetY()-v_offset.y));
    for(int i=0;i<branches;i++)
    {
        this->adjust_vertices(root->children[i]);
    }
}

void hierarchicFloor::loop_set_vertices(Node *Start_node,const int Index,const int Ref)
{   
    float w1 = Start_node->get_top_left_angle().GetX();
    float w2 = Start_node->get_bottom_right_angle().GetX();
    float h1 = Start_node->get_top_left_angle().GetY();
    float h2 = Start_node->get_bottom_right_angle().GetY();
    if(Index != 0)
    {
        float dif = (w2-w1)/(pow(2,Index));
        h2=dif + h1;
        w2=dif + w1;
        if(Start_node->children.size()!=0)
        {
            int count=0;
            for(int c=0;c<branches;c++)
            {
                Start_node->children[c]->set_vertices(CVector2(w1,h1),CVector2(w2,h2));
                this->loop_set_vertices(Start_node->children[c],Index,-1);
                w1=w2;
                w2=w2+dif;
                count++;
                if(count == pow(2,Index))
                {
                    count=0;
                    w1=Start_node->get_top_left_angle().GetX();
                    w2=dif + w1;
                    h1=h2;
                    h2=dif + h1;
                }
            }
        }
    }
    else
    {
        if(Ref==-1)
        {
            float dif = (w2-w1)/Start_node->children.size();
            w2=Start_node->get_bottom_right_angle().GetX()/(Start_node->children.size() + w1);
            for(int c=0;c<branches;c++)
            {
                Start_node->children[c]->set_vertices(CVector2(w1,h1),CVector2(w2,h2));
                this->loop_set_vertices(Start_node->children[c],Index,1);
                w1=w1+dif;
                w2=w2+dif;
            }
        }
        else
        {
            if(Ref==1)
            {
                float dif = (h2-h1)/Start_node->children.size();
                h2=h1+dif;
                if(Start_node->children.size()!=0)
                {
                    for(int c=0;c<branches;c++)
                    {
                        Start_node->children[c]->set_vertices(CVector2(w1,h1),CVector2(w2,h2));
                        this->loop_set_vertices(Start_node->children[c],Index,0);
                        h1=h1+dif;
                        h2=h2+dif;
                    }
                }
            }                   
            else
            {
                float dif = (w2-w1)/Start_node->children.size();
                w2=w1+dif;
                if(Start_node->children.size()!=0)
                {
                    for(int c=0;c<branches;c++)
                    {
                        Start_node->children[c]->set_vertices(CVector2(w1,h1),CVector2(w2,h2));
                        this->loop_set_vertices(Start_node->children[c],Index,1);
                        w1=w1+dif;
                        w2=w2+dif;
                    }
                }

            }
        }   
    }
}

void hierarchicFloor::adjust_vertices(Node *Start_node)
{   
    Start_node->set_vertices_offset(CVector2(Start_node->get_top_left_angle().GetX(),Start_node->get_top_left_angle().GetY()),CVector2(Start_node->get_bottom_right_angle().GetX(),Start_node->get_bottom_right_angle().GetY()));
    Start_node->set_vertices(CVector2(Start_node->get_top_left_angle().GetX()-v_offset.x,Start_node->get_top_left_angle().GetY()-v_offset.y),CVector2(Start_node->get_bottom_right_angle().GetX()-v_offset.x,Start_node->get_bottom_right_angle().GetY()-v_offset.y));
    if(Start_node->children.size()!=0)
    {
        for(int i=0;i<branches;i++)
        {
            this->adjust_vertices(Start_node->children[i]);
        }
    }
}

std::vector<Node *> hierarchicFloor::get_leafs_from_node(Node *Start_node)
{
    std::vector<Node *> out;
    if(Start_node->children.size()==branches)
    {
        for(int i=0;i<branches;i++)
        {
            std::vector<Node *> temp = this->get_leafs_from_node(Start_node->children[i]);
            for(int j=0;j<temp.size();j++)
            {
                out.push_back(temp[j]);
            }
        }
    }
    else
    {
        out.push_back(Start_node);
    }
    return out;
}

std::vector<Node *> hierarchicFloor::get_leafs()
{
    return leafs;
}

Node* hierarchicFloor::get_random_leaf()
{
    return leafs[rand()%leafs.size()];
}

Node* hierarchicFloor::get_best_leaf()
{
    for(int i=0;i<leafs.size();i++)
    {
        if(leafs[i]->distance_from_opt==0)
        {
            return leafs[i];
        }
    }
    return NULL;
}

Node* hierarchicFloor::get_node(const int Id)
{
    if(root->id==Id){
        return root;
    }
    Node *temp;
    for(int i=0;i<branches;i++)
    {
        temp = this->get_node(root->children[i],Id);
        if(temp!=NULL)
        {
            return temp;
        }
    }
    return NULL;
}

Node* hierarchicFloor::get_node(Node* Start_node,const int Id)
{
    if(Start_node->id==Id)
    {
        return Start_node;
    }
    if(Start_node->children.size()==branches)
    {
        Node *temp;
        for(int i=0;i<branches;i++)
        {
            temp = this->get_node(Start_node->children[i],Id);
            if(temp!=NULL)
            {
                return temp;
            }
        }
    }
    return NULL;
}

