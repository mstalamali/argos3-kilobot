#include "node.h"

Node::Node(){}

Node::Node(const std::string Env,const int SwarmSize,const int Depth,const int Id,const float Alpha,const float Utility,const float Noise)
{
    id = Id;

    if(Env=="arena")
    {
        utility = Utility;
        noise = Noise;
        for(int i=0;i<SwarmSize;i++)
        {
            committed_agents.push_back(0);
        }
    }
    else
    {
        if(Depth==0)
        {
            filter = new Filter(Alpha,1);
        }
        else
        {
            filter = new Filter(Alpha,0);
        }
    }
}

Node::~Node()
{
    delete [] filter,parent,committed_agents,tl_br;
    for(int i = 0; i < kernel_size.GetY(); ++i)
    {
        delete [] kernel[i];
    }
    for(int i = 0; i < children.size(); ++i)
    {
        delete [] children[i];
    }
}

void Node::set_parent(Node *Parent)
{
    parent = Parent;
}

void Node::add_child(Node *Child)
{
    children.push_back(Child);
}

void Node::set_distance_from_opt(const int Distance)
{
    distance_from_opt = Distance;
}

void Node::set_vertices(CVector2 Tl,CVector2 Br)
{
    tl_br.tl = Tl;
    tl_br.br = Br;
}

void Node::set_vertices_offset(CVector2 Tl,CVector2 Br)
{
    tl_br.tl_offset = Tl;
    tl_br.br_offset = Br;
}

void Node::init_kernel(const float Unit)
{   
    std::default_random_engine generator;
    std::normal_distribution<float> distribution(utility,noise);
    int X=0,Y=0;
    float difX = tl_br.br_offset.GetX()-tl_br.tl_offset.GetX();
    float difY = tl_br.br_offset.GetY()-tl_br.tl_offset.GetY();
    for(float y=0;y<difY;y+=Unit)
    {
        Y++;
        if(X==0)
        {
            for(float x=0;x<difX;x+=Unit)
            {
                X++;
            }
        }
    }
    kernel_size = CVector2(X,Y);
    float **Kernel = new float*[Y];
    for(int i = 0; i < Y; ++i)
    {
        Kernel[i] = new float[X];
    }
    for(int i=0;i<Y;i++)
    {
        for(int j=0;j<X;j++)
        {
            Kernel[i][j] = distribution(generator);
        }
    }
    kernel = Kernel;
}

void Node::update_utility(const float Utility)
{
    utility = Utility;
}

void Node::update_noise(const float Noise)
{
    noise = Noise;
}

void Node::update_filter(const float Sensed_utility,const float Ref_distance)
{
    filter->update_filter(Sensed_utility,Ref_distance);
}

int Node::get_distance_from_opt()
{
    return distance_from_opt;
}

int Node::get_id()
{
    return id;
}

CVector2 Node::get_top_left_angle()
{
    return tl_br.tl;
}

CVector2 Node::get_bottom_right_angle()
{
    return tl_br.br;
}

Node* Node::get_parent()
{
    return parent;
}

std::vector<Node *> Node::get_children()
{
    return children;
}

std::vector<Node *> Node::get_siblings()
{
    std::vector<Node *> temp = parent->get_children();
    std::vector<Node *> out;
    for(int i=0;i<temp.size();i++)
    {
        if(temp[i]->id != this->id)
        {
            out.push_back(temp[i]);
        }
    }
    return out;
}

float** Node::get_kernel()
{
    return kernel;
}

CVector2 Node::get_kernel_size()
{
    return kernel_size;
}