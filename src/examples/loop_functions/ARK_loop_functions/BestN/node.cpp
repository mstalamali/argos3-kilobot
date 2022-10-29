// @author Fabio Oddi <fabio.oddi@uniroma1.it>

#include "node.h"

Node::Node(){}

Node::Node(const int SwarmSize,const int Depth,const int Id,const float Utility,const float Noise)
{
    id = Id;
    utility = Utility;
    noise = Noise;
    for(int i=0;i<SwarmSize;i++)
    {
        committed_agents.push_back(0);
    }
}

Node::~Node()
{
    if(children.size()>0)
    {
        for(long unsigned int i = 0; i < children.size(); ++i)
        {
            this->children[i]->~Node();
            delete [] children[i];
        }
    }
    for(int i = 0; i < kernel_size.GetY(); ++i)
    {
        delete [] kernel[i];
    }
    delete [] kernel;
    delete [] parent;
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

float Node::get_kernel_value(const CVector2 Position,const float Unit)
{
    unsigned int X=0,Y=0;
    for(float y=tl_br.tl.GetY();y<tl_br.br.GetY();y+=Unit)
    {
        X=0;
        for(float x=tl_br.tl.GetX();x<tl_br.br.GetX();x+=Unit)
        {
            if(Position.GetX()>=x-.005 && Position.GetX()<=x+.005)
            {
                if(Position.GetY()>=y-.005 && Position.GetY()<=y+.005)
                {
                    return kernel[Y][X];
                }
            }
            X++;
        }
        Y++;
    }
    // std::cout<<X<<", "<<Y<<" - "<<Position<<"\n";
    return -1;
}

bool Node::isin(CVector2 Position)
{
    if((Position.GetX()>=this->tl_br.tl.GetX()) && (Position.GetX()<=this->tl_br.br.GetX()))
        {
            if((Position.GetY()>=this->tl_br.tl.GetY()) && (Position.GetY()<=this->tl_br.br.GetY()))
            {
                return true;
            }
        }
    return false;
}

float** Node::get_kernel()
{
    return kernel;
}

CVector2 Node::get_kernel_size()
{
    return kernel_size;
}

std::vector<int> Node::get_committed_agents()
{
    return committed_agents;
}