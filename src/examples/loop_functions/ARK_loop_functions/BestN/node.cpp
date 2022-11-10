// @author Fabio Oddi <fabio.oddi@diag.uniroma1.it>

#include "node.h"

Node::Node(){}

Node::Node(const int SwarmSize,const int Depth,const int Id,const float Utility,const float Noise)
{
    id = Id;
    utility = Utility;
    noise = Noise;
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