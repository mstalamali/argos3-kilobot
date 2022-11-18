#ifndef MESSAGE_STRCUCT_H
#define MESSAGE_STRCUCT_H

int expiring_time_messages=10000;

typedef struct message_structure
{
    unsigned int counter;
    int agent_id, agent_node, agent_leaf;
    float leaf_utility;
    struct message_structure *next,*prev;
} message_a;

void set_expiring_time_message(const int Expiring_time)
{
    expiring_time_messages=Expiring_time;
}

void add_a_message(message_a **Mymessage,message_a **Prev,const int Agent_id,const int Agent_node, const int Agent_leaf, const int Counter, const float Leaf_utility, int adding_to_bottom )
{
    if(adding_to_bottom==0)
    {
        if((*Mymessage)==NULL)
        {
            (*Mymessage)=(message_a*)malloc(sizeof(message_a));
            (*Mymessage)->agent_id=Agent_id;
            (*Mymessage)->agent_node=Agent_node;
            (*Mymessage)->agent_leaf=Agent_leaf;
            (*Mymessage)->counter=Counter;
            (*Mymessage)->leaf_utility=Leaf_utility;
            (*Mymessage)->prev=NULL;
            (*Mymessage)->next=NULL;
        }
        else
        {
            message_a *flag = (*Mymessage)->next;
            add_a_message(&flag,&(*Mymessage),Agent_id,Agent_node,Agent_leaf,Counter,Leaf_utility,1);
        }
    }
    else
    {
        if((*Mymessage)==NULL)
        {
            (*Mymessage)=(message_a*)malloc(sizeof(message_a));
            (*Mymessage)->agent_id=Agent_id;
            (*Mymessage)->agent_node=Agent_node;
            (*Mymessage)->agent_leaf=Agent_leaf;
            (*Mymessage)->counter=Counter;
            (*Mymessage)->leaf_utility=Leaf_utility;
            (*Mymessage)->prev=Prev;
            (*Mymessage)->next=NULL;
        }
        else
        {
            message_a *flag = (*Mymessage)->next;
            add_a_message(&flag,&(*Mymessage),Agent_id,Agent_node,Agent_leaf,Counter,Leaf_utility,1);
        }
    }
}

void erase_expired_messages(message_a **Mymessage)
{
    if((*Mymessage)!=NULL)
    {
        (*Mymessage)->counter=(*Mymessage)->counter+1;
        if((*Mymessage)->counter >= expiring_time_messages)
        {
            if((*Mymessage)->next == NULL && (*Mymessage)->prev == NULL) free(*Mymessage);
            else if((*Mymessage)->next == NULL)
            {
                (*Mymessage)->prev->next=NULL;
                (*Mymessage)->prev = NULL;
                free(*Mymessage);
            }
            else if((*Mymessage)->prev == NULL)
            {
                message_a *flag = (*Mymessage)->next;
                (*Mymessage)->next->prev = NULL;
                (*Mymessage)->next = NULL;
                free(*Mymessage);
                *Mymessage = flag;
            }
            else
            {
                (*Mymessage)->prev->next=(*Mymessage)->next;
                (*Mymessage)->next->prev=(*Mymessage)->prev;
                free(*Mymessage);
            }
        }
        erase_expired_messages(&(*Mymessage)->next);
    }
}

void erase_messages(message_a **Mymessage)
{
    if((*Mymessage!=NULL))
    {
        if((*Mymessage)->next!=NULL)
        {
            message_a *m=(*Mymessage)->next;
            erase_messages(&m);
            (*Mymessage)->next=NULL;
        }
        (*Mymessage)->prev=NULL;
        free(*Mymessage);
    }
}

int is_fresh_message(message_a **Mymessage,const int Agent_id,const int Agent_node, const int Agent_leaf, const int Counter, const float Leaf_utility)
{
    int out;
    out=1;
    if(*Mymessage!=NULL)
    {
        if((*Mymessage)->agent_id==Agent_id)
        {
            out=2;
            if((*Mymessage)->counter<Counter) out = 0;
            if(out==2)
            {
                (*Mymessage)->agent_node = Agent_node;
                (*Mymessage)->agent_leaf = Agent_leaf;
                (*Mymessage)->leaf_utility = Leaf_utility;
                (*Mymessage)->counter = Counter;
            }
        }
        if(out==1 && (*Mymessage)->next!=NULL)
        {
            message_a *flag=(*Mymessage)->next;
            out=is_fresh_message(&flag,Agent_id,Agent_node,Agent_leaf,Counter,Leaf_utility);
        }
    }
    return out;
}

int get_counter_from_id(message_a **Mymessage,const int Agent_id)
{
    int out;
    out=-1;
    if(*Mymessage!=NULL)
    {
        if((*Mymessage)->agent_id==Agent_id)
        {
            out=(*Mymessage)->counter;
        }
        else
        {
            message_a *flag=(*Mymessage)->next;
            out=get_counter_from_id(&flag,Agent_id);
        }
    }
    return out;
}
#endif