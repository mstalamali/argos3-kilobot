#ifndef MESSAGE_STRCUCT_H
#define MESSAGE_STRCUCT_H

int expiring_ticks_messages=10000;

typedef struct message_structure
{
    unsigned int counter;
    int agent_id, agent_node, agent_leaf;
    float leaf_utility, control_param;
    struct message_structure *next,*prev;
} message_a;

void set_expiring_ticks_message(const int Expiring_time)
{
    expiring_ticks_messages=Expiring_time;
}

void add_a_message(message_a **Mymessage,message_a **Prev,const int Agent_id,const int Agent_node, const int Agent_leaf, const int Counter, const float Leaf_utility,const float Control_param)
{
    if((*Mymessage)==NULL)
    {
        (*Mymessage)=(message_a*)malloc(sizeof(message_a));
        (*Mymessage)->agent_id=Agent_id;
        (*Mymessage)->agent_node=Agent_node;
        (*Mymessage)->agent_leaf=Agent_leaf;
        (*Mymessage)->counter=Counter;
        (*Mymessage)->leaf_utility=Leaf_utility;
        (*Mymessage)->control_param=Control_param;
        if (Prev!=NULL && *Prev!=NULL)
        {
            (*Mymessage)->prev=*Prev;
            (*Prev)->next=*Mymessage;
        }
        else (*Mymessage)->prev=NULL;
        (*Mymessage)->next=NULL;
    }
    else add_a_message(&(*Mymessage)->next,Mymessage,Agent_id,Agent_node,Agent_leaf,Counter,Leaf_utility,Control_param);
}

void increment_messages_counter(message_a **Mymessage)
{
    if((*Mymessage)!=NULL)
    {
        (*Mymessage)->counter=(*Mymessage)->counter+1;
        increment_messages_counter(&((*Mymessage)->next));
    }
}

void erase_expired_messages(message_a **Mymessage)
{
    if(*Mymessage!=NULL)
    {
        message_a *Next=(*Mymessage)->next;
        if((*Mymessage)->counter >= expiring_ticks_messages)
        {
            if((*Mymessage)->next == NULL && (*Mymessage)->prev == NULL)
            {
                free(*Mymessage);
                (*Mymessage)=NULL;
            }
            else if((*Mymessage)->next != NULL && (*Mymessage)->prev == NULL)
            {
                Next->prev = NULL;
                (*Mymessage)->next=NULL;
                free(*Mymessage);
                (*Mymessage)=Next;
            }
            else if((*Mymessage)->next == NULL && (*Mymessage)->prev != NULL)
            {
                (*Mymessage)->prev->next=NULL;
                (*Mymessage)->prev = NULL;
                free(*Mymessage);
                (*Mymessage)=NULL;
            }
            else
            {
                (*Mymessage)->prev->next = Next;
                Next->prev = (*Mymessage)->prev;
                free(*Mymessage);
                (*Mymessage)=NULL;
            }
        }
        erase_expired_messages(&Next);
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
        (*Mymessage)=NULL;
    }
}

int is_fresh_message(message_a **Mymessage,const int Agent_id,const int Agent_node, const int Agent_leaf, const int Counter, const float Leaf_utility, const float Control_param)
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
                (*Mymessage)->control_param = Control_param;
            }
        }
        if(out==1 && (*Mymessage)->next!=NULL)
        {
            message_a *flag=(*Mymessage)->next;
            out=is_fresh_message(&flag,Agent_id,Agent_node,Agent_leaf,Counter,Leaf_utility,Control_param);
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

int list_length_m(message_a **Mymessage)
{
    int out=0;
    if((*Mymessage!=NULL))
    {
        out=1;
        message_a *flag=(*Mymessage)->next;
        while (flag!=NULL)
        {
            out=out+1;
            flag=flag->next;
        }
        
    }
    return out;
}

message_a *select_a_random_msg(message_a **Mymessage)
{
    message_a *out=NULL;
    if(*Mymessage!=NULL)
    {
        if(list_length_m(Mymessage)==1) return *Mymessage;
        int index = rand()%(list_length_m(Mymessage)-1);
        out = *Mymessage;
        for(int i=0;i<index;i++) out = out->next;
    }
    return out;
}


#endif