#ifndef QUORUM_STRCUCT_H
#define QUORUM_STRCUCT_H

int expiring_ticks_quorum=20000;

typedef struct quorum_structure
{
    unsigned int agent_id, agent_node, counter;
    struct quorum_structure *next,*prev;
}quorum_a;

void set_expiring_ticks_quorum_item(const int Expiring_time)
{
    expiring_ticks_quorum=Expiring_time;
}

void add_an_item(quorum_a **Myquorum,quorum_a **Prev,const unsigned int Agent_id,const unsigned int Agent_node)
{
    if((*Myquorum)==NULL)
    {
        (*Myquorum)=(quorum_a*)malloc(sizeof(quorum_a));
        (*Myquorum)->agent_id=Agent_id;
        (*Myquorum)->agent_node=Agent_node;
        (*Myquorum)->counter=0;
        if (Prev!=NULL && *Prev!=NULL)
        {
            (*Myquorum)->prev=*Prev;
            (*Prev)->next=*Myquorum;
        }
        else (*Myquorum)->prev=NULL;
        (*Myquorum)->next=NULL;
    }
    else add_an_item(&(*Myquorum)->next,Myquorum,Agent_id,Agent_node);
}

void erase_expired_items(quorum_a **Myquorum)
{
    if(*Myquorum!=NULL)
    {
        quorum_a *Next=(*Myquorum)->next;
        if((*Myquorum)->counter >= expiring_ticks_quorum)
        {
            if((*Myquorum)->next == NULL && (*Myquorum)->prev == NULL)
            {
                // printf("erased_unique__%d__%d\n",(*Myquorum)->agent_id,(*Myquorum)->counter);
                free(*Myquorum);
                (*Myquorum)=NULL;
            }
            else if((*Myquorum)->next == NULL && (*Myquorum)->prev != NULL)
            {
                (*Myquorum)->prev->next=NULL;
                (*Myquorum)->prev = NULL;
                // printf("erased_end__%d__%d\n",(*Myquorum)->agent_id,(*Myquorum)->counter);
                free(*Myquorum);
                (*Myquorum)=NULL;
            }
            else if((*Myquorum)->next != NULL && (*Myquorum)->prev == NULL)
            {
                Next->prev = NULL;
                // printf("erased_head__%d__%d\n",(*Myquorum)->agent_id,(*Myquorum)->counter);
                free(*Myquorum);
                (*Myquorum)=Next;
            }
            else
            {
                (*Myquorum)->prev->next = Next;
                Next->prev = (*Myquorum)->prev;
                // printf("erased___%d__%d\n",(*Myquorum)->agent_id,(*Myquorum)->counter);
                free(*Myquorum);
                (*Myquorum)=NULL;
            }
        }
        erase_expired_items(&Next);
    }
}

void increment_quorum_counter(quorum_a **Myquorum)
{
    if((*Myquorum)!=NULL)
    {
        (*Myquorum)->counter=(*Myquorum)->counter+1;
        increment_quorum_counter(&((*Myquorum)->next));
    }
}

void erase_quorum_list(quorum_a **Myquorum)
{
    if((*Myquorum!=NULL))
    {
        if((*Myquorum)->next!=NULL)
        {
            quorum_a *q=(*Myquorum)->next;
            erase_quorum_list(&q);
            (*Myquorum)->next=NULL;
        }
        (*Myquorum)->prev=NULL;
        free(*Myquorum);
    }
}

int list_length_q(quorum_a **Myquorum)
{
    int out=0;
    if((*Myquorum!=NULL))
    {
        out=1;
        quorum_a *q=(*Myquorum)->next;
        while (q!=NULL)
        {
            out=out+1;
            q=q->next;
        }
        
    }
    return out;
}

int is_fresh_item(quorum_a **Myquorum,const int Agent_id,const int Agent_node)
{
    int out;
    out=1;
    if(*Myquorum!=NULL)
    {
        if((*Myquorum)->agent_id==Agent_id)
        {
            out=2;
            (*Myquorum)->agent_node = Agent_node;
            (*Myquorum)->counter = 0;
        }
        if(out==1 && (*Myquorum)->next!=NULL)
        {
            quorum_a *flag=(*Myquorum)->next;
            out=is_fresh_item(&flag,Agent_id,Agent_node);
        }
    }
    return out;
}

#endif