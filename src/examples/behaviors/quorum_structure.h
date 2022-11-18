#ifndef QUORUM_STRCUCT_H
#define QUORUM_STRCUCT_H

int expiring_time_quorum=20000;

typedef struct quorum_structure
{
    int agent_id, agent_node, counter;
    struct quorum_structure *next,*prev;
} quorum_a;

void set_expiring_time_quorum_item(const int Expiring_time)
{
    expiring_time_quorum=Expiring_time;
}

void add_an_item(quorum_a **myquorum,const int Agent_id,const int Agent_node)
{
    if((*myquorum)==NULL)
    {
        (*myquorum)=(quorum_a*)malloc(sizeof(quorum_a));
        (*myquorum)->agent_id=Agent_id;
        (*myquorum)->agent_node=Agent_node;
        (*myquorum)->counter=0;
        (*myquorum)->next=NULL;
        (*myquorum)->prev=NULL;
    }
    else
    {
        quorum_a *temp_quorum = (*myquorum);
        while (temp_quorum->next!=NULL) temp_quorum=temp_quorum->next;
        quorum_a *prev_quorum = temp_quorum;
        temp_quorum=temp_quorum->next;
        temp_quorum=(quorum_a*)malloc(sizeof(quorum_a));
        temp_quorum->agent_id=Agent_id;
        temp_quorum->agent_node=Agent_node;
        temp_quorum->counter=0;
        temp_quorum->next=NULL;
        temp_quorum->prev=prev_quorum;
    }
}

void erase_expired_items(quorum_a **myquorum)
{
    if((*myquorum!=NULL))
    {
        (*myquorum)->counter++;
        if((*myquorum)->counter >= expiring_time_messages)
        {
            if((*myquorum)->next == NULL && (*myquorum)->prev == NULL) free(*myquorum);
            else if((*myquorum)->next == NULL)
            {
                (*myquorum)->prev->next=NULL;
                (*myquorum)->prev = NULL;
                free(*myquorum);
            }
            else if((*myquorum)->prev == NULL)
            {
                quorum_a *flag = (*myquorum)->next;
                (*myquorum)->next->prev = NULL;
                (*myquorum)->next = NULL;
                free(*myquorum);
                *myquorum = flag;
            }
            else
            {
                (*myquorum)->prev->next=(*myquorum)->next;
                (*myquorum)->next->prev=(*myquorum)->prev;
                free(*myquorum);
            }
        }
        erase_expired_messages(&(*myquorum)->next);
    }
}

void erase_quorum_list(quorum_a **myquorum)
{
    if((*myquorum!=NULL))
    {
        if((*myquorum)->next!=NULL)
        {
            quorum_a *q=(*myquorum)->next;
            erase_quorum_list(&q);
            (*myquorum)->next=NULL;
        }
        (*myquorum)->prev=NULL;
        free(*myquorum);
    }
}

#endif