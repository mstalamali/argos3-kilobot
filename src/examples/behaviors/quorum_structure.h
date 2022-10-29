#ifndef QUORUM_STRCUCT_H
#define QUORUM_STRCUCT_H

int expiring_time_quorum=11;

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
    quorum_a *current_item=(*myquorum);
    if(current_item->counter >= expiring_time_quorum)
    {
        if(current_item->prev==NULL)
        {
            (*myquorum)=current_item->next;
            current_item->next=NULL;
            free(current_item);
            current_item=(*myquorum);
        }
        else if(current_item->next==NULL)
        {
            current_item->prev=NULL;
            free(current_item);
            current_item=NULL;
        }
        else
        {
            quorum_a *prev_item=current_item->prev;
            quorum_a *next_item=current_item->next;
            prev_item->next=next_item;
            next_item->prev=prev_item;
            current_item->prev=NULL;
            current_item->next=NULL;
            free(current_item);
            current_item=next_item;
        }
    }
    else current_item=current_item->next;
    if(current_item!=NULL) erase_expired_items(&current_item);
}

void increment_counter_quorum(quorum_a **myquorum)
{
    (*myquorum)->counter++;
    quorum_a *q=(*myquorum)->next;
    if (q!=NULL) increment_counter_quorum(&q);
}

void erase_quorum_list(quorum_a **myquorum)
{   
    quorum_a *q=(*myquorum)->next;
    if(q!=NULL)
    {
        erase_quorum_list(&q);
        q=NULL;
    }
    (*myquorum)->prev=NULL;
    free((*myquorum));
}

#endif