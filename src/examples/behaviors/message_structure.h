#ifndef MESSAGE_STRCUCT_H
#define MESSAGE_STRCUCT_H

int expiring_time_messages=10000;

typedef struct message_structure
{
    int agent_id, agent_node, agent_leaf, counter;
    float leaf_utility;
    struct message_structure *next,*prev;
} message_a;

void set_expiring_time_message(const int Expiring_time)
{
    expiring_time_messages=Expiring_time;
}

void erase_messages(message_a **mymessage)
{
    if((*mymessage)->next!=NULL)
    {
        message_a *m=(*mymessage)->next;
        erase_messages(&m);
        (*mymessage)->next=NULL;
    }
    (*mymessage)->prev=NULL;
    free((*mymessage));
}

int is_fresh(message_a **mymessage,const int Agent_id, const int Counter)
{
    int out;
    out=1;
    if((*mymessage)->agent_id==Agent_id)
    {
        if((*mymessage)->counter<Counter) out = 0;
        else out=2;
    }
    if(out==1 && (*mymessage)->next!=NULL)
    {
        message_a *flag=(*mymessage)->next;
        out=is_fresh(&flag,Agent_id,Counter);
    }
    if(out==2) out=1;
    return out;
}

void add_a_message(message_a **mymessage,const int Agent_id,const int Agent_node, const int Agent_leaf, const int Counter, const float Leaf_utility)
{
    if((*mymessage)==NULL)
    {
        (*mymessage)=(message_a*)malloc(sizeof(message_a));
        (*mymessage)->agent_id=Agent_id;
        (*mymessage)->agent_node=Agent_node;
        (*mymessage)->agent_leaf=Agent_leaf;
        (*mymessage)->counter=Counter;
        (*mymessage)->leaf_utility=Leaf_utility;
        (*mymessage)->next=NULL;
        (*mymessage)->prev=NULL;
    }
    else
    {
        message_a *temp_message = (*mymessage);
        while (temp_message->next!=NULL) temp_message = temp_message->next;
        message_a *prev_message = temp_message;
        temp_message=temp_message->next;
        temp_message=(message_a*)malloc(sizeof(message_a));
        temp_message->agent_id=Agent_id;
        temp_message->agent_node=Agent_node;
        temp_message->agent_leaf=Agent_leaf;
        temp_message->counter=Counter;
        temp_message->leaf_utility=Leaf_utility;
        temp_message->next=NULL;
        temp_message->prev=prev_message;
    }
}

void erase_expired_messages(message_a **mymessage)
{ //AGGIUSTA
    message_a *current_message=(*mymessage);
    if(current_message->counter >= expiring_time_messages)
    {
        if(current_message->prev==NULL)
        {
            (*mymessage)=current_message->next;
            current_message->next=NULL;
            free(current_message);
            current_message=(*mymessage);
        }
        else if(current_message->next==NULL)
        {
            current_message->prev=NULL;
            free(current_message);
            current_message=NULL;
        }
        else
        {
            message_a *prev_message=current_message->prev;
            message_a *next_message=current_message->next;
            prev_message->next=next_message;
            next_message->prev=prev_message;
            current_message->prev=NULL;
            current_message->next=NULL;
            free(current_message);
            current_message=next_message;
        }
    }
    else current_message=current_message->next;
    if(current_message!=NULL) erase_expired_messages(&current_message);
}

// void read_from_buffer(message_a **buffer, message_a **messages, const int myID)
// {
//     message_a *item_to_check=(*buffer);
//     while (item_to_check!=NULL)
//     {
//         if(item_to_check->agent_id!=myID)
//         {
//             int toADD=1;
//             message_a *current_message=(*messages);
//             while (current_message!=NULL)
//             {
//                 if(current_message->agent_id==item_to_check->agent_id)
//                 {
//                     toADD=0;
//                     if(item_to_check->counter<current_message->counter)
//                     {
//                         current_message->counter=item_to_check->counter;
//                         current_message->agent_node=item_to_check->agent_node;
//                         current_message->agent_leaf=item_to_check->agent_leaf;
//                         current_message->leaf_utility=item_to_check->leaf_utility;
//                     }
//                     break;
//                 }
//                 current_message=current_message->next;
//             }
//             if(toADD) add_a_message(messages,item_to_check->agent_id,item_to_check->agent_node,item_to_check->agent_leaf,item_to_check->counter,item_to_check->leaf_utility);
//         }
//         item_to_check=item_to_check->next;
//     }
//     erase_messages(buffer);
// }

void increment_counter_messages(message_a **mymessage)
{
    (*mymessage)->counter++;
    message_a *m=(*mymessage)->next;
    if (m!=NULL) increment_counter_messages(&m);
}

#endif