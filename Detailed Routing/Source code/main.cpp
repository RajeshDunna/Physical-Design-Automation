#include<iostream>
#include<vector>
#include<algorithm>
#include<math.h>
#include<stdlib.h>
#include<map>
#include<string>
#include<fstream>
#include<sstream>
#include<iterator>

using namespace std;

class point
{
    public:
        int net;
        int x,y;
        int r_flag;
        point* right, *up, *down, *left;
        point* v_jog, *down_jog;
        int trav;
};

class node
{
    public:
        int y_low, y_up;
        int yl_new, yu_new;
        int yl_flag, yu_flag;
        int net_value;
        int invalid, done;  // done is a flag
        int reduce_flag;
        int loj;    // length of jog
        int dfb;    // distance from bottom
        int dft;    // distance from top
        int high_prio;  // frees two tracks due to completion
        int dist;
        point* yy;
        vector<node*> overlapped, same_net, ovlap_reduce_len; // overlapped nets, same net
        vector<point*> free_tracks;
};

vector<vector<int>> route_pat;
map<int, vector<point*>> net_route;
vector<point*> r;
vector<int> top_edge, bottom_edge;
vector<pair<point*,point*>> edge, edge2;
vector<vector<point*>> bottom_band, top_band;
vector<node*> unc_net;  // uncollapsed net
vector<pair<int,int>> v_pattern;    // v_pattern = vertical pattern
vector<pair<point*,point*>> save_edge2_l, save_edge2_u;
vector<point*> coverage;

bool compare_coor_x(point* p1, point* p2)
{
    return p1->x<p2->x;
}
bool compare_coor_y(point* p1, point* p2)
{
    return p1->y<p2->y;
}
bool mycompare(node* ptr1, node* ptr2)
{
    if(ptr1->overlapped.size()!=ptr2->overlapped.size())
        return ptr1->overlapped.size()<ptr2->overlapped.size();
    else
        return ptr1->high_prio>ptr2->high_prio;    
}
bool mycompare2(node* ptr1, node* ptr2)
{
    return ptr1->overlapped.size()<ptr2->overlapped.size();
}
bool compare_first(pair<int,int> p1, pair<int,int> p2)
{
    return p1.first<p2.first;
}
bool compare_y(point* first_t, point* last_t)
{
    return first_t->y<last_t->y;
}
bool compare_dist(pair<int,vector<point*>>p1, pair<int,vector<point*>>p2)
{
    return abs(p1.first)<abs(p2.first);
}
bool compare_pin_dist(node* ptr1, node* ptr2)
{
    return ptr1->dist<ptr2->dist;
}

void reduceRangeUNC(point* bottom, point* top, int index, int w, vector<int>:: iterator top_it, vector<int>:: iterator bottom_it)    // step c reduce the range between split nets
{
    vector<point*> free_t;

    if(unc_net.size()==0)
        return;

    if(bottom->v_jog!=NULL && top->down_jog!=NULL)
    {
        point* ptr1=top->down_jog;
        for(point* ptr = bottom->v_jog; ptr->y<=ptr1->y; ptr=ptr->up)
        {
            if(ptr->v_jog!=NULL)
            {
                ptr=ptr->v_jog->down;
                continue;
            }

            if(ptr->net==0)
                free_t.push_back(ptr);      // free tracks not covered by present vertical jogs
        }
    }
    else if(bottom->v_jog!=NULL && top->down_jog==NULL)
    {
        for(point* ptr = bottom->v_jog; ptr!=top; ptr=ptr->up)
        {
            if(ptr->v_jog!=NULL)
            {
                ptr=ptr->v_jog->down;
                continue;
            }

            if(ptr->net==0)
                free_t.push_back(ptr);      // free tracks not covered by present vertical jogs
        }
    }
    else if(bottom->v_jog==NULL && top->down_jog!=NULL)
    {
        point* ptr1=top->down_jog;
        for(point* ptr = bottom->up; ptr->y<=ptr1->y; ptr=ptr->up)
        {
            if(ptr->v_jog!=NULL)
            {
                ptr=ptr->v_jog->down;
                continue;
            }

            if(ptr->net==0)
                free_t.push_back(ptr);      // free tracks not covered by present vertical jogs
        }
    }
    else
    {
        for(point* ptr = bottom->up; ptr!=top; ptr=ptr->up)
        {
            if(ptr->v_jog!=NULL)
            {
                ptr=ptr->v_jog->down;
                continue;
            }

            if(ptr->net==0)
                free_t.push_back(ptr);      // free tracks not covered by present vertical jogs
        }
    }
    if(free_t.size()==0)
        return;
    sort(free_t.begin(), free_t.end(), compare_y);

    for(int i=0;i<unc_net.size();i++)
    {
        int ov_flag=0;
        for(int j=i+1;j<unc_net.size();j++)
        {
            if(unc_net[i]->net_value==unc_net[j]->net_value)
            {
                continue;
            }
            else if(unc_net[i]->y_low>unc_net[j]->y_low && unc_net[i]->y_low<unc_net[j]->y_up)
            {
                ov_flag=1;
            }
            else if(unc_net[i]->y_up>unc_net[j]->y_low && unc_net[i]->y_up<unc_net[j]->y_up)
            {
                ov_flag=1;
            }
            else if(unc_net[j]->y_low>unc_net[i]->y_low && unc_net[j]->y_up<unc_net[i]->y_up)
            {
                ov_flag=1;
            }

            if(ov_flag==1)
            {
                if(find(unc_net[i]->overlapped.begin(),unc_net[i]->overlapped.end(),unc_net[j])==unc_net[i]->overlapped.end())
                {
                    unc_net[i]->overlapped.push_back(unc_net[j]);   
                }
                if(find(unc_net[j]->overlapped.begin(),unc_net[j]->overlapped.end(),unc_net[i])==unc_net[j]->overlapped.end())
                {
                    unc_net[j]->overlapped.push_back(unc_net[i]);   
                }
                ov_flag=0;
            }
        }
    }

    for(int i=0;i<unc_net.size();i++)
    {
        if(unc_net[i]->y_up<free_t[0]->y)  //comparing net and node
        {
            // no free tracks between this net
            unc_net[i]->reduce_flag=0;
            continue;   // unc_net[i]->free_track.size()==0
        }
        else if(unc_net[i]->y_low>free_t[free_t.size()-1]->y)  //comparing net and node
        {
            // no free tracks between this net
            unc_net[i]->reduce_flag=0;
            continue;   // unc_net[i]->free_track.size()==0
        }
        for(int j=0;j<free_t.size();j++)
        {
            if(unc_net[i]->y_low<free_t[j]->y && unc_net[i]->y_up>free_t[j]->y)
            {
                // the free track is not covered by any existing vertical jogs
                unc_net[i]->free_tracks.push_back(free_t[j]);
                unc_net[i]->reduce_flag=1;  // free track available
            }
        }
        if(unc_net[i]->free_tracks.size()==0)
        {
            unc_net[i]->reduce_flag=0;  // no free tracks between the net, cannot be reduced, no free tracks
        }
        else if(unc_net[i]->free_tracks.size()!=0)
        {
            sort(unc_net[i]->free_tracks.begin(),unc_net[i]->free_tracks.end(),compare_y);
        }
    }

    for(int i=0;i<unc_net.size();i++)
    {
        if(unc_net[i]->reduce_flag==0)
            continue;
        for(int j=0;j<v_pattern.size();j++)
        {
            if(v_pattern[j].first<unc_net[i]->y_low && v_pattern[j].second>unc_net[i]->y_up)
            {
                unc_net[i]->reduce_flag=0;
                break;
            }
            if(v_pattern[j].first<unc_net[i]->y_up && v_pattern[j].second>unc_net[i]->y_up)
            {
                unc_net[i]->yu_flag=1;  // cannot use
            } 
            if(v_pattern[j].first<unc_net[i]->y_low && v_pattern[j].second>unc_net[i]->y_low)
            {
                unc_net[i]->yl_flag=1;  // cannot use
            }
        }
        if(unc_net[i]->yu_flag==1 && unc_net[i]->yl_flag==1)
        {
            unc_net[i]->reduce_flag=0;
        }
    }

    for(int i=0;i<unc_net.size();i++)
    {
        if(unc_net[i]->reduce_flag==0)
            continue;
        for(int j=0;j<v_pattern.size();j++)
        {
            if(v_pattern[j].first>unc_net[i]->y_low && v_pattern[j].second<unc_net[i]->y_up)
            {
                if(v_pattern[j].first<unc_net[i]->free_tracks[0]->y)
                {
                    unc_net[i]->yl_flag=1;  // cannot use
                }
                if(v_pattern[j].second>unc_net[i]->free_tracks[unc_net[i]->free_tracks.size()-1]->y)
                {
                    unc_net[i]->yu_flag=1;  // cannot use
                }
            }
        }
        if(unc_net[i]->yu_flag==1 && unc_net[i]->yl_flag==1)
        {
            unc_net[i]->reduce_flag=0;
        }
    
    }
    vector<node*> unc_net2;
    for(int i=0;i<unc_net.size();i++)
    {
        if(unc_net[i]->reduce_flag==0)
            continue;
        if(unc_net[i]->free_tracks.size()==0)
            continue;
        else
            unc_net2.push_back(unc_net[i]);
    }

    unc_net=unc_net2;  

    // the above 3 loops can be merged to save runtime
    
    for(int i=0;i<unc_net.size();i++)
    {
        unc_net[i]->overlapped.clear();
        unc_net[i]->invalid=0;
        if(unc_net[i]->yl_flag!=1 && unc_net[i]->yu_flag==1)
        {
            for(int j=0;j<v_pattern.size();j++)
            {
                if(v_pattern[j].first<unc_net[i]->free_tracks[unc_net[i]->free_tracks.size()-1]->y && v_pattern[j].first>unc_net[i]->free_tracks[0]->y)
                {
                    while(unc_net[i]->free_tracks[unc_net[i]->free_tracks.size()-1]->y>v_pattern[j].first)
                    {
                        unc_net[i]->free_tracks.pop_back();
                    }
                }
            }
            unc_net[i]->yl_new=unc_net[i]->y_low;
            unc_net[i]->yu_new=unc_net[i]->free_tracks[unc_net[i]->free_tracks.size()-1]->y;
        }
        else if(unc_net[i]->yl_flag==1 && unc_net[i]->yu_flag!=1)
        {
            for(int j=0;j<v_pattern.size();j++)
            {
                if(v_pattern[j].second>unc_net[i]->free_tracks[0]->y && v_pattern[j].second<unc_net[i]->free_tracks[unc_net[i]->free_tracks.size()-1]->y) 
                {
                    while(v_pattern[j].second>unc_net[i]->free_tracks[0]->y)
                    {
                        unc_net[i]->free_tracks.erase(unc_net[i]->free_tracks.begin());
                    }
                }
            }
            unc_net[i]->yu_new=unc_net[i]->y_up;
            unc_net[i]->yl_new=unc_net[i]->free_tracks[0]->y;
        }
        else if(unc_net[i]->yl_flag!=1 && unc_net[i]->yu_flag!=1)
        {
            int top_up, top_low, top, bottom_up, bottom_low, bottom;
            top_up=w+1-unc_net[i]->y_up;
            top_low=unc_net[i]->y_up;
            top=top_up<top_low?top_up:top_low;

            bottom_up=w+1-unc_net[i]->y_low;
            bottom_low=unc_net[i]->y_low;
            bottom=bottom_up<bottom_low?bottom_up:bottom_low;

            if(top<bottom)
            {
                for(int j=0;j<v_pattern.size();j++)
                {
                    if(v_pattern[j].second>unc_net[i]->free_tracks[0]->y && v_pattern[j].second<unc_net[i]->free_tracks[unc_net[i]->free_tracks.size()-1]->y) 
                    {
                        while(v_pattern[j].second>unc_net[i]->free_tracks[0]->y)
                        {
                            unc_net[i]->free_tracks.erase(unc_net[i]->free_tracks.begin());
                        }
                    }
                }
                unc_net[i]->yu_new=unc_net[i]->y_up;
                unc_net[i]->yl_new=unc_net[i]->free_tracks[0]->y;
            }
            else    // taking the bottom one
            {
                for(int j=0;j<v_pattern.size();j++)
                {
                    if(v_pattern[j].first<unc_net[i]->free_tracks[unc_net[i]->free_tracks.size()-1]->y && v_pattern[j].first>unc_net[i]->free_tracks[0]->y)
                    {
                        while(unc_net[i]->free_tracks[unc_net[i]->free_tracks.size()-1]->y>v_pattern[j].first)
                        {
                            unc_net[i]->free_tracks.pop_back();
                        }
                    }
                }
                unc_net[i]->yl_new=unc_net[i]->y_low;
                unc_net[i]->yu_new=unc_net[i]->free_tracks[unc_net[i]->free_tracks.size()-1]->y;
            }
        }
    }

    for(int i=0;i<unc_net.size();i++)
    {
        int ov_flag=0;
        for(int j=i+1;j<unc_net.size();j++)
        {
            if(unc_net[i]->yl_new==unc_net[j]->yl_new || unc_net[i]->yl_new==unc_net[j]->yu_new)
            {
                ov_flag=1;
            }
            else if(unc_net[i]->yu_new==unc_net[j]->yu_new || unc_net[i]->yu_new==unc_net[j]->yl_new)
            {
                ov_flag=1;
            }
            else if(unc_net[i]->yl_new>unc_net[j]->yl_new && unc_net[i]->yl_new<unc_net[j]->yu_new)
            {
                ov_flag=1;
            }
            else if(unc_net[i]->yu_new>unc_net[j]->yl_new && unc_net[i]->yu_new<unc_net[j]->yu_new)
            {
                ov_flag=1;
            }
            else if(unc_net[j]->yl_new>unc_net[i]->yl_new && unc_net[j]->yu_new<unc_net[i]->yu_new)
            {
                ov_flag=1;
            }

            if(ov_flag==1)
            {
                unc_net[i]->overlapped.push_back(unc_net[j]);
                unc_net[j]->overlapped.push_back(unc_net[i]);
                ov_flag=0;
            }
        }
    }
    sort(unc_net.begin(),unc_net.end(), mycompare2);

    v_pattern.clear();
    for(int i=0;i<unc_net.size();i++)       // can be optimized but will become complicated
    {
        if(unc_net[i]->invalid==1)
            continue;
        if(unc_net[i]->overlapped.size()==0)
        {
            v_pattern.push_back({unc_net[i]->yl_new,unc_net[i]->yu_new});
        }
        else 
        {
            v_pattern.push_back({unc_net[i]->yl_new,unc_net[i]->yu_new});

            for(int j=0;j<unc_net[i]->overlapped.size();j++)
            {
                unc_net[i]->overlapped[j]->invalid=1;
            }
        }
    }
    if(v_pattern.size()==0)
        return;
    sort(v_pattern.begin(),v_pattern.end(),compare_first);
    vector<pair<int,int>>:: iterator iter=v_pattern.begin();
    for(point* ptr = bottom->up; ptr!=top; ptr=ptr->up)       // making connections in channel data structure
    {
        if(ptr->y==iter->first)
        {
            point* ptr_end=ptr->up;
            while(ptr_end->y!=iter->second)
                ptr_end=ptr_end->up;

            ptr->v_jog=ptr_end;
            ptr_end->down_jog=ptr;
            if(ptr->net==0)
                ptr->net=ptr_end->net;
            else if(ptr_end->net==0)
                ptr_end->net=ptr->net;
            iter++;
            if(iter==v_pattern.end())
                break;
            ptr=ptr_end->down;
        }
    }
}

void verticalConnections(point* bottom, point* top, int index, int w,vector<int>:: iterator top_it, vector<int>:: iterator bottom_it)   // step b
{
    map<int,vector<int>> H, Hf;

    vector<node*> partition;

    if(bottom->v_jog!=NULL && top->down_jog!=NULL)
    {
        point* ptr1=top->down_jog;
        for(point* ptr = bottom->v_jog; ptr->y<=ptr1->y; ptr=ptr->up)
        {
            map<int,vector<int>>::iterator it(H.find(ptr->net));
            if (it != H.end())
            {
                it->second.push_back(ptr->y);
            }
            else
            {
                H[ptr->net].push_back(ptr->y);
            }    
        }
    }
    else if(bottom->v_jog==NULL && top->down_jog!=NULL)
    {
        point* ptr1=top->down_jog;
        for(point* ptr = bottom->up; ptr->y<=ptr1->y; ptr=ptr->up)
        {
            map<int,vector<int>>::iterator it(H.find(ptr->net));
            if (it != H.end())
            {
                it->second.push_back(ptr->y);
            }
            else
            {
                H[ptr->net].push_back(ptr->y);
            }    
        }
    }
    else if(bottom->v_jog!=NULL && top->down_jog==NULL)
    {
        for(point* ptr = bottom->v_jog; ptr!=top; ptr=ptr->up)
        {
            map<int,vector<int>>::iterator it(H.find(ptr->net));
            if (it != H.end())
            {
                it->second.push_back(ptr->y);
            }
            else
            {
                H[ptr->net].push_back(ptr->y);
            }    
        }
    }
    else
    {
        for(point* ptr = bottom->up; ptr!=top; ptr=ptr->up)
        {
            map<int,vector<int>>::iterator it(H.find(ptr->net));
            if (it != H.end())
            {
                it->second.push_back(ptr->y);
            }
            else
            {
                H[ptr->net].push_back(ptr->y);
            }    
        }
    }
    
    int return_check=0;
    map<int,vector<int>>::iterator it;
    for(it=H.begin();it!=H.end();it++)
    {
        if(it->first==0)
        {
            //Hf.insert({it->first,it->second});
            continue;
        }
        else if(it->second.size()>1)
        {
            Hf.insert({it->first,it->second});
            return_check=1;
        }
    }
    if(return_check==0)
        return;

    
    for(it=Hf.begin();it!=Hf.end();it++)
    {
        int net_check=it->first;
        int net_flag=0;
        int j=index+1;
            
        while(j<edge.size())    // can be optimized by  using find()
        {
            if(net_check==edge[j].first->net || net_check==edge[j].second->net) // checkig for future pins
            {
                net_flag=1;
                break;
            }
            j++;
        }
        
        for(int i=1;i<it->second.size();i++)
        {
            node* ptr = new node;       // here node represents net (vertical jog)
            ptr->net_value=it->first;
            ptr->y_up=it->second[i];
            ptr->y_low=it->second[i-1];

            ptr->loj=it->second[i]-it->second[i-1];
            ptr->dfb=it->second[i-1];
            ptr->dft=w+1-it->second[i];

            ptr->invalid=0;
            ptr->done=0;
            ptr->reduce_flag=-1;
            ptr->yl_flag=-1;
            ptr->yu_flag=-1;
            ptr->yl_new=-1;
            ptr->yu_new=-1;
            
            if(net_flag==1)
                ptr->high_prio=0;
            else
                ptr->high_prio=1;

            partition.push_back(ptr);

        }      
    }

    for(int i=0;i<partition.size();i++)
    {
        int ov_flag=0;
        for(int j=i+1;j<partition.size();j++)
        {
            if(partition[i]->net_value==partition[j]->net_value)
            {
                partition[i]->same_net.push_back(partition[j]);
                partition[j]->same_net.push_back(partition[i]);
                continue;
            }
            else if(partition[i]->y_low>partition[j]->y_low && partition[i]->y_low<partition[j]->y_up)
            {
                ov_flag=1;
            }
            else if(partition[i]->y_up>partition[j]->y_low && partition[i]->y_up<partition[j]->y_up)
            {
                ov_flag=1;
            }
            else if(partition[j]->y_low>partition[i]->y_low && partition[j]->y_up<partition[i]->y_up)
            {
                ov_flag=1;
            }

            if(ov_flag==1)
            {
                partition[i]->overlapped.push_back(partition[j]);
                partition[j]->overlapped.push_back(partition[i]);
                ov_flag=0;
            }
        }
    }

    sort(partition.begin(), partition.end(),mycompare);     // check during debugging

    // Partition has list of vertical jogs, I ave to select vertical jogs giving maximum free tracks
    
    

    run_again:
    for(int i=0;i<partition.size();i++)
    {
        if(partition[i]->overlapped.size()==0)  // non overlapping vertical segments
        {
            v_pattern.push_back({partition[i]->y_low,partition[i]->y_up});
            partition[i]->invalid=1;
            partition[i]->done=1;   // this net is selected for vertical jog
            partition[i]=NULL;
        }
        else
            break;
    }

    for(int i=0;i<partition.size();i++)
    {
        if(partition[i]==NULL || partition[i]->invalid==1)
            continue;

        int connections = partition[i]->overlapped.size();
        vector<node*> same_conn;
        same_conn.push_back(partition[i]);
        for(int j=0;j<partition[i]->overlapped.size();j++)
        {
            if(partition[i]->overlapped[j]->overlapped.size()==connections)
            {
                same_conn.push_back(partition[i]->overlapped[j]);   // ties between multiple vertical jogs
            }
        }

        if(same_conn.size()==1)
        {
            v_pattern.push_back({partition[i]->y_low,partition[i]->y_up});
            partition[i]->done=1;
            partition[i]->invalid=1;

            for(int k=0;k<partition[i]->overlapped.size();k++)
            {
                partition[i]->overlapped[k]->invalid=1;
                unc_net.push_back(partition[i]->overlapped[k]);     // uncollapsed nets
                for(int l=0;l<partition[i]->overlapped[k]->overlapped.size();l++)
                {
                    node* ptr=partition[i]->overlapped[k]->overlapped[l];
                    if(ptr->invalid==1)
                        continue;
                    else    
                    {
                        vector<node*>:: iterator it = find(ptr->overlapped.begin(), ptr->overlapped.end(),partition[i]->overlapped[k]);
                        vector<node*>:: iterator it1= it;
                        it=ptr->overlapped.end()-1;     // it is pointing to last
                        *it1=*it;       // copying the value  of last one to the current index
                        ptr->overlapped.pop_back();
                    }

                }
            }
        }
        else
        {
            node* ptr=same_conn[0];
            node* final=ptr;
            for(int m=1;m<same_conn.size();m++)
            {                                               // resolving the ties in this loop
                ptr=final;
                int ptr_len, same_c_len;
                if(ptr->dfb<ptr->dft)
                    ptr_len=ptr->dfb;
                else
                    ptr_len=ptr->dft;

                if(same_conn[m]->dfb<same_conn[m]->dft)
                    same_c_len=same_conn[m]->dfb;
                else
                    same_c_len=same_conn[m]->dft;


                if(ptr_len<same_c_len)
                    final = ptr;
                else if(ptr_len>same_c_len)
                    final=same_conn[m];
                else if(ptr_len==same_c_len)        // another tie
                {
                    if(ptr->loj<same_conn[m]->loj)
                        final=same_conn[m];
                    else 
                        final=ptr;
                }
            }
            v_pattern.push_back({final->y_low,final->y_up});        // updating v_pattern
            final->done=1;
            final->invalid=1;

            for(int k=0;k<final->overlapped.size();k++)
            {
                final->overlapped[k]->invalid=1;
                unc_net.push_back(final->overlapped[k]);
                for(int l=0;l<final->overlapped[k]->overlapped.size();l++)
                {
                    node* ptr=final->overlapped[k]->overlapped[l];
                    if(ptr->invalid==1)
                        continue;
                    else    
                    {
                        vector<node*>:: iterator it = find(ptr->overlapped.begin(), ptr->overlapped.end(),final->overlapped[k]);
                        vector<node*>:: iterator it1= it;
                        it=ptr->overlapped.end()-1;     // it is pointing to last
                        *it1=*it;       // copying the value  of last one to the current index
                        ptr->overlapped.pop_back();
                    }

                }
            }


        }

        vector<node*> partition2;
        for(int m1=0;m1<partition.size();m1++)
        {
            if(partition[m1]==NULL)
                continue;
            else if(partition[m1]->invalid==1)
                continue;
            else
                partition2.push_back(partition[m1]);
        }
        if(partition2.size()==0)    // all nodes of partition have become either NULL or invalid
            break;
        else
            partition=partition2;
        
        sort(partition.begin(), partition.end(),mycompare);
        //i=-1;
        goto run_again;

    }

    sort(v_pattern.begin(),v_pattern.end(),compare_first);
    vector<pair<int,int>>:: iterator iter=v_pattern.begin();
    for(point* ptr = bottom->up; ptr!=top; ptr=ptr->up)       // making connections in channel data structure
    {
        if(ptr->y==iter->first)
        {
            point* ptr_end=ptr->up;
            while(ptr_end->y!=iter->second)
                ptr_end=ptr_end->up;

            ptr->v_jog=ptr_end;
            ptr_end->down_jog=ptr;
            iter++;
            if(iter==v_pattern.end())
                break;
            ptr=ptr_end->down;
        }
    }

}
void extendNextCol(point* bottom, point* top, int index, int w, vector<int>:: iterator top_it, vector<int>:: iterator bottom_it)
{
    map<int,vector<point*>> H;
    vector<point*> extend;

    for(point* ptr = bottom->up; ptr!=top; ptr=ptr->up)
    {
        map<int,vector<point*>>::iterator it(H.find(ptr->net));
        if (it != H.end())
        {
            it->second.push_back(ptr);
        }
        else
        {
            H[ptr->net].push_back(ptr);
        }    
    }
    map<int,vector<point*>>:: iterator it = H.begin();
    if(it->first==0)
        it++;
    while(it!=H.end())
    {
        if(it->second.size()==1)
        {
            if(find(bottom_it, bottom_edge.end(), it->first)!=bottom_edge.end() || find(top_it, top_edge.end(), it->first)!=top_edge.end())
            {
                extend.push_back(it->second[0]);
            }
        }
        else if(it->second.size()>1)
        {
            int t,b;
            if(find(bottom_it, bottom_edge.end(), it->first)!=bottom_edge.end())
                b=find(bottom_it, bottom_edge.end(), it->first)-bottom_it;
            else
                b=-1;
            if(find(top_it, top_edge.end(), it->first)!=top_edge.end())
                t=find(top_it, top_edge.end(), it->first)-top_it;
            else
                t=-1;

            if(b==-1 && t==-1)
            {
                int flag=0;
                for(int i=1;i<it->second.size();i++)
                {
                    if(it->second[i-1]->v_jog==NULL || it->second[i-1]->v_jog!=it->second[i])
                    {
                        flag=1;
                    }
                }
                if(flag==0)
                {
                    // complete collapsed
                }
                else if(flag==1)
                {
                    vector<vector<point*>> vec;
                    for(int i=0;i<it->second.size();i++)
                    {
                        vector<point*> v;
                        if(it->second[i]->v_jog==NULL)
                        {
                            v.push_back(it->second[i]);
                            vec.push_back(v);
                            extend.push_back(it->second[i]);
                        }
                        else
                        {
                            v.push_back(it->second[i]);
                            while(i<it->second.size() && it->second[i]->v_jog!=NULL)
                            {
                                i++;
                                
                            }
                            v.push_back(it->second[i]);
                            vec.push_back(v);
                        }
                    }
                    for(int j=0;j<vec.size();j++)
                    {
                        if(vec[j].size()==2)
                        {
                            if(j==0)
                            {
                                extend.push_back(vec[j][1]);
                            }
                            else if(j==(vec.size()-1))
                            {
                                extend.push_back(vec[j][0]);
                            }
                            else
                            {
                                if(vec[j-1].size()==1)
                                {
                                    extend.push_back(vec[j][0]);
                                }
                                else if(vec[j+1].size()==1)
                                {
                                    extend.push_back(vec[j][1]);
                                }
                                else
                                {
                                    extend.push_back(vec[j][0]);
                                }
                            }
                        }
                    }
                }
            }
            else    // pin exists in future
            {
                int flag=0;
                for(int i=1;i<it->second.size();i++)
                {
                    if(it->second[i-1]->v_jog==NULL || it->second[i-1]->v_jog!=it->second[i])
                    {
                        flag=1;
                    }
                }
                if(flag==0)
                {
                    // complete collapsed
                    if(it->first==bottom->net)
                    {
                        extend.push_back(it->second[it->second.size()-1]);
                    }
                    else if(it->first==top->net)
                    {
                        extend.push_back(it->second[0]);
                    }
                    else if(b==-1 && t!=-1)
                    {
                        extend.push_back(it->second[it->second.size()-1]);
                    }
                    else if(t==-1 && b!=-1)
                    {
                        extend.push_back(it->second[0]);
                    }
                    else if(b<t)
                    {
                        extend.push_back(it->second[0]);
                    }
                    else    //t<b
                    {
                        extend.push_back(it->second[it->second.size()-1]);
                    }
                }
                else if(flag==1)
                {
                    // uncollapsed
                    vector<vector<point*>> vec;
                    for(int i=0;i<it->second.size();i++)
                    {
                        vector<point*> v;
                        if(it->second[i]->v_jog==NULL)
                        {
                            v.push_back(it->second[i]);
                            vec.push_back(v);
                            extend.push_back(it->second[i]);
                        }
                        else
                        {
                            v.push_back(it->second[i]);
                            while(i<it->second.size() && it->second[i]->v_jog!=NULL)
                            {
                                i++;
                            }
                            v.push_back(it->second[i]);
                            vec.push_back(v);
                        }
                    }
                    for(int j=0;j<vec.size();j++)
                    {
                        if(vec[j].size()==2)
                        {
                            if(j==0)
                            {
                                extend.push_back(vec[j][1]);
                            }
                            else if(j==(vec.size()-1))
                            {
                                extend.push_back(vec[j][0]);
                            }
                            else
                            {
                                if(vec[j-1].size()==1)
                                {
                                    extend.push_back(vec[j][0]);
                                }
                                else if(vec[j+1].size()==1)
                                {
                                    extend.push_back(vec[j][1]);
                                }
                                else
                                {
                                    if(t==-1 && b!=-1)
                                    {
                                        extend.push_back(vec[j][0]);
                                    }
                                    else if(t!=-1 && b==-1)
                                    {
                                        extend.push_back(vec[j][1]);
                                    }
                                    else if(t<b)
                                        extend.push_back(vec[j][1]);
                                    else
                                        extend.push_back(vec[j][0]);
                                }
                            }
                        }
                    }
                }
            }
        }
        it++;
    }
    if(extend.size()==0)
        return;
    for(int i=0;i<extend.size();i++)
    {
        if(extend[i]->right==NULL)
        {
            // should not happen
        }
        extend[i]->right->net=extend[i]->net;
    }
}
void addTrackhigh(point* top, vector<int>:: iterator top_it)
{
        point* pu, *pd;
        vector<point*> range;
        range.push_back(top);
        point* range_start, *range_end;
        point* ptr_up=top;
        point* ptr_down=top->down; 
        pu=ptr_up;
        pd=ptr_down;
        // for left side
        while(ptr_up!=NULL)
        {
            point* new_point = new point;
            new_point->left=NULL;
            new_point->right=NULL;
            new_point->trav=0;

            ptr_down->up=new_point;
            new_point->down=ptr_down;

            ptr_up->down=new_point;
            new_point->up=ptr_up;

            new_point->x=ptr_down->x;
            new_point->y=ptr_down->y+1;

            for(point* p=new_point->up;p!=NULL;p=p->up)
            {
                p->y=p->y+1;
            }

            if(new_point->x==top->x)
            {
                top->down_jog=new_point;
                new_point->v_jog=top;
                new_point->net=top->net;

                range_start=new_point;
            }
            else
            {
                new_point->net=0;
                new_point->v_jog=NULL;
            }
            
            new_point->down_jog=NULL;
            

            if(new_point->x!=top->x)
            {
                new_point->right=ptr_down->right->up;
                ptr_down->right->up->left=new_point;
            }
            
            ptr_up=ptr_up->left;
            ptr_down=ptr_down->left;
            
        }

        if(pu->right!=NULL)
        {
            ptr_up=pu->right;
            ptr_down=pd->right;
        }
        
        // for right side
        while(pu->right!=NULL && ptr_up!=NULL)
        {
            point* new_point = new point;
            new_point->left=NULL;
            new_point->right=NULL;
            new_point->trav=0;

            ptr_down->up=new_point;
            new_point->down=ptr_down;

            ptr_up->down=new_point;
            new_point->up=ptr_up;

            new_point->x=ptr_down->x;
            new_point->y=ptr_down->y+1;

            for(point* p=new_point->up;p!=NULL;p=p->up)
            {
                p->y=p->y+1;
            }

            new_point->down_jog=NULL;
            new_point->v_jog=NULL;

            if(find(top_it, top_edge.end(),top->net)!=top_edge.end())
            {
                new_point->net=top->net;
                if(ptr_up->net==new_point->net)
                {
                    new_point->v_jog=ptr_up;
                    ptr_up->down_jog=new_point;

                    range_end=new_point;
                }
            }
            else
                new_point->net=0;
        
            new_point->left=ptr_down->left->up;
            ptr_down->left->up->right=new_point;
            
            
            ptr_up=ptr_up->right;
            ptr_down=ptr_down->right;
            top_it++;
            
        }
        range.push_back(range_start);
        range.push_back(range_end);
        top_band.push_back(range);
}
void addTracklow(point* bottom, vector<int>:: iterator bottom_it)
{
    
        point* pu, *pd;
        vector<point*> range;
        range.push_back(bottom);
        point* range_start, *range_end;
        point* ptr_down=bottom; 
        point* ptr_up=bottom->up;
        pu=ptr_up;
        pd=ptr_down;
        // for left side
        while(ptr_up!=NULL)
        {
            point* new_point = new point;
            new_point->left=NULL;
            new_point->right=NULL;
            new_point->trav=0;
            ptr_down->up=new_point;
            new_point->down=ptr_down;

            ptr_up->down=new_point;
            new_point->up=ptr_up;

            new_point->x=ptr_down->x;
            new_point->y=ptr_down->y+1;

            for(point* p=new_point->up;p!=NULL;p=p->up)
            {
                p->y=p->y+1;
            }

            if(new_point->x==bottom->x)
            {
                bottom->v_jog=new_point;
                new_point->down_jog=bottom;
                new_point->net=bottom->net;

                range_start=new_point;
            }
            else
            {
                new_point->down_jog=NULL;
                new_point->net=0;
            }
            new_point->v_jog=NULL;

            if(new_point->x!=bottom->x)
            {
                new_point->right=ptr_down->right->up;
                ptr_down->right->up->left=new_point;
            }
            
            ptr_up=ptr_up->left;
            ptr_down=ptr_down->left;
            
        }

        if(pu->right!=NULL)
        {
            ptr_up=pu->right;
            ptr_down=pd->right;
        }
        
        // for right side
        while(pu->right!=NULL && ptr_up!=NULL)
        {
            point* new_point = new point;
            new_point->left=NULL;
            new_point->right=NULL;
            new_point->trav=0;
            ptr_down->up=new_point;
            new_point->down=ptr_down;

            ptr_up->down=new_point;
            new_point->up=ptr_up;

            new_point->x=ptr_down->x;
            new_point->y=ptr_down->y+1;

            for(point* p=new_point->up;p!=NULL;p=p->up)
            {
                p->y=p->y+1;
            }

            new_point->down_jog=NULL;
            new_point->v_jog=NULL;

            if(find(bottom_it, bottom_edge.end(),bottom->net)!=bottom_edge.end())
            {
                new_point->net=bottom->net;
                if(ptr_down->net==new_point->net)
                {
                    ptr_down->v_jog=new_point;
                    new_point->down_jog=ptr_down;
                    range_end=new_point;
                }
            }
            else
                new_point->net=0;

            new_point->left=ptr_down->left->up;
            ptr_down->left->up->right=new_point;
            
            
            ptr_up=ptr_up->right;
            ptr_down=ptr_down->right;
            bottom_it++;
            
        }
        range.push_back(range_start);
        range.push_back(range_end);
        bottom_band.push_back(range); 
}
void addTrack(point* bottom, point* top, int index, int& w, vector<int>:: iterator top_it, vector<int>:: iterator bottom_it)
{
    if(bottom->v_jog==NULL)
    {
        point* ptr_up=bottom;
        point* pu, *pd;
        
        while(ptr_up->v_jog==NULL && (ptr_up->y-bottom->y)<(w+1)/2)
            ptr_up=ptr_up->up;
        point* ptr_down=ptr_up->down; 

        pu=ptr_up;
        pd=ptr_down;
        // for left side
        while(ptr_up!=NULL)
        {
            point* new_point = new point;
            new_point->left=NULL;
            new_point->right=NULL;
            new_point->trav=0;

            ptr_down->up=new_point;
            new_point->down=ptr_down;

            ptr_up->down=new_point;
            new_point->up=ptr_up;

            new_point->x=ptr_down->x;
            new_point->y=ptr_down->y+1;

            for(point* p=new_point->up;p!=NULL;p=p->up)
            {
                p->y=p->y+1;
            }

            if(new_point->x==bottom->x)
            {
                bottom->v_jog=new_point;
                new_point->down_jog=bottom;
                new_point->net=bottom->net;
            }
            else
            {
                new_point->down_jog=NULL;
                new_point->net=0;
            }
            new_point->v_jog=NULL;

            if(new_point->x!=bottom->x)
            {
                new_point->right=ptr_down->right->up;
                ptr_down->right->up->left=new_point;
            }
            
            ptr_up=ptr_up->left;
            ptr_down=ptr_down->left;
            
        }

        if(pu->right!=NULL)
        {
            ptr_up=pu->right;
            ptr_down=pd->right;
        }
        
        // for right side
        while(pu->right!=NULL && ptr_up!=NULL)
        {
            point* new_point = new point;
            new_point->left=NULL;
            new_point->right=NULL;
            new_point->trav=0;

            ptr_down->up=new_point;
            new_point->down=ptr_down;

            ptr_up->down=new_point;
            new_point->up=ptr_up;

            new_point->x=ptr_down->x;
            new_point->y=ptr_down->y+1;

            for(point* p=new_point->up;p!=NULL;p=p->up)
            {
                p->y=p->y+1;
            }

            new_point->down_jog=NULL;
            new_point->net=0;
            new_point->v_jog=NULL;

            
            new_point->left=ptr_down->left->up;
            ptr_down->left->up->right=new_point;
            
            
            ptr_up=ptr_up->right;
            ptr_down=ptr_down->right;
            
        }
        w++;  
        for(int k=0;k<save_edge2_u.size();k++)
        {
            point* ptr=save_edge2_u[k].second;
            while(ptr!=NULL)
            {
                ptr->y=ptr->y+1;
                ptr=ptr->up;
            }
        }
    }

    if(top->down_jog==NULL)
    {
        point* ptr_up=top;
        point* pu, *pd;
        
        while(ptr_up->down_jog==NULL && ptr_up->y>(((w+1)/2)+1))
            ptr_up=ptr_up->down;
        
        ptr_up=ptr_up->up;
        point* ptr_down=ptr_up->down; 

        pu=ptr_up;
        pd=ptr_down;
        // for left side
        while(ptr_up!=NULL)
        {
            point* new_point = new point;
            new_point->left=NULL;
            new_point->right=NULL;
            new_point->trav=0;

            ptr_down->up=new_point;
            new_point->down=ptr_down;

            ptr_up->down=new_point;
            new_point->up=ptr_up;

            new_point->x=ptr_down->x;
            new_point->y=ptr_down->y+1;

            for(point* p=new_point->up;p!=NULL;p=p->up)
            {
                p->y=p->y+1;
            }

            if(new_point->x==top->x)
            {
                top->down_jog=new_point;
                new_point->v_jog=top;
                new_point->net=top->net;
            }
            else
            {
                new_point->v_jog=NULL;
                new_point->net=0;
            }
            new_point->down_jog=NULL;

            if(new_point->x!=top->x)
            {
                new_point->right=ptr_down->right->up;
                ptr_down->right->up->left=new_point;
            }
            
            ptr_up=ptr_up->left;
            ptr_down=ptr_down->left;
            
        }

        if(pu->right!=NULL)
        {
            ptr_up=pu->right;
            ptr_down=pd->right;
        }
        
        // for right side
        while(pu->right!=NULL && ptr_up!=NULL)
        {
            point* new_point = new point;
            new_point->left=NULL;
            new_point->right=NULL;
            new_point->trav=0;

            ptr_down->up=new_point;
            new_point->down=ptr_down;

            ptr_up->down=new_point;
            new_point->up=ptr_up;

            new_point->x=ptr_down->x;
            new_point->y=ptr_down->y+1;

            for(point* p=new_point->up;p!=NULL;p=p->up)
            {
                p->y=p->y+1;
            }
            
            new_point->v_jog=NULL;
            new_point->net=0;
            
            new_point->down_jog=NULL;

            new_point->left=ptr_down->left->up;
            ptr_down->left->up->right=new_point;
            
            ptr_up=ptr_up->right;
            ptr_down=ptr_down->right;
            
        } 
        w++; 
        for(int k=0;k<save_edge2_u.size();k++)
        {
            point* ptr=save_edge2_u[k].second;
            while(ptr!=NULL)
            {
                ptr->y=ptr->y+1;
                ptr=ptr->up;
            }
        }
    }
}
void raiseLownets(point* bottom, point* top, int index, int w, vector<int>:: iterator top_it, vector<int>:: iterator bottom_it)
{
    // assuming all the v_pattern nets have been formed (need checking)

    map<int,vector<point*>> H;
    vector<pair<int, point*>> Hf;

    for(point* ptr = bottom->up; ptr!=top; ptr=ptr->up)
    {
        map<int,vector<point*>>::iterator it(H.find(ptr->net));
        if (it != H.end())
        {
            it->second.push_back(ptr);
        }
        else
        {
            H[ptr->net].push_back(ptr);
        }    
    }

    vector<point*> free_t;
    if(bottom->v_jog!=NULL && top->down_jog!=NULL)
    {
        point* ptr1=top->down_jog;
        for(point* ptr = bottom->v_jog; ptr->y<=ptr1->y; ptr=ptr->up)
        {
            if(ptr->v_jog!=NULL)
            {
                ptr=ptr->v_jog->down;
                continue;
            }

            if(ptr->net==0)
                free_t.push_back(ptr);      // free tracks not covered by present vertical jogs
        }
    }
    else if(bottom->v_jog==NULL && top->down_jog!=NULL)
    {
        point* ptr1=top->down_jog;
        for(point* ptr = bottom->up; ptr->y<=ptr1->y; ptr=ptr->up)
        {
            if(ptr->v_jog!=NULL)
            {
                ptr=ptr->v_jog->down;
                continue;
            }

            if(ptr->net==0)
                free_t.push_back(ptr);      // free tracks not covered by present vertical jogs
        }
    }
    else if(bottom->v_jog!=NULL && top->down_jog==NULL)
    {
        for(point* ptr = bottom->v_jog; ptr!=top; ptr=ptr->up)
        {
            if(ptr->v_jog!=NULL)
            {
                ptr=ptr->v_jog->down;
                continue;
            }

            if(ptr->net==0)
                free_t.push_back(ptr);      // free tracks not covered by present vertical jogs
        }
    }
    else
    {
        for(point* ptr = bottom->up; ptr!=top; ptr=ptr->up)
        {
            if(ptr->v_jog!=NULL)
            {
                ptr=ptr->v_jog->down;
                continue;
            }

            if(ptr->net==0)
                free_t.push_back(ptr);      // free tracks not covered by present vertical jogs
        }
    }
    
    if(free_t.size()==0)
        return;

    sort(free_t.begin(), free_t.end(), compare_y);
    

    map<int,vector<point*>>::iterator it = H.begin();

    if(it->first!=0)
        return;         // no free tracks, redundant

//    vector<int> ft = it->second;        // free tracks may have vertical overlaps

    it++;
    while(it!=H.end())
    {
        if(it->second.size()==1)
        {
            if(it->first==top->net || it->first==bottom->net)
            {
                if((find(top_it,top_edge.end(),it->first)!=top_edge.end()) || (find(bottom_it,bottom_edge.end(),it->first)!=bottom_edge.end()))
                {
                    Hf.push_back({it->first, it->second[0]});
                }
            }
            else
                Hf.push_back({it->first, it->second[0]});
        }
        else if(it->second.size()>1)
        {
            int flag=0;
            point* v;
            for(int i=1;i<it->second.size();i++)
            {
                if(it->second[i-1]->v_jog!=it->second[i])
                {
                    flag=1;
                    break;
                }
            }
            if(flag==1)
            {
                flag=0;
            }
            else
            {
                int t,b;
                if(find(top_it,top_edge.end(),it->first)!=top_edge.end())
                {
                    t=find(top_it,top_edge.end(),it->first)-top_it;
                }
                else
                    t=-1;
                
                if(find(bottom_it,bottom_edge.end(),it->first)!=bottom_edge.end())
                {
                    b=find(bottom_it,bottom_edge.end(),it->first)-bottom_it;
                }
                else
                    b=-1;

            
                if(t!=-1 && b==-1)
                {
                    if(it->first==top->net)
                    {
                        v=it->second[0];
                    }
                    else if(it->first==bottom->net)
                    {
                        v=it->second[it->second.size()-1];
                    }
                    else
                        v=it->second[it->second.size()-1];
                }
                else if(t==-1 && b!=-1)
                {
                    if(it->first==top->net)
                    {
                        v=it->second[0];
                    }
                    else if(it->first==bottom->net)
                    {
                        v=it->second[it->second.size()-1];
                    }
                    else
                        v=it->second[0];
                }
                else if(t!=-1 && b!=-1)
                {
                    if(it->first==top->net)
                    {
                        v=it->second[0];
                    }
                    else if(it->first==bottom->net)
                    {
                        v=it->second[it->second.size()-1];
                    }
                    else
                    {
                        if(t<b)
                        {
                            v=it->second[it->second.size()-1];
                        }
                        else
                        {
                            v=it->second[0];
                        }
                    }
                }
                else if(t==-1 && b==-1)
                {
                    // don't push, no pins in future
                }
                
                if(t!=-1 || b!=-1)
                    Hf.push_back({it->first,v});
            }
        }
        it++;
    }
    
    vector<pair<int, point*>>:: iterator iter;

    for(point* ptr=bottom; ptr!=top;ptr=ptr->up)
    {
      do_again:
        if(ptr->v_jog!=NULL)
        {
            ptr=ptr->v_jog;
            for(iter=Hf.begin();iter!=Hf.end();iter++)
            {
                if(iter->first!=-1)
                {
                    if(iter->second->y<ptr->y && iter->second->y>ptr->down_jog->y)
                    {
                        iter->first=-1;
                    }
                }
            }
            if(ptr==top)
                break;
            if(ptr->v_jog!=NULL)
                goto do_again;
        }
    }

    vector<pair<point*,vector<point*>>> H_mod;
    vector<node*> raise_low;
//  (find(top_it,top_edge.end(),it->first)!=top_edge.end()) || (find(bottom_it,bottom_edge.end(),it->first)!=bottom_edge.end())
    for(iter=Hf.begin();iter!=Hf.end();iter++)
    {
        if(iter->first==-1)
            continue;
        
        node* ptr = new node;
        ptr->net_value=iter->first;
        ptr->yy=iter->second;

        int t, b;
        if(find(top_it,top_edge.end(),iter->first)!=top_edge.end())
            t=find(top_it,top_edge.end(),iter->first)-top_it;
        else
            t=-1;
        if(find(bottom_it,bottom_edge.end(),iter->first)!=bottom_edge.end())
            b=find(bottom_it,bottom_edge.end(),iter->first)-bottom_it;
        else
            b=-1;

        if(t==-1 && b==-1)
        {
            // not possible
        }
        else if(t==-1 && b!=-1)
        {
            ptr->y_up=iter->second->y;
            ptr->y_low=-1;
            ptr->dist=b;
        }
        else if(t!=-1 && b==-1)
        {
            ptr->y_low=iter->second->y;
            ptr->y_up=-1;
            ptr->dist=t;
        }
        else if(t<b)
        {
            ptr->y_low=iter->second->y;
            ptr->y_up=-1;
            ptr->dist=t;
        }
        else if(b<t)
        {
            ptr->y_up=iter->second->y;
            ptr->y_low=-1;
            ptr->dist=b;
        }

        ptr->invalid=0;
        ptr->done=0;
        ptr->reduce_flag=-1;
        ptr->yl_flag=-1;
        ptr->yu_flag=-1;
        ptr->yl_new=-1;
        ptr->yu_new=-1;

        point* ptr1=ptr->yy;
        if(ptr->y_low==-1)
        {
            if(ptr1->down_jog!=NULL)
                ptr->invalid=1;
            else
            {
                int f=0;
                vector<point*> ftrack;
                while(ptr1->down_jog==NULL && ptr1->y>((bottom->y)+1))
                {
                    if(ptr1->net==0)
                    {
                        ftrack.push_back(ptr1);
                        f=1;
                    }
                    ptr1=ptr1->down;
                }
                if(f==1)
                {
                    if(ftrack.size()>2)
                        ptr->y_low=ftrack[(ftrack.size()/2)]->y;
                    else
                        ptr->y_low=ftrack[0]->y;
                    f=0;
                }
                else
                {
                    ptr->invalid=1;
                }
            }
        }
        else if(ptr->y_up==-1)      // ptr1 is pointing to grid point and ptr is the node-net
        {
            if(ptr1->v_jog!=NULL)
                ptr->invalid=1;
            else
            {
                int f=0;
                vector<point*> ftrack;
                while(ptr1->v_jog==NULL && ptr1->y<((top->y)-1))
                {
                    if(ptr1->net==0)
                    {
                        ftrack.push_back(ptr1);
                        f=1;
                    }
                    ptr1=ptr1->up;
                }
                if(f==1)
                {
                    if(ftrack.size()>2)
                        ptr->y_up=ftrack[(ftrack.size()/2)]->y;
                    else
                        ptr->y_up=ftrack[0]->y;
                    f=0;
                }
                else
                {
                    ptr->invalid=1;
                }
            }
        }

        if(ptr->invalid!=1)
            raise_low.push_back(ptr);
        
    }

    for(int i=0;i<raise_low.size();i++)
    {
        int ov_flag=0;
        for(int j=i+1;j<raise_low.size();j++)
        {
            if(raise_low[i]->net_value==raise_low[j]->net_value)
            {
                // not possible
            }
            else if(raise_low[i]->y_low>raise_low[j]->y_low && raise_low[i]->y_low<raise_low[j]->y_up)
            {
                ov_flag=1;
            }
            else if(raise_low[i]->y_up>raise_low[j]->y_low && raise_low[i]->y_up<raise_low[j]->y_up)
            {
                ov_flag=1;
            }
            else if(raise_low[j]->y_low>raise_low[i]->y_low && raise_low[j]->y_up<raise_low[i]->y_up)
            {
                ov_flag=1;
            }
            else if(raise_low[j]->y_low==raise_low[i]->y_low || raise_low[j]->y_up==raise_low[i]->y_up)
            {
                ov_flag=1;
            }
            else if(raise_low[j]->y_low==raise_low[i]->y_up || raise_low[j]->y_up==raise_low[i]->y_low)
            {
                ov_flag=1;
            }

            if(ov_flag==1)
            {
                raise_low[i]->overlapped.push_back(raise_low[j]);
                raise_low[j]->overlapped.push_back(raise_low[i]);
                ov_flag=0;
            }
        }
    }

    sort(raise_low.begin(),raise_low.end(), compare_pin_dist);

    v_pattern.clear();
    for(int i=0;i<raise_low.size();i++)       // can be optimized but will become complicated
    {
        if(raise_low[i]->invalid==1)
            continue;
        if(raise_low[i]->overlapped.size()==0)
        {
            v_pattern.push_back({raise_low[i]->y_low,raise_low[i]->y_up});
        }
        else 
        {
            v_pattern.push_back({raise_low[i]->y_low,raise_low[i]->y_up});

            for(int j=0;j<raise_low[i]->overlapped.size();j++)
            {
                raise_low[i]->overlapped[j]->invalid=1;
            }
        }
    }

    if(v_pattern.size()==0)
        return;
    sort(v_pattern.begin(),v_pattern.end(),compare_first);
    vector<pair<int,int>>:: iterator iter2=v_pattern.begin();
    for(point* ptr = bottom->up; ptr!=top; ptr=ptr->up)       // making connections in channel data structure
    {
        if(ptr->y==iter2->first)
        {
            point* ptr_end=ptr->up;
            while(ptr_end->y!=iter2->second)
                ptr_end=ptr_end->up;

            ptr->v_jog=ptr_end;
            ptr_end->down_jog=ptr;
            if(ptr->net==0)
                ptr->net=ptr_end->net;
            else if(ptr_end->net==0)
                ptr_end->net=ptr->net;
            iter2++;
            if(iter2==v_pattern.end())
                break;
            ptr=ptr_end->down;
        }
    }
}
void connections1(point* bottom, point* top, int index, int w, vector<int>:: iterator top_it, vector<int>:: iterator bottom_it)
{
    vector<int> h;
    map<int,vector<int>> H;

    for(point* ptr = bottom->up; ptr!=top; ptr=ptr->up)
    {
        h.push_back(ptr->net);

        map<int,vector<int>>::iterator it(H.find(ptr->net));
        if (it != H.end())
        {
            it->second.push_back(ptr->y);
        }
        else
        {
            H[ptr->net].push_back(ptr->y);
        }    
    }

    int t, b, t_track1, t_track2=0, b_track1, b_track2=0;   // track 1 is for net and track 2 is for empty track
    t=top->net;     // t is the top net
    b=bottom->net;      // b is the bottom net

    map<int,vector<int>>::iterator it0, it_top, it_bot;
    it0 = H.find(0);
    if(t!=0)
        it_top = H.find(t);
    if(b!=0)
        it_bot = H.find(b);
    if(it0 != H.end())
    {
        b_track2 = H[0][0];
        t_track2 = H[0][H[0].size()-1];
    }
    else
    {
        b_track2 = -1;
        t_track2 = -1;
    }
    if(t!=0 && it_top != H.end())
    {
        t_track1 = it_top->second[it_top->second.size()-1];
    }
    else
        t_track1=-1;
    if(b!=0 && it_bot != H.end())
    {
        b_track1 = it_bot->second[0];
    }
    else
        b_track1=-1;

    
    if(t==b && t!=0)
    {
        int flag=0;
        if(b_track2 == -1 && t_track2 == -1 && t_track1==-1 && b_track1==-1)
        {
            if(t==edge[index].first->net && t==edge[index].second->net)
            {
                int i=index+1;
                while(i<edge.size())
                {
                    if(t==edge[i].first->net || t==edge[i].second->net)
                    {
                        flag=1;
                        break;
                    }
                    i++;
                }
                if(flag==0)
                {
                    bottom->v_jog=top;
                    top->down_jog=bottom;
                }
                else
                {
                    // need 1 extra track step e
                }
            }
        }
    }

    if(t==b && t!=0)
    {
        int flag=0;
        if(b_track2 == -1 && t_track2 == -1 && t_track1!=-1 && b_track1!=-1)
        {
            if(t_track1==b_track1)
            {
                for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                {
                    if(t_track1==ptr->y)
                    {
                        bottom->v_jog=ptr;
                        ptr->down_jog=bottom;
                        ptr->net=bottom->net;
                        ptr->v_jog=top;
                        top->down_jog=ptr;
                        break;
                    }
                }
            }
            else
            {
                for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                {
                    if(b_track1==ptr->y)
                    {
                        bottom->v_jog=ptr;
                        ptr->down_jog=bottom;
                        ptr->net=bottom->net;
                    }
                    else if(t_track1==ptr->y)
                    {
                        ptr->v_jog=top;
                        ptr->net=top->net;
                        top->down_jog=ptr;
                        break;
                    }
                }
            }
            
        }
    }

    if(t==b && t!=0)
    {
        int flag=0;
        if(b_track2 != -1 && t_track2 != -1 && t_track1==-1 && b_track1==-1)
        {
            if(t_track2==b_track2)
            {
                for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                {
                    if(t_track2==ptr->y)
                    {
                        bottom->v_jog=ptr;
                        ptr->down_jog=bottom;
                        ptr->net=bottom->net;
                        ptr->v_jog=top;
                        top->down_jog=ptr;
                        break;
                    }
                }
            }
            else
            {
                for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                {
                    if(b_track2==ptr->y)
                    {
                        bottom->v_jog=ptr;
                        ptr->down_jog=bottom;
                        ptr->net=bottom->net;
                    }
                    else if(t_track2==ptr->y)
                    {
                        ptr->v_jog=top;
                        top->down_jog=ptr;
                        ptr->net=top->net;
                        break;
                    }
                }
            }
        }
        
    }

    if(t==b && t!=0)
    {
        int flag=0;
        if(b_track2 != -1 && t_track2 != -1 && t_track1!=-1 && b_track1!=-1)
        {
            if(b_track2<b_track1)
            {
                for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                {
                    if(b_track2==ptr->y)
                    {
                        bottom->v_jog=ptr;
                        ptr->down_jog=bottom;
                        ptr->net=bottom->net;
                        break;
                    }
                }
            }
            else    // b_track1<b_track2
            {
                for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                {
                    if(b_track1==ptr->y)
                    {
                        bottom->v_jog=ptr;
                        ptr->down_jog=bottom;
                        ptr->net=bottom->net;
                        break;
                    }
                }
            }

            if(t_track2<t_track1)
            {
                for(point* ptr=top; ptr!=bottom; ptr=ptr->down)
                {
                    if(t_track1==ptr->y)
                    {
                        ptr->v_jog=top;
                        top->down_jog=ptr;
                        ptr->net=top->net;
                        break;
                    }
                }                
            }
            else       // t_track1<t_track2
            {
                for(point* ptr=top; ptr!=bottom; ptr=ptr->down)
                {
                    if(t_track2==ptr->y)
                    {
                        ptr->v_jog=top;
                        top->down_jog=ptr;
                        ptr->net=top->net;
                        break;
                    }
                }
            }
        }  
    }

    // t=0 and b=0

    // t=0 and b!=0
    if(t==0 && b!=0)
    {
        if(b_track2 == -1 && t_track2 == -1 && b_track1==-1)
        {
            // step e
        }
        else if(b_track2 == -1 && t_track2 == -1 && b_track1!=-1)
        {
            for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
            {
                if(b_track1==ptr->y)
                {
                    bottom->v_jog=ptr;
                    ptr->down_jog=bottom;
                    ptr->net=bottom->net;
                    break;
                }
            }
        }
        else if(b_track2 != -1 && t_track2 != -1 && b_track1==-1)
        {
            for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
            {
                if(b_track2==ptr->y)
                {
                    bottom->v_jog=ptr;
                    ptr->down_jog=bottom;
                    ptr->net=bottom->net;
                    break;
                }
            }            
        }
        else if(b_track2 != -1 && t_track2 != -1 && b_track1!=-1)
        {
            if(b_track2<b_track1)
            {
                for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                {
                    if(b_track2==ptr->y)
                    {
                        bottom->v_jog=ptr;
                        ptr->down_jog=bottom;
                        ptr->net=bottom->net;
                        break;
                    }
                }
            }
            else // b_track1<b_track2
            {
                for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                {
                    if(b_track1==ptr->y)
                    {
                        bottom->v_jog=ptr;
                        ptr->down_jog=bottom;
                        ptr->net=bottom->net;
                        break;
                    }
                }                
            }
        }
    }

    // t!=0 and b=0
    if(t!=0 && b==0)
    {
        if(b_track2 == -1 && t_track2 == -1 && t_track1==-1)
        {
            // step e
        }
        else if(b_track2 == -1 && t_track2 == -1 && t_track1!=-1)
        {
            for(point* ptr=top; ptr!=bottom; ptr=ptr->down)
            {
                if(t_track1==ptr->y)
                {
                    ptr->v_jog=top;
                    top->down_jog=ptr;
                    ptr->net=top->net;
                    break;
                }
            }
        }
        else if(b_track2 != -1 && t_track2 != -1 && t_track1==-1)
        {
            for(point* ptr=top; ptr!=bottom; ptr=ptr->down)
            {
                if(t_track2==ptr->y)
                {
                    ptr->v_jog=top;
                    top->down_jog=ptr;
                    ptr->net=top->net;
                    break;
                }
            }            
        }
        else if(b_track2 != -1 && t_track2 != -1 && t_track1!=-1)
        {
            if(t_track2<t_track1)
            {
                for(point* ptr=top; ptr!=bottom; ptr=ptr->down)
                {
                    if(t_track1==ptr->y)
                    {
                        ptr->v_jog=top;
                        top->down_jog=ptr;
                        ptr->net=top->net;
                        break;
                    }
                }
            }
            else // t_track1<t_track2
            {
                for(point* ptr=top; ptr!=bottom; ptr=ptr->down)
                {
                    if(t_track2==ptr->y)
                    {
                        ptr->v_jog=top;
                        top->down_jog=ptr;
                        ptr->net=top->net;
                        break;
                    }
                }                
            }
        }        
    }

    if(t!=0 && b!=0 && t!=b)
    {
        if(b_track2 == -1 && t_track2 == -1 && b_track1==-1 && t_track1==-1)
        {
            // step e for b and t
        }
        if(b_track2 == -1 && t_track2 == -1 && b_track1!=-1 && t_track1==-1)
        {
            for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
            {
                if(b_track1==ptr->y)
                {
                    bottom->v_jog=ptr;
                    ptr->down_jog=bottom;
                    ptr->net=bottom->net;
                    break;
                }
            }
            // step e for t
        }
        if(b_track2 == -1 && t_track2 == -1 && b_track1==-1 && t_track1!=-1)
        {
            for(point* ptr=top; ptr!=bottom; ptr=ptr->down)
            {
                if(t_track1==ptr->y)
                {
                    ptr->v_jog=top;
                    top->down_jog=ptr;
                    ptr->net=top->net;
                    break;
                }
            }
            // step e for b
        }
        if(b_track2 == -1 && t_track2 == -1 && b_track1!=-1 && t_track1!=-1)
        {
            if(b_track1<t_track1)
            {
                for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                {
                    if(b_track1==ptr->y)
                    {
                        bottom->v_jog=ptr;
                        ptr->down_jog=bottom;
                        ptr->net=bottom->net;
                    }
                    else if(t_track1==ptr->y)
                    {
                        ptr->v_jog=top;
                        top->down_jog=ptr;
                        ptr->net=top->net;
                        break;
                    }
                }
            }
            else
            {
                if(b_track1<(w+1-t_track1))
                {
                    for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                    {
                        if(b_track1==ptr->y)
                        {
                            bottom->v_jog=ptr;
                            ptr->down_jog=bottom;
                            ptr->net=bottom->net;
                            break;
                        }
                    }
                    // step e - track for t
                }
                else
                {
                    for(point* ptr=top; ptr!=bottom; ptr=ptr->down)
                    {
                        if(t_track1==ptr->y)
                        {
                            ptr->v_jog=top;
                            top->down_jog=ptr;
                            ptr->net=top->net;
                            break;
                        }
                    }
                    // step e - track for b
                } 
            }
        }
        if(b_track2 != -1 && t_track2 != -1 && b_track1==-1 && t_track1==-1)
        {
            if(b_track2==t_track2)
            {
                if(b_track2<(w+1-t_track2))
                {
                    for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                    {
                        if(b_track2==ptr->y)
                        {
                            bottom->v_jog=ptr;
                            ptr->down_jog=bottom;
                            ptr->net=bottom->net;
                            break;
                        }
                    }
                    // step e for t
                }
                else
                {
                    for(point* ptr=top; ptr!=bottom; ptr=ptr->down)
                    {
                        if(t_track2==ptr->y)
                        {
                            ptr->v_jog=top;
                            top->down_jog=ptr;
                            ptr->net=top->net;
                            break;
                        }
                    }
                    // step e for b
                }
            }
            else    // b_track2!=t_track2
            {
                for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                {
                    if(b_track2==ptr->y)
                    {
                        bottom->v_jog=ptr;
                        ptr->down_jog=bottom;
                        ptr->net=bottom->net;
                    }
                    else if(t_track2==ptr->y)
                    {
                        ptr->v_jog=top;
                        top->down_jog=ptr;
                        ptr->net=top->net;
                        break;
                    }
                }
            }
        }
        if(b_track2 != -1 && t_track2 != -1 && b_track1!=-1 && t_track1==-1)
        {
            if(b_track2==t_track2)
            {
                if(t_track2>b_track1)
                {
                    for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                    {
                        if(b_track1==ptr->y)
                        {
                            bottom->v_jog=ptr;
                            ptr->down_jog=bottom;
                            ptr->net=bottom->net;
                        }
                        else if(t_track2==ptr->y)
                        {
                            ptr->v_jog=top;
                            top->down_jog=ptr;
                            ptr->net=top->net;
                            break;
                        }
                    }
                }
                else    // t_track2<b_track1
                {
                    if(b_track2<(w+1-b_track2))
                    {
                        for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                        {
                            if(b_track2==ptr->y)
                            {
                                bottom->v_jog=ptr;
                                ptr->down_jog=bottom;
                                ptr->net=bottom->net;
                                break;
                            }
                        }
                        // step e for t
                    }
                    else
                    {
                        for(point* ptr=top; ptr!=bottom; ptr=ptr->down)
                        {
                            if(b_track2==ptr->y)
                            {
                                ptr->v_jog=top;
                                top->down_jog=ptr;
                                ptr->net=top->net;
                                break;
                            }
                        }
                        // step e for b
                    }
                }
            }
            else    // b_track2!=t_track2
            {
                for(point* ptr=top; ptr!=bottom; ptr=ptr->down)
                {
                    if(t_track2==ptr->y)
                    {
                        ptr->v_jog=top;
                        top->down_jog=ptr;
                        ptr->net=top->net;
                        break;
                    }
                }
                if(b_track2<b_track1)
                {
                    for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                    {
                        if(b_track2==ptr->y)
                        {
                            bottom->v_jog=ptr;
                            ptr->down_jog=bottom;
                            ptr->net=bottom->net;
                            break;
                        }
                    }
                }
                else if(b_track2>b_track1)
                {
                    for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                    {
                        if(b_track1==ptr->y)
                        {
                            bottom->v_jog=ptr;
                            ptr->down_jog=bottom;
                            ptr->net=bottom->net;
                            break;
                        }
                    }
                }
            }
        }
        if(b_track2 != -1 && t_track2 != -1 && b_track1==-1 && t_track1!=-1)
        {
            if(b_track2==t_track2)
            {
                if(t_track1>b_track2)
                {
                    for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                    {
                        if(b_track2==ptr->y)
                        {
                            bottom->v_jog=ptr;
                            ptr->down_jog=bottom;
                            ptr->net=bottom->net;
                        }
                        else if(t_track1==ptr->y)
                        {
                            ptr->v_jog=top;
                            top->down_jog=ptr;
                            ptr->net=top->net;
                            break;
                        }
                    }
                }
                else    // t_track1<b_track2
                {
                    if(b_track2<(w+1-b_track2))
                    {
                        for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                        {
                            if(b_track2==ptr->y)
                            {
                                bottom->v_jog=ptr;
                                ptr->down_jog=bottom;
                                ptr->net=bottom->net;
                                break;
                            }
                        }
                        // step e for t
                    }
                    else
                    {
                        for(point* ptr=top; ptr!=bottom; ptr=ptr->down)
                        {
                            if(b_track2==ptr->y)
                            {
                                ptr->v_jog=top;
                                top->down_jog=ptr;
                                ptr->net=top->net;
                                break;
                            }
                        }
                        // step e for b
                    }
                }
            }
            else    // b_track2!=t_track2
            {
                for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                {
                    if(b_track2==ptr->y)
                    {
                        bottom->v_jog=ptr;
                        ptr->down_jog=bottom;
                        ptr->net=bottom->net;
                        break;
                    }
                }
                if(t_track2<t_track1)
                {
                    for(point* ptr=top; ptr!=bottom; ptr=ptr->down)
                    {
                        if(t_track1==ptr->y)
                        {
                            ptr->v_jog=top;
                            top->down_jog=ptr;
                            ptr->net=top->net;
                            break;
                        }
                    }
                }
                else if(t_track2>t_track1)
                {
                    for(point* ptr=top; ptr!=bottom; ptr=ptr->down)
                    {
                        if(t_track2==ptr->y)
                        {
                            ptr->v_jog=top;
                            top->down_jog=ptr;
                            ptr->net=top->net;
                            break;
                        }
                    }
                }
            }            
        }
        if(b_track2 != -1 && t_track2 != -1 && b_track1!=-1 && t_track1!=-1)
        {
            if(b_track2==t_track2)
            {
                if(b_track2>b_track1)
                {
                    for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                    {
                        if(b_track1==ptr->y)
                        {
                            bottom->v_jog=ptr;
                            ptr->down_jog=bottom;
                            ptr->net=bottom->net;
                        }
                    }

                    if(t_track1>b_track2)
                    {
                        for(point* ptr=top; ptr!=bottom; ptr=ptr->down)
                        {
                            if(t_track1==ptr->y)
                            {
                                ptr->v_jog=top;
                                top->down_jog=ptr;
                                ptr->net=top->net;
                            }
                        }
                    }
                    else    // t_track1<b_track2
                    {
                        for(point* ptr=top; ptr!=bottom; ptr=ptr->down)
                        {
                            if(b_track2==ptr->y)
                            {
                                ptr->v_jog=top;
                                top->down_jog=ptr;
                                ptr->net=top->net;
                            }
                        }
                    }
                }
                else if(b_track2<b_track1)
                {
                    if(t_track1>b_track2)
                    {
                        for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                        {
                            if(b_track2==ptr->y)
                            {
                                bottom->v_jog=ptr;
                                ptr->down_jog=bottom;
                                ptr->net=bottom->net;
                            }
                            else if(t_track1 == ptr->y)
                            {
                                ptr->v_jog=top;
                                top->down_jog=ptr;
                                ptr->net=top->net;
                                break;
                            }
                        }
                    }
                    else if(t_track1<b_track2)
                    {
                        if(b_track2<(w+1-b_track2))
                        {
                            for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                            {
                                if(b_track2==ptr->y)
                                {
                                    bottom->v_jog=ptr;
                                    ptr->down_jog=bottom;
                                    ptr->net=bottom->net;
                                    break;
                                }
                            }
                            // step e for t
                        }
                        else
                        {
                            for(point* ptr=top; ptr!=bottom; ptr=ptr->down)
                            {
                                if(b_track2==ptr->y)
                                {
                                    ptr->v_jog=top;
                                    top->down_jog=ptr;
                                    ptr->net=top->net;
                                    break;
                                }
                            }
                            // step e for b
                        }
                    }
                }
            }
            else    // b_track2!=t_track2
            {
                if(b_track2<b_track1)
                {
                    for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                    {
                        if(b_track2==ptr->y)
                        {
                            bottom->v_jog=ptr;
                            ptr->down_jog=bottom;
                            ptr->net=bottom->net;
                            break;
                        }
                    }
                }
                else
                {
                    for(point* ptr=bottom; ptr!=top; ptr=ptr->up)
                    {
                        if(b_track1==ptr->y)
                        {
                            bottom->v_jog=ptr;
                            ptr->down_jog=bottom;
                            ptr->net=bottom->net;
                            break;
                        }
                    }
                }

                if(t_track2<t_track1)
                {
                    for(point* ptr=top; ptr!=bottom; ptr=ptr->down)
                    {
                        if(t_track1==ptr->y)
                        {
                            ptr->v_jog=top;
                            top->down_jog=ptr;
                            ptr->net=top->net;
                            break;
                        }
                    }
                }
                else
                {
                    for(point* ptr=top; ptr!=bottom; ptr=ptr->down)
                    {
                        if(t_track2==ptr->y)
                        {
                            ptr->v_jog=top;
                            top->down_jog=ptr;
                            ptr->net=top->net;
                            break;
                        }
                    }
                }
            }
        }        
    }

}
void display(point* ptr)
{
    if(ptr==NULL)
        return;
    
    ptr->trav=1;
    r.push_back(ptr);
    
    if(ptr->v_jog!=NULL && ptr->v_jog->trav==0)
        display(ptr->v_jog);
    
    if(ptr->right!=NULL && ptr->right->net==ptr->net && ptr->right->trav==0)
        display(ptr->right);

    if(ptr->left!=NULL && ptr->left->net==ptr->net && ptr->left->trav==0)
        display(ptr->left);

    if(ptr->down_jog!=NULL && ptr->down_jog->trav==0)
        display(ptr->down_jog);   

}
int main( int argc, char* argv[])
{
    fstream input, output;
    input.open(argv[1],ios::in);
    output.open(argv[2],ios::out);

    if(argc!=3)
    {
        cout<<"Error in arguments"<<endl;
        exit(0);
    }

    if(!input.is_open())
    {
        cout<<"Error opening the file for taking input"<<endl;
        exit(0);
    } 
    if(!output.is_open())
    {
        cout<<"Error opening the output file"<<endl;
        exit(0);
    }

    string line;
    vector<vector<int>> input_data;
    while (getline(input,line)) 
    {
        istringstream is(line);
        input_data.push_back(vector<int>(istream_iterator<int>(is),istream_iterator<int>()));
    } 
    

    
    int w, mjl, snc;
    w=5;
    mjl=w/4;
    snc=20;
    vector<int> h;

    for(int j=0;j<input_data[0].size();j++)
    {
        point* ptr1 = new point;    // lower edge
        ptr1->r_flag=0;
        ptr1->v_jog=NULL;
        ptr1->down_jog=NULL;
        ptr1->right=NULL;
        ptr1->up=NULL;
        ptr1->down=NULL;
        ptr1->left=NULL;
        ptr1->trav=0;
        ptr1->x=j;
        ptr1->y=0;
        ptr1->net=input_data[1][j];
        bottom_edge.push_back(input_data[1][j]);    // pushing pin value

        point* ptr2 = new point;
        ptr2->r_flag=0;
        ptr2->v_jog=NULL;
        ptr2->down_jog=NULL;
        ptr2->right=NULL;
        ptr2->up=NULL;
        ptr2->down=NULL;
        ptr2->left=NULL;
        ptr2->trav=0;
        ptr2->x=j;
        ptr2->y=w+1;
        ptr2->net=input_data[0][j];     // upper edge
        top_edge.push_back(input_data[0][j]);   // pushing pin value

        edge.push_back({ptr1,ptr2});// ptr1 pointing to the bottom edge pin and ptr2 pointing to the top edge pin
        for(int k=1;k<=w;k++)
        {
            if(k==1)
            {
                ptr1->up = new point;
                ptr1->up->r_flag=0;
                ptr1->up->v_jog=NULL;
                ptr1->up->down=ptr1;
                ptr1=ptr1->up;
                ptr1->right=NULL;
                ptr1->left=NULL;
                ptr1->up=NULL;
                ptr1->down_jog=NULL;
                ptr1->trav=0;
                ptr1->net=0;
                ptr1->x=j;
                ptr1->y=k;
                continue;
            }
            if(k==w)
            {
                ptr1->up = new point;
                ptr1->up->r_flag=0;
                ptr1->up->v_jog=NULL;
                ptr1->up->down=ptr1;
                ptr1=ptr1->up;
                ptr1->right=NULL;
                ptr1->left=NULL;
                ptr1->down_jog=NULL;
                ptr1->trav=0;
                ptr1->net=0;
                ptr1->x=j;
                ptr1->y=k;
                ptr1->up=ptr2;
                ptr2->down=ptr1;    
                continue;
            }
            ptr1->up = new point;
            ptr1->up->r_flag=0;
            ptr1->up->v_jog=NULL;
            ptr1->up->down=ptr1;
            ptr1=ptr1->up;
            ptr1->right=NULL;
            ptr1->left=NULL;
            ptr1->up=NULL;
            ptr1->down_jog=NULL;
            ptr1->trav=0;
            ptr1->net=0;
            ptr1->x=j;
            ptr1->y=k;
        }

        if(j>0)
        {
            point* p1, *p2;
            p1=edge[j-1].first;    // p1 p2  
            p2=edge[j].first;
            for(int i=0;i<=w+1;i++)
            {
                p1->right=p2;
                p2->left=p1;

                p1=p1->up;
                p2=p2->up;
                
            }
        }
    }

    vector<point*> bottom_single, top_single;
    
    vector<int> topcheck, bottomcheck;

    point* top_coord, *bottom_coor;
    vector<int>:: iterator top_it, bottom_it;   // iterator for top edge and bottom edge pin values
    top_it=top_edge.begin();
    bottom_it=bottom_edge.begin(); 
    int fu,fl;
    fu=0;
    fl=0;
    for(int i=0;i<edge.size();i++)
    {
        bottom_it++;
        top_it++;
        if(edge[i].first->net!=0 && find(bottomcheck.begin(), bottomcheck.end(), edge[i].first->net)==bottomcheck.end() && find(bottom_it, bottom_edge.end(),edge[i].first->net)!=bottom_edge.end())
        {
            if(fl==0)
                bottom_coor=edge[i].first;
            bottomcheck.push_back(edge[i].first->net);
            addTracklow(edge[i].first, bottom_it);
            fl=1;
        }
        if(edge[i].second->net!=0 && find(topcheck.begin(), topcheck.end(), edge[i].second->net)==topcheck.end() && find(top_it, top_edge.end(),edge[i].second->net)!=top_edge.end())
        {
            if(fu==0)
                top_coord=edge[i].second;
            topcheck.push_back(edge[i].second->net);
            addTrackhigh(edge[i].second, top_it);
            fu=1;
        }
    }
    w=edge[0].second->y-edge[0].first->y;
    
    vector<vector<int>> tp, bt;

    int ymax;
    point* ptr;
    point* reach_up, *reach_low;
    vector<int> top_c, bot_c;

    ptr=bottom_coor->v_jog;
    ptr=ptr->up;

    ymax=ptr->y;
    reach_up=ptr;
    while(reach_up->left!=NULL)
        reach_up=reach_up->left;

    ptr=top_coord->down_jog;
    ptr=ptr->down;

    int ymin = ptr->y;
    reach_low=ptr;
    while(reach_low->left!=NULL)
        reach_low=reach_low->left;

    
    for(int i=0;i<edge.size();i++)
    {
        if(edge[i].first->net!=0 && edge[i].first->v_jog==NULL)
        {
            edge[i].first->v_jog=reach_up;
            reach_up->down_jog=edge[i].first;
            reach_up->net=edge[i].first->net;
        }
        else if(edge[i].first->net!=0 && edge[i].first->v_jog!=NULL && (find(top_edge.begin(), top_edge.end(),edge[i].first->net)!=top_edge.end()))
        {
            if(find(bot_c.begin(),bot_c.end(),edge[i].first->net)==bot_c.end())
            {
                point* p;
                p=edge[i].first->v_jog;
                p->v_jog=reach_up;
                reach_up->down_jog=p;
                reach_up->net=p->net;
                bot_c.push_back(reach_up->net);
            }
            
        }

        if(edge[i].second->net!=0 && edge[i].second->down_jog==NULL)
        {
            edge[i].second->down_jog=reach_low;
            reach_low->v_jog=edge[i].second;
            reach_low->net=edge[i].second->net;
        }
        else if(edge[i].second->net!=0 && edge[i].second->down_jog!=NULL && (find(bottom_edge.begin(), bottom_edge.end(),edge[i].second->net)!=bottom_edge.end()))
        {
            if(find(top_c.begin(),top_c.end(),edge[i].second->net)==top_c.end())
            {
                point* p;
                p=edge[i].second->down_jog;
                p->down_jog=reach_low;
                reach_low->v_jog=p;
                reach_low->net=p->net;
                top_c.push_back(reach_low->net);
            }
            
        }
        else if(find(bottom_edge.begin(), bottom_edge.end(),edge[i].second->net)==bottom_edge.end())
        {
            coverage.push_back(edge[i].second);
        }

        edge2.push_back({reach_up,reach_low});

        reach_up=reach_up->right;
        reach_low=reach_low->right;
    }

    top_edge.clear();
    bottom_edge.clear();
    
    for(int i=0;i<edge2.size();i++)
    {
        top_edge.push_back(edge2[i].second->net);

        save_edge2_u.push_back({edge2[i].second->v_jog,edge2[i].second->up});
        edge2[i].second->v_jog=NULL;
        edge2[i].second->up=NULL;

        bottom_edge.push_back(edge2[i].first->net);
        
        save_edge2_l.push_back({edge2[i].first->down_jog,edge2[i].first->down});
        edge2[i].first->down_jog=NULL;
        edge2[i].first->down=NULL;

    }
    w=edge2[0].second->y-edge2[0].first->y;
    top_it=top_edge.begin();
    bottom_it=bottom_edge.begin(); 
    for(int i=0;i<edge2.size();i++)
    {
        connections1(edge2[i].first,edge2[i].second, i,w, top_it, bottom_it);     // step a, i=x coordinate, w=channel width
        verticalConnections(edge2[i].first,edge2[i].second,i,w, top_it, bottom_it);  // step b, connections within the channel already made
        reduceRangeUNC(edge2[i].first,edge2[i].second,i,w, top_it, bottom_it);       // step c
        if((top_it+1)!=top_edge.end())
        {
            raiseLownets(edge2[i].first,edge2[i].second,i,w, top_it+1, bottom_it+1);
        }
        else
            raiseLownets(edge2[i].first,edge2[i].second,i,w, top_edge.end(), bottom_edge.end());
        if((edge2[i].first->net!=0 && edge2[i].first->v_jog==NULL) || (edge2[i].second->net!=0 && edge2[i].second->down_jog==NULL))
            addTrack(edge2[i].first,edge2[i].second,i,w, top_it+1, bottom_it+1);
        
        if((top_it+1)!=top_edge.end())
            extendNextCol(edge2[i].first,edge2[i].second,i,w, top_it+1, bottom_it+1);
        else
            extendNextCol(edge2[i].first,edge2[i].second,i,w, top_edge.end(), bottom_edge.end());

        top_it++;
        bottom_it++;
        v_pattern.clear();
        unc_net.clear();
    }


    tp.clear();
    bt.clear();
    bottomcheck.clear();
////////////////////////

    for(int i=0;i<edge2.size();i++)
    {
        edge2[i].second->v_jog=save_edge2_u[i].first;
        edge2[i].second->up=save_edge2_u[i].second;

        edge2[i].first->down_jog=save_edge2_l[i].first;
        edge2[i].first->down=save_edge2_l[i].second;

    }

    for(int j=0;j<edge.size();j++)
    {
        coverage.push_back(edge[j].first);
    }
///////////////////////
    for(int l=0;l<coverage.size();l++)
    {
        map<int,vector<point*>> horizontal, vertical;
        r.clear();
        vector<int> v1,v2;
        if(coverage[l]->net!=0 && find(bottomcheck.begin(),bottomcheck.end(),coverage[l]->net)==bottomcheck.end())
        {
            bottomcheck.push_back(coverage[l]->net);
            display(coverage[l]);
        }
        else
            continue;
        v1.push_back(r[0]->net);    // horizontal
        v2.push_back(r[0]->net);    // vertical

        for(int i=0;i<r.size();i++)
        {
            map<int,vector<point*>>::iterator it(horizontal.find(r[i]->y));
            if (it != horizontal.end())
            {
                it->second.push_back(r[i]);
            }
            else
            {
                horizontal[r[i]->y].push_back(r[i]);
            }

            map<int,vector<point*>>::iterator it1(vertical.find(r[i]->x));
            if (it1 != vertical.end())
            {
                it1->second.push_back(r[i]);
            }
            else
            {
                vertical[r[i]->x].push_back(r[i]);
            }
        }
        
        map<int,vector<point*>>::iterator iter;
        for(iter=horizontal.begin();iter!=horizontal.end();iter++)
        {
            if(iter->second.size()>1)
            {
                sort(iter->second.begin(), iter->second.end(), compare_coor_x);
            }
        }
        for(iter=vertical.begin();iter!=vertical.end();iter++)
        {
            if(iter->second.size()>1)
            {
                sort(iter->second.begin(), iter->second.end(), compare_coor_y);
            }
        }
        for(iter=horizontal.begin();iter!=horizontal.end();iter++)
        {
            if(iter->second.size()==1)
                continue;
            
            if(iter->second[0]->right!=iter->second[1])
                continue;
            int s=iter->second[0]->x;
            int l=s;
            for(int i=1;i<iter->second.size();i++)
            {
                if(iter->second[i]->x<s)
                    s=iter->second[i]->x;

                if(iter->second[i]->x>l)
                    l=iter->second[i]->x;
            }
            v1.push_back(s);
            v1.push_back(iter->first);
            v1.push_back(l);
        }
        bt.push_back(v1);
        
       for(iter=vertical.begin();iter!=vertical.end();iter++)
        {
            if(iter->second.size()==1)
                continue;

            int s;
            int l;

            for(int k=0;k<iter->second.size()-1;k++)
            {
                s=iter->second[k]->y;
                while(iter->second[k]->v_jog!=NULL && iter->second[k]->v_jog==iter->second[k+1] && iter->second[k+1]->down_jog==iter->second[k])
                    k++;
                l=iter->second[k]->y;
                if(l!=s)
                {
                    v2.push_back(iter->first);
                    v2.push_back(s);
                    v2.push_back(l);
                    
                }
                
            }
        }
        bt.push_back(v2); 
    }

    for(int i=0;i<bt.size();i=i+2)
    {
        output<<".begin"<<" "<<bt[i][0]<<endl;
        for(int j=1;j<bt[i].size();j=j+3)
        {
            output<<".H"<<" "<<bt[i][j]<<" "<<bt[i][j+1]<<" "<<bt[i][j+2]<<endl;
        }
        for(int j=1;j<bt[i+1].size();j=j+3)
        {
            output<<".V"<<" "<<bt[i+1][j]<<" "<<bt[i+1][j+1]<<" "<<bt[i+1][j+2]<<endl;
        }
        output<<".end"<<endl;

    }
    input.close();
    output.close();
    
}