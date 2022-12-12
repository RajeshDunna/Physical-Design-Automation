#include<iostream>
#include<fstream>
#include<map>
#include<unordered_map>
#include<algorithm>
#include <limits>
#include"cell.h"

using namespace std;

cell* top=NULL;
pair<int,int> die_size, die_rows;
map<string, vector<int>> terminals;
vector<cell*> cell_list;
vector<vector<cell*>> cell_list1;
vector<cell*> chip1, chip2;
vector<pair<string, vector<int>>> legal_cells, terminal1;

void parseData(fstream&);
void abacus(vector<cell*>&, int);
void placeRow(row* &, cell*);
void addCell(cluster*, cell*);
void collapse(cluster*,row* &);
void addCluster(cluster*, cluster*);
void computeOptimalx(vector<cluster*>&);

bool mycompare(cell* beg, cell* end)
{
    return beg->xl<end->xl;
}
bool compare_alpha(pair<string, vector<int>> p1, pair<string, vector<int>> p2)
{
    return stoi(p1.first.erase(0,1))<stoi(p2.first.erase(0,1));
}
bool compare_area(cell* start_cell, cell* end_cell)
{
    return start_cell->overlap_area>end_cell->overlap_area;
}

int main(int argc, char* argv[])
{
    fstream inFile, outFile;
    inFile.open(argv[1],ios::in);
    outFile.open(argv[2],ios::out);
    if(!inFile.is_open())
    {
        cout<<"Error opening the file for taking input"<<endl;
        exit(0);
    }  

    parseData(inFile);

    abacus(chip1, 0);
    abacus(chip2, 1);
    sort(legal_cells.begin(), legal_cells.end(), compare_alpha);
    for(int i=0;i<legal_cells.size();i++)
    {
        outFile<<legal_cells[i].first<<" "<<legal_cells[i].second[0]<<" "<<legal_cells[i].second[1]<<" "<<legal_cells[i].second[2]<<endl;
    }
    inFile.close();
    outFile.close();
    return 0;
}

void parseData(fstream& inFile)
{
    string str;
    inFile>>str;
    int chip_x, chip_y;
    inFile>>chip_x;
    inFile>>chip_y;
    die_size.first=chip_x;
    die_size.second=chip_y;

    inFile>>str;
    int row_height, rows;
    inFile>>row_height;
    inFile>>rows;  
    die_rows.first=row_height;
    die_rows.second=rows;

    inFile>>str;
    int n_terminals;
    inFile>>n_terminals;
    for(int i=0;i<n_terminals;i++)
    {
        int y;
        vector<int> v;
        inFile>>str;
        int xl, yl,xu, yu, width, height;
        inFile>>xl;
        inFile>>yl;
        y=yl;
        if(yl%die_rows.first!=0)
        {
            yl = (yl/die_rows.first)*die_rows.first;
        }
        int index = yl/die_rows.first;
        v.push_back(index);
        v.push_back(xl);  
        v.push_back(yl);
        inFile>>width;
        xu=xl+width;
        v.push_back(xu);
        inFile>>height;
        yu=y+height;
        if(yu%die_rows.first!=0)
        {
            yu = ((yu/die_rows.first)+1)*die_rows.first;
        }
        v.push_back(yu);
        terminals.insert({str,v});
    }

    int n_cells;
    inFile>>str;
    inFile>>n_cells;      //n = number of cells
    for(int i=0;i<n_cells;i++)
    {
        cell* ptr = new cell;
        inFile>>ptr->name;
        inFile>>ptr->xl;
        inFile>>ptr->yl;
        inFile>>ptr->width;
        inFile>>ptr->height;
        ptr->xu=ptr->xl+ptr->width;
        ptr->yu=ptr->yl+ptr->height;
        ptr->overlap_area=0;
        cell_list.push_back(ptr);        
    }

    for(int i=0;i<cell_list.size();i++)
    {
        int flag=0;
        vector<cell*> v;
        v.push_back(cell_list[i]);
        if(i==0)
        {
            cell_list1.push_back(v);
            continue;
        }
        
        for(int j=cell_list1.size()-1;j>=0;j--)
        {
            for(int k=cell_list1[j].size()-1;k>=0;k--)
            {
                // compare cell_list[i] and cell_list[j][k]
                int xl=-1, xu=-1, yl=-1, yu=-1;
                int area;
                cell* ptr1, *ptr2;

                // ptr1 with x lower
                if(cell_list[i]->xl<=cell_list1[j][k]->xl)
                {
                    ptr1=cell_list[i];
                    ptr2=cell_list1[j][k];
                }
                else if(cell_list[i]->xl>cell_list1[j][k]->xl)
                {
                    ptr1=cell_list1[j][k];
                    ptr2=cell_list[i];
                }
                //ptr1 has lower xl value
                if(ptr2->xl>=ptr1->xl && ptr2->xl<ptr1->xu)
                {
                    xl=ptr2->xl;
                    if(ptr2->xu<=ptr1->xu)
                        xu=ptr2->xu;
                    else
                        xu=ptr1->xu;
                }

                // ptr1 with y lower
                if(cell_list[i]->yl<=cell_list1[j][k]->yl)
                {
                    ptr1=cell_list[i];
                    ptr2=cell_list1[j][k];
                }
                else if(cell_list[i]->yl>cell_list1[j][k]->yl)
                {
                    ptr1=cell_list1[j][k];
                    ptr2=cell_list[i];
                }
                // ptr1 has lower yl value

                if(ptr2->yl>=ptr1->yl && ptr2->yl<ptr1->yu)
                {
                    yl=ptr2->yl;
                    if(ptr2->yu<=ptr1->yu)
                        yu=ptr2->yu;
                    else
                        yu=ptr1->yu;
                }

                if(xl==-1 || xu==-1 || yl==-1 || yu==-1)
                {
                    xl=-1;
                    xu=-1;
                    yl=-1;
                    yu=-1;
                }
                else
                {
                    // cell_list[i] overlapped with cell_list[j][k]
                    area=(xu-xl)*(yu-yl);   // Valid area 
                    cell_list1[j][k]->overlapped_cell.push_back(cell_list[i]);
                    cell_list[i]->overlapped_cell.push_back(cell_list1[j][k]);
                    cell_list[i]->overlap_area+=area;
                    cell_list1[j][k]->overlap_area+=area;
                    cell_list1[j].insert(cell_list1[j].end(), cell_list[i]);
                    flag=1;
                    goto another;
                }
            }
        }
        another:
        if(flag==0)
            cell_list1.push_back(v);
    }

    for(int i=0;i<cell_list1.size();i++)
    {
        sort(cell_list1[i].begin(), cell_list1[i].end(),compare_area);
    }

    for(int i=0;i<cell_list1.size();i++)
    {
        vector<cell*> v1, v2;
        for(int j=0;j<cell_list1[i].size();j++)
        {
            if(j==0)
            {
                v1.push_back(cell_list1[i][j]);
                for(int k=0;k<cell_list1[i][j]->overlapped_cell.size();k++)
                {
                    v2.push_back(cell_list1[i][j]->overlapped_cell[k]);
                }
                continue;
            }

            if((find(v1.begin(), v1.end(),cell_list1[i][j])==v1.end()) && (find(v2.begin(), v2.end(),cell_list1[i][j])==v2.end()))
            {
                v1.push_back(cell_list1[i][j]);
                for(int k=0;k<cell_list1[i][j]->overlapped_cell.size();k++)
                {
                    if((find(v2.begin(), v2.end(),cell_list1[i][j]->overlapped_cell[k])==v2.end()) && (find(v1.begin(), v1.end(),cell_list1[i][j]->overlapped_cell[k])==v1.end()))
                        v2.push_back(cell_list1[i][j]->overlapped_cell[k]);
                }
            }  
        }
        if(v1.size()<v2.size())
        {
            if((v1.size()+v2.size())%2==0)
            {
                while(v1.size()!=v2.size())
                {
                    v1.push_back(v2[v2.size()-1]);
                    v2.pop_back();
                }
            }
            else
            {
                while((v1.size()+1)!=v2.size())
                {
                    v1.push_back(v2[v2.size()-1]);
                    v2.pop_back();
                }
            }
        }
        else if(v1.size()>v2.size())
        {
            if((v1.size()+v2.size())%2==0)
            {
                while(v1.size()!=v2.size())
                {
                    v2.push_back(v1[v1.size()-1]);
                    v1.pop_back();
                }
            }
            else
            {
                while((v2.size()+1)!=v1.size())
                {
                    v2.push_back(v1[v1.size()-1]);
                    v1.pop_back();
                }
            }
        }

        if(v1.size()!=0)
            chip1.insert(chip1.end(),v1.begin(), v1.end());
        if(v2.size()!=0)
            chip2.insert(chip2.end(),v2.begin(),v2.end());
    }

}
void abacus(vector<cell*>& cell_list, int index)
{
    sort(cell_list.begin(),cell_list.end(),mycompare);

    int r = die_rows.second;

    vector<row*> row_ds[die_rows.second];
    
    for(int i=0;i<die_rows.second;i++)
    {
        row_ds[i].push_back(new row(0, i*die_rows.first, die_size.first, die_rows.first, i));
    }

    map<string, vector<int>>:: iterator it = terminals.begin();
    for(int i=0;i<terminals.size();i++)
    {
        int index1, index2;
        index1 = it->second[0];
        index2 = it->second[4]/die_rows.first;
        for(int j=index1;j<index2;j++)
        {
            int terminal_overlap=-1;
            search_again:
            for(int k=0;k<row_ds[j].size();k++)
            {
                if(terminal_overlap!=1)
                {
                    if(it->second[1]==row_ds[j][k]->xl && it->second[3]<row_ds[j][k]->xu)
                    {
                        row_ds[j][k]->xl=it->second[3];
                        row_ds[j][k]->vacant_width=row_ds[j][k]->xu-row_ds[j][k]->xl;
    
                        terminal_overlap=0;
                        break;
                    }
                    else if(it->second[1]>row_ds[j][k]->xl && it->second[3]==row_ds[j][k]->xu)
                    {
                        row_ds[j][k]->xu=it->second[1];
                        row_ds[j][k]->vacant_width=row_ds[j][k]->xu-row_ds[j][k]->xl;
    
                        terminal_overlap=0;
                        break;
                    }
                    else if(it->second[1]>=row_ds[j][k]->xl && it->second[3]<=row_ds[j][k]->xu)
                    {
                        if(it->second[1]==row_ds[j][k]->xl && it->second[3]==row_ds[j][k]->xu)
                        {
                            row_ds[j].erase(row_ds[j].begin()+k);
                            terminal_overlap=0;
                            break;
                        }

                        row* ptr = new row(row_ds[j][k]);

                        row_ds[j][k]->xu=it->second[1];
                        row_ds[j][k]->vacant_width=row_ds[j][k]->xu-row_ds[j][k]->xl;

                        ptr->xl=it->second[3];
                        ptr->vacant_width=ptr->xu-ptr->xl;
                        if(k==row_ds[j].size()-1)
                            row_ds[j].push_back(ptr);
                        else
                            row_ds[j].insert(row_ds[j].begin()+k+1,ptr);

                        terminal_overlap=0;
                        break;
                    }
                }
                else if(terminal_overlap==1)
                {
                    if(it->second[1]>=row_ds[j][k]->xl && it->second[1]<row_ds[j][k]->xu)
                    {
                        if(it->second[1]==row_ds[j][k]->xl)
                        {
                            row_ds[j].erase(row_ds[j].begin()+k);
                            terminal_overlap=0;
                            break;
                        }
                        row_ds[j][k]->xu=it->second[1];
                        row_ds[j][k]->vacant_width=row_ds[j][k]->xu-row_ds[j][k]->xl;

                        terminal_overlap=0;
                        break;
                    }
                    if(it->second[3]>row_ds[j][k]->xl && it->second[3]<=row_ds[j][k]->xu)
                    {
                        if(it->second[3]==row_ds[j][k]->xu)
                        {
                            row_ds[j].erase(row_ds[j].begin()+k);
                            terminal_overlap=0;
                            break;
                        }
                        row_ds[j][k]->xl=it->second[3];
                        row_ds[j][k]->vacant_width=row_ds[j][k]->xu-row_ds[j][k]->xl;

                        terminal_overlap=0;
                        break;
                    }                  
                } 
            }
            if(terminal_overlap==-1)
            {
                terminal_overlap=1;
                goto search_again;
            }
        }
        it++;
    }

    for(int i=0;i<cell_list.size();i++)
    {
        relocating_x:
        cell* p = cell_list[i];
        int temp = cell_list[i]->yl/die_rows.first;
        int flag=0;
        int row=0;
        int count=0;
        int shift=-1;
        int dist;
        find_row:
        if(cell_list[i]->yl-(temp*die_rows.first) == 0)
        {
            if(flag==-1 && count%2==1)
                row=row+count;
            else if(flag==-1 && count%2==0)
                row=row-count;
            else
                row=temp;
        }
        else if(abs(cell_list[i]->yl-(temp*die_rows.first)) <= abs(cell_list[i]->yl-((temp+1)*die_rows.first)))
        {
            if(flag==-1 && count%2==1)
                row=row+count;
            else if(flag==-1 && count%2==0)
                row=row-count;
            else
                row = temp;
        }
        else if(abs(cell_list[i]->yl-temp*die_rows.first) > abs(cell_list[i]->yl-(temp+1)*die_rows.first))
        {
            if(flag==-1 && count%2==1)
                row=row-count;
            else if(flag==-1 && count%2==0)
                row=row+count;
            else
                row = temp+1;
        }

        if(row>=0 && row<die_rows.second)
        {
            for(int j=0;j<row_ds[row].size();j++)
            {
                if(cell_list[i]->xl>=row_ds[row][j]->xl && cell_list[i]->xl<row_ds[row][j]->xu)
                {
                    if(row_ds[row][j]->vacant_width>=cell_list[i]->width)
                    {
                        flag=1;
                        cell_list[i]->yl=row*die_rows.first;
                        cell_list[i]->chip_index=index;
                        placeRow(row_ds[row][j],cell_list[i]);
                        row_ds[row][j]->vacant_width=row_ds[row][j]->vacant_width-cell_list[i]->width;
                    }
                    else 
                        flag=-1;
                    
                    break;
                }
            }
        }
        // if(row<((-1)*die_rows.second+1))
        // {
        //     cout<<"error"<<endl;
        //     exit(0);
        // }
        if(flag==0)
        {
            if(cell_list[i]->xl<row_ds[row][0]->xl)
            {
                int width=row_ds[row][0]->xu-row_ds[row][0]->xl;
                if(cell_list[i]->width<=width)
                {
                    shift=1;
                    dist=row_ds[row][0]->xl-cell_list[i]->xl;
                }
                else 
                    shift=0;
            }
            else if(cell_list[i]->xl>row_ds[row][row_ds[row].size()-1]->xu)
            {
                //Do nothing
            }
            else
            {
                for(int j=0;j<row_ds[row].size()-1;j++)
                {
                    if(cell_list[i]->xl>=row_ds[row][j]->xu && cell_list[i]->xl<row_ds[row][j+1]->xl)
                    {
                        int width=row_ds[row][j+1]->xu-row_ds[row][j+1]->xl;
                        if(cell_list[i]->width<=width)
                        {
                            shift=1;
                            dist=row_ds[row][j+1]->xl-cell_list[i]->xl;
                        }
                        else 
                            shift=0;
                    }
                }
            }       
            flag=-1;    // Blocked by terminal
        }
        if(flag==-1)
        {
            if(shift==1)
            {
                if(abs(cell_list[i]->yl-row*die_rows.first)> dist)
                {
                    cell_list[i]->xl=cell_list[i]->xl + dist;
                    cell *ptr=cell_list[i];
                    cell_list.erase(cell_list.begin()+i);
                    int check=0;
                    for(int k=i;k<cell_list.size();k++)
                    {
                        if(ptr->xl<=cell_list[k]->xl)
                        {
                            check=1;
                            cell_list.insert(cell_list.begin()+k, ptr);
                            break;
                        }
                    }
                    if(check==0)
                    {
                        cell_list.insert(cell_list.end(), ptr);
                    }
                    i--;
                    continue;
                    goto relocating_x;
                }
            }       
            count++;
            goto find_row;
        }
    }

    for(int i=0;i<die_rows.second;i++)
    {
        for(int j=0;j<row_ds[i].size();j++)
        {
            computeOptimalx(row_ds[i][j]->row_cluster);
        }
    }

    

    for(int i=0;i<cell_list.size();i++)
    {
        vector<int> v;
        v.push_back(cell_list[i]->xl);
        v.push_back(cell_list[i]->yl);
        v.push_back(cell_list[i]->chip_index);
        legal_cells.push_back({cell_list[i]->name,v});
    }
}

void computeOptimalx(vector<cluster*>& cluster_set)
{
    for(int i=0;i<cluster_set.size();i++)
    {
        int x=cluster_set[i]->xc;
        for(int j=0;j<cluster_set[i]->cluster_cell.size();j++)
        {
            cluster_set[i]->cluster_cell[j]->xl=x;
            x=x+cluster_set[i]->cluster_cell[j]->width;
        }
    }
}

void placeRow(row* & row_insert, cell* new_cell)
{
    if(row_insert->row_cluster.size()==0 || row_insert->row_cluster[row_insert->row_cluster.size()-1]->xc+row_insert->row_cluster[row_insert->row_cluster.size()-1]->wc <= new_cell->xl)
    {   // First cell or cell i does not overlap with the last cluster
        cluster* ptr = new cluster;
        ptr->xc=new_cell->xl;
        ptr->ec=0;
        ptr->wc=0;
        ptr->qc=0;
        //ptr->cluster_cell.push_back(new_cell);
        row_insert->row_cluster.push_back(ptr);
        addCell(ptr, new_cell);
        collapse(ptr,row_insert);
    }
    else
    {
        addCell(row_insert->row_cluster[row_insert->row_cluster.size()-1],new_cell);
        collapse(row_insert->row_cluster[row_insert->row_cluster.size()-1],row_insert);
    }
}

void addCell(cluster* ptr, cell* new_cell)
{
    ptr->cluster_cell.push_back(new_cell);
    ptr->ec=ptr->ec+1;  //ec=1
    ptr->qc=ptr->qc+(new_cell->xl-ptr->wc);
    ptr->wc=ptr->wc+new_cell->width;
}

void collapse(cluster* ptr, row* & row_insert)
{   // Place cluster c
    ptr->xc=ptr->qc/ptr->ec;

    // Limit positions between xmin and xmax-w(c)
    if(ptr->xc<row_insert->xl)
        ptr->xc=row_insert->xl;
    if(ptr->xc>row_insert->xu-ptr->wc)
        ptr->xc=row_insert->xu-ptr->wc;

    //Overlap between c and its predecessor c'
    if(row_insert->row_cluster.size()>1 && row_insert->row_cluster[row_insert->row_cluster.size()-2]->xc+row_insert->row_cluster[row_insert->row_cluster.size()-2]->wc > row_insert->row_cluster[row_insert->row_cluster.size()-1]->xc)
    {
        //Merge cluster c to c'
        addCluster(row_insert->row_cluster[row_insert->row_cluster.size()-2],row_insert->row_cluster[row_insert->row_cluster.size()-1]);
                   
        row_insert->row_cluster.erase(row_insert->row_cluster.begin()+row_insert->row_cluster.size()-1);

        collapse(row_insert->row_cluster[row_insert->row_cluster.size()-1],row_insert);
    }
}

void addCluster(cluster* cp, cluster* c)
{
    for(int i=0;i<c->cluster_cell.size();i++)
    {
        cp->cluster_cell.push_back(c->cluster_cell[i]);
    }
    cp->ec=cp->ec+c->ec;
    cp->qc=cp->qc+c->qc-(c->ec*cp->wc);
    cp->wc=cp->wc+c->wc;
}