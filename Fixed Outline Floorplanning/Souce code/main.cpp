#include<iostream>
#include<vector>
#include<stdlib.h>
#include<algorithm>
#include<fstream>
#include<map>
#include<cstring>
#include<time.h>
using namespace std;

int flag=0;
int y_upper=0;
int x_upper=0;

class node
{
    public:
    string name;
    int xl,yl,xu,yu, t_height;
    node* left, * right, *prev;
    int area;
    bool rotn;

    node(string,int,int,int,int); 
};

node :: node(string Name1, int Xl, int Yl, int width, int height)      //Parameterized constructor
{
    name=Name1;
    xl=Xl;
    yl=Yl;
    xu=xl+width;
    yu=yl+height;
    t_height=1;
    left=NULL;
    right=NULL;
    prev=NULL;
    area=(xu-xl)*(yu-yl);    
    rotn=false;
}

node* root=NULL;
node* position=NULL;
node* app_root=NULL;
vector<vector<int>> contourlist;    //using as pair
map<string,vector<int>> floorplan;
map<string,pair<int, int>> terminal;
vector<vector<string>> net;

void initialTree(vector<vector<int>> &, vector<pair<int, string>> &, int);
void display(node*);
void largestXUpper(node* );
void largestYUpper(node* );
void updateHeights(node* );
void updateContourDS(int , int , int , int, int , node*);
void findLocationHeight(node* , int, int);
int contourDS(int , int , int , int , node *);
void swapInput(vector<vector<int>> &);
vector<int> findBlock(vector<int>,vector<vector<int>> &, int);
void hpwlLength(int [][4]);
int calcLength(int [][4], int);
void readBlockData(fstream&,int&, int&, int&, int&, vector<pair<int,string>>&, vector<vector<int>>&, vector<vector<int>>&);
void readNetData(fstream&, int&);

bool sortColumn(const vector<int> &v1, const vector<int> &v2)
{
    return (v1[0]*v1[1])>(v2[0]*v2[1]);
}

void updateHeights(node* ptr)
{
    while(ptr!=NULL)
    {
        ptr->t_height++;
        ptr=ptr->prev;
    }
}
void initialTree(vector<vector<int>> &input, vector<pair<int, string>> &inputname, int x_out)
{
    for(int i=0;i<input.size();i++)
    {
        if(root==NULL)
        {
            root=new node(inputname[i].second,0,0,input[i][0],input[i][1]); //width=input[i][0], height=input[i][1]
            contourDS(0,0,input[i][0],input[i][1],NULL);
        }    
        else
        {
            node* ptr=root;
            app_root=root;
            flag=0;     // I want to find the location for new block addition
            findLocationHeight(ptr, x_out,input[i][0]);    //find location depending on the height each node of tree to form balanced tree
            int y;
            
            if(flag==1)
            {
                int x=position->xu;                                 // x coordinate of the new block
                y=contourDS(x,1,input[i][0],input[i][1],position);      // finding the coordinate of y
                position->left = new node(inputname[i].second,x,y,input[i][0],input[i][1]);
                position->left->prev=position;
                updateContourDS(x,y,input[i][0],input[i][1],1, position);
                
            } 
            else 
            {
                int x=position->xl;                                 // x coordinate of the new block
                y=contourDS(x,0,input[i][0],input[i][1],position);      // finding the coordinate of y
                position->right = new node(inputname[i].second,x,y,input[i][0],input[i][1]);
                position->right->prev=position;
                updateContourDS(x,y,input[i][0],input[i][1],0, position);
                
            }
                
        }
        
    }
}
int main(int argc, char* argv[])
{ 
    clock_t start,end;
    start = clock();

    int num_blocks;
    double alpha;
    vector<vector<int>> input;
    vector<pair<int, string>> inputname;
    vector<vector<int>> new_input, name_record;

    int x,y, terminal_no;

    fstream blockFile, netFile, outReport;
    alpha = stod(argv[1]); 
    blockFile.open(argv[2],ios::in);
    netFile.open(argv[3],ios::in);
    outReport.open(argv[4],ios::out);
    readBlockData(blockFile,x,y,num_blocks,terminal_no,inputname,input,name_record);

    int num_nets;

    readNetData(netFile,num_nets);

    int hpwl[num_nets][4];
    if(num_nets!=0)
    {
        memset(hpwl, -1, sizeof(hpwl[0][0]) * num_nets * 4);    //calculation after floorplan generation
    }

    sort(input.begin(),input.end(),sortColumn);     //Preprocessing on the input matrix

    swapInput(input);
    int count=0, x_vacant=x;
    vector<int> v = input[0];
    while(input.size()!=0)
    {
        if(count!=0)
        {
            v = findBlock(v,input,x_vacant);     // find the block that closely matches with the height of v
        }
        if(v[0]==0 && v[1]==0)
        {
            v = input[0];
            x_vacant=x;
        }
        new_input.push_back(v);
        x_vacant = x_vacant - v[0];
        vector<int> del;
        del.push_back(v[1]);
        del.push_back(v[0]);
        for(int i=0;i<input.size();i++)
        {
            if(v==input[i] || del==input[i])
            {
                input.erase(input.begin()+i);
                break;
            }

        }
        count++;
    }

    vector<pair<int,string>> block_name;
    for(int i=0;i<new_input.size();i++)
    {
        for(int j=0;j<name_record.size();j++)
        {
            if(new_input[i]==name_record[j])
            {
                block_name.push_back({0,inputname[j].second});
                name_record.erase(name_record.begin()+j);
                inputname.erase(inputname.begin()+j);
                break;
            }
            else if((new_input[i][0]==name_record[j][1]) && (new_input[i][1]==name_record[j][0]))
            {
                block_name.push_back({0,inputname[j].second});
                name_record.erase(name_record.begin()+j);
                inputname.erase(inputname.begin()+j);
                break;
            }
        }
    }    

    initialTree(new_input,block_name,x);     //Generating the tree

     display(root);      //floorplan is being generated during execution of this line

    int len;
    if(num_nets!=0)
    {
        hpwlLength(hpwl);
        len=calcLength(hpwl, num_nets);
    }
    else
        len=0;

    largestXUpper(root);
    
    largestYUpper(root);

    int cost=alpha*(x_upper*y_upper)+(1.0-alpha)*len;
    outReport<<cost<<endl;

    outReport<<len<<endl;
    outReport<<x_upper*y_upper<<endl;
    outReport<<x_upper<<" "<<y_upper<<endl;

    end = clock();
    double runtime=double(end-start)/CLOCKS_PER_SEC;
    outReport<<runtime<<endl;

    int size=floorplan.size();
    int check=0;
    for(auto fp : floorplan)
    {
        check++;
        outReport<<fp.first<<" ";
        outReport<<fp.second[0]<<" "<<fp.second[1]<<" "<<fp.second[2]<<" "<<fp.second[3];
         if(check<size)
             outReport<<endl;
    }

    blockFile.close();
    netFile.close();
    outReport.close();
    
    return 0;
}
int height(node *root)
{
    if(root==NULL)
        return 0; 
    int lh=height(root->left);
    int rh=height(root->right);
    return max(lh,rh)+1;
}
void largestXUpper(node* ptr)
{
    
    if(ptr==NULL)
        return;

    if(x_upper<ptr->xu)
        x_upper=ptr->xu;
    largestXUpper(ptr->left);
    largestXUpper(ptr->right);

}
void largestYUpper(node* ptr)
{
    
    if(ptr==NULL)
        return;

    if(y_upper<ptr->yu)
        y_upper=ptr->yu;
    largestYUpper(ptr->left);
    largestYUpper(ptr->right);

    
}
void display(node* ptr)
{
    if(ptr==NULL)
        return;

    vector<int> v;

    v.push_back(ptr->xl);
    v.push_back(ptr->yl);
    v.push_back(ptr->xu);
    v.push_back(ptr->yu);
    floorplan.insert(pair<string,vector<int>>(ptr->name,v));
    display(ptr->left);
    display(ptr->right);

}
void findLocationHeight(node* ptr, int x_out, int width)
{

    if(ptr->left==NULL)
    {
        if(((ptr->xu)+width)<=x_out)
        {
            position = ptr;
            flag=1;
            return;
        }
    }

    if(ptr->left!=NULL)
    {
        findLocationHeight(ptr->left, x_out, width);
    }
    if(ptr==app_root)
    {
        if(flag==1)
        {
            return;
        }
        else if(ptr->right==NULL)
        {
            position = ptr;        // condition as flag==0 while adding new block
            flag=0;
            return;
        }
        else
        {
            app_root = ptr->right;
            findLocationHeight(ptr->right, x_out, width);
        }
    }
    
}

int contourDS(int x, int left, int width, int height, node *ptr)
{
    int y=0;
    if(contourlist.size()==0)
    {
        contourlist.push_back({0,0});
        contourlist.push_back({0,height});
        contourlist.push_back({width,height});
        contourlist.push_back({width,0});
        return 0;
    }
    else if(left==1)
    {
        int count=0;
        bool present=false;
        vector<int> v;
        v.push_back(ptr->xu);
        v.push_back(ptr->yl);

        for(int i=0;i<contourlist.size();i++)
        {
            
            if(contourlist[i]==v)
            {
                present=true;
                break;
            }
            
        }

        for(int i=0;i<contourlist.size();i++)
        {
            if(contourlist[i][0]>=x && contourlist[i][0]<=(x+width-1))
            {              
                if(present == true && contourlist[i]!=v && count==0)
                    continue;
                
                if(contourlist[i][1]>=y)
                    y=contourlist[i][1];
                count++;
            }

        }

    }
    else
    {
        for(int i=0;i<contourlist.size();i++)
        {
            if(contourlist[i][0]>=x && contourlist[i][0]<=(x+width-1))
            {
                if(contourlist[i][1]>=y)
                    y=contourlist[i][1];
            }
        
        }

    }
    return y;
}

void updateContourDS(int x, int y, int width, int height, int left, node* ptr)
{
    bool present=false;
    bool parent_coor=false;
    vector<int> parent_right_coord;
    if(left==1)
    {
        parent_right_coord.push_back(ptr->xu);
        parent_right_coord.push_back(ptr->yl);
    }
    else
    {
        parent_right_coord.push_back(ptr->xl);
        parent_right_coord.push_back(ptr->yu);
    }
    vector<vector<int>> coord;
    vector<int> v;
    v.push_back(x);
    v.push_back(y+height);
    coord.push_back(v);     // pushed upper left coordinate

    v.clear();
    v.push_back(x+width);
    v.push_back(y+height);
    coord.push_back(v);     // pushed upper right coordinate 

    v.clear();
    v.push_back(x+width);
    v.push_back(y);         // pushed bottom right coordinate
    coord.push_back(v);
    v.clear();

    v.push_back(x);
    v.push_back(y);

    for(int i=0;i<coord.size();i++)
    {
        for(int j=0;j<contourlist.size();j++)
        {
            if(coord[i]==contourlist[j])
            {
                coord.erase(coord.begin()+i);
                i--;
            }
        }
    }
    for(int i=0;i<contourlist.size();i++)
    {
        if(contourlist[i]==parent_right_coord)
        {
            parent_coor = true;
            break;
        }
    }

    if(parent_coor==true && parent_right_coord==v)
    {
        for(int i=0;i<contourlist.size();i++)
        {
            if(contourlist[i]==v)
            {
                contourlist.erase(contourlist.begin()+i);
                for(int j=0;j<coord.size();j++)
                {
                    contourlist.insert(contourlist.begin()+i, coord[j]);
                    i++;
                }
                break;
                
            }
        }
    }
    else if(parent_coor==true && parent_right_coord!=v)
    {
        for(int i=0;i<contourlist.size();i++)
        {
            if(contourlist[i]==parent_right_coord)
            {
                while(contourlist[i][0]<=x+width-1)
                {
                    contourlist.erase(contourlist.begin()+i);
                }

                for(int j=0;j<coord.size();j++)
                {
                    contourlist.insert(contourlist.begin()+i, coord[j]);
                    i++;
                }
                break;
                
            }
        }
    }
    else if(parent_coor==false)
    {
        int flag=0;
        for(int i=0;i<contourlist.size();i++)
        {
            if(contourlist[i]==v)
            {
                flag=1;
                contourlist.erase(contourlist.begin()+i);
            
                while(contourlist[i][0]<=x+width-1)
                {
                    contourlist.erase(contourlist.begin()+i);
                }
                for(int j=0;j<coord.size();j++)
                {
                    contourlist.insert(contourlist.begin()+i, coord[j]);
                    i++;
                }
                break;
            }
        
        }
        if(flag==0)
        {
            for(int i=0;i<contourlist.size();i++)
            {
                if(contourlist[i][0]==x)
                {
                    i++;    
                    
                    while(contourlist[i][0]<=x+width-1)
                    {
                        contourlist.erase(contourlist.begin()+i);
                    }
                    for(int j=0;j<coord.size();j++)
                    {
                        contourlist.insert(contourlist.begin()+i, coord[j]);
                        i++;
                    }
                    break;
                }
                else if(contourlist[i][0]>x)
                {
                    while(contourlist[i][0]<=x+width-1)
                    {
                        contourlist.erase(contourlist.begin()+i);
                    }
                    coord.insert(coord.begin(),v);
                    for(int j=0;j<coord.size();j++)
                    {
                        contourlist.insert(contourlist.begin()+i, coord[j]);
                        i++;
                    }
                    break;                    
                }               
            }
        }
    }
}
void swapInput(vector<vector<int>> & input)
{
    for(int i=0;i<input.size();i++)
    {

        int temp;
        if(input[i][0]<input[i][1])
        {
            temp=input[i][0];
            input[i][0]=input[i][1];
            input[i][1]=temp;
        }
        
    }
}
vector<int> findBlock(vector<int> v,vector<vector<int>> & input, int x_vacant)
{
    vector<vector<int>> feasible_block;
    vector<int> height_matched_block;
    for(int i=0;i<input.size();i++)
    {
        if(input[i][0]<=x_vacant)
        {
            feasible_block.push_back(input[i]);
        }
        else if(input[i][1]<=x_vacant)
        {
            vector<int> temp;
            temp.push_back(input[i][1]);
            temp.push_back(input[i][0]);
            feasible_block.push_back(temp);
        }
    }       // By the end of this loop, I have all the blocks that can fit into the x_vacant

    if(feasible_block.size()==0)
    {
        height_matched_block.push_back(0);
        height_matched_block.push_back(0);
        return height_matched_block;
    }

    int height=v[1];
    int smallest=100000000;      // Assigned with the largest value in the set of blocks 

    // Now I will compare the heights of all blocks with height of V to get the block of closely matched height
    height_matched_block=feasible_block[0];
    for(int i=0;i<feasible_block.size();i++)
    {
        if(feasible_block[i][0]<=x_vacant && feasible_block[i][1]>x_vacant)
        {
            if(abs(feasible_block[i][1]-height)<=smallest)
            {
                if(abs(feasible_block[i][1]-height)==smallest)
                {
                    if((height_matched_block[0]*height_matched_block[1])<(feasible_block[i][0]*feasible_block[i][1]))
                    {
                        height_matched_block = feasible_block[i];
                    }
                    continue;
                }
                smallest=abs(feasible_block[i][1]-height);
                height_matched_block = feasible_block[i];
            }
        }
        else if(feasible_block[i][1]<=x_vacant && feasible_block[i][0]>x_vacant)
        {
            if(abs(feasible_block[i][0]-height)<=smallest)
            {
                if(abs(feasible_block[i][0]-height)==smallest)
                {
                    if((height_matched_block[0]*height_matched_block[1])<(feasible_block[i][0]*feasible_block[i][1]))
                    {
                        height_matched_block.clear();
                        height_matched_block.push_back(feasible_block[i][1]);
                        height_matched_block.push_back(feasible_block[i][0]);
                    }
                    continue;
                }
                smallest=abs(feasible_block[i][0]-height);
                height_matched_block.clear();
                height_matched_block.push_back(feasible_block[i][1]);
                height_matched_block.push_back(feasible_block[i][0]);
            }
        }
        else if(feasible_block[i][0]<=x_vacant && feasible_block[i][1]<=x_vacant)
        {
            if(abs(feasible_block[i][1]-height)<=smallest)
            {
                if(abs(feasible_block[i][1]-height)==smallest)
                {
                    if((height_matched_block[0]*height_matched_block[1])<(feasible_block[i][0]*feasible_block[i][1]))
                    {
                        height_matched_block = feasible_block[i];
                    }
                    goto label;
                }
                smallest=abs(feasible_block[i][1]-height);
                height_matched_block = feasible_block[i];
            }

            label:
            if(abs(feasible_block[i][0]-height)<=smallest)
            {
                if(abs(feasible_block[i][0]-height)==smallest)
                {
                    if((height_matched_block[0]*height_matched_block[1])<(feasible_block[i][0]*feasible_block[i][1]))
                    {
                        height_matched_block.clear();
                        height_matched_block.push_back(feasible_block[i][1]);
                        height_matched_block.push_back(feasible_block[i][0]);
                    }
                    continue;
                }
                smallest=abs(feasible_block[i][0]-height);
                height_matched_block.clear();
                height_matched_block.push_back(feasible_block[i][1]);
                height_matched_block.push_back(feasible_block[i][0]);
            }
        }
    }

    return height_matched_block;
}

void hpwlLength(int hpwl[][4])
{
    for(int i=0;i<net.size();i++)
    {
        for(auto it : floorplan)
        {
            if(find(net[i].begin(),net[i].end(),it.first)!=net[i].end())
            {
                int midx,midy;
                midx=(it.second[2]+it.second[0])/2;
                midy=(it.second[3]+it.second[1])/2;
                if(hpwl[i][0]==-1)
                {
                    hpwl[i][0]=midx;
                    hpwl[i][1]=midy;
                }
                else 
                {
                    if(hpwl[i][2]==-1)
                    {
                        if(midx<hpwl[i][0])
                        {
                            int temp;
                            temp = hpwl[i][0];
                            hpwl[i][0]=midx;
                            hpwl[i][2]=temp;
                        }
                        else if(midx>=hpwl[i][0])
                        {
                            hpwl[i][2]=midx;
                        }

                        if(midy<hpwl[i][1])
                        {
                            int temp;
                            temp = hpwl[i][1];
                            hpwl[i][1]=midy;
                            hpwl[i][3]=temp;
                        }
                        else if(midy>=hpwl[i][1])
                        {
                            hpwl[i][3]=midy;
                        }                    
                    }
                    else
                    {
                        if(midx<hpwl[i][0])
                        {
                            hpwl[i][0]=midx;
                        }
                        else if(midx>=hpwl[i][2])
                        {
                            hpwl[i][2]=midx;
                        }

                        if(midy<hpwl[i][1])
                        {
                            hpwl[i][1]=midy;
                        }
                        else if(midy>=hpwl[i][3])
                        {
                            hpwl[i][3]=midy;
                        }
                    }
                }
            }
        }
    }
    if(terminal.size()==0)
        return;
    // Now checking the terminals
    for(int i=0;i<net.size();i++)
    {
        for(auto it : terminal)
        {
            if(find(net[i].begin(),net[i].end(),it.first)!=net[i].end())
            {
                int midx,midy;
                midx=it.second.first;
                midy=it.second.second;
                if(hpwl[i][0]==-1)
                {
                    hpwl[i][0]=midx;
                    hpwl[i][1]=midy;
                }
                else 
                {
                    if(hpwl[i][2]==-1)
                    {
                        if(midx<hpwl[i][0])
                        {
                            int temp;
                            temp = hpwl[i][0];
                            hpwl[i][0]=midx;
                            hpwl[i][2]=temp;
                        }
                        else if(midx>=hpwl[i][0])
                        {
                            hpwl[i][2]=midx;
                        }

                        if(midy<hpwl[i][1])
                        {
                            int temp;
                            temp = hpwl[i][1];
                            hpwl[i][1]=midy;
                            hpwl[i][3]=temp;
                        }
                        else if(midy>=hpwl[i][1])
                        {
                            hpwl[i][3]=midy;
                        }                    
                    }
                    else
                    {
                        if(midx<hpwl[i][0])
                        {
                            hpwl[i][0]=midx;
                        }
                        else if(midx>=hpwl[i][2])
                        {
                            hpwl[i][2]=midx;
                        }

                        if(midy<hpwl[i][1])
                        {
                            hpwl[i][1]=midy;
                        }
                        else if(midy>=hpwl[i][3])
                        {
                            hpwl[i][3]=midy;
                        }
                    }
                }
            }
        }
    }
}
int calcLength(int hpwl[][4], int rows)
{
    int length=0;
    for(int i=0;i<rows;i++)
    {
        length=length+(hpwl[i][3]-hpwl[i][1])+(hpwl[i][2]-hpwl[i][0]);
    }
    return length;
}
void readBlockData(fstream& blockFile,int& x, int& y, int& num_blocks, int& terminal_no, vector<pair<int,string>>& inputname, vector<vector<int>>& input, vector<vector<int>>& name_record)
{
    string str, ign;
    blockFile>>ign;
    blockFile>>str;
    x=stod(str);
    blockFile>>str;
    y=stod(str);

    blockFile>>ign;
    blockFile>>str;
    num_blocks=stod(str);

    blockFile>>ign;
    blockFile>>str;
    terminal_no=stod(str);  
    
    for(int i=0;i<num_blocks;i++)
    {
        vector<int> dim;

        blockFile>>str;
        inputname.push_back({0,str});

        blockFile>>str;
        dim.push_back(stod(str));

        blockFile>>str;
        dim.push_back(stod(str));

        input.push_back(dim);
        name_record.push_back(dim);
    }

    if(terminal_no!=0)
    {
        string t_x,t_y;
        for(int i=0;i<terminal_no;i++)
        {   
            blockFile>>str;

            blockFile>>ign;

            blockFile>>t_x;

            blockFile>>t_y;

            terminal.insert(pair<string, pair<int,int>>(str,{stod(t_x),stod(t_y)}));
        }
    }
}
void readNetData(fstream& netFile, int& num_nets)
{
    string str, ign;
    netFile>>ign;
    netFile>>str;
    num_nets=stod(str);

    if(num_nets!=0)
    {
        for(int i=0;i<num_nets;i++)
        {
            netFile>>ign;
            netFile>>str;
            int net_deg=stod(str);
            vector<string> single_net;
            for(int j=0;j<net_deg;j++)
            {
                netFile>>str;
                single_net.push_back(str);
            }
            net.push_back(single_net);
        }
    }
}