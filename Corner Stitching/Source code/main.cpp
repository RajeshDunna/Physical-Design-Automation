#include<iostream>
#include<vector>
#include<stdlib.h>
#include<algorithm>
#include<fstream>
using namespace std;

int count_tiles=0;

class tile
{
    public:
    int index,xl,yl,xu,yu;
    tile *rot,*tor,*lob,*bol;
    bool status;

    tile(int,int,int,int,int,bool); 
    tile(tile* &); 
};
tile :: tile(int Index, int Xl,int Yl,int width,int height, bool stat)      //Parameterized constructor
{
    index=Index; xl=Xl; yl=Yl; xu=Xl+width; yu=Yl+height;
    rot=NULL; tor=NULL; lob=NULL; bol=NULL;
    status=stat;
}
tile :: tile(tile* &copyTile)       // Copy constructor
{
    index=copyTile->index; 
    xl = copyTile->xl; 
    yl = copyTile->yl; 
    xu = copyTile->xu; 
    yu = copyTile->yu;

    rot = copyTile->rot;
    tor = copyTile->tor;
    lob = copyTile->lob;
    bol = copyTile->bol;

    status=copyTile->status;       
}

tile* pointFind(int x, int y, tile* search)
{
    vertical_range:
    while((search->rot)!=NULL && y>=(search->yu))
        search=search->rot;

    while(y<(search->yl))
        search=search->lob;

    while(x<(search->xl))
    {
        search = search->bol;
        if(y>=(search->yu) || y<(search->yl))
        goto vertical_range;
    }

    while((search->tor)!=NULL && x>=(search->xu))
    {
        search=search->tor;
        if(y>=(search->yu) || y<(search->yl))
        goto vertical_range;
    }
    return search;
}

pair<int,int> findRightNeighbours(tile* new_tile, int yl)        
{
    tile* ptr=new_tile->tor;            //ptr is the topmost (first) right neighbour
    if(ptr==NULL)
        return {0,0};
    int block_count=0, vacant_count=0;

    do
    {
        if(ptr->status==true)
            block_count++;
        else
            vacant_count++;
        
        ptr=ptr->lob;
    }while(ptr!=NULL && ((ptr->yu)>yl));

    pair<int,int> p;
    p.first=block_count;
    p.second=vacant_count;
    return p;
}
pair<int,int> findTopNeighbours(tile* new_tile, int xl)
{
    tile* ptr=new_tile->rot;        // ptr is the first (rightmost) top neighbour 
    if(ptr==NULL)
        return {0,0};
    int block_count=0, vacant_count=0;
    //Counting top neighbours
    //ptr=start->rot; 
    
    do
    {
        if(ptr->status==true)
            block_count++;
        else
            vacant_count++;

        ptr=ptr->bol;
    }while(ptr!=NULL && ((ptr->xu)>xl));

    pair<int,int> p;
    p.first=block_count;
    p.second=vacant_count;
    return p;
}
pair<int,int> findBottomNeighbours(tile* new_tile, int xu)
{
    tile* ptr=new_tile->lob;            // ptr is the first (leftmost) bottom neighbour
    if(ptr==NULL)
        return {0,0};
    int block_count=0, vacant_count=0;
    //Counting bottom neighbours
    //ptr=start->lob;
    
    do
    {
        if(ptr->status==true)
            block_count++;
        else
            vacant_count++;

        ptr=ptr->tor;
    }while(ptr!=NULL && ((ptr->xl)<xu));

    pair<int,int> p;
    p.first=block_count;
    p.second=vacant_count;
    return p;
}
pair<int,int> findLeftNeighbours(tile* new_tile, int yu)
{
    tile* ptr=new_tile->bol;            // ptr is the first (bottom most) left neighbour 
    if(ptr==NULL)
        return {0,0};
    int block_count=0, vacant_count=0;
    //Counting left neighbour
    //ptr=start->bol;
    
    do
    {
        if(ptr->status==true)
            block_count++;
        else
            vacant_count++;
        
        ptr=ptr->rot;
    }while(ptr!=NULL && ((ptr->yl)<yu));

    pair<int,int> p;
    p.first=block_count;
    p.second=vacant_count;
    return p;
}
tile* rightEdgeStitch(tile* new_tile)
{
    //Considering that old_tile is at bottom and new_tile is at top 
    tile* ptr= new_tile->tor;
    if(ptr==NULL)
        return NULL;
    while(ptr!=NULL && ((ptr->yl)>=(new_tile->yl)))
    {
        ptr->bol=new_tile;
        ptr=ptr->lob;
    }
    return ptr;
}

void leftEdgeStitch(tile* old_tile, tile* new_tile)
{
    //Use when there is a horizontal split
    //Considering that old_tile is at bottom and new_tile is at top 
    tile* ptr=old_tile->bol;
    if(ptr==NULL)
    {
        new_tile->bol=NULL;
        return;
    }
    while((ptr->yu)<=(old_tile->yu))
        ptr=ptr->rot;

    new_tile->bol=ptr;
    while(ptr!=NULL && ((ptr->yu)<=(new_tile->yu)))
    {
        ptr->tor=new_tile;
        ptr=ptr->rot;
    }

}
tile* topEdgeStitch(tile* new_tile)
{

    tile* ptr = new_tile->rot;
    if(ptr==NULL)
        return NULL;
    while(ptr!=NULL && ((ptr->xl)>=(new_tile->xl)))
    {
        ptr->lob=new_tile;
        ptr=ptr->bol;
    } 
    return ptr;
}
void bottomEdgeStitch(tile* old_tile, tile* new_tile)
{
    tile* ptr = old_tile->lob;
    if(ptr==NULL)
    {
        new_tile->lob=NULL;
        return;
    }
    while((ptr->xu)<=(old_tile->xu))
        ptr=ptr->tor;
    new_tile->lob=ptr;

    while(ptr!=NULL && ((ptr->xu)<=(new_tile->xu)))
    {
        ptr->rot=new_tile;
        ptr=ptr->tor;
    }

}
void horizontalSplit(tile* old_tile,  int y)
{

    if((old_tile->yl)==y)       //If the horizontal split is already present then no need to do anything
        return;
    tile* new_tile = new tile(old_tile);
    count_tiles++;

    old_tile->yu=y;     //Making the existing tile as the bottom one
    new_tile->yl=y;     //Making the new tile as the upper one

    new_tile->lob = old_tile;
    old_tile->rot = new_tile;

    tile* ptr = rightEdgeStitch(new_tile);    //ptr pointing to nest to last neighbour of right edge

    old_tile->tor=ptr; 
    leftEdgeStitch(old_tile, new_tile);
    topEdgeStitch(new_tile);
}
void verticalSplit(tile* old_tile,  int x, int y)
{
    if(old_tile==NULL)
        return;
    if((old_tile->yu)>y)        // Base case for the recursion
        return;
    
    if((old_tile->xl)==x)       // If the vertical split is already present then no need to do anything
    {                           // It can execute only one time for the right edge and left edge
        tile* ptr=old_tile;
        old_tile=old_tile->bol;
        while((old_tile->rot)!=NULL && (old_tile->yu)<=(ptr->yu))    // For the left edge, the less than operator is not possible
            old_tile=old_tile->rot;

        verticalSplit(old_tile, x, y);
        return;                 // It can be executed only in the first call of verticalSplit
    }
    if(old_tile->xu==x)     //Alternative for below if statement, applicable to both left and right edges
    {
        verticalSplit(old_tile->rot, x, y);
        return;
    }     

    tile* new_tile = new tile(old_tile);
    count_tiles++;

    old_tile->xu=x;     //Making the existing tile as the right one
    new_tile->xl=x;     //Making the new tile as the left one

    new_tile->bol=old_tile;
    old_tile->tor=new_tile;

    tile* ptr = topEdgeStitch(new_tile);

    old_tile->rot=ptr;

    rightEdgeStitch(new_tile);
    bottomEdgeStitch(old_tile, new_tile);
    
    verticalSplit(old_tile->rot, x, y);        // Using recursion to split tiles along the left edge of area of interest
}
void mergeLeftEdgeStitch(tile* upper_tile, tile* bottom_tile)
{
    tile* ptr = upper_tile->bol;
    if(ptr==NULL)
        return;
    while(ptr!=NULL && ((ptr->yu)<=(upper_tile->yu)))
    {
        ptr->tor=bottom_tile;
        ptr=ptr->rot;
    }
}
void mergeRightEdgeStitch(tile* upper_tile, tile* bottom_tile)
{
    tile* ptr = upper_tile->tor;
    if(ptr==NULL)
        return;
    while((ptr->yl)>=(upper_tile->yl))
    {
        ptr->bol=bottom_tile;
        ptr=ptr->lob;
    }
}
void mergingTiles(tile* upper_tile, tile* bottom_tile, int y)
{

    if(upper_tile==NULL)
        return;

    if((upper_tile->yu)<=y)
    {
        if((upper_tile->status)==false && (bottom_tile->status)==false && (upper_tile->xl)==(bottom_tile->xl) && (upper_tile->xu)==(bottom_tile->xu))
        {
            bottom_tile->yu = upper_tile->yu;
            bottom_tile->rot = upper_tile->rot;
            bottom_tile->tor = upper_tile->tor;

            topEdgeStitch(bottom_tile);
            mergeLeftEdgeStitch(upper_tile,bottom_tile);
            mergeRightEdgeStitch(upper_tile,bottom_tile);
            upper_tile->rot=NULL;
            upper_tile->tor=NULL;
            upper_tile->bol=NULL;
            upper_tile->lob=NULL;
            count_tiles--;
            mergingTiles(bottom_tile->rot,bottom_tile,y);
            return;
        }
        mergingTiles(upper_tile->rot,upper_tile,y);       // The merge has not happened
    }
}
void mergingRightEdgeTiles(tile* upper_tile, tile* bottom_tile, int y)
{
    if(bottom_tile==NULL)
        return;

    if((bottom_tile->yl)>=y)
    {
        if((upper_tile->status)==false && (bottom_tile->status)==false && (upper_tile->xl)==(bottom_tile->xl) && (upper_tile->xu)==(bottom_tile->xu))
        {
            bottom_tile->yu = upper_tile->yu;
            bottom_tile->rot = upper_tile->rot;
            bottom_tile->tor = upper_tile->tor;
        
            topEdgeStitch(bottom_tile);
            mergeLeftEdgeStitch(upper_tile,bottom_tile);
            mergeRightEdgeStitch(upper_tile,bottom_tile);
            upper_tile->rot=NULL;
            upper_tile->tor=NULL;
            upper_tile->bol=NULL;
            upper_tile->lob=NULL;
            count_tiles--;
            mergingRightEdgeTiles(bottom_tile,bottom_tile->lob,y);
            return;
        }
        mergingRightEdgeTiles(bottom_tile,bottom_tile->lob,y);   
    }
}
void merge(tile* upper_tile, tile* bottom_tile)
{

    if(upper_tile==NULL || bottom_tile==NULL)
        return;

    if((upper_tile->status)==false && (bottom_tile->status)==false && (upper_tile->xl)==(bottom_tile->xl) && (upper_tile->xu)==(bottom_tile->xu))
    {
        bottom_tile->yu = upper_tile->yu;
        bottom_tile->rot = upper_tile->rot;
        bottom_tile->tor = upper_tile->tor;

        topEdgeStitch(bottom_tile);
        mergeLeftEdgeStitch(upper_tile,bottom_tile);
        mergeRightEdgeStitch(upper_tile,bottom_tile);
        upper_tile->rot=NULL;
        upper_tile->tor=NULL;
        upper_tile->bol=NULL;
        upper_tile->lob=NULL;
        count_tiles--;

    }
    
}

int out_width, out_height;

tile* createTile(vector<int> &single_block, tile* start)
{
//    tile* ptr = new tile;
//    ptr-
    if(start==NULL)
    {
        start = new tile(0,0,0,single_block[0],single_block[1],false);
        count_tiles++;
        return start;
    }
    tile* location = pointFind(single_block[1],single_block[2]+single_block[4],start);      //Finding the tile containing top edge
    if(single_block[2]+single_block[4] != out_height)
        horizontalSplit(location,single_block[2]+single_block[4]);      //Splitting the tile containing top edge
    
    location = pointFind(single_block[1],single_block[2],start);        //Finding the tile containing bottom edge
    if(single_block[2]!=0)
    horizontalSplit(location,single_block[2]);      //Splitting the tile containing bottom edge
    
    location = pointFind(single_block[1],single_block[2],start);        // Finding the bottom left corner for vertical splitting
    if(single_block[1]!=0)
    verticalSplit(location, single_block[1],single_block[2]+single_block[4]);      // Splitting the 1st bottom most space tile about left edge of area of interest

    location = pointFind(single_block[1],single_block[2],start); // Finding the bottom left corner

    if(location->bol!=NULL)
    {
        merge(location->bol,location->bol->lob);
        mergingTiles(location->bol->rot,location->bol,single_block[2]+single_block[4]);
        location=location->bol;
        while((location->yu)<(single_block[2]+single_block[4]))
            location=location->rot; 
        merge(location->rot,location);
    }
    

    location = pointFind(single_block[1]+single_block[3],single_block[2],start);    //Right bottom coordinate
    if(single_block[1]+single_block[3]!=out_width)
        verticalSplit(location, single_block[1]+single_block[3],single_block[2]+single_block[4]);   // Splitting the 1st bottom most space tile about right edge of area of interest

    location = pointFind(single_block[1],single_block[2],start);    //left bottom coordinate
    tile* final_tile = location; 
    while((location->yu)<(single_block[2]+single_block[4]))
        location=location->rot;                                     // Going to top most tile of area of interest

    if(location->tor!=NULL)
    {
        merge(location->tor->rot,location->tor);
        mergingRightEdgeTiles(location->tor,location->tor->lob,single_block[2]);
        location=location->tor;
        while((location->yl)>single_block[2])
            location=location->lob;
        merge(location,location->lob);
    }

    mergingTiles(final_tile->rot,final_tile,single_block[2]+single_block[4]);
    final_tile->status=true;
    final_tile->index=single_block[0];

    return start;

}
vector<vector<int>> findNeighbours(vector<vector<int>> &block, tile* start)
{

    vector<vector<int>> vec;
    for(int i=0;i<block.size();i++)
    {
        vector<int> single;
        tile* location = pointFind(block[i][1],block[i][2],start);
        
        pair<int,int> p, p_sum;
        p=findRightNeighbours(location, block[i][2]);    // yl = block[i][2]
        p_sum.first=p.first;
        p_sum.second=p.second;

        p=findTopNeighbours(location, block[i][1]);      // xl = block[i][1]
        p_sum.first=p_sum.first+p.first;
        p_sum.second=p_sum.second+p.second;
    
        p=findBottomNeighbours(location, block[i][1]+block[i][3]);   // xu = block[i][1]+block[i][3]
        p_sum.first=p_sum.first+p.first;
        p_sum.second=p_sum.second+p.second;
    
        p=findLeftNeighbours(location, block[i][2]+block[i][4]);     // yu = block[i][2]+block[i][4]
        p_sum.first=p_sum.first+p.first;
        p_sum.second=p_sum.second+p.second;

        single.push_back(block[i][0]);
        single.push_back(p_sum.first);
        single.push_back(p_sum.second);
        
        vec.push_back(single);

    }

    return vec;

}
bool sortColumn(const vector<int> &v1, const vector<int> &v2)
{
    return v1[0]<v2[0];
}


int main(int argc, char* argv[])
{  
    char c;
    vector<pair<int,int>> p;
    vector<vector<int>> block;
    tile *start=NULL;

    fstream inFile, outFile;
    
    if(argc!=3)
    {
        cout<<"Error in arguments"<<endl;
        exit(0);
    }
    inFile.open(argv[1],ios::in);

    if(!inFile.is_open())
    {
        cout<<"Error opening the file for taking input"<<endl;
        exit(0);
    }
    
    inFile>>out_width>>out_height;

    vector<int> outline;
    outline.push_back(out_width);
    outline.push_back(out_height);
    start = createTile(outline,start);

    int before = inFile.tellg();
    inFile>>c;
    int after = inFile.tellg();

    while(!inFile.eof())
    {
        if(c=='P')
        {
            int x,y;
            inFile>>x>>y;
            tile* locate = pointFind(x,y,start);
            p.push_back({locate->xl,locate->yl});
        }
        else
        {         
            inFile.seekg(-(after-before),ios::cur);
            vector<int> single_block;
            int a;
            for(int i=0;i<5;i++)
            {
                inFile>>a;
                single_block.push_back(a);
            }
            start=createTile(single_block,start);      
            block.push_back(single_block);
        }
        before = inFile.tellg();
        inFile>>c;
        after = inFile.tellg();   
    }
    inFile.close();
   
    outFile.open(argv[2],ios::out);
    if(!outFile.is_open())
    {
        cout<<"Error opening the output file for writing"<<endl;
        exit(0);
    }

    outFile<<count_tiles<<endl;

    vector<vector<int>> neighbour_tiles = findNeighbours(block,start);
    sort(neighbour_tiles.begin(),neighbour_tiles.end(),sortColumn);
    for(int i=0;i<neighbour_tiles.size();i++)
    {
        for(int j=0;j<neighbour_tiles[i].size();j++)
        {
            outFile<<neighbour_tiles[i][j];
            if(j<neighbour_tiles[i].size()-1)
                outFile<<" ";
        }
        outFile<<endl;
    }

    for(int i=0;i<p.size();i++)
    {
        outFile<<p[i].first<<" "<<p[i].second<<endl;
    }
    outFile.close();
    return 0;
    

} 