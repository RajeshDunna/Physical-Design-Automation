#pragma once
#include<string>
#include<vector>

using namespace std;

class cell
{
    public:
    string name;
    int xl, yl, xu, yu;
    int x,y;
    int width, height;
    int chip_index;
    vector<cell*> ov_cells;
    vector<cell*> overlapped_cell;
    int overlap_area;
};

class cluster
{
    public:
    double xc;
    int ec,qc;
    int wc;

    vector<cell*> cluster_cell;
};

class row
{
    public:
        int xl, yl, xu, yu;
        int row_index;
        int vacant_width;
        vector<cluster*> row_cluster;

    row(int, int, int, int, int);
    row(row* &);
};
