#include<iostream>
#include<fstream>
#include<sstream>
#include<iterator>
#include<string>
#include"cell.h"

row :: row(int x, int y, int w, int h, int index)
{
    xl=x;
    yl=y;
    xu=xl+w;
    yu=yl+h;
    row_index=index;
    vacant_width=xu-xl;;
}

row :: row(row* &copyRow)
{
   xl = copyRow->xl; 
   yl = copyRow->yl;
   xu = copyRow->xu;
   yu = copyRow->yu;
   row_index = copyRow->row_index;
   vacant_width = copyRow->vacant_width;
}