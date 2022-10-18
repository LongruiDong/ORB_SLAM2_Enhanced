/**
* 测试代码段 关于 io txt
*   
*/
#include<iostream>
#include<fstream>
#include<sstream>
#include<iomanip>
#include <vector>
#include<algorithm>

using namespace std;

int main() 
{
    string filename = "degugout.txt";
    //文件写入的准备工作
    ofstream f;
    f.open(filename.c_str());
    vector<float> data{0.,1.,2.,4.,5.};
    for (size_t i = 0; i < data.size(); i++)
    {
        
    }


    return 1;
}