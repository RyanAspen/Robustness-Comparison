#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <algorithm>
using namespace std;

struct Point{
  int x,y,z;
};

double getDist(Point p1, Point p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

int main()
{
    string line;
    
    int axis_index = 0;
    Point p;
    vector<Point> points1;
    ifstream mapInfoFile1("Map_Points_151.txt");
    while ( getline (mapInfoFile1,line) )
    {
        line.erase(remove(line.begin(), line.end(), ';'), line.end());
        line.erase(remove(line.begin(), line.end(), '['), line.end());
        line.erase(remove(line.begin(), line.end(), ']'), line.end());
        double measurement = std::stod(line);
        switch (axis_index)
        {
            case 0:
                p.x = measurement;
                break;
            case 1:
                p.y = measurement;
                break;
            case 2:
                p.z = measurement;
                break;
        }
        if (axis_index == 2)
        {
            points1.push_back(p);
        }
        axis_index = (axis_index + 1) % 3;
    }
    mapInfoFile1.close();
    cout << "Test" << endl;
    vector<Point> points2;
    ifstream mapInfoFile2("Map_Points_base.txt");
    while ( getline (mapInfoFile2,line) )
    {
        line.erase(remove(line.begin(), line.end(), ';'), line.end());
        line.erase(remove(line.begin(), line.end(), '['), line.end());
        line.erase(remove(line.begin(), line.end(), ']'), line.end());
        double measurement = std::stod(line);
        switch (axis_index)
        {
            case 0:
                p.x = measurement;
                break;
            case 1:
                p.y = measurement;
                break;
            case 2:
                p.z = measurement;
                break;
        }
        if (axis_index == 2)
        {
            points2.push_back(p);
        }
        axis_index = (axis_index + 1) % 3;
    }
    mapInfoFile2.close();

    double chamferDistance = 0;
    double hausdorffDistance = 0;

    cout << "Size 1 = " << points1.size() << " Size 2 = " << points2.size() << endl;
    double maxMinDist = 0;
    for (int i = 0; i < points1.size(); i++)
    {
        double minDist = 99999;
        for (int j = 0; j < points2.size(); j++)
        {
            double dist = getDist(points1[i], points2[j]);
            if (dist < minDist)
            {
                minDist = dist;
            }
        }
        if (minDist > maxMinDist)
        {
            maxMinDist = minDist;
        }
        chamferDistance += minDist;
    }
    chamferDistance /= points1.size() * 2;
    hausdorffDistance += maxMinDist / 2;

    maxMinDist = 0;
    for (int i = 0; i < points2.size(); i++)
    {
        double minDist = 99999;
        for (int j = 0; j < points1.size(); j++)
        {
            double dist = getDist(points2[i], points1[j]);
            if (dist < minDist)
            {
                minDist = dist;
            }
        }
        if (minDist > maxMinDist)
        {
            maxMinDist = minDist;
        }
        chamferDistance += minDist;
    }
    chamferDistance /= points2.size() * 2;
    hausdorffDistance += maxMinDist / 2;
    cout << "Chamfer Distance = " << chamferDistance << endl;
    cout << "Hausdorff Distance = " << hausdorffDistance << endl;
}