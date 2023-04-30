#ifndef POLYGON_INTERSECTION_H
#define POLYGON_INTERSECTION_H

#endif // POLYGON_INTERSECTION_H

#include "mapType.h"

using namespace std;
typedef struct Point
{
    int x;
    int y;
}Point;
//若点a大于点b,即点a在点b顺时针方向,返回true,否则返回false
bool PointCmp(const Point &a,const Point &b,const Point &center)
{
    if (a.x >= 0 && b.x < 0)
        return true;
    if (a.x == 0 && b.x == 0)
        return a.y > b.y;
    //向量OA和向量OB的叉积
    int det = (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
    if (det < 0)
        return true;
    if (det > 0)
        return false;
    //向量OA和向量OB共线，以距离判断大小
    int d1 = (a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y);
    int d2 = (b.x - center.x) * (b.x - center.y) + (b.y - center.y) * (b.y - center.y);
    return d1 > d2;
}
void ClockwiseSortPoints(std::vector<Point> &vPoints)
{
    //计算重心
    cv::Point center;
    double x = 0,y = 0;
    for (int i = 0;i < vPoints.size();i++)
    {
        x += vPoints[i].x;
        y += vPoints[i].y;
    }
    center.x = (int)x/vPoints.size();
    center.y = (int)y/vPoints.size();

    //冒泡排序
    for(int i = 0;i < vPoints.size() - 1;i++)
    {
        for (int j = 0;j < vPoints.size() - i - 1;j++)
        {
            if (PointCmp(vPoints[j],vPoints[j+1],center))
            {
                cv::Point tmp = vPoints[j];
                vPoints[j] = vPoints[j + 1];
                vPoints[j + 1] = tmp;
            }
        }
    }
}

//  The function will return YES if the point x,y is inside the polygon, or
//  NO if it is not.  If the point is exactly on the edge of the polygon,
//  then the function may return YES or NO.
bool IsPointInPolygon(std::vector<Point> poly,Point pt)
{
    int i,j;
    bool c = false;
    for (i = 0,j = poly.size() - 1;i < poly.size();j = i++)
    {
        if ((((poly[i].y <= pt.y) && (pt.y < poly[j].y)) ||
            ((poly[j].y <= pt.y) && (pt.y < poly[i].y)))
            && (pt.x < (poly[j].x - poly[i].x) * (pt.y - poly[i].y)/(poly[j].y - poly[i].y) + poly[i].x))
        {
            c = !c;
        }
    }
    return c;
}

//排斥实验
bool IsRectCross(const Point &p1,const Point &p2,const Point &q1,const Point &q2)
{
    bool ret = min(p1.x,p2.x) <= max(q1.x,q2.x)    &&
                min(q1.x,q2.x) <= max(p1.x,p2.x) &&
                min(p1.y,p2.y) <= max(q1.y,q2.y) &&
                min(q1.y,q2.y) <= max(p1.y,p2.y);
    return ret;
}
//跨立判断
bool IsLineSegmentCross(const Point &pFirst1,const Point &pFirst2,const Point &pSecond1,const Point &pSecond2)
{
    long line1,line2;
    line1 = pFirst1.x * (pSecond1.y - pFirst2.y) +
        pFirst2.x * (pFirst1.y - pSecond1.y) +
        pSecond1.x * (pFirst2.y - pFirst1.y);
    line2 = pFirst1.x * (pSecond2.y - pFirst2.y) +
        pFirst2.x * (pFirst1.y - pSecond2.y) +
        pSecond2.x * (pFirst2.y - pFirst1.y);
    if (((line1 ^ line2) >= 0) && !(line1 == 0 && line2 == 0))
        return false;

    line1 = pSecond1.x * (pFirst1.y - pSecond2.y) +
        pSecond2.x * (pSecond1.y - pFirst1.y) +
        pFirst1.x * (pSecond2.y - pSecond1.y);
    line2 = pSecond1.x * (pFirst2.y - pSecond2.y) +
        pSecond2.x * (pSecond1.y - pFirst2.y) +
        pFirst2.x * (pSecond2.y - pSecond1.y);
    if (((line1 ^ line2) >= 0) && !(line1 == 0 && line2 == 0))
        return false;
    return true;
}

bool GetCrossPoint(const Point &p1,const Point &p2,const Point &q1,const Point &q2,long &x,long &y)
{
    if(IsRectCross(p1,p2,q1,q2))
    {
        if (IsLineSegmentCross(p1,p2,q1,q2))
        {
            //求交点
            long tmpLeft,tmpRight;
            tmpLeft = (q2.x - q1.x) * (p1.y - p2.y) - (p2.x - p1.x) * (q1.y - q2.y);
            tmpRight = (p1.y - q1.y) * (p2.x - p1.x) * (q2.x - q1.x) + q1.x * (q2.y - q1.y) * (p2.x - p1.x) - p1.x * (p2.y - p1.y) * (q2.x - q1.x);

            x = (int)((double)tmpRight/(double)tmpLeft);

            tmpLeft = (p1.x - p2.x) * (q2.y - q1.y) - (p2.y - p1.y) * (q1.x - q2.x);
            tmpRight = p2.y * (p1.x - p2.x) * (q2.y - q1.y) + (q2.x- p2.x) * (q2.y - q1.y) * (p1.y - p2.y) - q2.y * (q1.x - q2.x) * (p2.y - p1.y);
            y = (int)((double)tmpRight/(double)tmpLeft);
            return true;
        }
    }
    return false;
}

bool PolygonClip(const vector<Point> &poly1,const vector<Point> &poly2, std::vector<Point> &interPoly)
{
    if (poly1.size() < 3 || poly2.size() < 3)
    {
        return false;
    }

    long x,y;
    //计算多边形交点
    for (int i = 0;i < poly1.size();i++)
    {
        int poly1_next_idx = (i + 1) % poly1.size();
        for (int j = 0;j < poly2.size();j++)
        {
            int poly2_next_idx = (j + 1) % poly2.size();
            if (GetCrossPoint(poly1[i],poly1[poly1_next_idx],
                poly2[j],poly2[poly2_next_idx],
                x,y))
            {
                interPoly.push_back(cv::Point(x,y));
            }
        }
    }

    //计算多边形内部点
    for(int i = 0;i < poly1.size();i++)
    {
        if (IsPointInpolygon(poly2,poly1[i]))
        {
            interPoly.push_back(poly1[i]);
        }
    }
    for (int i = 0;i < poly2.size();i++)
    {
        if (IsPointInpolygon(poly1,poly2[i]))
        {
            interPoly.push_back(poly2[i]);
        }
    }

    if(interPoly.size() <= 0)
        return false;

    //点集排序
    ClockwiseSortPoints(interPoly);
    return true;
}
