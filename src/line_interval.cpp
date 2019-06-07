#include "line_interval.hpp"
#include "shape2d.hpp"
#include <Eigen/Dense>
#include <vector>
#include <array>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace Eigen;
using namespace std;

array<double, 2> polarEquation(Vector2d v0, Vector2d v1){
    array<double, 3> line={v0[1]-v1[1], v1[0]-v0[0],-(v0[0]*v1[1]-v1[0]*v0[1])};

    double normalAngle=atan2(line[1], line[0]);
    double normalDist=line[2] / sqrt(line[0] * line[0] + line[1] * line[1]);
    array<double, 2> polar={normalDist, normalAngle};
    return polar;
}

LineInterval::LineInterval(Vector2d _point){
    point = _point;
    angleStart = atan2(point[1], point[0]);
    distLowerBound = 0.0;
    distUpperBound = 0.0;
    intervalMaxDists = {0.0, 0.0};
}

LineInterval::LineInterval(double _angleStart, double _angleEnd){
    angleStart=_angleStart;
    angleEnd=_angleEnd;
    distLowerBound = 0.0;
    distUpperBound = 0.0;
    intervalMaxDists = {0.0, 0.0};
}

double LineInterval::DistAt(array<double, 2> edge, int side){
    double angle=(side==0 ? angleStart : angleEnd);
    return edge[0]/cos(angle-edge[1]);
}

array<double, 3> LineInterval::Divide(){
    double mid=(angleStart+IntervalAngleEnd())/2;
    if(mid>=M_PI){
	mid -= 2 * M_PI;
    }
    return {angleStart, mid, angleEnd};
    /*if(angleStart>angleEnd){
	double intervalAngleEnd=angleEnd + (2 * M_PI);
	double mid=(angleStart+intervalAngleEnd)/2;
	if(mid>=M_PI){
	    mid -= 2 * M_PI;
	}
	return {angleStart, mid, angleEnd};
    } else{
	return {angleStart, (angleStart+angleEnd)/2, angleEnd};
    }*/
}

bool LineInterval::IntersectsEdge(double v0Angle, double v1Angle){
    double edgeAngleStart=v0Angle;
    double edgeAngleEnd=v1Angle;
    if(edgeAngleStart>edgeAngleEnd){
	if(angleStart>angleEnd){
	    edgeAngleStart -= M_PI * 2;
	} else {
	    edgeAngleEnd += M_PI * 2;
	}
    }
    return (edgeAngleStart<=angleStart && edgeAngleEnd>=IntervalAngleEnd());
}


double LineInterval::IntervalAngleEnd(){
    double intervalAngleEnd= (angleStart>angleEnd ? angleEnd + (2 * M_PI) : angleEnd);
    return intervalAngleEnd;
}

bool LineInterval::containsNormal(array<double, 2> edge){
    return edge[1] >= angleStart && edge[1] <= IntervalAngleEnd();
}

array<double, 3> LineInterval::FunctionsAt(array<double, 2> edge, int side){
    double angle=(side==0 ? angleStart : angleEnd);
    double dist=edge[0]/cos(angle-edge[1]);
    double distPrime=tan(angle-edge[1])*dist;
   
    double angleSin=sin(angle-edge[1]);
    double angleCos=cos(angle-edge[1]);

    double distPrime2=(1+angleSin*angleSin)/(angleCos*angleCos*angleCos);
    return {dist, distPrime, distPrime2};
}

double LineInterval::ApproxRoot(double distStart, double distEnd, double derivStart, double derivEnd){
    //double intervalAngleEnd= (angleStart>angleEnd ? angleEnd + (2 * M_PI) : angleEnd);

    double bStart=distStart - (angleStart * derivStart);
    double bEnd=distEnd - (IntervalAngleEnd() * derivEnd);

    Matrix2d A;
    A << derivStart, -1,   derivEnd, -1;
    Vector2d b;
    b << -bStart, -bEnd;

    return A.colPivHouseholderQr().solve(b)[1];
}

void LineInterval::SetAngleEnd(Vector2d _point){
    angleEnd = atan2(_point[1], _point[0]);
}

void LineInterval::update(double upperBound, double lowerBound, double distStart, double distEnd, unsigned long int shapeId){
    distUpperBound+=upperBound;
    distLowerBound+=lowerBound;

    if(distStart > max(intervalMaxDists[0], intervalMaxDists[1]) || distEnd > max(intervalMaxDists[0], intervalMaxDists[1])){
	intervalMaxDists={distStart, distEnd};
    }
    shapeIds.push_back(shapeId);
}

double LineInterval::MaxWidth(){
    //double intervalAngleEnd= (angleStart>angleEnd ? angleEnd + (2 * M_PI) : angleEnd);
    double b=intervalMaxDists[0];
    double c=intervalMaxDists[1];
    return sqrt( b*b + c*c - 2*b*c*cos(IntervalAngleEnd()-angleStart) );
}

array<Vector2d, 2> LineInterval::EndPoints(){
    Vector2d v0(intervalMaxDists[0]*cos(angleStart), intervalMaxDists[0]*sin(angleStart));
    Vector2d v1(intervalMaxDists[1]*cos(angleEnd), intervalMaxDists[1]*sin(angleEnd));
    return {v0, v1};
}


vector<unsigned long int> LineInterval::ShapeIds(){
    return shapeIds;
}

double LineInterval::LowerBound(){
    return distLowerBound;
}

double LineInterval::UpperBound(){
    return distUpperBound;
}

Vector2d LineInterval::Point(){
    return point;
}

