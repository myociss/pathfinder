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
    if(angleStart>angleEnd){
	double intervalAngleEnd=angleEnd + (2 * M_PI);
	double mid=(angleStart+intervalAngleEnd)/2;
	if(mid>=M_PI){
	    mid -= 2 * M_PI;
	}
	return {angleStart, mid, angleEnd};
    } else{
	return {angleStart, (angleStart+angleEnd)/2, angleEnd};
    }
}

void LineInterval::calculateTargetShape(Shape2d& shape){
    double intervalAngleEnd= (angleStart>angleEnd ? angleEnd + (2 * M_PI) : angleEnd);
    double intervalAngleStart=angleStart;

    vector<Vector2d> vertices=shape.VerticesArranged();
    array<double, 2> edgePolar;

    for(int i=0; i<vertices.size(); ++i){
	int next=i+1;
	if(next==vertices.size()){
	    next=0;
	}
	double edgeAngleEnd=atan2(vertices[i][1], vertices[i][0]);
	double edgeAngleStart=atan2(vertices[next][1], vertices[next][0]);
	if(edgeAngleStart>edgeAngleEnd){
	    if(angleStart>angleEnd){
		edgeAngleStart -= M_PI * 2;
	    } else {
		edgeAngleEnd += M_PI * 2;
	    }
	}
	if(edgeAngleStart<=intervalAngleStart && edgeAngleEnd>=intervalAngleEnd){
	    edgePolar=polarEquation(vertices[next], vertices[i]);
	    break;
	}
    }
    double startSide=DistAt(edgePolar, 0);
    double endSide=DistAt(edgePolar, 1);
    double upperBound=0.0;
    double lowerBound=0.0;
    double weight=shape.Weight();

    if(containsNormal(edgePolar)){
	upperBound=weight * max(startSide, endSide);
	lowerBound=weight * edgePolar[0];
    } else {
	upperBound=weight * max(startSide, endSide);
	lowerBound=weight * min(startSide, endSide);
	    }
    update(upperBound, lowerBound, startSide, endSide, shape.Id());
}

void LineInterval::calculateShape(Shape2d& shape){
    double intervalAngleEnd= (angleStart>angleEnd ? angleEnd + (2 * M_PI) : angleEnd);
    double intervalAngleStart=angleStart;

    vector<Vector2d> vertices=shape.VerticesArranged();
    array<double, 2> entryEdgePolar;
    array<double, 2> terminalEdgePolar;

    for(int i=1; i<=shape.EndVertex(); ++i){
	double edgeAngleStart=atan2(vertices[i-1][1], vertices[i-1][0]);
	double edgeAngleEnd=atan2(vertices[i][1], vertices[i][0]);
	if(edgeAngleStart>edgeAngleEnd){
	    if(angleStart>angleEnd){
		edgeAngleStart -= M_PI * 2;
	    } else {
		edgeAngleEnd += M_PI * 2;
	    }
	}
	if(edgeAngleStart<=intervalAngleStart && edgeAngleEnd>=intervalAngleEnd){
	    entryEdgePolar=polarEquation(vertices[i-1], vertices[i]);
	    break;
	}
    }

    for(int i=shape.EndVertex(); i<vertices.size(); ++i){
	int next=i+1;
	if(next==vertices.size()){
	    next=0;
	}
	double edgeAngleEnd=atan2(vertices[i][1], vertices[i][0]);
	double edgeAngleStart=atan2(vertices[next][1], vertices[next][0]);
	if(edgeAngleStart>edgeAngleEnd){
	    if(angleStart>angleEnd){
		edgeAngleStart -= M_PI * 2;
	    } else {
		edgeAngleEnd += M_PI * 2;
	    }
	}
	if(edgeAngleStart<=intervalAngleStart && edgeAngleEnd>=intervalAngleEnd){
	    terminalEdgePolar=polarEquation(vertices[next], vertices[i]);
	    break;
	}
    }

    array<double, 3> entryFStart=FunctionsAt(entryEdgePolar, 0);
    array<double, 3> terminalFStart=FunctionsAt(terminalEdgePolar, 0);

    array<double, 3> entryFEnd=FunctionsAt(entryEdgePolar, 1);
    array<double, 3> terminalFEnd=FunctionsAt(terminalEdgePolar, 1);
    
    FindShapeBounds(entryFStart, terminalFStart, entryFEnd, terminalFEnd, shape);
}

void LineInterval::FindShapeBounds(array<double, 3> entryFStart, array<double, 3> terminalFStart, array<double, 3> entryFEnd, array<double, 3> terminalFEnd, Shape2d& shape){
    double startDist=max(terminalFStart[0]-entryFStart[0], 0.0);
    double startDeriv=terminalFStart[1]-entryFStart[1];
    double startDeriv2=terminalFStart[2]-entryFStart[2];


    double endDist=max(terminalFEnd[0]-entryFEnd[0], 0.0);
    double endDeriv=terminalFEnd[1]-entryFEnd[1];
    double endDeriv2=terminalFEnd[2]-entryFEnd[2];

    double maxSide=max(startDist, endDist);
    double minSide=min(startDist, endDist);

    double upperBound = 0.0;
    double lowerBound = 0.0;

    double weight = shape.Weight();

    if(startDeriv2*endDeriv2 < 0){
	upperBound=weight * shape.maxDist();
    } else if(startDeriv*endDeriv < 0){
	double root=ApproxRoot(startDist, endDist, startDeriv, endDeriv);
	if(root < 0){
	    root=0.0;
	}

	if(startDeriv<0 || (startDeriv==0 && endDeriv>0)){
	    upperBound=weight*maxSide;
	    lowerBound=weight*root;
	} else{
	    upperBound=weight*root;
	    lowerBound=weight*minSide;
	}
    } else {
	upperBound=weight*maxSide;
	lowerBound=weight*minSide;
    }

    update(upperBound, lowerBound, terminalFStart[0], terminalFEnd[0], shape.Id());
}

bool LineInterval::containsNormal(array<double, 2> edge){
    double intervalAngleEnd= (angleStart>angleEnd ? angleEnd + (2 * M_PI) : angleEnd);
    return edge[1] >= angleStart && edge[1] <= intervalAngleEnd;
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
    double intervalAngleEnd= (angleStart>angleEnd ? angleEnd + (2 * M_PI) : angleEnd);

    double bStart=distStart - (angleStart * derivStart);
    double bEnd=distEnd - (intervalAngleEnd * derivEnd);

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
    double intervalAngleEnd= (angleStart>angleEnd ? angleEnd + (2 * M_PI) : angleEnd);
    double b=intervalMaxDists[0];
    double c=intervalMaxDists[1];
    return sqrt( b*b + c*c - 2*b*c*cos(intervalAngleEnd-angleStart) );
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

