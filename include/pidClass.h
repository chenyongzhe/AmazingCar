#pragma once

#include <iostream>
#include <math.h>
#include <vector>
#include <deque>
#include "wheelSpeed.h"
#include "magicVector.h"
#include "magicPoint.h"

using namespace std;

deque<magicPoint> pathToTraverse;


class pidClass{
public:
	static deque<magicPoint>& getPath(deque<magicPoint> &pathToTraverse);
	static int unitConversion(int distanceInMM);
	static wheelSpeed traverse(magicPoint carPoint1, float angle, int stopDistance);
	static wheelSpeed pid(magicPoint carPoint, magicPoint targetPoint, float alpha, int stopDistance);
	static wheelSpeed pidNoA(magicPoint carPoint, magicPoint targetPoint, int stopDistance);
	static wheelSpeed pidDoublePoints(magicPoint carPoint1, magicPoint carPoint2, magicPoint targetPoint, int stopDistance);
	
	static wheelSpeed simpleAlgorithm(magicPoint carPoint1, magicPoint targetPoint, float angle, int stopDistance);
};
