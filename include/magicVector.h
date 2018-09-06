/*
向量计算
向量角度为从x轴正方向，顺时针旋转得出。
*/
#pragma once
#include <math.h>
#include "magicPoint.h"
const float pi = 3.1415;

float calcAngle(magicPoint point1,magicPoint point2) {	//从1指向2的向量与x轴正方向的夹角
	if ((point1.y == point2.y) && (point1.x == point2.x))
		return -1;
	else
	{
		float alpha = 180 * acos(
			(point2.x - point1.x) / sqrt(
			(point1.x - point2.x)*(point1.x - point2.x) + 
			(point1.y - point2.y)*(point1.y - point2.y)
			)
			) / pi;
		alpha = (point2.y - point1.y) <= 0 ? alpha : 360 - alpha;
		return alpha;
	}
}

float calcDistance(magicPoint point1, magicPoint point2) {	//从1指向2的向量的大小
	return sqrt(
		(point1.x - point2.x)*(point1.x - point2.x) + 
		(point1.y - point2.y)*(point1.y - point2.y)
		);
}

struct magicVector
{
	magicVector() = default;
	explicit magicVector
		(magicPoint point2, magicPoint point1 = magicPoint{0,0})
		:p1(point1),p2(point2)
	{
		vectorLength = calcDistance(p1,p2);
		angle = calcAngle(p1,p2);
	};
	magicPoint p2,p1;
	float vectorLength;
	float angle;	//x1指向x2的方向的角度
};