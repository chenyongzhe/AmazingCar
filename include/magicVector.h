/*
��������
�����Ƕ�Ϊ��x��������˳ʱ����ת�ó���
*/
#pragma once
#include <math.h>
#include "magicPoint.h"
const float pi = 3.1415;

float calcAngle(magicPoint point1,magicPoint point2) {	//��1ָ��2��������x��������ļн�
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

float calcDistance(magicPoint point1, magicPoint point2) {	//��1ָ��2�������Ĵ�С
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
	float angle;	//x1ָ��x2�ķ���ĽǶ�
};