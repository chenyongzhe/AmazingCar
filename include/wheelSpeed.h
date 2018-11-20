/*
С�������ٶȱ���
�仯���ó����ϡ�����
*/
#pragma once

#include <stdio.h>

const float lowestSpeed = 200;
const float highestSpeed = 350;
struct wheelSpeed;
void fitWheelSpeed(wheelSpeed &w);

struct wheelSpeed
{
	wheelSpeed() = default;
	explicit wheelSpeed(float f1, float f2) :left(f1), right(f2) { fitWheelSpeed(*this); }
	void changeWheelSpeed(float u);
	float left;
	float right;
	float beta;
};

void fit(float &sp) {
	sp = sp < lowestSpeed ? lowestSpeed : sp;
	sp = sp > highestSpeed ? highestSpeed : sp;
}

void fitWheelSpeed(wheelSpeed &w) {
	fit(w.left); fit(w.right);
}

void wheelSpeed::changeWheelSpeed(float u) {
	this->left += u;
	this->right -= u;
	fitWheelSpeed(*this);
}

bool operator==(const wheelSpeed &l, const wheelSpeed &r) {
	return (l.left == r.left) && (l.right == r.right);
}

wheelSpeed moveSpeed((lowestSpeed + highestSpeed) / 2,(lowestSpeed + highestSpeed) / 2);
wheelSpeed stopSpeed( lowestSpeed,lowestSpeed );
wheelSpeed rightMax ( highestSpeed,lowestSpeed );
wheelSpeed leftMax( lowestSpeed,highestSpeed );
