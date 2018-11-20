#pragma once
struct magicPoint
{
	magicPoint() = default;
	explicit magicPoint(float tmpx, float tmpy) :x(tmpx), y(tmpy) {};
	float x;
	float y;
};