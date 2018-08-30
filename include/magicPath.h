#pragma once
#include <deque>
#include "magicPoint.h"

using namespace std;

struct magicPath
{
	deque<magicPoint> points;
	bool isNew = true;
};