#include "pidClass.h"
using namespace std;

const float T = 0.05;  //1ms
const float Kp = 0.8;
const float Ti = 200;
const float Td = 0.000625;


//0.01
//0.02
//0.02

//0.008
//0.001
//0.2

//0.05
//0.5
//0.05
//10

const int d = 50; //Ԥ���� ��λmm

float cur_e = 0;
float pre_e = 0;
float sum_e = 0;
float u = 0;

magicPoint carPoint_last(0,0) ;
float angle_last = 0;


int motion_mode = 1; //0 straight 1 turnleft or right
int count = 0;
int delay_count = 10;
float straight_tolerance = 8;
float turn_tolerance = 4;


deque<magicPoint>& pidClass::getPath(deque<magicPoint> &pathToTraverse) {
	return pathToTraverse;
}

int pidClass::unitConversion(int distanceInMM) {
	return distanceInMM / 10;	//mm->cm
}

wheelSpeed pidClass::pid(
	magicPoint carPoint,
	magicPoint targetPoint,
	float alpha,
	int stopDistance){
	magicVector carToTarget(targetPoint,carPoint);

	if (carToTarget.vectorLength <= stopDistance)
		return stopSpeed;
	
	float beta = (carToTarget.angle - alpha) >= 0 ? (carToTarget.angle - alpha) : 360 + (carToTarget.angle - alpha);
	if (beta > 90 && beta < 180)
		return rightMax;

	if (beta > 180 && beta < 270)
		return leftMax;

	cur_e = unitConversion(d)*sin(beta / 180 * pi);
	sum_e += cur_e;
	u = Kp * (cur_e + T / Ti * sum_e + Td / T * (cur_e - pre_e));
	moveSpeed.changeWheelSpeed(u);
	printf("left:%f right:%f angle:%f\n", moveSpeed.left, moveSpeed.right, alpha);
	return moveSpeed;
}

wheelSpeed pidClass::pidNoA(
	magicPoint carPoint,
	magicPoint targetPoint,
	int stopDistance){
	//С��λ��(carPosX1,carPosY1),Ŀ��λ��(targetPosX2,targetPosY2)
	//stopDistance:ֹͣ����
	magicVector carVecTmp(carPoint,carPoint_last);
	float alpha = carVecTmp.angle;	//x1_dir,x1������x���������ļн�
	carPoint_last = carPoint;
        printf("%f\n",alpha);
	return pid(carPoint,targetPoint, alpha, stopDistance);
}

wheelSpeed pidClass::pidDoublePoints(
	magicPoint carPoint1,
	magicPoint carPoint2,
	magicPoint targetPoint,
	int stopDistance){
	magicVector carVecTmp(carPoint2,carPoint1);
	//printf("point1: x = %f y = %f;pont2:x = %f y = %f \n",carPoint1.x,carPoint1.y,carPoint2.x,carPoint2.y);
	float alpha = carVecTmp.angle;	//x1_dir,x1������x���������ļн�
	magicPoint carPointTemp;
	carPointTemp.x = (carPoint1.x + carPoint2.x) / 2;
	carPointTemp.y = (carPoint1.y + carPoint2.y) / 2;
	wheelSpeed answer = pid(carPointTemp,targetPoint, alpha, stopDistance);
	//printf ("left speed : %f, right speed : %f, angle : %f\n",answer.left,answer.right,alpha);
	printf("%f\n", alpha);
	return answer;

}

wheelSpeed pidClass::simpleAlgorithm(magicPoint carPoint, magicPoint targetPoint, float angle, int stopDistance){
	magicVector carToTarget(targetPoint,carPoint);
	float beta = (carToTarget.angle - angle) >= 0 ? (carToTarget.angle - angle) : 360 + (carToTarget.angle - angle);
	wheelSpeed speed;
	speed.left = 200;
	speed.right = 200;
	speed.beta = beta;
	if (carToTarget.vectorLength <= stopDistance){
		speed.left = 200;
		speed.right = 200;
		return speed;
	}
	if(motion_mode == 1){
		if(count < delay_count){
			speed.left = 200;
			speed.right = 200;
			count++;
			return speed;
		}
		if(beta >= 180 && beta < 360 - turn_tolerance){
			//turn left
			speed.left = 50;
			speed.right = 350;
			return speed;
		} else if(beta >= turn_tolerance && beta < 180){
			//turn right
			speed.left = 350;
			speed.right = 50;
			return speed;
		}
		motion_mode = 0;
		count = 0;
	}
	if(motion_mode == 0) {
		if(count < delay_count){
			speed.left = 200;
			speed.right = 200;
			count++;
			return speed;
		}
		if(beta >= 180 && beta < 360 - straight_tolerance){
			//turn left
			speed.left = 200;
			speed.right = 200;
			motion_mode = 1;
			count = 0;
			return speed;
		} else if(beta >= straight_tolerance && beta < 180){
			//turn right
			speed.left = 200;
			speed.right = 200;
			motion_mode = 1;
			count = 0;
			return speed;
		}
		//go straight
		speed.left = 350;
		speed.right = 350;
		return speed;
	}
	return speed;
}

wheelSpeed pidClass::traverse(
	magicPoint carPoint1,
	float angle,
	int stopDistance){
	if (pathToTraverse.empty())
		return stopSpeed;

	wheelSpeed noneStop = pid(carPoint1, pathToTraverse.front(), angle,stopDistance);
	while (noneStop == stopSpeed){
		pathToTraverse.pop_front();
		if (pathToTraverse.empty()){
			return stopSpeed;
		}
		noneStop = pid(carPoint1, pathToTraverse.front(), angle,stopDistance);
	}
	return noneStop;
}

/*
int main() {
	int i;
	for(i = 0;i<100;i++){
		magicPoint carPoint1( 0,0 ), carPoint2(0.5,0);
		//wheelSpeed O1 = pidNoA(carPoint,targetPoint, 0);
		pathToTraverse.push_back(magicPoint(1.25,-1));
		wheelSpeed O1 = pidClass::traverse(carPoint1, carPoint2, 1);
		cout << O1.left << " " << O1.right;

		pathToTraverse.clear();
		O1 = pidClass::traverse(carPoint1, carPoint2, 1);
		cout << O1.left << " " << O1.right;
	}
}
*/
