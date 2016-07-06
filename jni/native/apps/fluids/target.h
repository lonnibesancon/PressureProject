#ifndef TARGET_H
#define TARGET_H



#include "global.h"

class Target
{
public:
	Target();
	Target(Vector3 pos, Quaternion q);

	Quaternion 	ComputeOrientationDifference(Quaternion q);
	Vector3 	ComputeEuclideanDistance(Vector3 t);

private:
	
	Matrix4 mat ;
	Quaternion orientation ;
	Vector3 position ;


};

#endif /* TARGET_H */
