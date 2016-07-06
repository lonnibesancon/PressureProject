#include "target.h"

Target::Target(){
	orientation = Quaternion(Vector3::unitX(), M_PI));
	position = Vector3::zero();
}

Target::Target(Vector3 pos, Quaternion q){
	orientation = q ;
	position = pos ;
}

Vector3 Target::ComputeEuclideanDistance(Vector3 t){
	return (pos - t) ;
}

Quaternion Target::ComputeOrientationDifference(Quaternion q){
	// See http://answers.unity3d.com/questions/35541/problem-finding-relative-rotation-from-one-quatern.html
	return (orientation.inverse() * q) ;
}