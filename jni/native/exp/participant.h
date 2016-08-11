#ifndef PARTICIPANT_H
#define PARTICIPANT_H

#include "global.h"
#include <algorithm>  // for std::sort, std::next_permutation
#include <string>

class Participant{

public:
	int getNextCondition();
	std::vector <std::tuple<Matrix4,Quaternion>> getNextTarget();
	int getAllTargetsString();

	Participant(int p, std::string path);
	void addData(Matrix4 currentPos, Quaternion currentRot, precision);


private:
	
	int currentConditionID;
	int pID ;
	int currentTargetID ;
	std::string filepath ;

	std::vector<int> conditions ;
	std::vector<std::tuple<Matrix4,Quaternion>> logPositions ;
	std::vector<std::tuple<Matrix4,Quaternion>> logDifference ;
	std::vector<int> timestamps ;
	std::vector<float> precision ;


	std::vector<std::tuple<Matrix4,Quaternion>> targets ;


	void getPermutationTrials();
	void getPermutationCondition();
	void generateAllTargets();



}

#endif /* LOADER_OBJ_H */
