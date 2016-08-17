#ifndef PARTICIPANT_H
#define PARTICIPANT_H

#include "global.h"
#include <algorithm>  // for std::sort, std::next_permutation
#include <string>

class Participant{

public:
	int getNextCondition();
	std::vector <std::tuple<Vector3,Quaternion>> getNextTarget();
	int getAllTargetsString();

	Participant(int p, std::string path);
	Participant();
	void setValues(int p, std::string path);
	void addData(Vector3 currentPos, Quaternion currentRot, float prec, int timestamp);
	void resetTrial();
	void resetCondition();
	int getCondition();


private:
	
	int currentConditionID;
	int pID ;
	int currentTargetID ;
	std::string filepath ;

	std::vector<int> conditions ;
	std::vector<std::tuple<Vector3,Quaternion>> logPositions ;
	std::vector<std::tuple<Vector3,Quaternion>> logDifference ;
	std::vector<std::tuple<double,double>> logDiffValues ;
	std::vector<int> timestamps ;
	std::vector<float> precision ;


	std::vector<std::tuple<Vector3,Quaternion>> targets ;


	void getPermutationTrials();
	void getPermutationCondition();
	void generateAllTargets();
	void performLog();



};

#endif /* LOADER_OBJ_H */
