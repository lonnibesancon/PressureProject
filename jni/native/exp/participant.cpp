#include "participant.h"
#include <algorithm>  // for std::sort, std::next_permutation
#include <string>
#include "../apps/fluids/interactionMode.h"

Participant::Participant(int p, std::string path){
	pID = p ;
	filepath = path ;
	conditions = {1,2,3,4} ;
	getPermutationCondition();
	currentConditionID = 0 ;
	currentTargetID = 0 ;
}

void Participant::getPermutationCondition(){
	std::sort(conditions.begin(), conditions.end()); // already sorted but does not hurt for 4

    for(int i=0;i<pID;i++){
        //std::cout << cond[0] << " " << cond[1] << " " << cond[2] << '\n';
        std::next_permutation(conditions.begin(), conditions.end());
    }

}

void Participant::getPermutationTrials(){
    int seed = pID * conditions[currentConditionID] * 40 ;
    //cout << "SEED = " << seed << endl ;
    srand(seed);
    for (int i=0; i<(NBTRIALS); i++) {
        int r = i + (rand() % (NBTRIALS-i)); // Random remaining position.
        if( i != 0 && r != 0){
            std::tuple<Matrix4,Quaternion> temp = targets[i]; 
            targets[i] = targets[r]; 
            targets[r] = temp;
        }
        
    }

    /*for(int i=0;i<NBTRIALS;i++){
        std::cout << get<0>(trialTargetsMouse[i]) << " ; " ;
        //*outfile << get<0>(trialTargets[i]) << " ; " ;
    }*/

}

void Participant::generateAllTargets(){
	targets.push_back(std::tuple<Matrix4,Quaternion>(Matrix4::identity(),Quaternion::identity()));
	targets.push_back(std::tuple<Matrix4,Quaternion>(Matrix4::identity(),Quaternion::identity()));
	targets.push_back(std::tuple<Matrix4,Quaternion>(Matrix4::identity(),Quaternion::identity()));
	targets.push_back(std::tuple<Matrix4,Quaternion>(Matrix4::identity(),Quaternion::identity()));
	targets.push_back(std::tuple<Matrix4,Quaternion>(Matrix4::identity(),Quaternion::identity()));
	targets.push_back(std::tuple<Matrix4,Quaternion>(Matrix4::identity(),Quaternion::identity()));
	targets.push_back(std::tuple<Matrix4,Quaternion>(Matrix4::identity(),Quaternion::identity()));
	targets.push_back(std::tuple<Matrix4,Quaternion>(Matrix4::identity(),Quaternion::identity()));
	targets.push_back(std::tuple<Matrix4,Quaternion>(Matrix4::identity(),Quaternion::identity()));
	targets.push_back(std::tuple<Matrix4,Quaternion>(Matrix4::identity(),Quaternion::identity()));
	targets.push_back(std::tuple<Matrix4,Quaternion>(Matrix4::identity(),Quaternion::identity()));
	targets.push_back(std::tuple<Matrix4,Quaternion>(Matrix4::identity(),Quaternion::identity()));
	targets.push_back(std::tuple<Matrix4,Quaternion>(Matrix4::zero(),Quaternion::identity()));
	targets.push_back(std::tuple<Matrix4,Quaternion>(Matrix4::zero(),Quaternion::identity()));
	targets.push_back(std::tuple<Matrix4,Quaternion>(Matrix4::zero(),Quaternion::identity()));

}