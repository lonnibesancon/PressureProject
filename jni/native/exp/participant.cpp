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
            std::tuple<Vector3,Quaternion> temp = targets[i]; 
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
	targets.push_back(std::tuple<Vector3,Quaternion>(Vector3(34.4052, 20.5995, 110.312)		,Quaternion(0.17284, -0.027112, 0.982298, -0.0668766)));
	targets.push_back(std::tuple<Vector3,Quaternion>(Vector3(-29.6198, 17.7426, 90.7622)	,Quaternion(-0.119085, 0.160843, 0.919453, 0.338448)));
	targets.push_back(std::tuple<Vector3,Quaternion>(Vector3(-25.3372, 2.28942, 75.3072)	,Quaternion(-0.463481, -0.118306, 0.875867, -0.0635474)));
	targets.push_back(std::tuple<Vector3,Quaternion>(Vector3(24.8166, -2.20706, 75.3191)	,Quaternion(-0.356861, 0.233901, 0.837369, -0.34167)));
	targets.push_back(std::tuple<Vector3,Quaternion>(Vector3(26.937, 1.94701, 109.407)		,Quaternion(0.210963, 0.809764, 0.245368, -0.489443)));
	targets.push_back(std::tuple<Vector3,Quaternion>(Vector3(-3.45967, -12.5448, 91.4293)	,Quaternion(0.728339, 0.0348798, 0.436378, -0.527114)));
	targets.push_back(std::tuple<Vector3,Quaternion>(Vector3(34.212, -11.3861, 130.689)		,Quaternion(0.564828, 0.113259, 0.703096, -0.416848)));
	targets.push_back(std::tuple<Vector3,Quaternion>(Vector3(13.1964, 2.97829, 107.047)		,Quaternion(-0.191473, 0.427081, 0.842703, 0.265989)));
	targets.push_back(std::tuple<Vector3,Quaternion>(Vector3(-22.2263, -0.264936, 110.766)	,Quaternion(-0.0805761, 0.229052, 0.335052, 0.910351)));
	targets.push_back(std::tuple<Vector3,Quaternion>(Vector3(16.9274, 18.0633, 158.765)		,Quaternion(-0.959303, 0.22141, 0.0488697, 0.168163)));
	targets.push_back(std::tuple<Vector3,Quaternion>(Vector3(30.2101, 7.40269, 86.2055)		,Quaternion(-0.748447, 0.645197, 0.134274, -0.0739392)));
	targets.push_back(std::tuple<Vector3,Quaternion>(Vector3(-13.2588, 5.63171, 139.974)	,Quaternion(-0.541399, 0.653179, -0.291624, -0.441754)));
	targets.push_back(std::tuple<Vector3,Quaternion>(Vector3(11.206, 8.74678, 76.2941)		,Quaternion(0.751457, 0.222842, 0.268716, -0.5598)));
	targets.push_back(std::tuple<Vector3,Quaternion>(Vector3(33.7071, 3.05983, 101.892)		,Quaternion(0.692698, -0.176285, -0.0476433, -0.697677)));
	targets.push_back(std::tuple<Vector3,Quaternion>(Vector3(-3.92666, -12.46, 98.5911)		,Quaternion(-0.236857, -0.725444, 0.576961, -0.29098)));
	//targets.push_back(std::tuple<Vector3,Quaternion>(Vector3(-4.42321, 13.177, 93.1823)		,Quaternion(-0.637159, -0.714029, 0.249659, -0.147583)));
	//targets.push_back(std::tuple<Vector3,Quaternion>(Vector3(31.721, -8.42904, 124.926)		,Quaternion(-0.920161, -0.359409, -0.148246, -0.0454646)));
	//targets.push_back(std::tuple<Vector3,Quaternion>(Vector3(30.2151, 21.5581, 102.932)		,Quaternion(-0.933026, -0.275187, -0.226801, -0.0470352)));

}