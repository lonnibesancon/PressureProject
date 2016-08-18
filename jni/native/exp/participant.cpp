#include "participant.h"
#include <algorithm>  // for std::sort, std::next_permutation
#include <string>
#include "../apps/fluids/interactionMode.h"
#include <stdio.h>
#include <fstream>
#include <sys/stat.h>
#include <cerrno>

#include<sstream>
template <typename T>
std::string to_string(T value)
{
  //create an output string stream
  std::ostringstream os ;

  //throw the value into the string stream
  os << value ;

  //convert the string stream into a string and return
  return os.str() ;
}


Participant::Participant(int p, std::string path){
	generateAllTargets();
	pID = p ;
	filepath = path ;
	conditions = {1,2,3,4} ;
	getPermutationCondition();
	currentConditionID = 0 ;
	currentTargetID = 0 ;
	logWritten = false ;
	shouldLog = false ;
}

Participant::Participant(){
	generateAllTargets();
	conditions = {1,2,3,4} ;
	currentConditionID = 0 ;
	currentTargetID = 0 ;
	logWritten = false ;
	shouldLog = false ;
}

void Participant::setValues(int p, std::string path){
	pID = p ;
	filepath = path+"log/" ;
	printAny(filepath, "FilePath Set to ");
	logWritten = false ;
	getPermutationCondition();
	getPermutationTrials();
	shouldLog = false ;
}

void Participant::logging(bool b){
	shouldLog = b ;
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
        printAny(i, "AAAAA");
        if( i != 0 && r != 0){
            std::tuple<Vector3,Quaternion,int> temp = targets[i]; 
            targets[i] = targets[r]; 
            targets[r] = temp;
        }
        
    }

}

//Has to be called after the reset
int Participant::getCondition(){
	return conditions[currentConditionID] ;
}

void Participant::performLog(){
	//See http://stackoverflow.com/questions/11294487/android-writing-saving-files-from-native-code-only

    // First we check whether log exists
    struct stat sb;
    int32_t res = stat(filepath.c_str(), &sb);
    LOGD("LOGWRITING path = %s",filepath.c_str());
    LOGD("LOGWRITING trial number %d",currentTargetID);
    if (0 == res && sb.st_mode & S_IFDIR)
    {
        LOGD("LOGWRITING 'log/' dir already in app's internal data storage.");
    }
    else if (ENOENT == errno)
    {
        res = mkdir(filepath.c_str(), 0770);
        LOGD("LOGWRITING no directory log");
    }

    if (0 == res)
    {

    	std::string cond = "";
    	switch(conditions[currentConditionID]){
    		case 1:
    			cond = "rate_control";
    			break ;
    		case 2:
    			cond = "speed_control";
    			break ;
    		case 3:
    			cond = "pressure_control";
    			break ;
    		case 4:
    			cond = "slider_control";
    			break ;
    	}
    	

    	//Then we check whether this participant folder exists
    	std::string finalPath = filepath +"P"+to_string(pID) + "/";
    	LOGD("LOGWRITING path = %s",finalPath.c_str());
    	res = stat(finalPath.c_str(), &sb);
	    if (0 == res && sb.st_mode & S_IFDIR)
	    {
	        LOGD("LOGWRITING 'PiD/' dir already in app's internal data storage.");
	    }
	    else if (ENOENT == errno)
	    {
	        res = mkdir(finalPath.c_str(), 0770);
	        LOGD("LOGWRITING no directory Pid");
	    }

	    if (0 == res){


	    	//Finally we check for the condition folder
	    	finalPath += cond + "/";
	    	LOGD("LOGWRITING path = %s",finalPath.c_str());
	    	res = stat(finalPath.c_str(), &sb);
		    if (0 == res && sb.st_mode & S_IFDIR)
		    {
		        LOGD("LOGWRITING 'Condition' dir already in app's internal data storage.");
		    }
		    else if (ENOENT == errno)
		    {
		        res = mkdir(finalPath.c_str(), 0770);
		        LOGD("LOGWRITING no directory condition");
		    }

		    if (0 == res){
		    	finalPath += to_string(currentTargetID)+".csv" ;
				LOGD("LOGWRITING File log is %s \n", finalPath.c_str());
				
		    	FILE* logFile = std::fopen(finalPath.c_str(), "w+");
		    	std::string line ;

		    	if(logFile!=NULL){
		    		//The header first
		    		line = "Timestamp;CurrentConditionID;CurrentTargetID;Precision;DataPosition;DataOrientation;EuclideanDist;AngularDist;TargetPosition;TargetOrientation" ;
			    	fputs(line.c_str(), logFile);
			        fflush(logFile);
			        //The we populate the file with the data
		    		for(int i = 0 ; i < logPositions.size(); i++){
		    			line = 	 to_string(timestamps[i])+";"+to_string(conditions[currentConditionID])+";"+to_string(std::get<2>(targets[currentTargetID]))+";"
		    					+to_string(precision[i])+";"+to_string(std::get<0>(logPositions[i]))+";"+to_string(std::get<1>(logPositions[i]))+";"
		    					+to_string(std::get<0>(logDiffValues[i]))+";"+to_string(std::get<1>(logDiffValues[i]))
		    					+";"+to_string(std::get<0>(targets[currentTargetID]))+";"+to_string(std::get<1>(targets[currentTargetID]))+"\n" ;
		    			
		    			fputs(line.c_str(), logFile);
			        	fflush(logFile);
		    		}
		    	}
		    	std::fclose(logFile);

		    }
	    }

    }
}

/*void Participant::writeLog(std::string finalPath){
	FILE* logFile = std::fopen(finalPath.c_str(), "w+");
	std::string line ;

	if(logFile!=NULL){
		//The header first
		line = "Timestamp;CurrentConditionID;CurrentTargetID;Precision;DataPosition;DataOrientation;EuclideanDist;AngularDist;TargetPosition;TargetOrientation" ;
    	fputs(line.c_str(), logFile);
        fflush(logFile);
        //The we populate the file with the data
		for(int i = 0 ; i < logPositions.size(); i++){
			line = 	 to_string(timestamps[i])+";"+to_string(conditions[currentConditionID])+";"+to_string(std::get<2>(targets[currentTargetID]))+";"
					+to_string(precision[i])+";"+to_string(std::get<0>(logPositions[i]))+";"+to_string(std::get<1>(logPositions[i]))+";"
					+to_string(std::get<0>(logDiffValues[i]))+";"+to_string(std::get<1>(logDiffValues[i]))
					+";"+to_string(std::get<0>(targets[currentTargetID]))+";"+to_string(std::get<1>(targets[currentTargetID]))+"\n" ;
			
			fputs(line.c_str(), logFile);
        	fflush(logFile);
		}
	}
	std::fclose(logFile);
}*/


bool Participant::hasFinishedLog(){
	return logWritten ;
}

void Participant::resetTrial(){
	//First we need to log everything
	shouldLog = false ;
	performLog();
	currentTargetID ++ ;
	LOGD("LOGWRITING has been done and currentTargetID has been increased and equals %d", currentTargetID);
	if(currentTargetID%15 == 0){
		LOGD("LOG Changed condition");
		resetCondition();
	}
	logWritten = true ;
	
}

void Participant::clearVectors(){
	logPositions.clear();
	precision.clear();
	timestamps.clear();
	logDiffValues.clear();
}
void Participant::resetCondition(){
	currentTargetID = 0 ;
	currentConditionID ++ ;
	generateAllTargets();
	getPermutationTrials();

}
void Participant::addData(Vector3 currentPos, Quaternion currentRot, float prec, int timestamp){
	if(shouldLog){
		logWritten = false ;

		logPositions.push_back(std::tuple<Vector3,Quaternion>(currentPos, currentRot));
		precision.push_back(prec);
		timestamps.push_back(timestamp*TIMELOG);
		
		float eucli = euclideandist(currentPos,std::get<0>(targets[currentConditionID]));
		Quaternion directionRot = currentRot * std::get<1>(targets[currentConditionID]).inverse();
		float angular = 2 * safe_acos(directionRot.w);

		logDiffValues.push_back(std::tuple<float,float>(eucli,angular));
	}
	
}

Matrix4 Participant::getTargetMatrix(){
	return Matrix4::makeTransform(std::get<0>(targets[currentTargetID]), std::get<1>(targets[currentTargetID]));
}

void Participant::generateAllTargets(){
	targets.clear(); 	//Just to make sure that if called twice it won't mess up the target generation
	targets.push_back(std::tuple<Vector3,Quaternion,int>(Vector3(34.4052, 20.5995, 110.312)		,Quaternion(0.17284, -0.027112, 0.982298, -0.0668766)	,1));
	targets.push_back(std::tuple<Vector3,Quaternion,int>(Vector3(-29.6198, 17.7426, 90.7622)	,Quaternion(-0.119085, 0.160843, 0.919453, 0.338448)	,2));
	targets.push_back(std::tuple<Vector3,Quaternion,int>(Vector3(-25.3372, 2.28942, 75.3072)	,Quaternion(-0.463481, -0.118306, 0.875867, -0.0635474)	,3));
	targets.push_back(std::tuple<Vector3,Quaternion,int>(Vector3(24.8166, -2.20706, 75.3191)	,Quaternion(-0.356861, 0.233901, 0.837369, -0.34167)	,4));
	targets.push_back(std::tuple<Vector3,Quaternion,int>(Vector3(26.937, 1.94701, 109.407)		,Quaternion(0.210963, 0.809764, 0.245368, -0.489443)	,5));
	targets.push_back(std::tuple<Vector3,Quaternion,int>(Vector3(-3.45967, -12.5448, 91.4293)	,Quaternion(0.728339, 0.0348798, 0.436378, -0.527114)	,6));
	targets.push_back(std::tuple<Vector3,Quaternion,int>(Vector3(34.212, -11.3861, 130.689)		,Quaternion(0.564828, 0.113259, 0.703096, -0.416848)	,7));
	targets.push_back(std::tuple<Vector3,Quaternion,int>(Vector3(13.1964, 2.97829, 107.047)		,Quaternion(-0.191473, 0.427081, 0.842703, 0.265989)	,8));
	targets.push_back(std::tuple<Vector3,Quaternion,int>(Vector3(-22.2263, -0.264936, 110.766)	,Quaternion(-0.0805761, 0.229052, 0.335052, 0.910351)	,9));
	targets.push_back(std::tuple<Vector3,Quaternion,int>(Vector3(16.9274, 18.0633, 158.765)		,Quaternion(-0.959303, 0.22141, 0.0488697, 0.168163)	,10));
	targets.push_back(std::tuple<Vector3,Quaternion,int>(Vector3(30.2101, 7.40269, 86.2055)		,Quaternion(-0.748447, 0.645197, 0.134274, -0.0739392)	,11));
	targets.push_back(std::tuple<Vector3,Quaternion,int>(Vector3(-13.2588, 5.63171, 139.974)	,Quaternion(-0.541399, 0.653179, -0.291624, -0.441754)	,12));
	targets.push_back(std::tuple<Vector3,Quaternion,int>(Vector3(11.206, 8.74678, 76.2941)		,Quaternion(0.751457, 0.222842, 0.268716, -0.5598)		,13));
	targets.push_back(std::tuple<Vector3,Quaternion,int>(Vector3(33.7071, 3.05983, 101.892)		,Quaternion(0.692698, -0.176285, -0.0476433, -0.697677)	,14));
	targets.push_back(std::tuple<Vector3,Quaternion,int>(Vector3(-3.92666, -12.46, 98.5911)		,Quaternion(-0.236857, -0.725444, 0.576961, -0.29098)	,15));
	//targets.push_back(std::tuple<Vector3,Quaternion,int>(Vector3(-4.42321, 13.177, 93.1823)		,Quaternion(-0.637159, -0.714029, 0.249659, -0.147583)		,16));
	//targets.push_back(std::tuple<Vector3,Quaternion,int>(Vector3(31.721, -8.42904, 124.926)		,Quaternion(-0.920161, -0.359409, -0.148246, -0.0454646)	,17));
	//targets.push_back(std::tuple<Vector3,Quaternion,int>(Vector3(30.2151, 21.5581, 102.932)		,Quaternion(-0.933026, -0.275187, -0.226801, -0.0470352)	,18));

}