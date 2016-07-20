package fr.limsi.ARViewer;

public final class Utils{

	public static float convertIntoNewRange(float newRangeMin, float newRangeMax,
											float oldRangeMin, float oldRangeMax, float value){

		float oldRange = oldRangeMax - oldRangeMin ;
		float newRange = newRangeMax - newRangeMin ;
		value = (((value - oldRangeMin) * newRange)/oldRange)+newRangeMin;
		
		return value ;

	}

}


