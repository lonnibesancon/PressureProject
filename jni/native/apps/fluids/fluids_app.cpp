#include "fluids_app.h"

#include "apps/app.h"

#include "vtk_output_window.h"
#include "vtk_error_observer.h"
#include "volume.h"
#include "volume3d.h"
#include "isosurface.h"
#include "slice.h"
#include "rendering/cube.h"
#include "tracking/multi_marker.h"
#include "tracking/multi_marker_objects.h"
#include "loaders/loader_obj.h"
#include "loaders/loader_ply.h"
#include "rendering/mesh.h"
#include "rendering/lines.h"

#include <array>
#include <time.h> 

#include <vtkSmartPointer.h>
#include <vtkNew.h>
#include <vtkDataSetReader.h>
#include <vtkXMLImageDataReader.h>
#include <vtkImageData.h>
#include <vtkImageResize.h>
#include <vtkPointData.h>
#include <vtkDataArray.h>
#include <vtkProbeFilter.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>

#include <QCAR/QCAR.h>
#include <QCAR/CameraDevice.h>
#include <QCAR/Renderer.h>
#include <QCAR/VideoBackgroundConfig.h>
#include <QCAR/Trackable.h>
#include <QCAR/TrackableResult.h>
#include <QCAR/Tool.h>
#include <QCAR/Tracker.h>
#include <QCAR/TrackerManager.h>
#include <QCAR/ImageTracker.h>
#include <QCAR/CameraCalibration.h>
#include <QCAR/UpdateCallback.h>
#include <QCAR/DataSet.h>
#include "interactionMode.h"
// #include <QCAR/Image.h>

#include <functional>
#include <chrono>
#include <future>
#include <cstdio>



#include <vtkXMLPolyDataReader.h>

#include "exp/participant.h"

#define NEW_STYLUS_RENDER
//#define VTK

// ======================================================================
// JNI interface

extern "C" {
	JNIEXPORT jboolean JNICALL Java_fr_limsi_ARViewer_FluidMechanics_loadDataset(JNIEnv* env, jobject obj, jstring filename);
	JNIEXPORT jboolean JNICALL Java_fr_limsi_ARViewer_FluidMechanics_loadVelocityDataset(JNIEnv* env, jobject obj, jstring filename);

	// JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_initQCAR(JNIEnv* env, jobject obj);


	JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_buttonPressed(JNIEnv* env, jobject obj);
	JNIEXPORT jfloat JNICALL Java_fr_limsi_ARViewer_FluidMechanics_buttonReleased(JNIEnv* env, jobject obj);

	JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_getSettings(JNIEnv* env, jobject obj, jobject settingsObj);
	JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_setSettings(JNIEnv* env, jobject obj, jobject settingsObj);

	JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_getState(JNIEnv* env, jobject obj, jobject stateObj);
	JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_setTangoValues(JNIEnv* env, jobject obj, jdouble tx, jdouble ty, jdouble tz, jdouble rx, jdouble ry, jdouble rz, jdouble q);
    JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_setGyroValues(JNIEnv* env, jobject obj, jdouble rx, jdouble ry, jdouble rz, jdouble q);
    JNIEXPORT jstring JNICALL Java_fr_limsi_ARViewer_FluidMechanics_getData(JNIEnv* env, jobject obj);
    JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_setInteractionMode(JNIEnv* env, jobject obj, jint mode);
    JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_updateFingerPositions(JNIEnv* env, jobject obj, jfloat x, jfloat y, jint fingerID);
    JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_addFinger(JNIEnv* env, jobject obj, jfloat x, jfloat y, jint fingerID);
    JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_removeFinger(JNIEnv* env, jobject obj, jint fingerID);
    JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_reset(JNIEnv* env, jobject obj);
    JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_launchTrial(JNIEnv* env, jobject obj);
    JNIEXPORT jboolean JNICALL Java_fr_limsi_ARViewer_FluidMechanics_isTrialOver(JNIEnv* env, jobject obj);
    JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_setPId(JNIEnv* env, jobject obj, jint p);
    JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_hasFinishedLog(JNIEnv* env, jobject obj);
    JNIEXPORT int JNICALL Java_fr_limsi_ARViewer_FluidMechanics_getCondition(JNIEnv* env, jobject obj);
    //Initialize everything to call a java function
    JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_initJNI(JNIEnv* env, jobject obj);
    JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_endTrialJava();
    JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_isEgo(JNIEnv* env, jobject obj, jboolean ego);
    JNIEXPORT int JNICALL Java_fr_limsi_ARViewer_FluidMechanics_getTime(JNIEnv* env, jobject obj);
    JNIEXPORT jfloat JNICALL Java_fr_limsi_ARViewer_FluidMechanics_getValueSlider(JNIEnv* env, jobject obj);

}

// (end of JNI interface)
// ======================================================================

struct Particle
{
	Vector3 pos;
	bool valid;
	int delayMs, stallMs;
	timespec lastTime;
};

//For calling java functions from NDK

jclass javaClassRef;
jmethodID javaMethodRef;
JNIEnv* env ;
jobject obj ;
static JavaVM* g_jvm = 0;
int nbSeconds = TIME/1000000 ;


// See http://stackoverflow.com/questions/14650885/how-to-create-timer-events-using-c-11

/*template <class callable, class... arguments>
void later(int after, bool async, callable&& f, arguments&&... args)
{
    std::function<typename std::result_of<callable(arguments...)>::type()> task(std::bind(std::forward<callable>(f), std::forward<arguments>(args)...));

    if (async)
    {
        std::thread([after, task]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(after));
            task();
        }).detach();
    }
    else
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(after));
        task();
    }
}*/



struct FluidMechanics::Impl
{
	Impl(const std::string& baseDir);

	bool loadDataSet(const std::string& fileName);
	bool loadVelocityDataSet(const std::string& fileName);

	// void initQCAR();

	template <typename T>
	vtkSmartPointer<vtkImageData> loadTypedDataSet(const std::string& fileName);

	// (GL context)
	void rebind();

	// void detectObjects(ARMarkerInfo* markerInfo, int markerNum);
	void setMatrices(const Matrix4& volumeMatrix, const Matrix4& stylusMatrix);
	void updateMatrices();

	// (GL context)
	void renderObjects();

	void updateSurfacePreview();

	void buttonPressed();
	float buttonReleased();

	Vector3 particleJitter();

	bool computeCameraClipPlane(Vector3& point, Vector3& normal);
	bool computeAxisClipPlane(Vector3& point, Vector3& normal);
	bool computeStylusClipPlane(Vector3& point, Vector3& normal);

	void setTangoValues(double tx, double ty, double tz, double rx, double ry, double rz, double q);
	void setGyroValues(double rx, double ry, double rz, double q);
	std::string getData();
	void setInteractionMode(int mode);
	void updateFingerPositions(float x, float y, int fingerID);
	void addFinger(float x, float y, int fingerID);
	void removeFinger(int fingerID);
	void computeFingerInteraction();
	void reset();
	int getFingerPos(int fingerID);
	void onTranslateBar(float pos);
	float convertIntoNewRange(float oldRangeMin, float oldRangeMax, float value);
	void computeAngular();
	void computeEucli();
	void setPId(int p);
	bool hasFinishedLog();
	int getCondition();
	float getValueSlider();




	Vector3 posToDataCoords(const Vector3& pos); // "pos" is in eye coordinates
	Vector3 dataCoordsToPos(const Vector3& dataCoordsToPos);

	Quaternion currentSliceRot ;
	Vector3 currentSlicePos ;
	Quaternion currentDataRot ;
	Vector3 currentDataPos ;

	//Quaternion and vector to represent previous positions and orientation so as to compute angular speed and speed
	Quaternion previousRot ;
	Vector3 previousPos ;
	

	//Quaternion and vector to represent the position and orientation for the rate control mode
	//Also quaternion and vector to represent the currentTab which is not synchronized anymore with the data
	Quaternion centerRot ;
	Vector3 centerPos ;
	Quaternion tabRot;
	Vector3 tabPos ;
	Vector3 directionMov = Vector3::zero();

	float teta = 1 ;
	float eucli = 1 ;
	
	Quaternion directionRot ;


	//Quaternion representing the orientation of the tablet no matter the interaction mode or constrains
	Quaternion currentTabRot ;

	Synchronized<std::vector<Vector3> > fingerPositions;
	Synchronized<std::vector<Vector3> > prevFingerPositions ;
	Vector2 initialVector ;
	Vector3 prevVec ;
	float translateBarPrevPos ;
	bool isAboveThreshold = false ;
	bool mInitialPinchDistSet = false ;
	Synchronized<Vector3> seedingPoint ;
	float screenW = 1920 ;
	float screenH = 1104 ;
	float mInitialZoomFactor ;
	float mInitialPinchDist ;



	bool tangoEnabled = false ;
	int interactionMode = dataTangible ;
	bool seedPointPlacement = false ;



	FluidMechanics* app;
	std::shared_ptr<FluidMechanics::Settings> settings;
	std::shared_ptr<FluidMechanics::State> state;

	Synchronized<MultiMarker> tangible;
	Synchronized<MultiMarker> stylus;

	CubePtr cube, axisCube;

	vtkSmartPointer<vtkImageData> data, dataLow;
	int dataDim[3];
	Vector3 dataSpacing;

	vtkSmartPointer<vtkImageData> velocityData;

	typedef LinearMath::Vector3<int> DataCoords;
	// static constexpr int particleReleaseDuration = 500; // ms

	// static constexpr float stylusEffectorDist = 20.0f;
	static constexpr float stylusEffectorDist = 24.0f;
	// static constexpr float stylusEffectorDist = 30.0f;

	Synchronized<VolumePtr> volume;
	//Synchronized<MeshPtr> volume;
	// Synchronized<Volume3dPtr> volume;
	Synchronized<IsoSurfacePtr> isosurface, isosurfaceLow;
	Synchronized<SlicePtr> slice;
	Synchronized<CubePtr> outline;
	Vector3 slicePoint, sliceNormal;
	float sliceDepth;
	Synchronized<std::vector<Vector3>> slicePoints; // max size == 6

	MeshPtr particleSphere, cylinder;
	MeshPtr bunny ;
	MeshPtr bunnytarget;
	LinesPtr lines;


	//Matching part
	std::vector<Matrix4> planeMatrices ;
	std::vector<Matrix4> dataMatrices ;
	int targetId = 0 ;

	// Matrix4 qcarProjMatrix;
	// QCAR::DataSet* dataSetStonesAndChips;
	// Synchronized<Matrix4> qcarModelMatrix; // XXX: test
	// bool qcarVisible; // XXX: test

	vtkSmartPointer<vtkProbeFilter> probeFilter;

	Synchronized<Vector3> effectorIntersection;
	// Vector3 effectorIntersectionNormal;
	bool effectorIntersectionValid;

	bool buttonIsPressed;

	bool ego = false ;
	void isEgo(bool ego);


	//Trial Handling
	void launchTrial();
	bool isTrialOver();
	void endTrial();
	void log();
	void timer();
	void initJNI();
	bool isOver = true ;
	Participant participant ;
	int logNumber = 0 ;
	std::string directory ;
	Matrix4 tm ;
	Matrix4 targetMatrix = Matrix4::makeTransform(Vector3(34.4052, 20.5995, 100.312), Quaternion(0.17284, -0.127112, 0.982298, -0.0668766));
	Quaternion forSliderQ ;
	Vector3 forSliderV ;
	float getMaxComponentofVector(Vector3 v);

};


FluidMechanics::Impl::Impl(const std::string& baseDir)
 : currentSliceRot(Quaternion(Vector3::unitX(), M_PI)),
   currentSlicePos(Vector3::zero()),
   currentDataPos(Vector3::zero()),
   currentDataRot(Quaternion(Vector3::unitX(), M_PI)),
   prevVec(Vector3::zero()),
   currentTabRot(Quaternion(Vector3::unitX(), M_PI)),
   buttonIsPressed(false) 
{
	seedingPoint = Vector3(-10000.0,-10000.0,-10000.0);
	cube.reset(new Cube);
	axisCube.reset(new Cube(true));
	particleSphere = LoaderOBJ::load(baseDir + "/sphere.obj");
	cylinder = LoaderOBJ::load(baseDir + "/cylinder.obj");
	bunny = LoaderOBJ::load(baseDir +"/bunny.obj");
	bunnytarget = LoaderOBJ::load(baseDir+"/bunny.obj");
	lines.reset(new Lines);


	previousPos = currentDataPos ;
	previousRot = currentDataRot ;
	

	isAboveThreshold = false ;
	directionMov = Vector3(1,1,1);
	directionRot = currentDataRot ;

	
	targetId = 0 ;
	directory = baseDir +"/";

}

void FluidMechanics::Impl::initJNI(){

}

void FluidMechanics::Impl::launchTrial(){
	settings->isTraining = false ;
	settings->precision = 1 ;
	LOGD("LaunchTrial with pID value = %d", settings->pID);
	//participant.setValues(settings->pID,directory);
	isOver = false ;
	interactionMode = dataTangible ;
	reset();
	//participant.clearVectors();
	participant.logging(true);	
	std::thread timerTrial(&FluidMechanics::Impl::endTrial,this);
	//std::thread timerLog(&FluidMechanics::Impl::log,this);
	//timerLog.detach();
	timerTrial.detach();
	//main is blocked until funcTest1 is not finished
	//timerTrial.join();
}

bool FluidMechanics::Impl::isTrialOver(){
	//TOFIX
	//endTrial();
	printAny(isOver,"TrialOver");
	
	return isOver;
}

bool FluidMechanics::Impl::hasFinishedLog(){
	printAny(participant.hasFinishedLog(), "TrialOver Log done");
	return participant.hasFinishedLog() ;
}

void FluidMechanics::Impl::timer(){
	
}

void FluidMechanics::Impl::endTrial(){
	int nbSteps = TIME/TIMELOG ;
	for(int i = 0 ; i < nbSteps ; i++){
		std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(50));
		participant.addData(currentDataPos, currentDataRot, settings->precision, logNumber);
		logNumber++ ;
		if(i % 20 == 0){
			nbSeconds -- ;	
		}

	}
	participant.addData(currentDataPos, currentDataRot, settings->precision, logNumber);
	isOver = true ;
	reset();
	participant.resetTrial();
	settings->controlType = participant.getCondition();
	interactionMode = 0 ; 
	return ;
}

void FluidMechanics::Impl::isEgo(bool b){
	ego = b ;
}

void FluidMechanics::Impl::log(){
	while(isOver == false){
		usleep(TIMELOG);
		LOGD("Second timer for collecting data");	
		participant.addData(currentDataPos, currentDataRot, settings->precision, logNumber);
		printAny(settings->precision, "TESTPRESS");
		logNumber++ ;
	}
	return ;

}

void FluidMechanics::Impl::setPId(int p){
	participant.setValues(p,directory);
}

int FluidMechanics::Impl::getCondition(){
	/*std::string techniqueName ;
	LOGD("TECHNIQUENAME %d",settings->controlType);
	LOGD("TECHNIQUENAME Participant %d",participant.getCondition());
	switch(participant.getCondition()){
        case PRESSURE_CONTROL:
            techniqueName = "Pressure Control";
            break;
        case PRESSURE_CONTROL_REVERSE:
            techniqueName = "Reverse Pressure Control";
            break ;
        case SPEED_CONTROL:
            techniqueName = "Speed control";
            break;
        case RATE_CONTROL:
            techniqueName = "Rate Control";
            break ;
        case SLIDER_CONTROL:
            techniqueName = "Slider Control";
            break ;
    }
	return techniqueName;*/
	return participant.getCondition();
}




void FluidMechanics::Impl::reset(){
	seedingPoint = Vector3(-1,-1,-1);
	currentSliceRot = Quaternion(Vector3::unitX(), M_PI);
	currentDataRot = Quaternion(Vector3::unitX(), M_PI);
	currentSlicePos = Vector3(0, 0, 400);
	currentDataPos = Vector3(0,0,400);
	buttonIsPressed = false ;
	previousPos = currentDataPos ;
	previousRot = currentDataRot ;
	centerPos = currentDataPos ;
	centerRot = currentDataRot ;
	directionMov = Vector3::zero();
	logNumber = 0 ;
	nbSeconds = TIME/1000000 ;

	setMatrices(Matrix4::makeTransform(Vector3(0, 0, 400)),Matrix4::makeTransform(Vector3(0, 0, 400)));
	
}

float FluidMechanics::Impl::convertIntoNewRange(float oldRangeMin, float oldRangeMax, float value){
	//http://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio
	float oldRange = oldRangeMax - oldRangeMin ;
	float newRange = MAXPRECISION - MINPRECISION ;
	value = (((value - oldRangeMin) * newRange)/oldRange)+MINPRECISION ;
	return value ;
}


void FluidMechanics::Impl::rebind()
{
	LOGD("OpenGL version: %s", glGetString(GL_VERSION));
	LOGD("GLSL version: %s", glGetString(GL_SHADING_LANGUAGE_VERSION));
	LOGD("OpenGL extensions: %s", glGetString(GL_EXTENSIONS));

	cube->bind();
	axisCube->bind();
	lines->bind();
	particleSphere->bind();
	cylinder->bind();

	synchronized_if(volume) { volume->bind(); }
	synchronized_if(isosurface) { isosurface->bind(); }
	synchronized_if(isosurfaceLow) { isosurfaceLow->bind(); }
	synchronized_if(slice) { slice->bind(); }
	synchronized_if(outline) { outline->bind(); }
}

template <typename T>
vtkSmartPointer<vtkImageData> FluidMechanics::Impl::loadTypedDataSet(const std::string& fileName)
{
	vtkNew<T> reader;

	LOGI("Loading file: %s...", fileName.c_str());
	reader->SetFileName(fileName.c_str());

	vtkNew<VTKErrorObserver> errorObserver;
	reader->AddObserver(vtkCommand::ErrorEvent, errorObserver.GetPointer());

	reader->Update();

	if (errorObserver->hasError()) {
		// TODO? Throw a different type of error to let Java code
		// display a helpful message to the user
		throw std::runtime_error("Error loading data: " + errorObserver->getErrorMessage());
	}

	vtkSmartPointer<vtkImageData> data = vtkSmartPointer<vtkImageData>::New();
	data->DeepCopy(reader->GetOutputDataObject(0));

	return data;
}

bool FluidMechanics::Impl::loadDataSet(const std::string& fileName)
{
	// // Unload mesh data
	// mesh.reset();

	VTKOutputWindow::install();



	std::string filename("teapot.vtp") ;
	//const std::string ext = filename.substr(fileName.find_last_of(".") + 1);
	const std::string ext = fileName.substr(fileName.find_last_of(".") + 1);
	if (ext == "vtk")
		data = loadTypedDataSet<vtkDataSetReader>(fileName);
	else if (ext == "vti")
		data = loadTypedDataSet<vtkXMLImageDataReader>(fileName);
	else

		#if 0
			throw std::runtime_error("Error loading data: unknown extension: \"" + ext + "\"");
		#else
			data = loadTypedDataSet<vtkXMLPolyDataReader>(fileName);
		#endif


	data->GetDimensions(dataDim);

	double spacing[3];
	data->GetSpacing(spacing);
	dataSpacing = Vector3(spacing[0], spacing[1], spacing[2]);

	// Compute a default zoom value according to the data dimensions
	// static const float nativeSize = 128.0f;
	static const float nativeSize = 110.0f;
	state->computedZoomFactor = nativeSize / std::max(dataSpacing.x*dataDim[0], std::max(dataSpacing.y*dataDim[1], dataSpacing.z*dataDim[2]));
	// FIXME: hardcoded value: 0.25 (minimum zoom level, see the
	// onTouch() handler in Java code)
	state->computedZoomFactor = std::max(state->computedZoomFactor, 0.25f);

	dataLow = vtkSmartPointer<vtkImageData>::New();
	vtkNew<vtkImageResize> resizeFilter;
	resizeFilter->SetInputData(data.GetPointer());
	resizeFilter->SetOutputDimensions(std::max(dataDim[0]/3, 1), std::max(dataDim[1]/3, 1), std::max(dataDim[2]/3, 1));
	resizeFilter->InterpolateOn();
	resizeFilter->Update();
	dataLow->DeepCopy(resizeFilter->GetOutput());

	probeFilter = vtkSmartPointer<vtkProbeFilter>::New();
	probeFilter->SetSourceData(data.GetPointer());

	synchronized(outline) {
		LOGD("creating outline...");
		outline.reset(new Cube(true));
		outline->setScale(Vector3(dataDim[0]/2, dataDim[1]/2, dataDim[2]/2) * dataSpacing);
	}

	synchronized(volume) {
		LOGD("creating volume...");
		volume.reset(new Volume(data));
		// volume.reset(new Volume3d(data));
		if (fileName.find("FTLE7.vtk") != std::string::npos) { // HACK
			// volume->setOpacity(0.25f);
			volume->setOpacity(0.15f);
		}
	}

	if (fileName.find("FTLE7.vtk") == std::string::npos) { // HACK
		synchronized(isosurface) {
			LOGD("creating isosurface...");
			isosurface.reset(new IsoSurface(data));
			isosurface->setPercentage(settings->surfacePercentage);
		}

		synchronized(isosurfaceLow) {
			LOGD("creating low-res isosurface...");
			isosurfaceLow.reset(new IsoSurface(dataLow, true));
			isosurfaceLow->setPercentage(settings->surfacePercentage);
		}
	} else {
		isosurface.reset();
		isosurfaceLow.reset();
	}

	synchronized(slice) {
		LOGD("creating slice...");
		slice.reset(new Slice(data));
	}

	return true;
}

bool FluidMechanics::Impl::loadVelocityDataSet(const std::string& fileName)
{
	if (!data)
		throw std::runtime_error("No dataset currently loaded");

	VTKOutputWindow::install();

	const std::string ext = fileName.substr(fileName.find_last_of(".") + 1);

	if (ext == "vtk")
		velocityData = loadTypedDataSet<vtkDataSetReader>(fileName);
	else if (ext == "vti")
		velocityData = loadTypedDataSet<vtkXMLImageDataReader>(fileName);
	else
		throw std::runtime_error("Error loading data: unknown extension: \"" + ext + "\"");

	int velocityDataDim[3];
	velocityData->GetDimensions(velocityDataDim);

	if (velocityDataDim[0] != dataDim[0]
	    || velocityDataDim[1] != dataDim[1]
	    || velocityDataDim[2] != dataDim[2])
	{
		throw std::runtime_error(
			"Dimensions do not match: "
			"vel: " + Utility::toString(velocityDataDim[0]) + "x" + Utility::toString(velocityDataDim[1]) + "x" + Utility::toString(velocityDataDim[2])
			+ ", data: " + Utility::toString(dataDim[0]) + "x" + Utility::toString(dataDim[1]) + "x" + Utility::toString(dataDim[2])
		);
	}

	int dim = velocityData->GetDataDimension();
	if (dim != 3)
		throw std::runtime_error("Velocity data is not 3D (dimension = " + Utility::toString(dim) + ")");

	if (!velocityData->GetPointData() || !velocityData->GetPointData()->GetVectors())
		throw std::runtime_error("Invalid velocity data: no vectors found");

	return true;
}

Vector3 FluidMechanics::Impl::posToDataCoords(const Vector3& pos)
{
	Vector3 result;

	synchronized(state->modelMatrix) {
		// Transform "pos" into object space
		result = state->modelMatrix.inverse() * pos;
	}

	// Compensate for the scale factor
	result *= 1/settings->zoomFactor;

	// The data origin is on the corner, not the center
	result += Vector3(dataDim[0]/2, dataDim[1]/2, dataDim[2]/2) * dataSpacing;

	return result;
}

Vector3 FluidMechanics::Impl::particleJitter()
{
	return Vector3(
		(float(std::rand()) / RAND_MAX),
		(float(std::rand()) / RAND_MAX),
		(float(std::rand()) / RAND_MAX)
	) * 1.0f;
	// ) * 0.5f;
}

void FluidMechanics::Impl::buttonPressed()
{
	LOGD("BUTTON PRESSED");
	if(tangoEnabled == false){
		tangoEnabled = true ;

		centerPos = currentDataPos ;
		centerRot = currentDataRot ;
		tabRot = currentDataRot ;
		tabPos = currentDataPos ;
	}
	

	//For the rate control we have to memorize the initial position to be able to compute
	//The precision factor


}

float FluidMechanics::Impl::buttonReleased()
{
	LOGD("BUTTON RELEASED");
	if(tangoEnabled == true){
		tangoEnabled = false ;
		centerPos = currentDataPos ;
		centerRot = currentDataRot ;
	}
	

	return 0 ;
}

bool FluidMechanics::Impl::computeCameraClipPlane(Vector3& point, Vector3& normal)
{
	// static const float weight = 0.3f;
	// static const float weight = 0.5f;
	static const float weight = 0.8f;
	static bool wasVisible = false;
	static Vector3 prevPos;

	if (!state->tangibleVisible) {
		wasVisible = false;
		return false;
	}

	Matrix4 slicingMatrix;
	// synchronized(modelMatrix) { // not needed since this thread is the only one to write to "modelMatrix"
	// Compute the inverse rotation matrix to render this
	// slicing plane
	slicingMatrix = Matrix4((app->getProjMatrix() * state->modelMatrix).inverse().get3x3Matrix());
	// }

	// Compute the slicing origin location in data coordinates:

	// Center of the screen (at depth "clipDist")
	Vector3 screenSpacePos = Vector3(0, 0, settings->clipDist);

	// Transform the position in object space
	Vector3 pos = state->modelMatrix.inverse() * screenSpacePos;

	// Transform the screen normal in object space
	Vector3 n = (state->modelMatrix.transpose().get3x3Matrix() * Vector3::unitZ()).normalized();

	// Filter "pos" using a weighted average, but only in the
	// "n" direction (the screen direction)
	// TODO: Kalman filter?
	if (wasVisible)
		pos += -n.project(pos) + n.project(pos*weight + prevPos*(1-weight));
	wasVisible = true;
	prevPos = pos;

	// Transform the position back in screen space
	screenSpacePos = state->modelMatrix * pos;

	// Store the computed depth
	sliceDepth = screenSpacePos.z;

	// Unproject the center of the screen (at the computed depth
	// "sliceDepth"), then convert the result into data coordinates
	Vector3 pt = app->getProjMatrix().inverse() * Vector3(0, 0, app->getDepthValue(sliceDepth));
	Vector3 dataCoords = posToDataCoords(pt);
	slicingMatrix.setPosition(dataCoords);

	synchronized(slice) {
		slice->setSlice(slicingMatrix, sliceDepth, settings->zoomFactor);
	}

	point = pt;
	normal = -Vector3::unitZ();

	return true;
}

bool FluidMechanics::Impl::computeAxisClipPlane(Vector3& point, Vector3& normal)
{
	if (state->tangibleVisible) {
		Matrix3 normalMatrix = state->modelMatrix.inverse().transpose().get3x3Matrix();
		float xDot = (normalMatrix*Vector3::unitX()).normalized().dot(Vector3::unitZ());
		float yDot = (normalMatrix*Vector3::unitY()).normalized().dot(Vector3::unitZ());
		float zDot = (normalMatrix*Vector3::unitZ()).normalized().dot(Vector3::unitZ());
		// Prevent back and forth changes between two axis (unless no
		// axis is defined yet)
		const float margin = (state->clipAxis != CLIP_NONE ? 0.1f : 0.0f);
		if (std::abs(xDot) > std::abs(yDot)+margin && std::abs(xDot) > std::abs(zDot)+margin) {
			state->clipAxis = (xDot < 0 ? CLIP_AXIS_X : CLIP_NEG_AXIS_X);
		} else if (std::abs(yDot) > std::abs(xDot)+margin && std::abs(yDot) > std::abs(zDot)+margin) {
			state->clipAxis = (yDot < 0 ? CLIP_AXIS_Y : CLIP_NEG_AXIS_Y);
		} else if (std::abs(zDot) > std::abs(xDot)+margin && std::abs(zDot) > std::abs(yDot)+margin) {
			state->clipAxis = (zDot < 0 ? CLIP_AXIS_Z : CLIP_NEG_AXIS_Z);
		}

		if (state->lockedClipAxis != CLIP_NONE) {
			Vector3 axis;
			ClipAxis neg;
			switch (state->lockedClipAxis) {
				case CLIP_AXIS_X: axis = Vector3::unitX(); neg = CLIP_NEG_AXIS_X; break;
				case CLIP_AXIS_Y: axis = Vector3::unitY(); neg = CLIP_NEG_AXIS_Y; break;
				case CLIP_AXIS_Z: axis = Vector3::unitZ(); neg = CLIP_NEG_AXIS_Z; break;
				case CLIP_NEG_AXIS_X: axis = -Vector3::unitX(); neg = CLIP_AXIS_X; break;
				case CLIP_NEG_AXIS_Y: axis = -Vector3::unitY(); neg = CLIP_AXIS_Y; break;
				case CLIP_NEG_AXIS_Z: axis = -Vector3::unitZ(); neg = CLIP_AXIS_Z; break;
				default: android_assert(false);
			}
			float dot = (normalMatrix*axis).normalized().dot(Vector3::unitZ());
			if (dot > 0)
				state->lockedClipAxis = neg;
		}

	} else {
		state->clipAxis = state->lockedClipAxis = CLIP_NONE;
	}

	const ClipAxis ca = (state->lockedClipAxis != CLIP_NONE ? state->lockedClipAxis : state->clipAxis);

	if (ca == CLIP_NONE)
		return false;

	Vector3 axis;
	Quaternion rot;
	switch (ca) {
		case CLIP_AXIS_X: axis = Vector3::unitX(); rot = Quaternion(Vector3::unitY(), -M_PI/2)*Quaternion(Vector3::unitZ(), M_PI); break;
		case CLIP_AXIS_Y: axis = Vector3::unitY(); rot = Quaternion(Vector3::unitX(),  M_PI/2)*Quaternion(Vector3::unitZ(), M_PI); break;
		case CLIP_AXIS_Z: axis = Vector3::unitZ(); rot = Quaternion::identity(); break;
		case CLIP_NEG_AXIS_X: axis = -Vector3::unitX(); rot = Quaternion(Vector3::unitY(),  M_PI/2)*Quaternion(Vector3::unitZ(), M_PI); break;
		case CLIP_NEG_AXIS_Y: axis = -Vector3::unitY(); rot = Quaternion(Vector3::unitX(), -M_PI/2)*Quaternion(Vector3::unitZ(), M_PI); break;
		case CLIP_NEG_AXIS_Z: axis = -Vector3::unitZ(); rot = Quaternion(Vector3::unitX(),  M_PI); break;
		default: android_assert(false);
	}

	// Project "pt" on the chosen axis in object space
	Vector3 pt = state->modelMatrix.inverse() * app->getProjMatrix().inverse() * Vector3(0, 0, app->getDepthValue(settings->clipDist));
	Vector3 absAxis = Vector3(std::abs(axis.x), std::abs(axis.y), std::abs(axis.z));
	Vector3 pt2 = absAxis * absAxis.dot(pt);

	// Return to eye space
	pt2 = state->modelMatrix * pt2;

	Vector3 dataCoords = posToDataCoords(pt2);

	// static const float size = 128.0f;
	const float size = 0.5f * std::max(dataSpacing.x*dataDim[0], std::max(dataSpacing.y*dataDim[1], dataSpacing.z*dataDim[2]));

	Matrix4 proj = app->getProjMatrix(); proj[0][0] = -proj[1][1] / 1.0f; // same as "projMatrix", but with aspect = 1
	Matrix4 slicingMatrix = Matrix4((proj * Matrix4::makeTransform(dataCoords, rot)).inverse().get3x3Matrix());
	slicingMatrix.setPosition(dataCoords);
	synchronized(slice) {
		slice->setSlice(slicingMatrix, -proj[1][1]*size*settings->zoomFactor, settings->zoomFactor);
	}

	synchronized(state->sliceModelMatrix) {
		state->sliceModelMatrix = Matrix4(state->modelMatrix * Matrix4::makeTransform(state->modelMatrix.inverse() * pt2, rot, settings->zoomFactor*Vector3(size, size, 0.0f)));
	}

	if (!slice->isEmpty())
		state->lockedClipAxis = ca;
	else
		state->lockedClipAxis = CLIP_NONE;

	point = pt2;
	normal = state->modelMatrix.inverse().transpose().get3x3Matrix() * axis;

	return true;
}

// From: "Jittering Reduction in Marker-Based Augmented Reality Systems"
Matrix4 filter(const Matrix4& in, const Matrix4& prev, float posWeight, float rotWeight)
{
	// TODO: Kalman filter for position?

	Matrix4 result;

	for (unsigned int col = 0; col < 4; ++col) {
		for (unsigned int row = 0; row < 4; ++row) {
			if (row == 3) {
				// The last row is left unchanged
				result[col][row] = in[col][row];

			} else if (row == 0 && col < 3) {
				// Skip the first axis (side vector)
				continue;

			} else if (col < 3) { // orientation
				// Average the last 1/rotWeight values
				result[col][row] = in[col][row]*rotWeight + prev[col][row]*(1-rotWeight);

			} else { // position
				// Average the last 1/posWeight values
				result[col][row] = in[col][row]*posWeight + prev[col][row]*(1-posWeight);
			}
		}
	}

	Vector3 forward(result[0][2], result[1][2], result[2][2]);
	forward.normalize();
	result[0][2] = forward.x;
	result[1][2] = forward.y;
	result[2][2] = forward.z;

	Vector3 up(result[0][1], result[1][1], result[2][1]);
	up.normalize();

	// Recompute the side vector, then the up vector, to make sure the
	// coordinate system remains orthogonal

	Vector3 side = forward.cross(-up);
	side.normalize();
	result[0][0] = side.x;
	result[1][0] = side.y;
	result[2][0] = side.z;

	up = forward.cross(side);
	up.normalize();
	result[0][1] = up.x;
	result[1][1] = up.y;
	result[2][1] = up.z;

	return result;
}

bool FluidMechanics::Impl::computeStylusClipPlane(Vector3& point, Vector3& normal)
{
#if 0
	
#else
	if (!state->stylusVisible)
		return false;
#endif

	// FIXME: state->stylusModelMatrix may be invalid (non-invertible) in some cases
	try {

	const float size = 0.5f * (60.0f + std::max(dataSpacing.x*dataDim[0], std::max(dataSpacing.y*dataDim[1], dataSpacing.z*dataDim[2])));

	Matrix4 planeMatrix = state->stylusModelMatrix;

	// The slice will be rendered from the viewpoint of the plane
	Matrix4 proj = app->getProjMatrix(); proj[0][0] = -proj[1][1] / 1.0f; // same as "projMatrix", but with aspect = 1
	Matrix4 slicingMatrix = Matrix4((proj * planeMatrix.inverse() * state->modelMatrix).inverse().get3x3Matrix());

	Vector3 pt2 = planeMatrix * Vector3::zero();

	// Position of the stylus tip, in data coordinates
	Vector3 dataCoords = posToDataCoords(pt2);
	// LOGD("dataCoords = %s", Utility::toString(dataCoords).c_str());
	slicingMatrix.setPosition(dataCoords);

	synchronized(slice) {
		slice->setSlice(slicingMatrix, -proj[1][1]*size*settings->zoomFactor, settings->zoomFactor);
	}

	synchronized(state->sliceModelMatrix) {
		state->sliceModelMatrix = Matrix4(planeMatrix * Matrix4::makeTransform(Vector3::zero(), Quaternion::identity(), settings->zoomFactor*Vector3(size, size, 0.0f)));
	}

	point = pt2;
	normal = state->stylusModelMatrix.inverse().transpose().get3x3Matrix() * Vector3::unitZ();

	} catch (const std::exception& e) { LOGD("%s", e.what()); return false; }

	return true;
}

Vector3 FluidMechanics::Impl::dataCoordsToPos(const Vector3& dataCoords)
{
	Vector3 result = dataCoords;

	// The data origin is on the corner, not the center
	result -= Vector3(dataDim[0]/2, dataDim[1]/2, dataDim[2]/2) * dataSpacing;

	// Compensate for the scale factor
	result *= settings->zoomFactor;

	synchronized(state->modelMatrix) {
		// Transform "result" into eye space
		result = state->modelMatrix * result;
	}

	return result;
}

template <typename T>
T lowPassFilter(const T& cur, const T& prev, float alpha)
{ return prev + alpha * (cur-prev); }

void FluidMechanics::Impl::setMatrices(const Matrix4& volumeMatrix, const Matrix4& stylusMatrix)
{
	synchronized(state->modelMatrix) {
		state->modelMatrix = volumeMatrix;
	}

	synchronized(state->stylusModelMatrix) {
		state->stylusModelMatrix = stylusMatrix;
	}

}

void FluidMechanics::Impl::setInteractionMode(int mode){
	this->interactionMode = mode ;
}

void FluidMechanics::Impl::setTangoValues(double tx, double ty, double tz, double rx, double ry, double rz, double q){
	if(!data ){
		return ;
	}

	//settings->controlType = SPEED_CONTROL ;

	Vector3 vec(tx,ty,tz);

	if(tangoEnabled){
		//LOGD("Tango Enabled");
		Quaternion quat(rx,ry,rz,q);

		//LOGD("autoConstraint == %d",settings->autoConstraint);

		//Normal interaction
		Vector3 trans = quat.inverse() * (vec-prevVec);
		trans *= Vector3(1,-1,-1);	//Tango... -_-"
		trans *= 300 ;
		//trans.z *= -1 ;

		if(settings->controlType == NO_CONTROL){
			if(ego){
				trans *= -1 ;	
			}
			trans *= settings->precision ;
			currentDataPos +=trans ;	
			printAny(settings->precision,"NO_CONTROL");
			prevVec = vec ;
			return ;
		}

		/*if(settings->controlType == RATE_CONTROL_SIMPLE){
		//if(participant.getCondition() == RATE_CONTROL_SIMPLE){
			/*tabPos+=trans ;
			Vector3 coordinateCorrection(1,-1,-1);
			Vector3 diff = (tabPos-centerRot.inverse()*centerPos) ;
			printAny(diff,"RATE_CONTROL BEFORE");
			/*diff.x = convertIntoNewRange(0,100, diff.x);
			diff.y = convertIntoNewRange(0,100, diff.y);
			diff.z = convertIntoNewRange(0,100, diff.z);
			//currentDataPos = (centerRot.inverse()*coordinateCorrection * diff * 0.005) + currentDataPos ;
			currentDataPos = (diff * 0.02) + currentDataPos ;
			printAny(diff,"RATE_CONTROL");
			LOGD("RATE_CONTROL_SIMPLE");
			forSliderV = diff ;*/
			/*tabPos+=trans ;
			Vector3 coordinateCorrection(1,-1,-1);
			currentDataPos = (centerRot.inverse()*coordinateCorrection * (tabPos-centerPos) * 0.04) + currentDataPos ;
			LOGD("RATE_CONTROL_SIMPLE");*/
			tabPos+=trans ;
			Vector3 coordinateCorrection(1,-1,-1);
			Vector3 diff = (tabPos-/*centerRot.inverse()**/centerPos) ;
			printAny(diff,"RATE_CONTROL BEFORE");
			/*diff.x = convertIntoNewRange(1,100, diff.x);
			diff.y = convertIntoNewRange(1,100, diff.y);
			diff.z = convertIntoNewRange(1,100, diff.z);
			//currentDataPos = (centerRot.inverse()*coordinateCorrection * diff * 0.005) + currentDataPos ;
			currentDataPos = (diff * 0.02) + currentDataPos ;
			forSliderV = diff ;
			LOGD("RATE_CONTROL_SIMPLE");
			prevVec = vec ;
			return ;
		}

		


		if(settings->controlType == SPEED_CONTROL){
					computeEucli();
					trans*=eucli ;
					currentDataPos +=trans ;


					prevVec = vec ;
					return ;
		}

		if(settings->controlType == PRESSURE_CONTROL_REVERSE || settings->controlType == SLIDER_CONTROL){
			LOGD("SLIDERORPRESSURE %f",settings->precision) ;
			trans *= settings->precision ;
			currentDataPos +=trans ;

			prevVec = vec ;
			return ;	
		}

		

			if(settings->controlType == RATE_CONTROL){
				//if(participant.getCondition() == RATE_CONTROL){
					tabPos+=trans ;
					Vector3 diff = tabPos-centerPos ;
					diff.x = abs(diff.x);
					diff.y = abs(diff.y);
					diff.z = abs(diff.z);
					printAny(tabPos,"POSTAB");
					printAny(centerPos,"POSCENTER");
					printAny(diff,"RATE_CONTROL BEFORE");
					diff.x = convertIntoNewRange(0,50, diff.x);
					diff.y = convertIntoNewRange(0,50, diff.y);
					diff.z = convertIntoNewRange(0,50, diff.z);
					currentDataPos += trans * diff;
					printAny(diff,"RATE_CONTROL");



					prevVec = vec ;
					return ;
			}
		


		// END TESTING*/ 
		if( interactionMode == dataTangible || interactionMode == dataTouchTangible )
		{

			//if(settings->controlType == SPEED_CONTROL){

			if(participant.getCondition() == SPEED_CONTROL){
				if(ego){
					trans *= -1 ;	
				}
				computeEucli();
				trans*=eucli ;
				currentDataPos +=trans ;
			}
			//if(settings->controlType == RATE_CONTROL_SIMPLE){
			else if(participant.getCondition() == RATE_CONTROL_SIMPLE){
				/*tabPos+=trans ;
				Vector3 coordinateCorrection(1,-1,-1);
				currentDataPos = (centerRot.inverse()*coordinateCorrection * (tabPos-centerPos) * 0.04) + currentDataPos ;
				LOGD("RATE_CONTROL_SIMPLE");*/
				tabPos+=trans ;
				Vector3 coordinateCorrection(1,-1,-1);
				
				Vector3 diff = (tabPos-/*centerRot.inverse()**/centerPos) ;
				if(ego){
					diff *= -1 ;
				}
				printAny(diff,"RATE_CONTROL BEFORE");
				/*diff.x = convertIntoNewRange(1,100, diff.x);
				diff.y = convertIntoNewRange(1,100, diff.y);
				diff.z = convertIntoNewRange(1,100, diff.z);*/
				//currentDataPos = (centerRot.inverse()*coordinateCorrection * diff * 0.005) + currentDataPos ;
				currentDataPos = (diff * 0.02) + currentDataPos ;
				forSliderV = diff ;
				LOGD("RATE_CONTROL_SIMPLE");
			}
			else if(settings->controlType == RATE_CONTROL){
			//if(participant.getCondition() == RATE_CONTROL){
				tabPos+=trans ;
				Vector3 diff = tabPos-centerPos ;
				diff.x = abs(diff.x);
				diff.y = abs(diff.y);
				diff.z = abs(diff.z);
				diff.x = convertIntoNewRange(0, 100, diff.x);
				diff.y = convertIntoNewRange(0, 100, diff.y);
				diff.z = convertIntoNewRange(0, 100, diff.z);
				currentDataPos += trans * diff ;
				LOGD("RATE_CONTROL");
			}
			else if(participant.getCondition() == PRESSURE_CONTROL_REVERSE || participant.getCondition() == SLIDER_CONTROL){
				if(ego){
					trans *= -1 ;	
				}
				printAny(settings->precision, "PRECISIONVALUE");
				trans *= settings->precision ;
				currentDataPos +=trans ;	
			}
			else{
			currentDataPos +=trans ;
			}
		

			printAny(currentDataPos, "Data Pos");
		}
		
		//updateMatrices();
	}
	prevVec = vec ;

}


void FluidMechanics::Impl::setGyroValues(double rx, double ry, double rz, double q){
	if(!data){
		return ;
	}
	
	if(tangoEnabled){

		if(settings->controlType == NO_CONTROL){
			rz *=settings->precision ;
			ry *=settings->precision ;
			rx *=settings->precision ;

			if(ego){
				rz *= -1 ;
				ry *= -1 ;
				rx *= -1 ;
			}

			Quaternion rot = currentDataRot;
			rot = rot * Quaternion(rot.inverse() * (-Vector3::unitZ()), rz);
			rot = rot * Quaternion(rot.inverse() * -Vector3::unitY(), ry);
			rot = rot * Quaternion(rot.inverse() * Vector3::unitX(), rx);

			currentDataRot = rot;
			return ;
		}
		/*if(settings->controlType == RATE_CONTROL_SIMPLE ){
			Quaternion rot = tabRot;
			rot = rot * Quaternion(rot.inverse() * (-Vector3::unitZ()), rz);
			rot = rot * Quaternion(rot.inverse() * -Vector3::unitY(), ry);
			rot = rot * Quaternion(rot.inverse() * Vector3::unitX(), rx);
			tabRot = rot ;//* tabRot ;
			//currentDataRot = centerRot.inverse() * slerp(Quaternion::identity(),(tabRot*centerRot.inverse()),0.08) * centerRot *currentDataRot ;
			currentDataRot = slerp(Quaternion::identity(),(tabRot*centerRot.inverse()),0.02) *currentDataRot ;
			forSliderQ = tabRot*centerRot.inverse();
			return ;
		}

		if(settings->controlType == PRESSURE_CONTROL_REVERSE || settings->controlType == SLIDER_CONTROL){
				rz *=settings->precision ;
				ry *=settings->precision ;
				rx *=settings->precision ;

				Quaternion rot = currentDataRot;
				rot = rot * Quaternion(rot.inverse() * (-Vector3::unitZ()), rz);
				rot = rot * Quaternion(rot.inverse() * -Vector3::unitY(), ry);
				rot = rot * Quaternion(rot.inverse() * Vector3::unitX(), rx);

				currentDataRot = rot;
				
				return ;
			}

		if(settings->controlType == SPEED_CONTROL){
				rz *= teta ;
				ry *= teta ;
				rx *= teta ;


				Quaternion rot = currentDataRot;
				rot = rot * Quaternion(rot.inverse() * (-Vector3::unitZ()), rz);
				rot = rot * Quaternion(rot.inverse() * -Vector3::unitY(), ry);
				rot = rot * Quaternion(rot.inverse() * Vector3::unitX(), rx);

				currentDataRot = rot;
				computeAngular();
				
				return ;
			}

		if(settings->controlType == RATE_CONTROL){
		//if(participant.getCondition() == RATE_CONTROL_SIMPLE){
			Quaternion rot = currentDataRot;
			rot = rot * Quaternion(rot.inverse() * (-Vector3::unitZ()), rz);
			rot = rot * Quaternion(rot.inverse() * -Vector3::unitY(), ry);
			rot = rot * Quaternion(rot.inverse() * Vector3::unitX(), rx);

			tabRot = rot ;//* tabRot ;
			//currentDataRot = centerRot.inverse() * slerp(Quaternion::identity(),(tabRot*centerRot.inverse()),0.08) * centerRot *currentDataRot ;
			Quaternion diff = centerRot.inverse() * rot ;
			Vector3 diffAngles ;
			diffAngles.x = diff.x /sqrt(1-diff.w*diff.w);
			diffAngles.y = diff.y /sqrt(1-diff.w*diff.w);
			diffAngles.z = diff.z /sqrt(1-diff.w*diff.w);
			printAny(diffAngles,"ANGLES");
			return ;
		}

		//END TRAINING*/

		if(interactionMode == dataTangible || interactionMode == dataTouchTangible)
		{

			if(ego){
				rz *= -1 ;
				ry *= -1 ;
				rx *= -1 ;
			}


			if(participant.getCondition() == PRESSURE_CONTROL_REVERSE || participant.getCondition() == SLIDER_CONTROL){
				rz *=settings->precision ;
				ry *=settings->precision ;
				rx *=settings->precision ;
				Quaternion rot = currentDataRot;
				rot = rot * Quaternion(rot.inverse() * (-Vector3::unitZ()), rz);
				rot = rot * Quaternion(rot.inverse() * -Vector3::unitY(), ry);
				rot = rot * Quaternion(rot.inverse() * Vector3::unitX(), rx);
				currentDataRot = rot;
			}
			else if(participant.getCondition() == SPEED_CONTROL){
				rz *= teta ;
				ry *= teta ;
				rx *= teta ;
				Quaternion rot = currentDataRot;
				rot = rot * Quaternion(rot.inverse() * (-Vector3::unitZ()), rz);
				rot = rot * Quaternion(rot.inverse() * -Vector3::unitY(), ry);
				rot = rot * Quaternion(rot.inverse() * Vector3::unitX(), rx);
				currentDataRot = rot;
				computeAngular();
			}
			
			/*Quaternion rot = currentDataRot;
			rot = rot * Quaternion(rot.inverse() * (-Vector3::unitZ()), rz);
			rot = rot * Quaternion(rot.inverse() * -Vector3::unitY(), ry);
			rot = rot * Quaternion(rot.inverse() * Vector3::unitX(), rx);*/

			
			
			//if(settings->controlType == RATE_CONTROL_SIMPLE ){


			else if(participant.getCondition() == RATE_CONTROL_SIMPLE ){
				Quaternion rot = tabRot;
				rot = rot * Quaternion(rot.inverse() * (-Vector3::unitZ()), rz);
				rot = rot * Quaternion(rot.inverse() * -Vector3::unitY(), ry);
				rot = rot * Quaternion(rot.inverse() * Vector3::unitX(), rx);
				tabRot = rot ;//* tabRot ;
				//currentDataRot = centerRot.inverse() * slerp(Quaternion::identity(),(tabRot*centerRot.inverse()),0.08) * centerRot *currentDataRot ;
				currentDataRot = slerp(Quaternion::identity(),(tabRot*centerRot.inverse()),0.02) *currentDataRot ;
				forSliderQ = tabRot*centerRot.inverse();
			}
			else{
				Quaternion rot = currentDataRot;
				rot = rot * Quaternion(rot.inverse() * (-Vector3::unitZ()), rz);
				rot = rot * Quaternion(rot.inverse() * -Vector3::unitY(), ry);
				rot = rot * Quaternion(rot.inverse() * Vector3::unitX(), rx);
				currentDataRot = rot;	
			}
			
			printAny(currentDataRot, "Data Rot");

		}
	}
	
}

float FluidMechanics::Impl::getValueSlider(){
	if(participant.getCondition() == SPEED_CONTROL){
	//if(settings->controlType == SPEED_CONTROL){
		if(teta > eucli){
			LOGD("VALUESLIDER: TETA is bigger + %f", teta);

			return teta ;
		}
		else{
			LOGD("VALUESLIDER: EUCLI is bigger + %f", eucli);
			return eucli ;
		}
	}
	else if(participant.getCondition() == RATE_CONTROL_SIMPLE){
	//else if(settings->controlType== RATE_CONTROL_SIMPLE){
		if(forSliderQ.w < 0.0){
			forSliderQ.w *= -1.0 ;
		}
		float angle = 2 * safe_acos(forSliderQ.w);
		angle = angle * 180/3.14159 ;
		float vector = getMaxComponentofVector(forSliderV);
		if(angle/120 > abs(vector)){
			LOGD("VALUESLIDER: ANGLE is bigger + %f", angle);
			return angle ;
		}
		else{
			LOGD("VALUESLIDER: Vector is bigger + %f", vector);
			vector = convertIntoNewRange(0, 100, abs(vector));

			return vector;
		}
	}
}

float FluidMechanics::Impl::getMaxComponentofVector(Vector3 v){
	if(abs(v.x)>= abs(v.y) && abs(v.x) >= abs(v.z)){
		LOGD("VALUESLIDER: Biggest value is v.x = %f",v.x);
		return v.x ;
	}
	else if(abs(v.y)>= abs(v.x) && abs(v.y) >= abs(v.z)){
		LOGD("VALUESLIDER: Biggest value is v.y = %f",v.y);
		return v.y ;
	}
	else{
		LOGD("VALUESLIDER: Biggest value is v.z = %f",v.z);
		return v.z ;
	}
}



void FluidMechanics::Impl::computeAngular(){
	//See http://lost-found-wandering.blogspot.fr/2011/09/revisiting-angular-velocity-from-two.html


	//if(settings->controlType == SPEED_CONTROL ){
	if(participant.getCondition() == SPEED_CONTROL ){
		directionRot = currentDataRot * previousRot.inverse();
		float tmp = 2 * safe_acos(directionRot.w);
		
		LOGD("ANGULAR TMP = %f",tmp);
		if(tmp < MINANGULAR)		tmp = MINANGULAR ;
		if(tmp > MAXANGULAR)		tmp = MAXANGULAR ;

		teta =convertIntoNewRange(MINANGULAR, MAXANGULAR, tmp);
		LOGD("VALUE = %f  ----   ANGULAR SPEED = %f",tmp, teta);

		//We update the previous positions and orientation
		//Will only be used if the control mode is Speed
		previousRot = currentDataRot ;
	}

	/*else if(settings->controlType == RATE_CONTROL ){
	//else if(participant.getCondition() == RATE_CONTROL ){
		directionRot = currentDataRot * centerRot.inverse();
		float tmp = 2 * safe_acos(directionRot.w);
		

		if(tmp < MINANGULARRATE)		tmp = MINANGULARRATE ;
		if(tmp > MAXANGULARRATE)		tmp = MAXANGULARRATE ;

		teta =convertIntoNewRange(MINANGULARRATE, MAXANGULARRATE, tmp);
		LOGD("VALUE = %f  ----   ANGULAR SPEED = %f",tmp, teta);

		//We update the previous positions and orientation
		//Will only be used if the control mode is Speed
		previousRot = currentDataRot ;
	}

	//else if(settings->controlType == RATE_CONTROL_SIMPLE){
	else if(participant.getCondition() == RATE_CONTROL_SIMPLE){
		directionRot = tabRot * centerRot.inverse();
		float tmp = 2 * safe_acos(directionRot.w);
		

		if(tmp < MINANGULAR)		tmp = MINANGULAR ;
		if(tmp > MAXANGULAR)		tmp = MAXANGULAR ;

		teta =convertIntoNewRange(MINANGULAR, MAXANGULAR, tmp);
		LOGD("VALUE = %f  ----   ANGULAR SPEED = %f",tmp, teta);


		/*
		directionRot = tabRot * centerRot.inverse();
		float tmp = 2 * safe_acos(directionRot.w);

		//if(tmp < MINANGULARRATE)		tmp = MINANGULARRATE ;
		//if(tmp > MAXANGULARRATE)		tmp = MAXANGULARRATE ;

		teta =convertIntoNewRange(MINANGULARRATE, MAXANGULARRATE, tmp);
		LOGD("VALUE = %f  ----   ANGULAR SPEED = %f",tmp, teta);
		printAny(centerRot, "centerRot");
		printAny(tabRot,"tabRot");
		printAny(directionRot,"directionRot");
	}*/
	else{

		teta = 1 ;
	}
	 
}

void FluidMechanics::Impl::computeEucli(){

	//if(settings->controlType == SPEED_CONTROL){
	if(participant.getCondition() == SPEED_CONTROL){
		float tmp = euclideandist(currentDataPos, previousPos);
		if(tmp < MINEUCLIDEAN)	tmp = MINEUCLIDEAN ;	
		if(tmp > MAXEUCLIDEAN)	tmp = MAXEUCLIDEAN ;
		//if(eucli < MINEUCLIDEAN)	eucli = MINEUCLIDEAN ;	
		//if(eucli > MAXEUCLIDEAN)	eucli = MAXEUCLIDEAN ;
		eucli = convertIntoNewRange(MINEUCLIDEAN, MAXEUCLIDEAN, tmp);
		LOGD("VALUE = %f  ----	 EUCLI = %f",tmp,eucli);

		//We update the previous positions and orientation
		//Will only be used if the control mode is Speed
		previousPos = currentDataPos ;
	}
	/*else if(settings->controlType == RATE_CONTROL_SIMPLE){
	//else if(participant.getCondition() == RATE_CONTROL_SIMPLE){
		float tmp = euclideandist(currentDataPos, centerPos);
		if(tmp < MINEUCLIDEAN)	tmp = MINEUCLIDEAN ;	
		if(tmp > MAXEUCLIDEAN)	tmp = MAXEUCLIDEAN ;
		//if(eucli < MINEUCLIDEAN)	eucli = MINEUCLIDEAN ;	
		//if(eucli > MAXEUCLIDEAN)	eucli = MAXEUCLIDEAN ;
		eucli = convertIntoNewRange(MINEUCLIDEAN, MAXEUCLIDEAN, tmp);

		LOGD("VALUE = %f  ----	 EUCLI = %f",tmp,eucli);

		

		/*float tmp = euclideandist(tabPos, centerPos);
		if(tmp < MINEUCLIDEANRATE)	tmp = MINEUCLIDEANRATE ;
		if(tmp > MAXEUCLIDEANRATE) tmp = MAXEUCLIDEANRATE;
		eucli = convertIntoNewRange(MINEUCLIDEANRATE, MAXEUCLIDEANRATE, tmp);
		directionMov = tabPos - centerPos ;
		directionMov.normalize();
		directionMov *= eucli ;
		LOGD("VALUE = %f  ----	 EUCLI = %f",tmp,eucli);
	}*/
	else{

		eucli = 1 ;
	}
	previousPos = currentDataPos ;
	
}




//Code adapted from 
//http://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-plane-and-ray-disk-intersection
bool intersectPlane(const Vector3& n, const Vector3& p0, const Vector3& l0, const Vector3& l, float& t) 
{ 
	// Here we consider that if the normal in the direction of the screen, there is no intersection
    // assuming vectors are all normalized
    float denom = n.dot(l);//dotProduct(n, l); 
    //LOGD("DENOM = %f", denom);
    if (denom < 1e-6) { 
        Vector3 p0l0 = p0 - l0; 
        t = p0l0.dot(n)/denom ;
        //printAny(p0l0, "POLO = ");
        //printAny(p0l0.dot(n), "p0l0.dot(n) = ");
        //LOGD("t == %f", t);
        return (t >= 0); 
    } 
 
    return false; 
} 

void FluidMechanics::Impl::onTranslateBar(float pos){

	/*float trans = pos-translateBarPrevPos ;
	if(interactionMode == planeTouch || interactionMode == planeHybrid ){

	}

	translateBarPrevPos = pos ;*/

}

void FluidMechanics::Impl::computeFingerInteraction(){
	Vector2 currentPos ;
	Vector2 prevPos ;
	if(fingerPositions.size() == 0){
		return ;
	}

	if(fingerPositions.size() == 1){
		//LOGD("Normal Finger Interaction 1 finger");


		synchronized(fingerPositions){
			currentPos = Vector2(fingerPositions[0].x,fingerPositions[0].y);
		}
		synchronized(prevFingerPositions){
			prevPos = Vector2(prevFingerPositions[0].x, prevFingerPositions[0].y);
		}

		Vector2 diff = currentPos - prevPos ;
		//printAny(currentPos,"Current Pos = ");
		//printAny(prevPos,"Previous Pos =");

		diff /=1000 ;
		diff *= settings->precision ;

		if( interactionMode == dataTouch || 
			interactionMode == dataTouchTangible )
		{
					Quaternion rot = currentDataRot;
					rot = rot * Quaternion(rot.inverse() * Vector3::unitZ(), 0);
					rot = rot * Quaternion(rot.inverse() * Vector3::unitY(), -diff.x);
					rot = rot * Quaternion(rot.inverse() * Vector3::unitX(), diff.y);
					currentDataRot = rot;
		}

	}

	else if(fingerPositions.size() == 2){
		//LOGD("Two finger interaction");
		Vector2 diff = Vector2(0,0);

		float x1,x2,y1,y2 ;
		x1 = fingerPositions[0].x ;
		x2 = fingerPositions[1].x ;
		y1 = fingerPositions[0].y ;
		y2 = fingerPositions[1].y ;

		Vector2 newVec = Vector2(x2-x1,y2-y1);
		

		//Translation Computation

		for(int i = 0 ; i < 2 ; i++){
			synchronized(fingerPositions){
			currentPos = Vector2(fingerPositions[i].x,fingerPositions[i].y);
			}
			synchronized(prevFingerPositions){
				prevPos = Vector2(prevFingerPositions[i].x, prevFingerPositions[i].y);
			}

			diff += currentPos - prevPos ;
		}

	
	
		//FIXME Hardcoded
		diff /=2 ;
		diff /=4 ;
		diff *= settings->precision ;
		diff *= settings->considerTranslation * settings->considerX * settings->considerY;

		Vector3 trans = Vector3(diff.x, diff.y, 0);
		//LOGD("Diff = %f -- %f", diff.x, diff.y);
		
		//Compute distance between the two fingers
		float distance = sqrt(    (x1-x2)*(x1-x2) + (y1-y2) * (y1-y2)    );
		//LOGD("Distance %f", distance);
		if(interactionMode == dataTouch || 
			    interactionMode == dataTouchTangible )
		{
			currentDataPos +=trans ;
		}
		//Rotation on the z axis --- Spinning 
		if(distance> thresholdRST || isAboveThreshold){
			isAboveThreshold = true ;
			
			//LOGD("Spinning considered");
			
	        
	        float dot = initialVector.x * newVec.x + initialVector.y * newVec.y ;
	        float det = initialVector.x * newVec.y - initialVector.y * newVec.x ;

	        float angle = atan2(det,dot);
	        //FIXME : to correct for the case when I remove one finger and put it back, rotate 360°
	        if(angle == 3.141593){	//FIXME hardcoded
	        	angle = 0 ;	
	        }

	        angle *=settings->precision ;
	        angle *= settings->considerZ * settings->considerRotation ;

	        //if(interactionMode == planeTouch){
	      	if(interactionMode == dataTouch || 
			    	interactionMode == dataTouchTangible)
			{
				Quaternion rot = currentDataRot;
				rot = rot * Quaternion(rot.inverse() * Vector3::unitZ(), angle);
				rot = rot * Quaternion(rot.inverse() * Vector3::unitY(), 0);
				rot = rot * Quaternion(rot.inverse() * Vector3::unitX(), 0);
				currentDataRot = rot;
			}



	        if(mInitialPinchDistSet == false){
	        	mInitialPinchDist = distance ;
	        	mInitialPinchDistSet = true ;
	        }
        	LOGD("Initial Pinch = %f, && zoom = %f",mInitialPinchDist,settings->zoomFactor);
            settings->zoomFactor = mInitialZoomFactor * distance/mInitialPinchDist;
            if (settings->zoomFactor <= 0.25f){
            	settings->zoomFactor = 0.25f;
            }
                
		}

		//Should be done no matter what is happening with the distance between the two fingers
		//Otherwise, spinning happens at one
		initialVector = newVec ;

	}
	
}

void FluidMechanics::Impl::updateMatrices(){
	Matrix4 statem ;
	Matrix4 slicem ;

	LOGD("CONTROL TYPE = %d", settings->controlType);

	if( interactionMode == dataTouch ||
		interactionMode == dataTouchTangible
		)
	{

			//LOGD("Interaction Needs touch");
			computeFingerInteraction();
	}
	

	statem = Matrix4::makeTransform(currentDataPos, currentDataRot);

	//Plane not moving on the tablet
	/*synchronized(state->modelMatrix) {
		state->modelMatrix = m ;
	}*/
	


	//Then the data
	synchronized(state->modelMatrix) {
		state->modelMatrix = statem ;
	}
	
	
	
	
	

}

std::string FluidMechanics::Impl::getData(){
	
  	std::ostringstream oss;
  	Matrix4 m ;

  	//First we set the zoomFactor
  	oss << settings->zoomFactor << ";" ;
  	int tmp = (settings->showVolume) ? 1 : 0 ;
  	oss << tmp << ";" ;
  	tmp = (settings->showSurface) ? 1 : 0 ;
  	oss << tmp << ";" ;
  	tmp = (settings->showStylus) ? 1 : 0 ;
  	oss << tmp << ";" ;
  	tmp = (settings->showSlice) ? 1 : 0 ;
  	oss << tmp << ";" ;
  	tmp = (settings->showOutline) ? 1 : 0 ;
  	oss << tmp << ";" ;

  	synchronized(state->modelMatrix){
  		m = state->modelMatrix ;	
  	}  	
	oss << m.data_[0] << ";" 
		<< m.data_[1] << ";" 
		<< m.data_[2] << ";" 
		<< m.data_[3] << ";" 
		<< m.data_[4] << ";" 
		<< m.data_[5] << ";" 
		<< m.data_[6] << ";" 
		<< m.data_[7] << ";" 
		<< m.data_[8] << ";" 
		<< m.data_[9] << ";" 
		<< m.data_[10] << ";" 
		<< m.data_[11] << ";" 
		<< m.data_[12] << ";" 
		<< m.data_[13] << ";" 
		<< m.data_[14] << ";" 
		<< m.data_[15] << ";" ;

	synchronized(state->stylusModelMatrix){
  		m = state->stylusModelMatrix ;	
  	}

	oss << m.data_[0] << ";" 
		<< m.data_[1] << ";" 
		<< m.data_[2] << ";" 
		<< m.data_[3] << ";" 
		<< m.data_[4] << ";" 
		<< m.data_[5] << ";" 
		<< m.data_[6] << ";" 
		<< m.data_[7] << ";" 
		<< m.data_[8] << ";" 
		<< m.data_[9] << ";" 
		<< m.data_[10] << ";" 
		<< m.data_[11] << ";" 
		<< m.data_[12] << ";" 
		<< m.data_[13] << ";" 
		<< m.data_[14] << ";" 
		<< m.data_[15] << ";" ;

	oss << seedingPoint.x << ";"
		<< seedingPoint.y << ";"  
		<< seedingPoint.z << ";" ;

	std::string s = oss.str();

	return s ;
}

void FluidMechanics::Impl::addFinger(float x, float y, int fingerID){
	Vector3 pos(x,y, fingerID);
	synchronized(prevFingerPositions){
		prevFingerPositions.push_back(pos);
	}
	synchronized(fingerPositions){
		fingerPositions.push_back(pos);
	}
	if(fingerPositions.size() == 2){
		float x1,x2,y1,y2 ;
		synchronized(fingerPositions){
			x1 = fingerPositions[0].x ;
			x2 = fingerPositions[1].x ;
			y1 = fingerPositions[0].y ;
			y2 = fingerPositions[1].y ;
		}
		initialVector = Vector2(x2-x1, y2-y1);
		float dx = x1  - x2;
        float dy = y1 - y2;
        float dist = sqrt(dx*dx + dy*dy);
        mInitialPinchDist = dist;
        mInitialZoomFactor = settings->zoomFactor;
	}
}
void FluidMechanics::Impl::removeFinger(int fingerID){
	int position = getFingerPos(fingerID);
	if(position == -1){
		return ;
	}
	//LOGD("Size = %d, position == %d", fingerPositions.size(), position);
	synchronized(prevFingerPositions){
		prevFingerPositions.erase(prevFingerPositions.begin()+position);
	}
	synchronized(fingerPositions){
		fingerPositions.erase(fingerPositions.begin()+position);
	}

	//Reset all bools that are related to 2-finger interaction
	if(fingerPositions.size()<2){
		mInitialPinchDistSet = false ;
		isAboveThreshold = false ;
	}
	
	/*if(fingerPositions.size() == 0){
		seedPointPlacement = false ;
	}*/

}

int FluidMechanics::Impl::getFingerPos(int fingerID){
	for(int i = 0 ; i < fingerPositions.size() ; i++){
		if(fingerPositions[i].z == fingerID){
			return i ;
		}
	}
	return -1 ;
}


void FluidMechanics::Impl::updateFingerPositions(float x, float y, int fingerID){
	Vector3 pos(x,y, fingerID);
	/*if(fingerID >= fingerPositions.size()){
		LOGD("Error in Finger ID");
		return ;
	}*/
	
	int position = getFingerPos(fingerID);
	if(position == -1){
		//LOGD("Error in Finger ID");
		return ;
	}
	synchronized(prevFingerPositions){
		prevFingerPositions[position] = fingerPositions[position];	
	}
	synchronized(fingerPositions){
		fingerPositions[position] = pos ;	
	}
	//LOGD("Finger %d has moved from %f -- %f     to     %f -- %f",fingerID,prevFingerPositions[position].x,prevFingerPositions[position].y, x, y);
}

// (GL context)
void FluidMechanics::Impl::renderObjects()
{
	updateMatrices();
	const Matrix4 proj = app->getProjMatrix();
	//printAny(state->modelMatrix, "Model Matrix Object = ");
	glEnable(GL_DEPTH_TEST);


	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_CULL_FACE);
	glDepthMask(true); // requires "discard" in the shader where alpha == 0

	// LOGD("render 4");

	if (state->tangibleVisible) {
		Matrix4 mm;
		Matrix4 tm;
		if(settings->isTraining == false){
			tm= participant.getTargetMatrix();
			synchronized(state->modelMatrix) {
				mm = state->modelMatrix;
			}
		}
		else{
			tm= targetMatrix;
			synchronized(state->modelMatrix) {
				mm = state->modelMatrix;
			}
		}
		

		// Apply the zoom factor
		mm = mm * Matrix4::makeTransform(
			Vector3::zero(),
			Quaternion::identity(),
			Vector3(settings->zoomFactor)
		);
		tm = tm * Matrix4::makeTransform(
			Vector3::zero(),
			Quaternion::identity(),
			Vector3(settings->zoomFactor)
		);




#ifdef VTK
		//Render the outline
		if(settings->showOutline){
			synchronized_if(outline) {
				glDepthMask(true);
				glLineWidth(2.0f);
				if(tangoEnabled && (interactionMode == dataTangible ||
									interactionMode == dataTouchTangible))
				{
					outline->setColor(Vector3(0, 1.0f, 0));
				}
				else{
					outline->setColor(Vector3(1.0f, 0, 0));	
				}
				//outline->setColor(tangoEnabled ? Vector3(1.0f, 0, 0) : Vector3(0, 1.0f, 0));
				//outline->setColor(!velocityData ? Vector3(1.0f, 0, 0) : Vector3(0, 1.0f, 0));
				outline->render(proj, mm);
			}
		}

		// Render the surface
		if (settings->showSurface) {
			glDisable(GL_CULL_FACE);
			glDisable(GL_BLEND);
			glDepthMask(true);

			if (!settings->surfacePreview || !isosurfaceLow) {
				synchronized_if(isosurface) {
					isosurface->render(proj, mm);
				}
			}

			if (settings->surfacePreview) {
				synchronized_if(isosurfaceLow) {
					isosurfaceLow->render(proj, mm);
				}
			}
		}

		// Render the volume
		if (settings->showVolume) {// && sliceDot <= 0) {
			glEnable(GL_DEPTH_TEST);
			synchronized_if(volume) {
				// glDepthMask(false);
				glDepthMask(true);
				glEnable(GL_BLEND);
				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // modulate
				// glBlendFunc(GL_SRC_ALPHA, GL_ONE); // additive
				glDisable(GL_CULL_FACE);
				volume->render(proj, mm);
			}
		}

#else

		// Render the volume
		if (settings->showVolume) {// && sliceDot <= 0) {
			glEnable(GL_DEPTH_TEST);
			synchronized_if(volume) {
				// glDepthMask(false);
				glDepthMask(true);
				glEnable(GL_BLEND);
				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // modulate
				// glBlendFunc(GL_SRC_ALPHA, GL_ONE); // additive
				glDisable(GL_CULL_FACE);
				bunny->render(proj, mm);

				bunnytarget->renderTarget(proj,tm);
			}
		}

#endif

	}

	
}


void FluidMechanics::Impl::updateSurfacePreview()
{
	if (!settings->surfacePreview) {
		synchronized_if(isosurface) {
			isosurface->setPercentage(settings->surfacePercentage);
		}
	} else if (settings->surfacePreview) {
		synchronized_if(isosurfaceLow) {
			isosurfaceLow->setPercentage(settings->surfacePercentage);
		}
	}
	if(settings->considerX+settings->considerY+settings->considerZ == 3){
		state->clipAxis = CLIP_NONE ;
	}

	else if(settings->considerX == 1 ){
		state->clipAxis = CLIP_AXIS_X ;
	}
	else if(settings->considerY == 1 ){
		state->clipAxis = CLIP_AXIS_Y ;
	}
	else if(settings->considerZ == 1 ){
		state->clipAxis = CLIP_AXIS_Z ;
	}
}

JNIEXPORT jboolean JNICALL Java_fr_limsi_ARViewer_FluidMechanics_loadDataset(JNIEnv* env,
	jobject obj, jstring filename)
{
	try {
		// LOGD("(JNI) [FluidMechanics] loadDataSet()");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		const char* javaStr = env->GetStringUTFChars(filename, nullptr);
		if (!javaStr)
			throw std::runtime_error("GetStringUTFChars() returned null");
		const std::string filenameStr(javaStr);
		env->ReleaseStringUTFChars(filename, javaStr);

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		return instance->loadDataSet(filenameStr);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
		return false;
	}
}

JNIEXPORT jboolean JNICALL Java_fr_limsi_ARViewer_FluidMechanics_loadVelocityDataset(JNIEnv* env,
	jobject obj, jstring filename)
{
	try {
		// LOGD("(JNI) [FluidMechanics] loadVelocityDataSet()");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		const char* javaStr = env->GetStringUTFChars(filename, nullptr);
		if (!javaStr)
			throw std::runtime_error("GetStringUTFChars() returned null");
		const std::string filenameStr(javaStr);
		env->ReleaseStringUTFChars(filename, javaStr);

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		return instance->loadVelocityDataSet(filenameStr);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
		return false;
	}
}


JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_getSettings(JNIEnv* env, jobject obj, jobject settingsObj)
{
	try {
		// LOGD("(JNI) [FluidMechanics] getSettings()");

		if (!settingsObj)
			throw std::runtime_error("\"Settings\" object is null");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		jclass cls = env->GetObjectClass(settingsObj);
		if (!cls)
			throw std::runtime_error("GetObjectClass() returned null");

		FluidMechanics::Settings* settings = dynamic_cast<FluidMechanics::Settings*>(App::getInstance()->getSettings().get());
		android_assert(settings);
		return settings->read(env, settingsObj, cls);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_setSettings(JNIEnv* env, jobject obj, jobject settingsObj)
{
	try {
		// LOGD("(JNI) [FluidMechanics] setSettings()");

		if (!settingsObj)
			throw std::runtime_error("\"Settings\" object is null");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		jclass cls = env->GetObjectClass(settingsObj);
		if (!cls)
			throw std::runtime_error("GetObjectClass() returned null");

		FluidMechanics::Settings* settings = dynamic_cast<FluidMechanics::Settings*>(App::getInstance()->getSettings().get());
		android_assert(settings);
		settings->write(env, settingsObj, cls);

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(settings);
		instance->updateSurfacePreview();

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_getState(JNIEnv* env, jobject obj, jobject stateObj)
{
	try {
		// LOGD("(JNI) [FluidMechanics] getState()");

		if (!stateObj)
			throw std::runtime_error("\"State\" object is null");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		jclass cls = env->GetObjectClass(stateObj);
		if (!cls)
			throw std::runtime_error("GetObjectClass() returned null");

		FluidMechanics::State* state = dynamic_cast<FluidMechanics::State*>(App::getInstance()->getState().get());
		android_assert(state);
		state->read(env, stateObj, cls);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_buttonPressed(JNIEnv* env, jobject obj)
{
	try {
		 //LOGD("(JNI) [FluidMechanics] buttonPressed()");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->buttonPressed();

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_setTangoValues(JNIEnv* env, jobject obj,jdouble tx, jdouble ty, jdouble tz, jdouble rx, jdouble ry, jdouble rz, jdouble q){
	try {

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->setTangoValues(tx,ty,tz,rx,ry,rz,q);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_setGyroValues(JNIEnv* env, jobject obj, jdouble rx, jdouble ry, jdouble rz, jdouble q){
	try {

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->setGyroValues(rx,ry,rz,q);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT jfloat JNICALL Java_fr_limsi_ARViewer_FluidMechanics_buttonReleased(JNIEnv* env, jobject obj)
{
	try {
		 //LOGD("(JNI) [FluidMechanics] buttonReleased()");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		return instance->buttonReleased();

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
		return 0.0f;
	}
}

JNIEXPORT jstring Java_fr_limsi_ARViewer_FluidMechanics_getData(JNIEnv* env, jobject obj)
{
	try {

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		return (env->NewStringUTF(instance->getData().c_str())) ;

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
	
}


JNIEXPORT void Java_fr_limsi_ARViewer_FluidMechanics_setInteractionMode(JNIEnv* env, jobject obj, jint mode){
	try {

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->setInteractionMode(mode);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void Java_fr_limsi_ARViewer_FluidMechanics_updateFingerPositions(JNIEnv* env, jobject obj,jfloat x, jfloat y, jint fingerID){
	try {

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->updateFingerPositions(x,y,fingerID);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void Java_fr_limsi_ARViewer_FluidMechanics_addFinger(JNIEnv* env, jobject obj,jfloat x, jfloat y, jint fingerID){
	try {

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->addFinger(x,y,fingerID);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void Java_fr_limsi_ARViewer_FluidMechanics_reset(JNIEnv* env, jobject obj){
	try {

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->reset();

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void Java_fr_limsi_ARViewer_FluidMechanics_removeFinger(JNIEnv* env, jobject obj, jint fingerID){
	try {

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->removeFinger(fingerID);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}


JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_launchTrial(JNIEnv* env, jobject obj)
{
	try {
		// LOGD("(JNI) [FluidMechanics] loadVelocityDataSet()");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");


		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->launchTrial();

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT jboolean JNICALL Java_fr_limsi_ARViewer_FluidMechanics_isTrialOver(JNIEnv* env, jobject obj)
{
	try {
		// LOGD("(JNI) [FluidMechanics] loadVelocityDataSet()");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		return instance->isTrialOver();

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
		return false;
	}
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_setPId(JNIEnv* env, jobject obj, int id){
	try {
		// LOGD("(JNI) [FluidMechanics] loadVelocityDataSet()");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->setPId(id);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_initJNI(JNIEnv* en, jobject ob)
{
	try {
		// LOGD("(JNI) [FluidMechanics] loadVelocityDataSet()");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		
		/*FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		jclass dataClass = en->FindClass("fr/limsi/ARViewer/MainActivity");
	    javaClassRef = (jclass) env->NewGlobalRef(dataClass);
	    javaMethodRef = env->GetMethodID(javaClassRef, "endTrial", "()V");


	    int status = env->GetJavaVM(env, &jvm);
	    if(status != 0) {
	        LOGD("JNIERROR");
	    }*/
	     // insted of the env store the VM
	    int status = en->GetJavaVM(&g_jvm);
	    jclass dataClass = en->FindClass("fr/limsi/ARViewer/MainActivity");
	    javaClassRef = (jclass) en->NewGlobalRef(dataClass);
	    javaMethodRef = en->GetMethodID(javaClassRef, "endTrial", "()V");
	    if(status != 0) {
	        LOGD("-------->JNIERROR");
	    }
	    else if(javaMethodRef == NULL){
	    	LOGD("-------->METHODREFERROR");
	    }
	    else{
	    	LOGD("-------->JNIOK");
	    }
	    /*obj = en->NewGlobalRef(ob); // I don't think you need this
	    // and at some point you must delete it again*/

	    LOGD("JNIINITDONE");

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT jint JNICALL Java_fr_limsi_ARViewer_FluidMechanics_getCondition(JNIEnv* env, jobject obj){
	try {
		// LOGD("(JNI) [FluidMechanics] loadVelocityDataSet()");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		return instance->getCondition() ;


	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_endTrialJava(){
	/*jobject javaObjectRef = env->NewObject(javaClassRef, javaMethodRef);
	env->CallVoidMethod(javaObjectRef, javaMethodRef);*/
	/*JNIEnv *env;
    jvm->AttachCurrentThread((void **)&env, jvm);
    jobject javaObjectRef = env->NewObject(javaClassRef, javaMethodRef);
    env->CallVoidMethod(javaObjectRef, javaMethodRef);*/
    
    return ;

    LOGD("EndTrialJava");
    // See 
    JNIEnv* env;

    int getEnvStat = g_jvm->GetEnv((void **)&env, JNI_VERSION_1_6);
    if (getEnvStat == JNI_EDETACHED) {
        LOGD("GetEnv: not attached");
        //if (g_jvm->AttachCurrentThread((void **) &env, NULL) != 0) {
        if (g_jvm->AttachCurrentThread(&env, NULL) != 0) {
            LOGD("Failed to attach");
        }
        else{
        	jobject javaObjectRef = env->NewObject(javaClassRef, javaMethodRef);
        	env->CallVoidMethod(javaObjectRef, javaMethodRef);
        }
    } else if (getEnvStat == JNI_OK) {
        //
    } else if (getEnvStat == JNI_EVERSION) {
        LOGD("GetEnv: version not supported");
    }

    //See http://stackoverflow.com/questions/26534304/android-jni-call-attachcurrentthread-without-detachcurrentthread
    g_jvm->DetachCurrentThread();

    //g_jvm->AttachCurrentThread(&env, NULL); // check error etc
    //jobject javaObjectRef = env->NewObject(javaClassRef, javaMethodRef);
    // this line makes not much sense. I think you don't need it if you use the global
    // with the global it would be more like this
    //env->CallVoidMethod(javaObjectRef, javaMethodRef);
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_hasFinishedLog(JNIEnv* env, jobject obj){
	try {
		// LOGD("(JNI) [FluidMechanics] loadVelocityDataSet()");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->hasFinishedLog();

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}

}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_isEgo(JNIEnv* env, jobject obj, jboolean ego){
	try {
		// LOGD("(JNI) [FluidMechanics] loadVelocityDataSet()");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->isEgo(ego);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT int JNICALL Java_fr_limsi_ARViewer_FluidMechanics_getTime(JNIEnv* env, jobject obj){
	if (!App::getInstance())
			throw std::runtime_error("init() was not called");

	if (App::getType() != App::APP_TYPE_FLUID)
		throw std::runtime_error("Wrong application type");

	return nbSeconds;

}

JNIEXPORT jfloat JNICALL Java_fr_limsi_ARViewer_FluidMechanics_getValueSlider(JNIEnv* env, jobject obj){
	try {
		// LOGD("(JNI) [FluidMechanics] loadVelocityDataSet()");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		return instance->getValueSlider();

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}




FluidMechanics::FluidMechanics(const InitParams& params)
 : NativeApp(params, SettingsPtr(new FluidMechanics::Settings), StatePtr(new FluidMechanics::State)),
   impl(new Impl(params.baseDir))
{
	impl->app = this;
	impl->settings = std::static_pointer_cast<FluidMechanics::Settings>(settings);
	impl->state = std::static_pointer_cast<FluidMechanics::State>(state);
}

bool FluidMechanics::loadDataSet(const std::string& fileName)
{
	impl->setMatrices(Matrix4::makeTransform(Vector3(0, 0, 400)),
	                  Matrix4::makeTransform(Vector3(0, 0, 400)));
	impl->currentSlicePos = Vector3(0, 0, 400);
	impl->currentDataPos = Vector3(0,0,400);
	return impl->loadDataSet(fileName);
}

bool FluidMechanics::loadVelocityDataSet(const std::string& fileName)
{
	return impl->loadVelocityDataSet(fileName);
}

void FluidMechanics::buttonPressed()
{
	impl->buttonPressed();
}

float FluidMechanics::buttonReleased()
{
	return impl->buttonReleased();
}

void FluidMechanics::rebind()
{
	NativeApp::rebind();
	impl->rebind();
}



void FluidMechanics::setMatrices(const Matrix4& volumeMatrix, const Matrix4& stylusMatrix)
{
	impl->setMatrices(volumeMatrix, stylusMatrix);
}

void FluidMechanics::renderObjects()
{
	impl->renderObjects();
}

void FluidMechanics::updateSurfacePreview()
{
	impl->updateSurfacePreview();
}

void FluidMechanics::reset(){
	impl->reset();
}


void FluidMechanics::setInteractionMode(int mode){
	impl->setInteractionMode(mode);
}

std::string FluidMechanics::getData(){
	return impl->getData();
}

void FluidMechanics::updateFingerPositions(float x, float y, int fingerID){
	impl->updateFingerPositions(x,y,fingerID);
}

void FluidMechanics::addFinger(float x, float y, int fingerID){
	impl->addFinger(x,y,fingerID);
}

void FluidMechanics::removeFinger(int fingerID){
	impl->removeFinger(fingerID);
}

void FluidMechanics::setTangoValues(double tx, double ty, double tz, double rx, double ry, double rz, double q){
	impl->setTangoValues(tx,ty,tz,rx,ry,rz,q);
}

void FluidMechanics::setGyroValues(double rx, double ry, double rz, double q){
	impl->setGyroValues(rx,ry,rz,q);
}

void FluidMechanics::launchTrial(){
	impl->launchTrial();
}

bool FluidMechanics::isTrialOver(){
	return impl->isTrialOver();
}

void FluidMechanics::setPId(int p){
	impl->setPId(p);
}

bool FluidMechanics::hasFinishedLog(){
	impl->hasFinishedLog();
}

int FluidMechanics::getCondition(){
	return impl->getCondition();
}

void FluidMechanics::isEgo(bool b){
	impl->isEgo(b);
}

float FluidMechanics::getValueSlider(){
	return impl->getValueSlider();
}
/*
void FluidMechanics::initJNI(){
	impl->initJNI();
}*/

