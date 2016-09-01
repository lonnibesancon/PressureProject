package fr.limsi.ARViewer;

import java.io.PrintWriter;

import android.app.AlertDialog;
import android.content.res.Resources;
import android.graphics.drawable.GradientDrawable;
import android.graphics.drawable.ColorDrawable;
import android.hardware.Camera;
import android.os.Bundle;
import android.util.Log;
import android.util.TypedValue;
import android.view.GestureDetector;
import android.view.GestureDetector.SimpleOnGestureListener;
import android.view.ContextMenu.ContextMenuInfo;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.widget.ToggleButton;
import android.widget.ImageButton;

import android.app.*;
import android.bluetooth.*;
import android.content.*;
import android.graphics.*;
import android.hardware.*;
import android.hardware.Camera;
import android.hardware.usb.*;
import android.os.*;
import android.util.*;
import android.view.*;
import android.widget.*;
import java.io.*;
import java.net.*;
import java.lang.*;
import java.util.*;
import android.view.View.OnClickListener;
import java.lang.Object;


import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.os.Bundle;
import android.os.IBinder;
import android.view.KeyEvent;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;
 

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanFilter;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;


import java.util.ArrayList;
import java.util.List;

import java.util.UUID;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;


import java.text.DecimalFormat ;

import com.google.atap.tangoservice.*;



// import com.qualcomm.QCAR.QCAR;
// // import com.qualcomm.ar.pl.CameraPreview;
// import com.lannbox.rfduinotest.RFduinoService;

public class MainActivity extends BaseARActivity
 implements View.OnTouchListener, Tango.OnTangoUpdateListener, SensorEventListener, InteractionMode, OnClickListener, BluetoothAdapter.LeScanCallback  //,GestureDetector.OnDoubleTapListener, View.OnLongClickListener, BluetoothAdapter.LeScanCallback
    // , CameraPreview.SizeCallback
{
    private static final String TAG = Config.APP_TAG;
    private static final boolean DEBUG = Config.DEBUG;

    private ImageButton tangibleBtn ;
    private ToggleButton tangibleToggle ;
    private ToggleButton touchToggle ;


    private boolean isTangibleOn = false ;
    private boolean isTouchOn = false;
    private boolean dataORplaneTangible = true ; //Data
    private boolean dataORplaneTouch = true ;    //Data
    private boolean isTangiblePressed = false ;

    
    public Object lock = new Object() ;

    // FIXME: static?
    private static FluidMechanics.Settings fluidSettings = new FluidMechanics.Settings();
    private static FluidMechanics.State fluidState = new FluidMechanics.State();

    private FluidMechanics.Settings tempSettings = new FluidMechanics.Settings();

    // private static boolean mFluidDataLoaded = false;
    private static int mDataSet = head;
    private static boolean mDatasetLoaded = false;
    private static boolean mVelocityDatasetLoaded = false;

    // Menu            
    private static boolean menuInitialized = false;
    private MenuItem mAxisClippingMenuItem, mStylusClippingMenuItem;

    // Pinch zoom
    private float mInitialPinchDist = 0;
    private float mInitialZoomFactor = 0;
    private boolean mZoomGesture = false;

    // Gestures
    private GestureDetector mGestureDetector;

    private ARSurfaceView mView;

    private Tango mTango;
    private TangoConfig mConfig;
    private boolean mIsTangoServiceConnected;

    // Gyro part
    private SensorManager mSensorManager;
    private Sensor mGyro ;
    private Sensor mRotation ;
    private static final float NS2S = 1.0f / 1000000000.0f;
    private long mLastTimestamp = 0;

    //private Client client ;

    private int interactionMode = nothing;
    private boolean tangibleModeActivated = false ;

    //LOGGING
    private static final String FILENAME = "myFile.txt";
    private Logging logging ;
    private short interactionType ;
    private FileOutputStream fOut ;
    private OutputStreamWriter outputStreamWriter ;
    private boolean isInteracting = false ;
    private long initialTime ;
    private long previousLogTime = 0 ;
    private long logrefreshrate = 50 ;
    private int nbOfFingers = 0;
    private int nbOfFingersButton = 0 ;
    private int nbOfResets = 0 ;
    private int pId = 0 ;

    private final ScheduledThreadPoolExecutor executor = new ScheduledThreadPoolExecutor(1);;

    //Constrain interaction part 

    private boolean constrainRotation ;
    private boolean constrainTranslation ;
    private boolean autoConstraint ;
    private boolean dataOrTangibleValue = true ;
    
    /*** BLE PART***/
    // State machine
    final private static int STATE_BLUETOOTH_OFF = 1;
    final private static int STATE_DISCONNECTED = 2;
    final private static int STATE_CONNECTING = 3;
    final private static int STATE_CONNECTED = 4;

    private TextView bluetoothOverlay ;

    private int state;

    private boolean scanStarted;
    private boolean scanning;
    private boolean bluestarted = false ;

    private BluetoothAdapter bluetoothAdapter;
    private BluetoothDevice bluetoothDevice;

    private RFduinoService rfduinoService;

    private boolean areReceiverUnregistered = false ;

    private Button enableBluetoothButton;
    private TextView scanStatusText;
    private Button scanButton;
    private TextView deviceInfoText;
    private TextView connectionStatusText;
    private Button connectButton;
    private EditData valueEdit;
    private Button sendZeroButton;
    private Button sendValueButton;
    private Button clearButton;
    private LinearLayout dataLayout;

    private float value ;
    private short trialNumber = 0 ;
    private boolean trialFinished = false ;
    private boolean mAlertVisible = false ;
    private boolean idRegistered = false ;
    final private Context context = this ;
    private Handler mHandler ;


    final private static short NO_CONTROL               = 0 ;
    final private static short RATE_CONTROL_SIMPLE      = 1 ;
    final private static short RATE_CONTROL             = 11 ;
    final private static short SPEED_CONTROL            = 2 ;
    final private static short PRESSURE_CONTROL_REVERSE = 3 ;
    final private static short PRESSURE_CONTROL         = 31 ;
    final private static short SLIDER_CONTROL           = 4 ;


    //Training part
    private Button endTrainingBtn ;
    private Button resetBtn ;
    private boolean isEgo = false ;
    private Button egoBtn ;
    private boolean isPopUp = false ;
    private boolean trialStarted = false ;

    @Override
    protected int getAppType() {
        // return NativeApp.APP_TYPE_STUB;
        return NativeApp.APP_TYPE_FLUID;
    }


    private void getStartInfo(){
        AlertDialog.Builder alert = new AlertDialog.Builder(this);
        
        alert.setTitle("Participant ID");
        alert.setMessage("Please enter the ID");

        // Set an EditText view to get user input 
        final EditText input = new EditText(this);
        alert.setView(input);

        alert.setPositiveButton("Ok", new DialogInterface.OnClickListener() {
        public void onClick(DialogInterface dialog, int whichButton) {
            if(input.getText()!= null){
                int p = Integer.parseInt(input.getText().toString());
                if(p < 0){
                    System.exit(0);
                }
                else{
                    pId = p ;
                    fluidSettings.pID = p ;
                    updateSettings();
                    FluidMechanics.setPId(p);
                    openFile();
                    idRegistered = true ;
                    //showAlerts();
                }
            }
            
          
          }
        });

        alert.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
          public void onClick(DialogInterface dialog, int whichButton) {
            System.exit(0);
          }
        });

        alert.show();
    }

    private void openFile(){
        try{
            fOut = new FileOutputStream("/sdcard/test"+"/"+pId+".csv");
            outputStreamWriter = new OutputStreamWriter(fOut);
        }
        catch (IOException e) {
            Log.e(TAG, "File write failed: " + e.toString());
        } 

        this.executor.scheduleWithFixedDelay(new Runnable() {
            @Override
            public void run() {
                try{
                    loggingFunction();
                }
                catch (IOException e) {
                    Log.e(TAG, "File write failed: " + e.toString());
                } 
            }
        }, 0L, logrefreshrate , TimeUnit.MILLISECONDS);

    }

    public void showAlerts(){
        //When it's done
        isPopUp = true ;
        if( trialNumber == 4*NBTRIALS ){
            AlertDialog.Builder alert = new AlertDialog.Builder(MainActivity.this);
            alert.setTitle("Thanks for your participation");
            alert.setMessage("Thanks a lot for participating in the study!!!!");

            alert.setPositiveButton("Ok", new DialogInterface.OnClickListener() {
            public void onClick(DialogInterface dialog, int whichButton) {
                System.exit(0);
                return ;
              
              }
            });
            AlertDialog alertdialog = alert.create();
            alertdialog.show();
        }

        //We need to show that they're gonna use a new technique
        if(trialNumber%NBTRIALS == 0 ){
            alertBeforeNewTechnique();
        }
        else{
            alertBeforeTrial();    
        }
        
    }

    public void alertBeforeNewTechnique(){
        AlertDialog.Builder alert = new AlertDialog.Builder(MainActivity.this);
        fluidSettings.controlType = FluidMechanics.getCondition() ;
        String techniqueName = getConditionName(fluidSettings.controlType);
        /*if(techniqueName.equals("Pressure Control")){
            Log.d("Bluetooth","started because new technique is pressure");
            connectBluetooth();
            bluestarted = true ;
        }
        else{
            if(bluestarted){
                Log.d("Bluetooth","stopped cause new technique is different");
                disconnectBluetooth(); 
                bluestarted = false ;   
            }
        }*/
        alert.setTitle("Next technique: "+techniqueName);
        alert.setMessage("You are now done with the current technique, it's time to evaluate it! \nYou will now try an other technique. Touch ok when you're ready\n");

        alert.setPositiveButton("Ok", new DialogInterface.OnClickListener() {
        public void onClick(DialogInterface dialog, int whichButton) {
            alertBeforeTrial();
            return ;
          
          }
        });
        AlertDialog alertdialog = alert.create();
        alertdialog.show();
        alertdialog.setCancelable(false);
        //alert.show();
    }

    public void launchTrial(){
       
        FluidMechanics.launchTrial();
        trialNumber ++ ; 
        Log.d("TrialNumber",""+trialNumber);
        trialStarted = true ;
    }

    public void endTrial(){
        if(trialFinished == false){
            Log.d("TEST","End Trial");
            trialFinished = true ;
            mHandler.post(new Runnable(){
                public void run(){
                    showAlerts();
                    /*//Be sure to pass your Activity class, not the Thread
                    AlertDialog.Builder builder = new AlertDialog.Builder(MyActivity.this);
                    //... setup dialog and show*/
                }
            });
            
               
        }
        
    }

    public void alertBeforeEndOfTraining(){
        AlertDialog.Builder alert = new AlertDialog.Builder(MainActivity.this);
        alert.setTitle("Are you sure?");
        alert.setMessage("Click ok if you're ready to start the experiment. Cancel otherwise");

        alert.setPositiveButton("Ok", new DialogInterface.OnClickListener() {
        public void onClick(DialogInterface dialog, int whichButton) {
            
            fluidSettings.isTraining = false ;
            updateSettings();
            endTrainingBtn.setVisibility(View.GONE);
            getStartInfo();
            resetBtn.setVisibility(View.GONE);
            return ;   
          
          }
        });
        alert.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
        public void onClick(DialogInterface dialog, int which) {
            return ;
        } });

        AlertDialog alertdialog = alert.create();
        alertdialog.show();
        alertdialog.setCancelable(false);
    }



    public void alertBeforeTrial(){
        AlertDialog.Builder alert = new AlertDialog.Builder(MainActivity.this);
        alert.setTitle("Next Trial # "+trialNumber%NBTRIALS);
        alert.setMessage("Click ok when you're ready for the next trial");

        alert.setPositiveButton("Ok", new DialogInterface.OnClickListener() {
        public void onClick(DialogInterface dialog, int whichButton) {
        launchTrial();
            mAlertVisible = false ; // So that we'll display the next alert when the trial is Over
            isPopUp = false ;
            return ;   
          
          }
        });
        AlertDialog alertdialog = alert.create();
        alertdialog.show();
        alertdialog.setCancelable(false);
        /*try{
            Thread.sleep(1000); // A little pause for the logging to be saved
        }
        catch(InterruptedException e){
            Log.d("Error on sleep","Error while puttin the UI thread to sleep waiting for the logging");
        }*/
        //alert.show();
        
    }

    @Override
    @SuppressWarnings("deprecation")
    public void onCreate(Bundle savedInstanceState) {
        boolean wasInitialized = isInitialized();

        super.onCreate(savedInstanceState);
        mHandler=new Handler();

        if (!isInitialized()) // || !isCameraAvailable())
            return;

        //Since the training is done first, we do not use that here but call it when training is done
        //getStartInfo();

        FluidMechanics.getSettings(fluidSettings);
        FluidMechanics.getState(fluidState);
        fluidSettings.precision = MAXPRECISION ;
        fluidSettings.translatePlane = false ;
        fluidSettings.controlType = RATE_CONTROL;
        fluidSettings.isTraining = true ;
        //fluidSettings.dataORplane = 0 ; //Data 

        //this.client = new Client();
        //this.client.execute();
        // Request an overlaid action bar (title bar)
        getWindow().requestFeature(Window.FEATURE_ACTION_BAR_OVERLAY);

        // setContentView(R.layout.main);
        setContentView(R.layout.main_noar);

        mView = (ARSurfaceView)findViewById(R.id.glSurfaceView);
        setView(mView);

        // ContextMenuFragment content = new ContextMenuFragment();
        // getFragmentManager().beginTransaction().add(android.R.id.content, content).commit();
        registerForContextMenu(mView);

        mView.setOnTouchListener(this);

        mGestureDetector = new GestureDetector(new SimpleOnGestureListener() {});
        //mGestureDetector.setOnDoubleTapListener(this);
        // view.setOnLongClickListener(this);

        setupActionBar();
        //setupSlider();
        setupSliderPrecision();

        if (!wasInitialized) {
            mDataSet = 0;
            loadNewData();
        }

        // =================================================
        // Tango service

        // Instantiate Tango client
        mTango = new Tango(this);

        // Set up Tango configuration for motion tracking
        // If you want to use other APIs, add more appropriate to the config
        // like: mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true)
        mConfig = mTango.getConfig(TangoConfig.CONFIG_TYPE_CURRENT);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_MOTIONTRACKING, true);

        mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        mGyro = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mRotation = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);

        
        FluidMechanics.setInteractionMode(this.interactionMode);

        this.tangibleBtn = (ImageButton) findViewById(R.id.tangibleBtn);
        //this.tangibleBtn.setOnClickListener(this);
        
        //Fix for the experiment
        this.tangibleBtn.setVisibility(View.GONE);
        //this.tangibleBtn.setOnTouchListener(this);

        //End fix

        touchToggle = (ToggleButton) findViewById(R.id.touchToggle);
        //Fix for the experiment
        isTouchOn = false ;
        touchToggle.setVisibility(View.GONE);

        /*touchToggle.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                
                if (isChecked) {
                    isTouchOn = true ;
                } else {
                    isTouchOn = false ;
                }
                setInteractionMode();
            }

        });*/
        //End Fix

        tangibleToggle = (ToggleButton) findViewById(R.id.tangibleToggle);
        //Fix for the experiment
        tangibleToggle.setVisibility(View.GONE);
        isTangibleOn = true ;
        setInteractionMode();
        /*tangibleToggle.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    isTangibleOn = true ;
                } else {
                    isTangibleOn = false ;
                }
                setInteractionMode();
            }

        });*/
        //End Fix


        bluetoothOverlay = (TextView) findViewById(R.id.textOverlay);
        /*this.tangibleBtn.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View view) {
                Log.d(TAG,"Button Clicked");
                Toast.makeText(MainActivity.this, "Button Clicked", Toast.LENGTH_SHORT).show();
            }
        });*/

        resetBtn = (Button) findViewById(R.id.resetBtn);
        this.resetBtn.setOnClickListener(new OnClickListener(){
            @Override
            public void onClick(View view){
                Log.d("ResetBtn","ResetBtn clicked");
                reset();
            } 
        });

        endTrainingBtn = (Button) findViewById(R.id.endTrainingBtn);
        this.endTrainingBtn.setOnClickListener(new OnClickListener(){
            @Override
            public void onClick(View view){
                Log.d("End Training","End Training clicked");
                alertBeforeEndOfTraining();
            } 
        });

        egoBtn = (Button) findViewById(R.id.egoBtn);
        this.egoBtn.setOnClickListener(new OnClickListener(){
            @Override
            public void onClick(View view){
                isEgo=!isEgo;
                if(isEgo){
                    egoBtn.setText("Ego");
                }
                else{
                    egoBtn.setText("Allo");
                }
                FluidMechanics.isEgo(isEgo);
            } 
        });

        


        Display display = getWindowManager().getDefaultDisplay();
        Point size = new Point();
        display.getSize(size);
        int width = size.x;
        int height = size.y;

        Log.d(TAG,"width = "+width+" height = "+height);

        Date date = new Date();
        initialTime = date.getTime();
        logging = new Logging();


        KeyguardManager keyguardManager = (KeyguardManager) getSystemService(Activity.KEYGUARD_SERVICE);  
        KeyguardManager.KeyguardLock lock = keyguardManager.newKeyguardLock(KEYGUARD_SERVICE);  
        lock.disableKeyguard();  


        //BLE
        /*bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        scanStarted = true;
        bluetoothAdapter.startLeScan(new UUID[]{ RFduinoService.UUID_SERVICE },MainActivity.this);
        //Intent rfduinoIntent = new Intent(MainActivity.this, RFduinoService.class);
        //bindService(rfduinoIntent, rfduinoServiceConnection, BIND_AUTO_CREATE);
        //launchBluetooth();
        //scanBluetooth();
        setStateOverlay();*/
        FluidMechanics.initJNI();
        connectBluetooth();

    }

    private void setInteractionMode(){
        if(isTangibleOn == false){
            if(isTouchOn == false){
                interactionMode = nothing ;
                //fluidSettings.interactionMode = nothing ;
                Log.d(TAG,"No Interaction");
            }
            else{
                interactionMode = dataTouch ;
                Log.d(TAG,"dataTouch");
                //fluidSettings.interactionMode = dataTouch ;
            }
        }

        else{
            if(isTouchOn == false){

                interactionMode = dataTangible ;
                Log.d(TAG,"dataTangible");
                //fluidSettings.interactionMode = dataTangible ;
            }
            else{

                interactionMode = dataTouchTangible ;
                Log.d(TAG,"dataTouchTangible");
                //fluidSettings.interactionMode = dataTouchTangible ;
            }
        }

        FluidMechanics.setInteractionMode(interactionMode);
    }


    protected void loggingFunction()throws IOException{
        /*if(this.isInteracting == false){
            return ;
        }
        else{*/
            Date date = new Date();
            long current = date.getTime();
            long timestamp = current - initialTime ;
            synchronized(lock){
                writeLine(timestamp);
            }
            this.isInteracting = false ;
            //Log.d(TAG,"Logging");
        //}
        
        

    }

    private void closeFile(){
        try{
            outputStreamWriter.close();
            fOut.close();
        }
        
        catch (IOException e) {
            Log.e(TAG, "Close failed" + e.toString());
        } 
    }

    private String getLogString(long timestamp){
        //Log.d(TAG,"LOGGING");
        return (""+timestamp+";"
                 +mDataSet+";"
                 +interactionMode+";"
                 +fluidSettings.precision+";"
                 +interactionType+";"
                 +nbOfFingers+";"
                 +nbOfFingers+";"
                 +nbOfFingersButton+";"
                 +nbOfResets+";"
                 +"\n") ;
                //LOG NBFINGERS ON SCREEN
    }

    private void writeLine(long timestamp) throws IOException{
        outputStreamWriter.write(getLogString(timestamp));
        //outputStreamWriter.flush();
    }


    private void writeLogging(){
        Log.d(TAG,"Writing Info to SD CARD to "+"/sdcard/test"+"/"+FILENAME);
        try {
             //OutputStreamWriter outputStreamWriter = new OutputStreamWriter(openFileOutput("/sdcard/test"+"/"+FILENAME, Context.MODE_PRIVATE));
            FileOutputStream fOut = new FileOutputStream("/sdcard/test"+"/"+FILENAME);
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(fOut);
            
            for(int i = 0 ; i < logging.size ; i++){
                outputStreamWriter.write(logging.getString(i));    
            }
            outputStreamWriter.close();
            fOut.close();
        }
        catch (IOException e) {
            Log.e(TAG, "File write failed: " + e.toString());
        } 
    }

    private void setTangoListeners() {
        // Select coordinate frame pairs
        ArrayList<TangoCoordinateFramePair> framePairs = new ArrayList<TangoCoordinateFramePair>();
        framePairs.add(new TangoCoordinateFramePair(
                           TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE, // OK?
                           // TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION, // needs KEY_BOOLEAN_LEARNINGMODE == true
                           TangoPoseData.COORDINATE_FRAME_DEVICE
                           ));
        // Add a listener for Tango pose data
        mTango.connectListener(framePairs, this);
    }

    // private static final int SECS_TO_MILLISECS = 1000;
    // private static final double UPDATE_INTERVAL_MS = 50.0;
    // private double mPreviousTimeStamp;
    // private double mTimeToNextUpdate = UPDATE_INTERVAL_MS;
    @Override
    public void onPoseAvailable(TangoPoseData pose) {
        // Log.d(TAG, "onPoseAvailable");

        // if (pose.statusCode == TangoPoseData.POSE_INVALID) {
        if (pose.statusCode != TangoPoseData.POSE_VALID && mDatasetLoaded == true) {
            Log.w(TAG, "Invalid pose state");

            // if (mTranslationsEnabled) {
            //     runOnUiThread(new Runnable() {
            //         @Override
            //         public void run() {
            //             mTextOverlay.setVisibility(View.VISIBLE);
            //             mTextOverlay.setText("Position non détectée !");
            //         }});
            // }
        } else {
            //if(isTangibleOn){     //Can't use it because we still need to monitor the position of the tango

                this.interactionType = tangibleInteraction ;
                FluidMechanics.setTangoValues(pose.translation[0],pose.translation[1],pose.translation[2],
                                          pose.rotation[0],pose.rotation[1],pose.rotation[2],pose.rotation[3] ) ;
                
                //synchronized(lock){
                    
                //}
            //}
            
            // runOnUiThread(new Runnable() {
            //     @Override
            //     public void run() {
            //         mTextOverlay.setVisibility(View.INVISIBLE);
            //     }});
        }
        if(isTangiblePressed){
            this.isInteracting = true ;
        }
        //requestRender();

    }

    @Override
    public void onXyzIjAvailable(TangoXyzIjData arg0) {
        // Ignoring XyzIj data
    }

    @Override
    public void onTangoEvent(TangoEvent arg0) {
        // Ignoring TangoEvents
    }

    @Override
    public void onFrameAvailable(int arg0) {
        // Ignoring onFrameAvailable Events

    }

    @Override
   public void onSensorChanged(SensorEvent event) {
       if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE && mDatasetLoaded == true ) {
           if (mLastTimestamp != 0) {
               float dt = (event.timestamp - mLastTimestamp) * NS2S;
               if (dt != 0 && isTangiblePressed) {
                   FluidMechanics.setGyroValues(dt * event.values[0],   // rx
                                                dt * event.values[1],   // ry
                                                dt * event.values[2],0);  // rz
                   this.interactionType = tangibleInteraction ;
                   requestRender();

                   
               }
           }
           mLastTimestamp = event.timestamp;
       } else if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
           // Attention aux axes qui peuvent varier selon le device !

            double x = Math.asin(event.values[0]);
            float y = event.values[1];
            float z = event.values[2];

            if(x <= (Math.PI/4 + Math.PI/8) && x >= (Math.PI/4 - Math.PI/8) ){
                //Log.d(TAG,"ZZZZZZZZZZZZZZZZZZZZZZ");
            }
            if(x >= (0 + Math.PI/8) && x <= (Math.PI/2 - Math.PI/8) ){
                //Log.d(TAG,"YYYYYYYYYYYYYYYYYYYYYY");
            }
       }
       
   }

   @Override
   public void onAccuracyChanged(Sensor sensor, int accuracy) {
       // Log.d(TAG, "onAccuracyChanged");
   }

   @Override
    protected void onStart() {
        super.onStart();

        /*registerReceiver(scanModeReceiver, new IntentFilter(BluetoothAdapter.ACTION_SCAN_MODE_CHANGED));
        registerReceiver(bluetoothStateReceiver, new IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED));
        registerReceiver(rfduinoReceiver, RFduinoService.getIntentFilter());

        updateState(bluetoothAdapter.isEnabled() ? STATE_DISCONNECTED : STATE_BLUETOOTH_OFF);
        setStateOverlay();*/
        //onStartBluetooth();
    }


    @Override
    protected void onPause() {
        super.onPause();
        try {
            mTango.disconnect();
            mIsTangoServiceConnected = false;
        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), "Tango Error!",
                           Toast.LENGTH_SHORT).show();
        }
        mSensorManager.unregisterListener(this);
        disconnectBluetooth();

    }


    @Override
    protected void onResume() {
        super.onResume();
        if (!mIsTangoServiceConnected) {
            try {
                setTangoListeners();
            } catch (TangoErrorException e) {
                Toast.makeText(this, "Tango Error! Restart the app!",
                               Toast.LENGTH_SHORT).show();
            }
            try {
                mTango.connect(mConfig);
                mIsTangoServiceConnected = true;
            } catch (TangoOutOfDateException e) {
                Toast.makeText(getApplicationContext(),
                               "Tango Service out of date!", Toast.LENGTH_SHORT)
                    .show();
            } catch (TangoErrorException e) {
                Toast.makeText(getApplicationContext(),
                               "Tango Error! Restart the app!", Toast.LENGTH_SHORT)
                    .show();
            }
        }
        mSensorManager.registerListener(this, mGyro, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, mRotation, SensorManager.SENSOR_DELAY_UI);
        setStateOverlay();
        connectBluetooth();

    }

    @Override
    public void onStop () {
        Log.d(TAG,"Finish Activity");
        //writeLogging();
        super.onStop() ;
//        bluetoothAdapter.stopLeScan(this);
//        unregisterReceiver(scanModeReceiver);
//        unregisterReceiver(bluetoothStateReceiver);
//        unregisterReceiver(rfduinoReceiver);
        disconnectBluetooth();
    }

   

    private void loadNewData() {
        loadDataset(++mDataSet);
    }

    private void loadDataset(int id) {
        mDatasetLoaded = false;

        mDataSet = (id % 4);
        //client.dataset = mDataSet ;
/*
        FluidMechanics.loadDataset(copyAssetsFileToStorage("bunny.vtk", false));
        mVelocityDatasetLoaded = false;

        return ;*/

        // TODO: check exceptions + display error message
        // TODO: load in background?
        switch (mDataSet) {
        // switch ((mDataSet++ % 3)) {
            case ftle:
                FluidMechanics.loadDataset(copyAssetsFileToStorage("ftlelog.vtk", false));
                mVelocityDatasetLoaded = false;
                //client.dataset = ftle ;
                break;
            case ironprot:
                // FluidMechanics.loadDataset(copyAssetsFileToStorage("head.vti", false));
                FluidMechanics.loadDataset(copyAssetsFileToStorage("ironProt.vtk", false));
                mVelocityDatasetLoaded = false;
                //client.dataset = ironprot ;
                break;
            case head:
                FluidMechanics.loadDataset(copyAssetsFileToStorage("head.vti", false));
                mVelocityDatasetLoaded = false;
                //client.dataset = head ;
                break;
            case velocities:
                FluidMechanics.loadDataset(copyAssetsFileToStorage("FTLE7.vtk", false));
                FluidMechanics.loadVelocityDataset(copyAssetsFileToStorage("Velocities7.vtk", false));
                mVelocityDatasetLoaded = true;
                //client.dataset = velocities ;
                break;


        }

        // We want the large display to change as well:
        //client.valuesupdated = true ; 


        // Apply the computed zoom factor
        updateDataState();
        // settings.zoomFactor = fluidState.computedZoomFactor;
        settings.zoomFactor = fluidState.computedZoomFactor * 0.75f;
        fluidSettings.showSlice = true ;
        fluidSettings.sliceType = FluidMechanics.SLICE_STYLUS ;
        updateSettings();
        mDatasetLoaded = true;
        requestRender();

    }

        @Override
        public void onCreateContextMenu(ContextMenu menu, View v, ContextMenuInfo menuInfo) {
            super.onCreateContextMenu(menu, v, menuInfo);
            menu.add(Menu.NONE, 0, Menu.NONE, "FTLElog");
            menu.add(Menu.NONE, 1, Menu.NONE, "Iron prot");
            menu.add(Menu.NONE, 2, Menu.NONE, "Head");
            menu.add(Menu.NONE, 3, Menu.NONE, "FTLE + velocity");
        }

        @Override
        public boolean onContextItemSelected(MenuItem item) {
            /*MainActivity.this.*/loadDataset(item.getItemId());
            return super.onContextItemSelected(item);
        }
    // }

    private void setupActionBar() {
        // Remove the title text and icon from the action bar
        getActionBar().setDisplayShowTitleEnabled(false);
        getActionBar().setDisplayShowHomeEnabled(false);
        // // Make the action bar (partially) transparent
        // // getActionBar().setBackgroundDrawable(new ColorDrawable(Color.argb(0, 0, 0, 0)));
        // getActionBar().setBackgroundDrawable(new ColorDrawable(Color.argb(100, 0, 0, 0)));
        getActionBar().setBackgroundDrawable(getResources().getDrawable(R.drawable.actionbar_bg));
    }

    private float dpToPixels(float dp) {
        Resources r = getResources();
        return TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_DIP, dp, r.getDisplayMetrics());
    }

    @SuppressWarnings("deprecation")
    private void setupSlider() {
        // "Jet" color map
        int[] colors = new int[] {
            0xFF00007F, // dark blue
            0xFF0000FF, // blue
            0xFF007FFF, // azure
            0xFF00FFFF, // cyan
            0xFF7FFF7F, // light green
            0xFFFFFF00, // yellow
            0xFFFF7F00, // orange
            0xFFFF0000, // red
            0xFF7F0000  // dark red
        };
        GradientDrawable colormap = new GradientDrawable(GradientDrawable.Orientation.BOTTOM_TOP, colors);
        colormap.setGradientType(GradientDrawable.LINEAR_GRADIENT);
        final VerticalSeekBar slider = (VerticalSeekBar)findViewById(R.id.verticalSlider);
        slider.setBackgroundDrawable(colormap);
        slider.setProgressDrawable(new ColorDrawable(0x00000000)); // transparent

        slider.setProgress((int)(fluidSettings.surfacePercentage * 100));

        final TextView sliderTooltip = (TextView)findViewById(R.id.sliderTooltip);
        sliderTooltip.setVisibility(View.INVISIBLE);

        slider.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
            double mProgress = -1;
            boolean mPressed = false;

			@Override
			public void onStartTrackingTouch(SeekBar seekBar) {
                sliderTooltip.setVisibility(View.VISIBLE);
                mPressed = true;
			}

			@Override
			public void onStopTrackingTouch(SeekBar seekBar) {
                sliderTooltip.setVisibility(View.INVISIBLE);
                if (mProgress != -1) {
                    // Log.d(TAG, "setSurfaceValue " + mProgress);
                    fluidSettings.surfacePreview = false;
                    fluidSettings.surfacePercentage = mProgress;
                    updateDataSettings();
                    mProgress = -1;
                }
                mPressed = false;
			}

			@Override
			public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                // Only handle events called from VerticalSeekBar. Other events, generated
                // from the base class SeekBar, contain bogus values because the Android
                // SeekBar was not meant to be vertical.
                if (fromUser)
                    return;

                if (!mPressed)
                    return;

                sliderTooltip.setText(progress + "%");
                mProgress = (double)progress/seekBar.getMax();
                // Log.d(TAG, "mProgress = " + mProgress);
                int pos = seekBar.getTop() + (int)(seekBar.getHeight() * (1.0 - mProgress));
                sliderTooltip.setPadding((int)dpToPixels(5), 0, (int)dpToPixels(5), 0);
                LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(sliderTooltip.getLayoutParams());
                lp.topMargin = pos - sliderTooltip.getHeight()/2;
                lp.rightMargin = (int)dpToPixels(25);
                sliderTooltip.setLayoutParams(lp);

                
                fluidSettings.surfacePreview = true;
                fluidSettings.surfacePercentage = mProgress;
                updateDataSettings();
                
			}
		});
    }


    private void setupSliderPrecision() {
        // "Jet" color map
            int[] colors = new int[] {
                0xFF00007F, // dark blue
                0xFF0000FF, // blue
                0xFF007FFF, // azure
                0xFF00FFFF, // cyan
                0xFF7FFF7F, // light green
                0xFFFFFF00, // yellow
                0xFFFF7F00, // orange
                0xFFFF0000, // red
                0xFF7F0000  // dark red
            };
            //Have to use int
            final int step = 1;
            final int max = (int)(MAXPRECISION*100);
            final int min = (int)(MINPRECISION*100);
            final int initialValue = (int)(MINPRECISION*100) ;
            final double initialPosition = (int)(MINPRECISION*100);

            GradientDrawable colormap = new GradientDrawable(GradientDrawable.Orientation.BOTTOM_TOP, colors);
            colormap.setGradientType(GradientDrawable.LINEAR_GRADIENT);
            final VerticalSeekBar sliderPrecision = (VerticalSeekBar)findViewById(R.id.verticalSliderPrecision);
            //sliderPrecision.setBackgroundDrawable(colormap);
            //sliderPrecision.setProgressDrawable(new ColorDrawable(0x00000000)); // transparent

            sliderPrecision.setMax( (max - min) / step );
            sliderPrecision.setProgress((int)(initialPosition));

            final TextView sliderTooltipPrecision = (TextView)findViewById(R.id.sliderTooltipPrecision);
            sliderTooltipPrecision.setVisibility(View.INVISIBLE);

            sliderPrecision.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
                double mProgress = -1;
                boolean mPressed = false;

                @Override
                public void onStartTrackingTouch(SeekBar seekBar) {
                    if(fluidSettings.controlType == SLIDER_CONTROL){
                        sliderTooltipPrecision.setVisibility(View.VISIBLE);
                        mPressed = true;
                        Log.d(TAG, "Precision Java = " + mProgress);
                    }
                    
                }

                @Override
                public void onStopTrackingTouch(SeekBar seekBar) {
                    if(fluidSettings.controlType == SLIDER_CONTROL){
                        sliderTooltipPrecision.setVisibility(View.INVISIBLE);
                        if (mProgress != -1) {
                            // Log.d(TAG, "setSurfaceValue " + mProgress);
                            fluidSettings.precision = (float)mProgress;
                            updateDataSettings();
                            mProgress = -1;
                        }
                    }
                    mPressed = false;
                }

                @Override
                public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                    // Only handle events called from VerticalSeekBar. Other events, generated
                    // from the base class SeekBar, contain bogus values because the Android
                    // SeekBar was not meant to be vertical.                    
                    
                    mProgress = (double)progress/seekBar.getMax();
                    // Log.d(TAG, "mProgress = " + mProgress);
                    int pos = seekBar.getTop() + (int)(seekBar.getHeight() * (1.0 - mProgress));
                    sliderTooltipPrecision.setPadding((int)dpToPixels(5), 0, (int)dpToPixels(5), 0);
                    LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(sliderTooltipPrecision.getLayoutParams());
                    lp.topMargin = pos - sliderTooltipPrecision.getHeight()/2;
                    lp.rightMargin = (int)dpToPixels(25);
                    sliderTooltipPrecision.setLayoutParams(lp);

                    //fluidSettings.surfacePreview = true;
                    double value = min/(100.0) + (progress * (step/100.0));
                    fluidSettings.precision = (float)value ;
                    Log.d("ValueP"," = "+fluidSettings.precision);
                    DecimalFormat df = new DecimalFormat("0.00##");
                    String tooltipvalue = df.format(value);
                    sliderTooltipPrecision.setText(tooltipvalue + "");
                    updateDataSettings();
                    //Log.d(TAG, "Precision Java = " + mProgress);
                }
            });
        
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.main_menu, menu);

        mAxisClippingMenuItem = menu.findItem(R.id.action_axisClipping);
        mStylusClippingMenuItem = menu.findItem(R.id.action_stylusClipping);

        if (!menuInitialized) {
            fluidSettings.showVolume = menu.findItem(R.id.action_showVolume).isChecked();
            fluidSettings.showSurface = menu.findItem(R.id.action_showSurface).isChecked();
            fluidSettings.showStylus = true;
            fluidSettings.showSlice = menu.findItem(R.id.action_showSlice).isChecked();
            fluidSettings.showOutline = menu.findItem(R.id.action_showOutline).isChecked();
            settings.showCamera = menu.findItem(R.id.action_showCamera).isChecked();
            fluidSettings.showCrossingLines = menu.findItem(R.id.action_showLines).isChecked();

            updateSettings();
            updateDataSettings();

            menuInitialized = true;

        } else {
            menu.findItem(R.id.action_showVolume).setChecked(fluidSettings.showVolume);
            menu.findItem(R.id.action_showSurface).setChecked(fluidSettings.showSurface);
            menu.findItem(R.id.action_showSlice).setChecked(fluidSettings.showSlice);
            menu.findItem(R.id.action_showCamera).setChecked(settings.showCamera);
            menu.findItem(R.id.action_showLines).setChecked(fluidSettings.showCrossingLines);
            menu.findItem(R.id.action_showOutline).setChecked(fluidSettings.showOutline);
            menu.findItem(R.id.action_axisClipping).setChecked(fluidSettings.sliceType == FluidMechanics.SLICE_AXIS);
            menu.findItem(R.id.action_stylusClipping).setChecked(fluidSettings.sliceType == FluidMechanics.SLICE_STYLUS);

        }

        return true;
    }

    @Override
    public boolean onPrepareOptionsMenu(Menu menu) {
        menu.findItem(R.id.action_clipDist).setEnabled(
            !menu.findItem(R.id.action_stylusClipping).isChecked()
            && menu.findItem(R.id.action_showSlice).isChecked()
        );
        menu.findItem(R.id.action_axisClipping).setEnabled(
            menu.findItem(R.id.action_showSlice).isChecked()
        );
        menu.findItem(R.id.action_stylusClipping).setEnabled(
            menu.findItem(R.id.action_showSlice).isChecked()
        );
        return true;
    }

    @Override
	public boolean onOptionsItemSelected(MenuItem item) {
        boolean handledSetting = false;
        boolean handledDataSetting = false;
        int tmp ;

		switch (item.getItemId()) {

// Menu show
            case R.id.action_showVolume:
                fluidSettings.showVolume = !fluidSettings.showVolume;
                item.setChecked(fluidSettings.showVolume);
                handledDataSetting = true;
                break;

            case R.id.action_showSurface:
                fluidSettings.showSurface = !fluidSettings.showSurface;
                item.setChecked(fluidSettings.showSurface);
                handledDataSetting = true;
                break;

            case R.id.action_showSlice:
                fluidSettings.showSlice = !fluidSettings.showSlice;
                item.setChecked(fluidSettings.showSlice);
                handledDataSetting = true;
                break;

            case R.id.action_showOutline:
                fluidSettings.showOutline = !fluidSettings.showOutline;
                item.setChecked(fluidSettings.showOutline);
                handledDataSetting = true ;
                break;

            case R.id.action_showCamera:
                settings.showCamera = !settings.showCamera;
                item.setChecked(settings.showCamera);
                handledSetting = true;
                break;

            case R.id.action_showLines:
                fluidSettings.showCrossingLines = !fluidSettings.showCrossingLines;
                item.setChecked(fluidSettings.showCrossingLines);
                handledDataSetting = true;
                break;

//End Menu Show
//Start Menu Action
            case R.id.action_axisClipping:
                item.setChecked(!item.isChecked());
                if (item.isChecked() && mStylusClippingMenuItem.isChecked())
                    mStylusClippingMenuItem.setChecked(false);
                handledDataSetting = true;
                break;

            case R.id.action_stylusClipping:
                item.setChecked(!item.isChecked());
                if (item.isChecked() && mAxisClippingMenuItem.isChecked())
                    mAxisClippingMenuItem.setChecked(false);
                handledDataSetting = true;
                break;

            case R.id.action_clipDist:
                showDistanceDialog();
                break;

            case R.id.change_IP:
                //changeIP();
                break ;

            case R.id.action_reset:
                Log.d(TAG,"Reset");
                reset();
                break;

            case R.id.action_bluetooth:
                item.setChecked(!item.isChecked());
                if(item.isChecked()){
                    connectBluetooth();
                }
                else{
                    disconnectBluetooth();
                }
                
                setStateOverlay();
                break ;

            case R.id.action_quit:
                Log.d(TAG,"Quit");
                //writeLogging();
                closeFile();
                //this.finish();
                //android.os.Process.killProcess(android.os.Process.myPid());
                System.exit(0);
                break ;        

            /* Dataset */
            case R.id.action_dataset_ftle:
                loadDataset(ftle);
                reset();
                break ;
            case R.id.action_dataset_head:
                loadDataset(head);
                reset();
                break ;
            case R.id.action_dataset_ironprot:
                loadDataset(ironprot);
                reset();
                break ;
            case R.id.action_dataset_velocities:
                loadDataset(velocities);
                reset();
                break ;


            case R.id.action_ratecontrol:
                item.setChecked(!item.isChecked());
                if(item.isChecked()){
                    //changeControlType(RATE_CONTROL);
                    changeControlType(RATE_CONTROL_SIMPLE);
                }
                break;

            case R.id.action_speedcontrol:
                item.setChecked(!item.isChecked());
                if(item.isChecked()){
                    changeControlType(SPEED_CONTROL);
                }
                break;

            case R.id.action_pressurecontrol:
                item.setChecked(!item.isChecked());
                if(item.isChecked()){
                    changeControlType(PRESSURE_CONTROL_REVERSE);
                }
                break;

            case R.id.action_slidercontrol:
                item.setChecked(!item.isChecked());
                if(item.isChecked()){
                    changeControlType(SLIDER_CONTROL);
                }
                break;




        }

        if (handledSetting) {
            updateSettings();
            return true;
        } else if (handledDataSetting) {
            updateDataSettings();
            return true;
        } else {
            return super.onOptionsItemSelected(item);
        }
    }

    private void reset(){
        FluidMechanics.reset();
        
        //fluidSettings.dataORplane = 0 ; //Data 
        
        this.interactionMode = nothing ;
        this.tangibleToggle.setChecked(false);
        this.touchToggle.setChecked(false);

        //Fix For the experiment
            //isTangibleOn = false ;
            //isTouchOn = false;
        //End
        dataORplaneTangible = true ; //Data
        dataORplaneTouch = true ;    //Data

        this.nbOfResets += 1 ;

        if(this.nbOfResets%2 == 0){
            fluidSettings.precision = MAXPRECISION ;    
        }
        else{
            fluidSettings.precision = MINPRECISION ;
        }
        updateDataSettings();
        requestRender();

    }

    private void changeControlType(int s){
        fluidSettings.controlType = s ;
        updateDataSettings();
        //TODO ADD changes for the native part as well and propagate
    }


    private void showDistanceDialog() {
        LayoutInflater inflater = getLayoutInflater();
        View layout = inflater.inflate(R.layout.seekbar_dialog, null);

        SeekBar seekbar = (SeekBar)layout.findViewById(R.id.seekbar_dialog_seekbar);
        final TextView text = (TextView)layout.findViewById(R.id.seekbar_dialog_text);

        seekbar.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
                // Log.d(TAG, "tracking started");
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                // Log.d(TAG, "tracking stopped");
            }

            @Override
            public void onProgressChanged(SeekBar seekbar, int progress, boolean fromUser) {
                // Log.d(TAG, "progress changed: " + progress + " (fromUser = " + fromUser + ")");
                text.setText("Distance: " + progress);
                if (fromUser) {
                    fluidSettings.clipDist = progress;
                    updateDataSettings();
                }
            }
        });

        seekbar.setMax(2000);
        seekbar.setProgress((int)fluidSettings.clipDist);

        new AlertDialog.Builder(this)
            .setView(layout)
            .setTitle("Slicing distance")
            .setPositiveButton("Close", null)
            .show();
    }

    private void updateDataState() {
        FluidMechanics.getState(fluidState);
    }

    private void updateSettings() {
        NativeApp.setSettings(settings);
    }

    private void getSettings(){
        NativeApp.getSettings(settings);
    }

    private void updateDataSettings() {
        fluidSettings.sliceType =
            mStylusClippingMenuItem.isChecked() ? FluidMechanics.SLICE_STYLUS
            : mAxisClippingMenuItem.isChecked() ? FluidMechanics.SLICE_AXIS
            : FluidMechanics.SLICE_CAMERA;

        FluidMechanics.setSettings(fluidSettings);

        VerticalSeekBar verticalSlider = (VerticalSeekBar)findViewById(R.id.verticalSlider);
        if (verticalSlider != null) {
            verticalSlider.setVisibility(
                fluidSettings.showSurface ? View.VISIBLE : View.INVISIBLE);
        }
    }

    @Override
    protected void onFrameProcessed() {
        FluidMechanics.getState(fluidState);
    }

    @Override
    public void onBackPressed() {
        // (ignore the back button)
    }

    private boolean isButton(View v){
        int id = v.getId();
        if(id == R.id.tangibleBtn ){

                return true ;
        }
        return false ;
    }

    private boolean isOnTouchButton(float x, float y){
        if(x < 270){
            //Log.d("Seed or Tangible");
            return true ;
        }
        else if(x<=500 && y>=820){
            //Log.d(TAG,"Buttons on the left");
            return true ;
        }
        else if(x>=1500 && y >=820){
            //Log.d(TAG,"Buttons on the right");
            return true ;
        }
        return false ;
    }

    @Override
    public boolean onTouch(View v, MotionEvent event) {
        //First compute RAW coordinates of each touch event
        float rawPosX[] = new float[5];
        float rawPosY[] = new float[5];
        float ofsX = event.getRawX() - event.getX();
        float ofsY = event.getRawY() - event.getY();
        boolean removedButtonFinger = false ;
        for (int i = 0; i < event.getPointerCount(); i++) {
            rawPosX[i] = event.getX(i) + ofsX;
            rawPosY[i] = event.getY(i) + ofsY;
            //isOnTouchButton(rawPosX[i],rawPosY[i]);
            //Log.d(TAG,"Finger "+i+" X = "+rawPosX[i]+" Y = "+rawPosY[i]);
        }

        //FIX For the Experiment
        /*if(v.getId() == R.id.tangibleBtn){
            int index = event.getActionIndex();
            //Log.d(TAG,"INDEX = "+fingerOnButtonIndex);
            if (event.getAction() == MotionEvent.ACTION_DOWN ){
                isTangiblePressed = true ;
                FluidMechanics.buttonPressed();
                this.tangibleBtn.setPressed(true);
                this.nbOfFingersButton+=1 ;
            }
            else if(event.getAction() == MotionEvent.ACTION_UP ){
                FluidMechanics.buttonReleased();
                isTangiblePressed = false ;
                this.tangibleBtn.setPressed(false);
                this.nbOfFingersButton-=1 ;
                removedButtonFinger = true ;
            }
            
            //return true ;
        }*/


        //Log.d(TAG,"X = "+fluidSettings.considerX+"  -- Y = "+fluidSettings.considerY
        //                +"  -- Z = "+ fluidSettings.considerZ+"  - Rotation  = "+fluidSettings.considerRotation);
        mGestureDetector.onTouchEvent(event);

        //Log.d(TAG, "View == "+(v.getId() == R.id.glSurfaceView));
        if(mDatasetLoaded ){
            this.interactionType = touchInteraction ;
            switch (event.getActionMasked()) {
                case MotionEvent.ACTION_DOWN:
                case MotionEvent.ACTION_POINTER_DOWN:
                {   
                    int index = event.getActionIndex();
                    int id = event.getPointerId(index);
                    //Log.d("Finger ID", "Finger ID = "+id);
                    //Log.d("Finger Index", "Finger Index = "+index);
                    //Log.d(TAG, "Finger X = "+event.getRawX()+" Finger Y = "+event.getRawY());
                    if(isOnTouchButton(rawPosX[index], rawPosY[index]) == false){
                        //Log.d(TAG, "Add Finger X = "+event.getX(index)+"  Y = "+event.getY(index));
                        Log.d(TAG, "Add Finger X = "+rawPosX[index]+"  Y = "+rawPosY[index]);
                        this.nbOfFingers +=1 ;
                        //FluidMechanics.addFinger(event.getX(index), event.getY(index), id);    
                        FluidMechanics.addFinger(rawPosX[index], rawPosY[index], id);    
                    }


                    //Fix for the experiment
                    isTangiblePressed = true ;
                    //FluidMechanics.buttonPressed();
                    this.tangibleBtn.setPressed(true);
                    this.nbOfFingersButton+=1 ;



                    break ;

                }

                case MotionEvent.ACTION_UP:
                case MotionEvent.ACTION_POINTER_UP:
                {
                    int index = event.getActionIndex();
                    int id = event.getPointerId(index);
                    //Log.d("Finger ID", "Finger ID = "+id);
                    //Log.d("Finger Index", "Finger Index = "+index);
                    //if(isOnTouchButton(rawPosX[index], rawPosY[index]) == false){
                        //Log.d(TAG, "Remove Finger");
                        FluidMechanics.removeFinger(id);
                        if(removedButtonFinger == false ){
                            this.nbOfFingers -= 1 ;    
                        }
                        
                    //}

                    //Fix for the experiment
                    //FluidMechanics.buttonReleased();
                    isTangiblePressed = false ;
                    this.tangibleBtn.setPressed(false);
                    this.nbOfFingersButton-=1 ;
                    removedButtonFinger = true ;


                    break ;

                }

                case MotionEvent.ACTION_MOVE:
                {
                    int numPtrs = event.getPointerCount();
                    float [] xPos = new float[numPtrs];
                    float [] yPos = new float[numPtrs];
                    int [] ids = new int[numPtrs];
                    for (int i = 0; i < numPtrs; ++i)
                    {
                        ids[i]  = event.getPointerId(i);
                        xPos[i] = event.getX(i);
                        yPos[i] = event.getY(i);
                            //FluidMechanics.updateFingerPositions(xPos[i],yPos[i],ids[i]);
                            FluidMechanics.updateFingerPositions(rawPosX[i], rawPosY[i], ids[i]);  
                        
                    }
                    break ;
                }
            }

            this.isInteracting = true ;
            Log.d(TAG,"NB OF FINGERS = "+this.nbOfFingers);
            Log.d(TAG,"Nb of fingers button = "+this.nbOfFingersButton);
            // NativeApp.setZoom(mZoomFactor);

        }

        //requestRender();
        setStateOverlay();
        return true;
    }


   public void requestRender(){
        if (mView != null){
            Log.d(TAG,"RequestRender");
            if(trialStarted){
                setStateOverlay();    
            }
            mView.requestRender();
            //client.setData(FluidMechanics.getData());
            //Log.d(TAG,"Request Render");
            //loggingFunction(); 
            if(FluidMechanics.isTrialOver() && !mAlertVisible && idRegistered ){ 
                //The last one is here to prevent the dialogs from showing up first
                trialStarted = false ;
                mAlertVisible = true;
                /*while(FluidMechanics.hasFinishedLog() == false ){
                            try{
                                Thread.sleep(100); 
                                Log.d("LOG","Waiting");
                            }
                            catch(InterruptedException e){
                                Log.d("Error on sleep","Error while puttin the UI thread to sleep waiting for the logging");
                            }
                            //alert.show()

                }*/
                mHandler.post(new Runnable() {
                    public void run(){
                        
                        Log.d("Runnable","Show Alert");
                        showAlerts();
                        Log.d("Runnable","End Show Alert");
                    }
                });
                
            }
        }
            
   } 

   public void changeInteractionMode(int mode){
        this.interactionMode = mode ;
        FluidMechanics.setInteractionMode(mode);
   }



    @Override
    public void onClick(View v) {
    
    }


    public String getConditionName(int condID){
        String techniqueName = "";
        switch(condID){
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
        return techniqueName ;
    }














    //BLE

    private final void launchBluetooth(){
        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        bluetoothAdapter.enable();
    }

    private final void scanBluetooth(){
        scanStarted = true;
        bluetoothAdapter.startLeScan(new UUID[]{ RFduinoService.UUID_SERVICE }, MainActivity.this);
    }

    private final void connectBluetooth(){
        if(!bluestarted){
            launchBluetooth();
            onStartBluetooth();
            scanBluetooth();
            Intent rfduinoIntent = new Intent(MainActivity.this, RFduinoService.class);
            bindService(rfduinoIntent, rfduinoServiceConnection, BIND_AUTO_CREATE);
            bluestarted = true ;
        }
        
    }

    private final void disconnectBluetooth(){
        if(bluestarted){
            onStopBluetooth();
            bluetoothAdapter.stopLeScan(this);
        }
    }

    private final void onStartBluetooth(){
        registerReceiver(scanModeReceiver, new IntentFilter(BluetoothAdapter.ACTION_SCAN_MODE_CHANGED));
        registerReceiver(bluetoothStateReceiver, new IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED));
        registerReceiver(rfduinoReceiver, RFduinoService.getIntentFilter());

        updateState(bluetoothAdapter.isEnabled() ? STATE_DISCONNECTED : STATE_BLUETOOTH_OFF);
        this.areReceiverUnregistered = false ;
    }

    protected void onStopBluetooth() {
        if(!this.areReceiverUnregistered && bluestarted){
            bluetoothAdapter.stopLeScan(this);

            unregisterReceiver(scanModeReceiver);
            unregisterReceiver(bluetoothStateReceiver);
            unregisterReceiver(rfduinoReceiver);
            
            this.areReceiverUnregistered = true ;    
        }
        
    }

    private final BroadcastReceiver bluetoothStateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            int state = intent.getIntExtra(BluetoothAdapter.EXTRA_STATE, 0);
            if (state == BluetoothAdapter.STATE_ON) {
                upgradeState(STATE_DISCONNECTED);
            } else if (state == BluetoothAdapter.STATE_OFF) {
                downgradeState(STATE_BLUETOOTH_OFF);
            }
        }
    };

    private final BroadcastReceiver scanModeReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            scanning = (bluetoothAdapter.getScanMode() != BluetoothAdapter.SCAN_MODE_NONE);
            scanStarted &= scanning;
        }
    };

    private final ServiceConnection rfduinoServiceConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            rfduinoService = ((RFduinoService.LocalBinder) service).getService();
            if (rfduinoService.initialize()) {
                if (rfduinoService.connect(bluetoothDevice.getAddress())) {
                    upgradeState(STATE_CONNECTING);
                }
            }
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
            rfduinoService = null;
            downgradeState(STATE_DISCONNECTED);
        }
    };

    private final BroadcastReceiver rfduinoReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            Log.d("Bluetooth", "on Receive");
            final String action = intent.getAction();
            if (RFduinoService.ACTION_CONNECTED.equals(action)) {
                Log.d("Bluetooth", "Action connected");
                upgradeState(STATE_CONNECTED);
            } else if (RFduinoService.ACTION_DISCONNECTED.equals(action)) {
                Log.d("Bluetooth", "Action disconnected");
                downgradeState(STATE_DISCONNECTED);
            } else if (RFduinoService.ACTION_DATA_AVAILABLE.equals(action)) {
                Log.d("Bluetooth", "Action data available");
                getData(intent.getByteArrayExtra(RFduinoService.EXTRA_DATA));
            }
        }
    };


     private void upgradeState(int newState) {
        if (newState > state) {
            updateState(newState);
        }
    }

    private void downgradeState(int newState) {
        if (newState < state) {
            updateState(newState);
        }
    }

    private void updateState(int newState) {
        state = newState;
    }


    private void getData(byte[] data) {

        if(isPopUp || mAlertVisible){
            Log.d("TrialLaunched","PopUPVISIBLE");
            return ; //To avoid the bug of the technique not changing
        }
        Log.d("Bluetooth","test");

        Log.d("Bluetooth", "Control Type = "+fluidSettings.controlType);
        //if(fluidSettings.controlType == PRESSURE_CONTROL || fluidSettings.controlType == PRESSURE_CONTROL_REVERSE){
            value = HexAsciiHelper.HexToFloat(HexAsciiHelper.bytesToHex(data));
            Log.d("Bluetooth","value = "+value);
            if(value < 0 ){
                //Case with no contact at all
                FluidMechanics.buttonReleased();
                isTangiblePressed = false ;
            }
            else{
                if(fluidSettings.controlType == PRESSURE_CONTROL || fluidSettings.controlType == PRESSURE_CONTROL_REVERSE){

                    if(value < MINPRESSURE)     value = MINPRESSURE ;
                    if(value > MAXPRESSURE)     value = MAXPRESSURE ;

                    //Have to use int
                    final int step = 1;
                    final int max = (int)(MAXPRECISION * 100);
                    final int min = (int)(MINPRECISION * 100);
                    final int initialValue = max ;
                    final double initialPosition = (double)max ;
                    final int valueInt ;
                    if(fluidSettings.controlType == PRESSURE_CONTROL_REVERSE){
                        value = Utils.convertIntoNewRange(MINPRECISION,MAXPRECISION,MINPRESSURE,MAXPRESSURE,value);
                        valueInt = (int) (value * 100) - min; //Because the slider cannot be set at something else than 0    
                    }
                    else{
                        value = Utils.convertIntoNewRange(MAXPRECISION,MINPRECISION,MINPRESSURE,MAXPRESSURE,value);
                        //value = MAXPRESSURE - value ;
                        valueInt = (int) (value * 100) - min;
                    }

                    final VerticalSeekBar sliderPrecision = (VerticalSeekBar)findViewById(R.id.verticalSliderPrecision);

                    sliderPrecision.setMax( (max - min) / step );
                    sliderPrecision.customSetProgress(valueInt);
                    sliderPrecision.setMax( (max - min) / step );
                    //Log.d("Bluetooth ValueSlider","Value = "+value+";;;;;Value Int = "+valueInt+" ;;;; Max - Min = "+(max-min)+" ;;;;;Value  Slider = "+(max-valueInt));
                    Log.d("ValueSlider","Value = "+value+";;;;;Value Int = "+valueInt+" Min = "+min );

                    final TextView sliderTooltipPrecision = (TextView)findViewById(R.id.sliderTooltipPrecision);
                    sliderTooltipPrecision.setVisibility(View.INVISIBLE);
                    sliderTooltipPrecision.setText(""+value);

                    fluidSettings.precision = value;
                    updateDataSettings();
                }
                
                FluidMechanics.buttonPressed();
                requestRender();
                isTangiblePressed = true ;

            }
            
        //}
        
    }

    @Override
    public void onLeScan(BluetoothDevice device, final int rssi, final byte[] scanRecord) {
        bluetoothAdapter.stopLeScan(this);
        bluetoothDevice = device;
        MainActivity.this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                bluetoothOverlay.setText(BluetoothHelper.getDeviceInfoText(bluetoothDevice, rssi, scanRecord));
            }
        });
    }

    private void setStateOverlay(){

        String timeText = ""+FluidMechanics.getTime();
        /*if (state == STATE_CONNECTING) {
            connectionText = "Connecting...";
        } else if (state == STATE_CONNECTED) {
            connectionText = "Connected";
        }
        */
        bluetoothOverlay.setText(timeText);
    }




}
