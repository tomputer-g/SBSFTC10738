package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class BaseAuto extends BaseOpMode {

    protected double landerAwayTurningAmount, landerAndCraterOrientation, landerAndWallOrientation;
    private double landerAwayTurningAmountR = 56, landerAndCraterOrientationR = -140, landerAndWallOrientationR = -94;
    private double landerAwayTurningAmountB = 58, landerAndCraterOrientationB = 44, landerAndWallOrientationB = 86;

    //Autonomous params
    protected boolean useTFOD = true, useVuMark = true, useLight = true;
    protected int TFOD_result = -1; //set useTFOD=false and override this value (0~2 int).
    private int TFOD_fallback = 2;

    private boolean VuforiaEngineInit = false;
    //Gyro Pturn params
    protected static final double THRESHOLD_90 = 16.5, THRESHOLD_45 = 15; //turning tolerance
    //TFOD
    protected TFObjectDetector tfod;
    private final int TFOD_TIMEOUT_LIMIT = 500; //ms
    //IMU params
    //imho dummy
    protected static BNO055IMU imu;
    protected static double imuHeading;
    protected static double imuOffset;
    //VuMark params
    protected static final float mmPerInch        = 25.4f;
    protected static final double[][] targetPos = {{-13.8,-50.2},{-29,-37.8},{-44.1,-28.2}}; //TODO: calibrate left vumark position

    //protected static final String[] ImageNames= {"Red-Footprint", "Back-Space", "Blue-Rover", "Front-Craters"};

    protected static final double[] targetOri = {-90,-90,0};
    protected OpenGLMatrix lastLocation = null;
    protected boolean targetVisible = false;
    private VuforiaLocalizer vuforia;
    protected List<VuforiaTrackable> allTrackables;
    //un-inherited from subclasses
    protected boolean firstVumarkUsed = false;
    protected boolean secondVumarkUsed= false;
    protected double currHeading = 0;
    protected double turningAmount = 0;
    protected double[] currOrientation = {0,0,0};
    protected double[] currPosition = {0,0,0};
    protected double XOffset = 0;
    protected double YOffset = 0;
    protected double threshold = 3;


    protected int grabberShoulderFinalPos = -5150;
    protected int grabberSlideFinalPos = -650;
    //-----------------------------------------Overrides here--------------------------------------------

    @Override public void init() {
        msStuckDetectInitLoop = 10000;
        super.init();
        initIMU();
        if(useTFOD)initTFOD();
        if(useLight){
            light = hardwareMap.get(Servo.class,"light");
        }
        if(useVuMark)initVumark();
    }

    @Override
    public void init_loop(){
        if(useTFOD) {
            ElapsedTime t = new ElapsedTime();
            wait(1000);
            TFOD_result = TFOD_double();
            if (TFOD_result != -1) {
                //initLoopFlag = true;
                telemetry.addData("TFOD result", TFOD_result);
                telemetry.addData("Time in init loop", t.milliseconds());

            } else {
                TFOD_result = TFOD_fallback;
                telemetry.addLine("TFOD -1, fallback to " + TFOD_fallback);
            }
        }
        telemetry.update();
    }

    @Override public void stop() {
        grabber_shoulder.setPower(0);
        if(useTFOD)stopTFOD();
        if(useLight)light.setPosition(0.5);
        super.stop();
    }

    protected void getParams(boolean isRed){
        if(isRed){
            landerAwayTurningAmount = landerAwayTurningAmountR;
            landerAndCraterOrientation = landerAndCraterOrientationR;
            landerAndWallOrientation = landerAndWallOrientationR;
        }else{
            landerAwayTurningAmount = landerAwayTurningAmountB;
            landerAndCraterOrientation = landerAndCraterOrientationB;
            landerAndWallOrientation = landerAndWallOrientationB;
        }
    }

    //-------------------------------------------TFOD Code here------------------------------------------
    //Initializes TFOD w/ compatible VuForia engine; cannot use VuMark detection with this @ same time
    protected void initTFOD() {
        if(!VuforiaEngineInit)initVuforiaEngine();
        if(ClassFactory.getInstance().canCreateTFObjectDetector()){
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset("RoverRuckus.tflite", "Gold", "Silver");
        }
        tfod.activate();
    }

    //This should be called in stop() and before VuMark detection is initialized.
    protected void stopTFOD(){
        if(useLight) light.setPosition(0.5);
        if (tfod != null) {
            tfod.deactivate();
            tfod.shutdown();
        }
    }

    //assumes phone camera to be horizontal, Left is bottom
    protected int TFOD_double() {
        if(useLight)light.setPosition(1);
        List<Recognition> recs;
        Recognition r1 = null, r2 = null;
        ElapsedTime timeout = new ElapsedTime();

        do{
            recs = tfod.getRecognitions();
            Collections.sort(recs, new Comparator<Recognition>() {@Override public int compare(Recognition lhs, Recognition rhs) { return (int)(lhs.getLeft() - rhs.getLeft()); }});

            if(recs != null && recs.size() > 0){
                ArrayList<Recognition> arr = new ArrayList<>();
            for (int i = 0;i < recs.size(); i++){
                    Recognition r = recs.get(i);
                if (r.getLeft() > 200 || getTFODArea(r) < 11000 || r.getTop() > 1000){
                    arr.add(r);
                }
            }
            for(Recognition r : arr)
                recs.remove(r);

                if(recs.size() >= 2){
                    r1 = recs.get(0);
                    r2 = recs.get(1);
                }
            }
        }while((recs.size() < 2 || (r1.getLabel().equals("Gold") && r2.getLabel().equals("Gold"))) && timeout.milliseconds() < TFOD_TIMEOUT_LIMIT);
        if(useLight)light.setPosition(0.5);

        if(r1 == null || r2 == null)
            return -1; //timeout without result

        if(r1.getTop() < r2.getTop()){//r1 is left
            if(r1.getLabel().equals("Gold")){//left is gold
                return 0;
            }else if(r2.getLabel().equals("Gold")){//middle gold
                return 1;
            }else{//no gold (right gold)
                return 2;
            }
        }else{//r2 is left
            if(r2.getLabel().equals("Gold")){//left is gold
                return 0;
            }else if(r1.getLabel().equals("Gold")){//middle gold
                return 1;
            }else{//no gold (right gold)
                return 2;
            }
        }
    }

    private double getTFODArea(Recognition r){
        if(r == null) return 0;
        return r.getWidth() * r.getHeight();
    }

    //-----------------------------------------VuMark Code here------------------------------------------

    protected void initVumark(){
        if(!VuforiaEngineInit)initVuforiaEngine();
        final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
        final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

        VuforiaTrackables targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        allTrackables = new ArrayList<>(targetsRoverRuckus);
        allTrackables.addAll(targetsRoverRuckus);

        blueRover.setLocation(OpenGLMatrix.translation(0, mmFTCFieldWidth, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        redFootprint.setLocation(OpenGLMatrix.translation(0, -mmFTCFieldWidth, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
        frontCraters.setLocation(OpenGLMatrix.translation(-mmFTCFieldWidth, 0, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
        backSpace.setLocation(OpenGLMatrix.translation(mmFTCFieldWidth, 0, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

                                                                            // wheel (current)          center
        final int CAMERA_FORWARD_DISPLACEMENT  = (int)(mmPerInch * -6.5);   // -6.5 in          -6.5 in
        final int CAMERA_VERTICAL_DISPLACEMENT = (int)(mmPerInch * 8.75);   // 8.75 in          8.75 in
        final int CAMERA_LEFT_DISPLACEMENT     = (int)(mmPerInch * -7.0);   // -7 in            1 in

        //  Let all the trackable listeners know where the phone is.
        for (VuforiaTrackable trackable : allTrackables)
        {((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,-90, 0, 0)), BACK); }

        targetsRoverRuckus.activate();
    }

    protected int waitUntilVumarkRecognized(){//0: rover, 1: footprint, 2: crater, 3: space
        if(useLight)light.setPosition(1);
        ElapsedTime t = new ElapsedTime();
        while (!targetVisible && t.milliseconds() <= 750){
            telemetry.addLine("waiting for VuMark...");

            for (VuforiaTrackable trackable : allTrackables)
            {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible())
                {
                    targetVisible = true;
                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) lastLocation = robotLocationTransform;
                    light.setPosition(0.5);
                    switch(trackable.getName()){
                        case "Blue-Rover":
                            return 2;
                        case "Red-Footprint":
                            return 0;
                        case "Front-Craters":
                            return 3;
                        case "Back-Space":
                            return 1;
                    }
                }
            }
        }
        if(useLight)light.setPosition(0.5);
        return -1;

    }

    protected void refreshVumarkReading(){
        if(useLight)light.setPosition(1);
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.clearAll();
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                telemetry.addLine("Refresh robotLocationTransform");
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                    telemetry.addLine("Updated Last Location");
                }
                telemetry.addData("last location", lastLocation);
                break;
            }
        }
        if(useLight)light.setPosition(0.5);
    }

    protected boolean vumarkIsVisible() {
        for (VuforiaTrackable trackable : allTrackables)
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible())
                return true;
        return false;
    }

    protected double[] getPositionFromVumark(){//inches
        VectorF translation = lastLocation.getTranslation();
        double x = (int)(translation.get(0) / mmPerInch * 10) / 10.0;
        double y = (int)(translation.get(1) / mmPerInch * 10) / 10.0;
        double z = (int)(translation.get(2) / mmPerInch * 10) / 10.0;
        return new double[]{x, y, z};
    }

    protected double[] getOrientationFromVumark(){//degrees
        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        double roll = (int)(rotation.firstAngle * 10) / 10.0;
        double pitch = (int)(rotation.secondAngle * 10) / 10.0;
        double heading = (int)(rotation.thirdAngle * 10) / 10.0;
        return new double[]{roll, pitch, heading};
    }

    protected double[] posOffset(double[] currPos, double[] targetPos) {
        return new double[]{targetPos[0] - currPos[0], targetPos[1] - currPos[1]};
    }

    protected double headingOffset(double currHeading, double targetheading){
        currHeading = stupidAngleToUnitCircle(currHeading);
        targetheading = stupidAngleToUnitCircle(targetheading);
        double ccwTurningAmount;
        if (currHeading <= targetheading)
        {
            ccwTurningAmount = Math.abs(currHeading-targetheading);
        }
        else
        {
            ccwTurningAmount = 360 - Math.abs(currHeading-targetheading);
        }
        double cwTurningAmount = 360-ccwTurningAmount;
        if (cwTurningAmount <= ccwTurningAmount)
        {
            return -cwTurningAmount;
        }
        else
        {
            return ccwTurningAmount;
        }
    }

    private double stupidAngleToUnitCircle(double stupid){
        if (stupid < 0)
            stupid += 360;
        return stupid;
    }

    protected double[] fieldOffsetToRobotOffset(double fieldX, double fieldY, double fieldAngleOffset) {
        double robotX = - fieldX * Math.cos(fieldAngleOffset) + fieldY * Math.sin(fieldAngleOffset);
        double robotY = fieldX * Math.sin(fieldAngleOffset) + fieldY * Math.cos(fieldAngleOffset);

        return new double[]{robotX, robotY};
    }
    //-----------------------------------------Vuforia Code here-----------------------------------------

    protected void initVuforiaEngine(){//used in init(). Used for both TFOD and Vumark.
        if(vuforia == null) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parameters.vuforiaLicenseKey = "ATRaSk7/////AAAAGYLgjlimQklSl+oDhmYIEjhzUV34Rljx8+M72Lzbu408S2XaUuMmL8Z0SRdMoKdoQ0dZ4/MeKas+GaC6AGw9GOFc4XyrUVlne2Cue3tTjC75ZTPbhh4odsJQVBlXkb88Ww38LX0oWeUnRS9b2GhGhCqPwhKA+HlZk6SCPSBqMVQg/T3TLKPSpouwA74gpdbw0wtdgp+X6K/1zUkSkp43hx7DATnoDEy467aFKlC/V/vgpVfxMbEVZbiHp8rSgmiVlEfPQuIGSq/pMWdmSNEor5LNY1SpV8BBwSp65OxB6ct9WdmOHJHxlhHdPhqpNRtJSdleNSCO4xAjmXuZ+8dkaU+kmTV/+x/4Po4yxuJVBGKo";
            parameters.cameraDirection = BACK;
            vuforia = ClassFactory.getInstance().createVuforia(parameters);
        }
        VuforiaEngineInit = true;
    }

    //----------------------------------------IMU/Gyro Code here-----------------------------------------
    //Should be called in start().
    protected void initIMU(){
        BNO055IMU.Parameters BNOParameters = new BNO055IMU.Parameters();
        BNOParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        BNOParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(BNOParameters);
    }

    //UPDATES imuHeading value with offset.
    protected void getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES);
        imuHeading = Double.parseDouble(String.format(Locale.getDefault(), "%.2f", AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)))) - imuOffset;
    }

    protected void setNewGyro0(){
        imuOffset = 0;
        getHeading();
        imuOffset = imuHeading;
    }


    //----------------------------------------Movement Code here-----------------------------------------
    //for phone: phone camera facing x-, extended grabber is y+

    //requires: motor, encoder
    protected void moveInchesPolar(double r, double theta, double speed){
        double conversion_fct = 3.14159265 / 180;
        moveInches(r*Math.cos(theta * conversion_fct), r*Math.sin(theta * conversion_fct), speed);
    }

    protected void turn(double angle, double speed, double threshold) {
        setMode_RUN_WITH_ENCODER();
        setNewGyro0();
        double p_TURN = 0.25;
        while(!onHeading(speed, angle, p_TURN, threshold));
    }

    private boolean onHeading(double turnSpeed, double angle, double PCoeff, double threshold) {
        double   error = getError(angle), steer, speed;
        boolean  onTarget = false;

        if (Math.abs(error) <= threshold) {
            steer = 0.0;
            speed = 0.0;
            onTarget = true;
            telemetry.addData("ON TARGET!", error);
        }
        else {
            steer = Range.clip(error * PCoeff, -1, 1);
            speed  = turnSpeed * steer;
        }
        setAllDrivePower(speed);
        telemetry.update();
        return onTarget;
    }

    private double getError(double targetAngle) {
        getHeading();
        double robotError = targetAngle - imuHeading;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    //-----------------------------------------Modularized Auto Code here--------------------------------

    protected void dropMarker(){//TODO: re-test this
        grabber.setPower(1);//out
        wait(300);
        grabber.setPower(0);
    }

    protected void lowerRobot(){


        ElapsedTime t = new ElapsedTime();
        lander_lifter.setTargetPosition(14500);
        lander_lifter.setPower(1);
        lander_lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);//!!


        grabber_slide.setTargetPosition(-125);
        grabber_slide.setPower(1);
        grabber_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wait(500);
        grabber_shoulder.setPower(1);
        grabber_shoulder.setTargetPosition(-1000);
        grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wait(800);
        grabber_slide.setTargetPosition(10);

        while(lander_lifter.isBusy() && t.milliseconds() < 7000);

        /*
        grabber_slide.setPower(1);
        lander_lifter.setPower(1);
        grabber_shoulder.setPower(1);

        wait(500);
        grabber_shoulder.setTargetPosition(560);
        grabber_slide.setTargetPosition(0);

        //lander_lifter.setPower(0);*/
    }

    protected double[] getRobotPositionOffset(double VMDeltaX, double VMDeltaY, double VMDeltaTheta) {
        double distanceToTarget = Math.sqrt(VMDeltaX*VMDeltaX + VMDeltaY*VMDeltaY);
        double targetBearing = Math.atan(VMDeltaY/VMDeltaX);

        double dx = distanceToTarget*Math.cos(targetBearing+90-VMDeltaTheta);
        double dy = distanceToTarget*Math.sin(targetBearing+90-VMDeltaTheta);

        return new double[]{dx, dy};
    }

    protected void initGrabber(){
        grabber_shoulder = hardwareMap.get(DcMotor.class,"grabber_shoulder");
        grabber_shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabber_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabber_slide = hardwareMap.get(DcMotor.class, "grabber_slide");
        grabber_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabber_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabber = hardwareMap.get(DcMotor.class, "grabber");
        slideLimitSW = hardwareMap.get(RevTouchSensor.class, "slideLimitSW");
    }
    protected void redCraterMineral(int TFOD_result){
        switch(TFOD_result){
            case 0:
                moveInchesHighSpeedEncoder(0,-15.7,0.3,3,6,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(-9.5,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(10,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(0,-28,0.8,8,16,0.1,0.65,0.15);
                break;
            case 1:
                moveInchesHighSpeedEncoder(-10,0,0.3,3,6,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(10,0,0.3,2,4,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(0,-49,0.8,8,16,0.1,0.55,0.1);
                break;
            case 2:
                moveInchesHighSpeedEncoder(0,15,0.3,3,6,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(-10,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(9,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(0,-62.5,0.8,8,16,0.1,0.55,0.1);
                break;
            default:
                moveInchesHighSpeedEncoder(0,15,0.3,3,6,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(-11,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(9.5,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(0,-61,0.8,8,16,0.1,0.55,0.1);
                break;
        }
    }

    protected void blueCraterMineral(int TFOD_result){
        switch(TFOD_result){
            case 0:
                moveInchesHighSpeedEncoder(0,-15.7,0.3,3,6,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(-9.5,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(10,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(0,-28,0.8,8,16,0.1,0.65,0.15);
                break;
            case 1:
                moveInchesHighSpeedEncoder(-10,0,0.3,3,6,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(10,0,0.3,2,4,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(0,-49,0.8,8,16,0.1,0.55,0.1);
                break;
            case 2:
                moveInchesHighSpeedEncoder(0,15,0.3,3,6,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(-11,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(8,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(0,-62.5,0.8,8,16,0.1,0.55,0.1);
                break;
        }
    }

}
