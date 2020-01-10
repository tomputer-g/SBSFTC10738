package org.firstinspires.ftc.teamcode20;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class BaseAuto extends BaseOpMode {

    //Vuforia
    private VuforiaLocalizer vuforia;
    private boolean VuforiaInit = false;
    protected static final float mmPerInch = 25.4f;
    protected OpenGLMatrix lastLocation = null;
    protected VuforiaTrackables targetsSkyStone;
    protected List<VuforiaTrackable> allTrackables;
    private final float CAMERA_FORWARD_DISPLACEMENT  = 0f * mmPerInch;//2.5 in from end + 1 in correction
    private final float CAMERA_VERTICAL_DISPLACEMENT = 0f * mmPerInch;// eg: Camera is 8 Inches above ground
    private final float CAMERA_LEFT_DISPLACEMENT     = 0f * mmPerInch; // eg: Camera is ON the robot's center line

    //TFOD
    protected TFObjectDetector tfod;
    //Sensors
    protected ModernRoboticsI2cRangeSensor rangeSensorFront, rangeSensorSide;
    protected Rev2mDistanceSensor left,right;

    //IMU
    protected static BNO055IMU imu;
    protected static double imuHeading;
    protected static double imuOffset=0;

    //Vuforia
    private ElapsedTime VuforiaPositionTime;
    private double[] displacements = {2, 7};//+ = forward; + = right
    private double headingDisplacement = -90;

    //Encoders
    protected double xmult = 1430.5/72, ymult = 1305.25/72;
    protected double[] coo={0,0};
    private double xpre=0,ypre=0,theta=0;

    protected void initVuforia(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AZlAJSf/////AAABmf5BgAyil0t8tK506wQNabJX0SH5ekkamom8UybSLKgtsYTY/0/AB5n0Db9/JRrUDLEhDRXJgx5osNHZt6kVKSIF5cdge/dE9OgOunoX6LWBqk8cHGwBlKCXl1eGuvBPwQa3OaJDC7neKLmlZf2/NJiJKMvi9VBqKEDsS74Dp0tFbJka5cJa8YpKyrJh8593SN8p2qcYxXRORCWzmdMdD2xHUJXw28foxuNOotp2onbDmpnfH7x4oegFalegxvQbJ3J0cFqOuP8pboEjoN0Zl64xFVu6ZCc2uvsnXECEgWtycA+bWmQZNG6BD4SLYN/LWVYBp6U5MrIHsNeOOQfwTAZNVDcLELke77iK1XuWnCzG";
        parameters.cameraDirection   = BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix.translation(0, 0, 2.00f * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix.translation(-5.18f * mmPerInch, 23 * mmPerInch, 6.42f * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, 59, 180)));
        blueRearBridge.setLocation(OpenGLMatrix.translation(-5.18f * mmPerInch, 23 * mmPerInch, 6.42f * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -59, 180)));
        redFrontBridge.setLocation(OpenGLMatrix.translation(-5.18f * mmPerInch, -23 * mmPerInch, 6.42f * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -59, 0)));
        redRearBridge.setLocation(OpenGLMatrix.translation(5.18f * mmPerInch, -23 * mmPerInch, 6.42f * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, 59, 0)));
        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix.translation(36 * mmPerInch, -72 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
        red2.setLocation(OpenGLMatrix.translation(-36 * mmPerInch, -72 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
        front1.setLocation(OpenGLMatrix.translation(-72 * mmPerInch, -36 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
        front2.setLocation(OpenGLMatrix.translation(-72 * mmPerInch, 36 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
        blue1.setLocation(OpenGLMatrix.translation(-36 * mmPerInch, 72 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        blue2.setLocation(OpenGLMatrix.translation(36 * mmPerInch, 72 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        rear1.setLocation(OpenGLMatrix.translation(72 * mmPerInch, 36 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        rear2.setLocation(OpenGLMatrix.translation(72 * mmPerInch, -36 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        OpenGLMatrix robotFromCamera = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, 0,0,0));


        for (VuforiaTrackable trackable : allTrackables)
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
    }

    protected void initVuforiaWebcam(){
        VuforiaInit = true;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AZlAJSf/////AAABmf5BgAyil0t8tK506wQNabJX0SH5ekkamom8UybSLKgtsYTY/0/AB5n0Db9/JRrUDLEhDRXJgx5osNHZt6kVKSIF5cdge/dE9OgOunoX6LWBqk8cHGwBlKCXl1eGuvBPwQa3OaJDC7neKLmlZf2/NJiJKMvi9VBqKEDsS74Dp0tFbJka5cJa8YpKyrJh8593SN8p2qcYxXRORCWzmdMdD2xHUJXw28foxuNOotp2onbDmpnfH7x4oegFalegxvQbJ3J0cFqOuP8pboEjoN0Zl64xFVu6ZCc2uvsnXECEgWtycA+bWmQZNG6BD4SLYN/LWVYBp6U5MrIHsNeOOQfwTAZNVDcLELke77iK1XuWnCzG";
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix.translation(0, 0, 2.00f * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix.translation(-5.18f * mmPerInch, 23 * mmPerInch, 6.42f * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, 59, 180)));
        blueRearBridge.setLocation(OpenGLMatrix.translation(-5.18f * mmPerInch, 23 * mmPerInch, 6.42f * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -59, 180)));
        redFrontBridge.setLocation(OpenGLMatrix.translation(-5.18f * mmPerInch, -23 * mmPerInch, 6.42f * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -59, 0)));
        redRearBridge.setLocation(OpenGLMatrix.translation(5.18f * mmPerInch, -23 * mmPerInch, 6.42f * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, 59, 0)));
        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix.translation(36 * mmPerInch, -72 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
        red2.setLocation(OpenGLMatrix.translation(-36 * mmPerInch, -72 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
        front1.setLocation(OpenGLMatrix.translation(-72 * mmPerInch, -36 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
        front2.setLocation(OpenGLMatrix.translation(-72 * mmPerInch, 36 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
        blue1.setLocation(OpenGLMatrix.translation(-36 * mmPerInch, 72 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        blue2.setLocation(OpenGLMatrix.translation(36 * mmPerInch, 72 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        rear1.setLocation(OpenGLMatrix.translation(72 * mmPerInch, 36 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        rear2.setLocation(OpenGLMatrix.translation(72 * mmPerInch, -36 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        OpenGLMatrix robotFromCamera = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, 0,0,180));

        for (VuforiaTrackable trackable : allTrackables)
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
    }

    protected void shutdownVuforia(){
        targetsSkyStone.deactivate();
    }

    protected int skystonePosition(){
        VuforiaPositionTime = new ElapsedTime();
        targetsSkyStone.activate();
        while(VuforiaPositionTime.milliseconds() < 1500){
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    if (trackable.getName().equals("Stone Target")) {
                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        if(showTelemetry)telemetry.addLine("Turn " + (int) Math.abs(rotation.thirdAngle + headingDisplacement) + (rotation.thirdAngle + headingDisplacement > 0 ? "deg. CW" : "deg. CCW"));
                        VectorF translation = lastLocation.getTranslation();
                        double dist = translation.get(1) / mmPerInch + displacements[1];
                        if(showTelemetry)telemetry.addLine("Move " + Math.abs(dist) + (dist > 0 ? "in. Right" : "in. Left"));
                        if (dist > 5) {
                            if(showTelemetry)telemetry.addData("Capture time", VuforiaPositionTime.milliseconds());
                            return 2;
                        }else{
                            if(showTelemetry)telemetry.addData("Capture time", VuforiaPositionTime.milliseconds());
                            return 1;
                        }
                    }
                }
            }
        }
        if(showTelemetry)telemetry.addLine("Vuforia exceeded 1s wait.");
        return 0;
    }



    //TFOD
    protected void initTfod() {
        if(!VuforiaInit)initVuforia();
        if(ClassFactory.getInstance().canCreateTFObjectDetector()){
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minimumConfidence = 0.8;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset("Skystone.tflite", "Stone", "Skystone");
        }
        tfod.activate();
    }

    protected void stopTFOD(){
        if (tfod != null) {
            tfod.deactivate();
            tfod.shutdown();
        }
    }

    //daxie sensers
    protected void initSensors(){
        rangeSensorSide = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "side");
        rangeSensorFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front");
        left = hardwareMap.get(Rev2mDistanceSensor.class,"left");
        right = hardwareMap.get(Rev2mDistanceSensor.class,"right");
        tower_top = hardwareMap.get(Rev2mDistanceSensor.class, "tower_top");
    }

    //IMU
    protected void initIMU(){
        BNO055IMU.Parameters BNOParameters = new BNO055IMU.Parameters();
        BNOParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        BNOParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        BNOParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        BNOParameters.loggingEnabled = true;
        BNOParameters.loggingTag = "imu";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(BNOParameters);
    }

    protected double getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES);
        imuHeading = getError(Double.parseDouble(String.format(Locale.getDefault(), "%.2f", AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)))),imuOffset);
        return imuHeading;
    }

    protected void setNewGyro0(){
        imuOffset = 0;
        getHeading();
        imuOffset = imuHeading;
    }

    protected void setNewGyro(double target){
        imuOffset = target;
    }

    private double getError(double targetAngle) {
        return getError(targetAngle,getHeading());
    }

    private double getError(double target, double cur) {
        double robotError =target-cur;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    private boolean onHeading(double turnSpeed, double angle, double PCoeff, double threshold) {
        double   error = getError(angle), steer, speed;
        boolean  onTarget = false;
        if (Math.abs(error) <= threshold) {
            steer = 0.0;
            speed = 0.0;
            onTarget = true;
            if(showTelemetry)telemetry.addData("ON TARGET!", error);
        }
        else {
            if(showTelemetry)telemetry.addData("not ON TARGET!", error);
            steer = Range.clip(error/180 * PCoeff, -1, 1);
            speed  = turnSpeed * steer;
        }
        setAllDrivePower(speed);
        if(showTelemetry)telemetry.update();
        return onTarget;
    }

    protected void turn(double angle, double speed, double threshold) {
        setMode_RUN_WITHOUT_ENCODER();
        setNewGyro0();
        double p_TURN = 6;
        while(!onHeading(speed, angle, p_TURN, threshold));
    }

    protected void updateCoo(){
        getHeading();
        double dtheta=imuHeading-theta;
        double dx=platform_grabber.getCurrentPosition()-xpre;
        double dy=L2.getCurrentPosition()-ypre;
        coo[0]+=(dx*Math.cos(imuHeading)+dy*Math.sin(imuHeading))*Math.PI/4096;
        coo[1]+=(dx*Math.sin(imuHeading)+dy*Math.cos(imuHeading))*Math.PI/4096;
        xpre=platform_grabber.getCurrentPosition();
        ypre=L2.getCurrentPosition();
        theta=imuHeading;
    }

    protected void setAllDrivePowerG(double a, double b, double c, double d,double Kp){
        double p=Kp*(getHeading()*0.1/9);
        setAllDrivePower(a-p,b-p,c-p,d-p);
    }

    protected void setAllDrivePowerG(double a, double b, double c, double d){
        setAllDrivePowerG(a,b,c,d,0.8);
    }

    protected void setAllDrivePowerG(double power){
        setAllDrivePowerG(power,power,power,power);
    }

    protected void setAllDrivePowerSlow(double dir,double x,double w){
            w *= 1.6;
            double highp = 0.03 / .18;
            try {
                sleep(0, (int) (100 * (1 - highp)));
            } catch (InterruptedException e) {
                telemetry.addLine("Error0");
            }
            setAllDrivePower(-.2 * dir - .5 * x + .2 * w, -.2 * dir + .5 * x + .2 * w, .2 * dir - .5 * x + .2 * w, .2 * dir + .5 * x + .2 * w);
            try {
                sleep(0, (int) (100 * highp));
            } catch (InterruptedException e) {
                telemetry.addLine("Error1");
            }
            setAllDrivePower(-.02 * dir - .05 * x + .02 * w, -.02 * dir + .05 * x + .02 * w, .02 * dir - 0.05 * x + .02 * w, .2 * dir + .05 * x + .2 * w);
    }

    protected void moveInchesG(double xInch, double yInch, double speed,double Kp){
        reset_ENCODER();
        setMode_RUN_WITHOUT_ENCODER();
        //ElapsedTime t = new ElapsedTime();
        //int p_time = (int) (sqrt(xInch*xInch + yInch*yInch)*100);
        speed=Math.abs(speed);
        double fgt=1;
        int encoder_x=(int)(xInch*xmult),encoder_y=(int)(yInch*ymult);//232/12,
        double theta=(yInch==0)?90:Math.abs(Math.atan(xInch/yInch));
        double vx=(xInch==0)?0:xInch/Math.abs(xInch)*Math.sin(theta)*speed;
        double vy=(yInch==0)?0:(yInch/Math.abs(yInch)*Math.cos(theta)*speed);
        boolean elf=Math.abs(-encoder_x-encoder_y)>Math.abs(-LF.getCurrentPosition()),elb=Math.abs(encoder_x-encoder_y)>Math.abs(-LB.getCurrentPosition()),erf=Math.abs(-encoder_x+encoder_y)>Math.abs(-RF.getCurrentPosition()),erb=Math.abs(encoder_x+encoder_y)>Math.abs(-RB.getCurrentPosition());
        while(elf|| elb|| erf|| erb){
            elf=Math.abs(-encoder_x-encoder_y)>Math.abs(-LF.getCurrentPosition());
            elb=Math.abs(encoder_x-encoder_y)>Math.abs(-LB.getCurrentPosition());
            erf=Math.abs(-encoder_x+encoder_y)>Math.abs(-RF.getCurrentPosition());
            erb=Math.abs(encoder_x+encoder_y)>Math.abs(-RB.getCurrentPosition());
            setAllDrivePowerG(fgt*(-vx-vy),fgt*(vx-vy),fgt*(-vx+vy),fgt*(vx+vy),Kp);
        }
        //brake
        setAllDrivePower(-LF.getPower()/Math.abs(LF.getPower()),-LB.getPower()/Math.abs(LB.getPower()),-RF.getPower()/Math.abs(RF.getPower()),-RB.getPower()/Math.abs(RB.getPower()));
        wait(75);
        setAllDrivePower(0);
        reset_ENCODER();
        setMode_RUN_WITHOUT_ENCODER();
    }

    public void moveInchesG(double xInch, double yInch, double speed){
        moveInchesG(xInch,yInch,speed,.8);
    }

    public void brake(){
        double speed = LF.getPower();
        setAllDrivePower(-speed,-speed,speed,speed);
        wait(40+(int)(400*Math.abs(speed)));
    }
}