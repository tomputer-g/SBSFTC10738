package org.firstinspires.ftc.teamcode20;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraManager;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

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
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.nio.ByteBuffer;
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

import org.firstinspires.ftc.teamcode20.Roadrunner.drive.mecanum.SampleMecanumDriveREV;
import org.openftc.revextensions2.ExpansionHubEx;

public class BaseAuto extends BaseOpMode {

    //Vuforia
    protected VuforiaLocalizer vuforia;
    protected static final float mmPerInch = 25.4f;
    protected OpenGLMatrix lastLocation = null;
    protected VuforiaTrackables targetsSkyStone;
    protected List<VuforiaTrackable> allTrackables= new ArrayList<>();
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    private VuforiaTrackable rear1,rear2;

    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;
    private ElapsedTime VuforiaPositionTime;
    private double[] displacements = {2, 7};//+ = forward; + = right
    private double headingDisplacement = -90;

    //Sensors
    protected ModernRoboticsI2cRangeSensor rangeSensorFront, rangeSensorSide;
    protected Rev2mDistanceSensor left,right;

    //Roadrunner
    public SampleMecanumDriveREV drive;

    //IMU
    protected static BNO055IMU imu;
    protected Orientation angles;
    protected double angle;
    protected static double imuHeading;
    protected static double imuAbsolute=0;
    protected static double imuOffset=0;
    protected static double acctarget=0;

    //Odometry
    protected double xmult = 1430.5/72, ymult = 18.65;
    protected final double xOdoEnable = 0.8, xOdoDisable = 1;

    //Threads
    protected CooThread cooThread;

    //Misc
    protected double[] n_pass ={0,0};
    private double xpre=0,y1pre=0,y2pre=0,theta=0;

    //-------------------------------------------------------------Multithreading------------------------------------------------------------------
/*
    class AutonomousInitThread extends Thread{
        @Override
        public void run() {
            Log.i("Auto init thread","started at "+System.currentTimeMillis());
            Log.i("Auto init thread", "finished at "+ System.currentTimeMillis());
        }
    }

 */

    //------------------------------------------------------------------Init-----------------------------------------------------------------------
    protected void initAutonomous() {
        showTelemetry = false;
        initDrivetrain();//181.64ms
        initIMU();//!!!!!1.259s : stops if thread has Interrupt flag
        initGrabber();//1.14ms
        initPlatformGrabber();//34.20ms
        initOdometry();//100.89ms
        xOdometryEnableServo.setPosition(xOdoEnable);
        setNewGyro0();
        initHubs();
        initVuforia();
        //initViewMarks();
        Log.i("Auto init", "done");
    }

    protected void initHubs(){
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub4 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 4");
    }

    //----------------------------------------------------------------Vuforia----------------------------------------------------------------------

    protected void initVuforia(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AZlAJSf/////AAABmf5BgAyil0t8tK506wQNabJX0SH5ekkamom8UybSLKgtsYTY/0/AB5n0Db9/JRrUDLEhDRXJgx5osNHZt6kVKSIF5cdge/dE9OgOunoX6LWBqk8cHGwBlKCXl1eGuvBPwQa3OaJDC7neKLmlZf2/NJiJKMvi9VBqKEDsS74Dp0tFbJka5cJa8YpKyrJh8593SN8p2qcYxXRORCWzmdMdD2xHUJXw28foxuNOotp2onbDmpnfH7x4oegFalegxvQbJ3J0cFqOuP8pboEjoN0Zl64xFVu6ZCc2uvsnXECEgWtycA+bWmQZNG6BD4SLYN/LWVYBp6U5MrIHsNeOOQfwTAZNVDcLELke77iK1XuWnCzG";
        parameters.cameraDirection = BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(6);

        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");
        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        allTrackables.addAll(targetsSkyStone);
        targetsSkyStone.activate();
        VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
        boolean PHONE_IS_PORTRAIT = false  ;
        float phoneXRotate    = 0;
        float phoneYRotate    = 0;
        float phoneZRotate    = 0;
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 8.0f * mmPerInch;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
    }

    protected double[] adjustToViewMark(boolean isBlue) throws InterruptedException {
        double[] xy=new double[2];
        double x,y;
        targetsSkyStone.activate();
        boolean targetVisible=false;
        VuforiaTrackable trackable;
        if(isBlue) trackable=rear1;
        else trackable=rear2;

        ElapsedTime ti=new ElapsedTime();
        while ((!targetVisible) && ti.milliseconds()<1000) {
            Thread.sleep(0);
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }
        }
        if (targetVisible) {
            VectorF translation = lastLocation.getTranslation();
            x=translation.get(0) / mmPerInch; y= translation.get(1) / mmPerInch;
            hub4.setLedColor(0,255,0);
        }
        else {
            if(isBlue){ x=38;y=39.2333333;}
            else{ x=-55.35;y=-24.7233333;}
            hub4.setLedColor(255,0,0);
        }
        targetsSkyStone.deactivate();
        xy[0]=x;
        xy[1]=y;
        return xy;
    }

    /*protected void initVuforiaWebcam(){
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
        front1.setLocation(OpenGLMatrix.translation(-72 * mmPerInch, -36 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
        front2.setLocation(OpenGLMatrix.translation(-72 * mmPerInch, 36 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
        blue1.setLocation(OpenGLMatrix.translation(-36 * mmPerInch, 72 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        blue2.setLocation(OpenGLMatrix.translation(36 * mmPerInch, 72 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        rear1.setLocation(OpenGLMatrix.translation(72 * mmPerInch, 36 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        rear2.setLocation(OpenGLMatrix.translation(72 * mmPerInch, -36 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        OpenGLMatrix robotFromCamera = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, 0, 0, 180));

        for (VuforiaTrackable trackable : allTrackables)
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
    }

     */
    protected void shutdownVuforia(){
        targetsSkyStone.deactivate();
    }

    protected int skystonePosition() {
        CameraManager cameraManager = (CameraManager) hardwareMap.appContext.getSystemService(Context.CAMERA_SERVICE);
        try {
            cameraManager.setTorchMode("0", true);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
        VuforiaPositionTime = new ElapsedTime();
        targetsSkyStone.activate();
        while (VuforiaPositionTime.milliseconds() < 2500) {
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    if (trackable.getName().equals("Stone Target")) {
                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        if (showTelemetry)
                            telemetry.addLine("Turn " + (int) Math.abs(rotation.thirdAngle + headingDisplacement) + (rotation.thirdAngle + headingDisplacement > 0 ? "deg. CW" : "deg. CCW"));
                        VectorF translation = lastLocation.getTranslation();
                        double dist = translation.get(1) / mmPerInch + displacements[1];
                        if (showTelemetry)
                            telemetry.addLine("Move " + Math.abs(dist) + (dist > 0 ? "in. Right" : "in. Left"));
                        if (dist > 5) {
                            if (showTelemetry)
                                telemetry.addData("Capture time", VuforiaPositionTime.milliseconds());
                            try {
                                cameraManager.setTorchMode("0", false);
                            } catch (CameraAccessException e) {
                                e.printStackTrace();
                            }
                            return 2;
                        } else {
                            if (showTelemetry)
                                telemetry.addData("Capture time", VuforiaPositionTime.milliseconds());
                            try {
                                cameraManager.setTorchMode("0", false);
                            } catch (CameraAccessException e) {
                                e.printStackTrace();
                            }
                            return 1;
                        }
                    }
                }
            }
        }
        if (showTelemetry) telemetry.addLine("Vuforia exceeded 1s wait.");
        try {
            cameraManager.setTorchMode("0", false);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
        return 0;
    }

    protected int new_skystoneposition() throws InterruptedException {
        VuforiaLocalizer.CloseableFrame frame = null;
        Image image = null;
        int result = 0;
        int red_L = 0, red_R = 0, blue_L = 0, blue_R = 0, green_L = 0, green_R = 0, red_M = 0, blue_M = 0, green_M = 0;
        int curpixel_L, curpixel_R, curpixel_M;

        frame = this.vuforia.getFrameQueue().take();


        for (int i = 0; i < frame.getNumImages(); ++i) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                image = frame.getImage(i);
                break;
            }
        }

        if (image != null) {
            ByteBuffer pixels = image.getPixels();
            byte[] pixelArray = new byte[pixels.remaining()];
            pixels.get(pixelArray, 0, pixelArray.length);
            //1280x720
            int imageWidth = image.getWidth();
            int imageHeight = image.getHeight();
            pixels.rewind();
            Bitmap bm = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(pixels);
            for (int i = -10; i < 10; ++i) {
                for (int j = -10; j < 10; ++j) {
                    curpixel_L = bm.getPixel(1147 + i, 180 + j);//"left"
                    curpixel_M = bm.getPixel(927 + i, 180 + j);//"mid"
                    curpixel_R = bm.getPixel(662 + i, 207 + j);//"right"
                    red_L += Color.red(curpixel_L);
                    blue_L += Color.blue(curpixel_L);
                    green_L += Color.green(curpixel_L);
                    red_M += Color.red(curpixel_M);
                    blue_M += Color.blue(curpixel_M);
                    green_M += Color.green(curpixel_M);
                    red_R += Color.red(curpixel_R);
                    blue_R += Color.blue(curpixel_R);
                    green_R += Color.green(curpixel_R);
                }
            }
            red_L /= 400;red_R /= 400;red_M /= 400;blue_M /= 400;blue_L /= 400;blue_R /= 400;green_M /= 400;green_L /= 400;green_R /= 400;

            telemetry.addData("rgb@L", "red: %d blue: %d green: %d", red_L, blue_L, green_L);
            telemetry.addData("rgb@M", "red: %d blue: %d green: %d", red_M, blue_M, green_M);
            telemetry.addData("rgb@R", "red: %d blue: %d green: %d", red_R, blue_R, green_R);
        }
        double L = red_L+blue_L+green_L, R = red_R+blue_R+green_R, M = red_M+blue_M+green_M;
        /*
        if (red_L > 100 && green_L > 90) l = false;
        if (red_R > 100 && green_R > 90) r = false;
        if (red_M > 100 && green_M > 90) m = false;

        if (l) result = 0;
        if (m) result = 1;
        if (r) result = 2;
        */
        if(L<M&&L<R)result = 0;
        else if(M<L&&M<R)result = 1;
        else if(R<M&&R<L)result = 2;

        telemetry.addData("position: ", result);
        telemetry.update();

        frame.close();
        return result;
    }

    protected int new_skystonepositionR() throws InterruptedException {
        VuforiaLocalizer.CloseableFrame frame = null;
        Image image = null;
        int result = 0;
        int red_L = 0, red_R = 0, blue_L=0,blue_R=0,green_L=0,green_R=0, red_M = 0,blue_M=0,green_M=0;
        int curpixel_L, curpixel_R, curpixel_M;
        boolean l = true, r=true, m = true;

        frame = this.vuforia.getFrameQueue().take();

        for(int i = 0;i<frame.getNumImages();++i){
            if(frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565){
                image = frame.getImage(i);
                break;
            }
        }

        if(image!=null){
            ByteBuffer pixels = image.getPixels();
            byte[] pixelArray = new byte[pixels.remaining()];
            pixels.get(pixelArray, 0, pixelArray.length);
            //1280x720
            int imageWidth = image.getWidth();
            int imageHeight = image.getHeight();
            pixels.rewind();
            Bitmap bm = Bitmap.createBitmap(imageWidth,imageHeight,Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(pixels);
            for(int i = -10;i<10;++i){
                for(int j = -10;j<10;++j){
                    curpixel_L = bm.getPixel(1177+i,180+j);//"left"
                    curpixel_M = bm.getPixel(938+i,180+j);//"mid"
                    curpixel_R = bm.getPixel(682+i,185+j);//"right"
                    red_L+=Color.red(curpixel_L);
                    blue_L+=Color.blue(curpixel_L);
                    green_L+=Color.green(curpixel_L);
                    red_M+=Color.red(curpixel_M);
                    blue_M+=Color.blue(curpixel_M);
                    green_M+=Color.green(curpixel_M);
                    red_R+=Color.red(curpixel_R);
                    blue_R+=Color.blue(curpixel_R);
                    green_R+=Color.green(curpixel_R);
                }
            }
            red_L/=400;red_R/=400;red_M/=400;blue_M/=400;blue_L/=400;blue_R/=400;green_M/=400;green_L/=400;green_R/=400;
            telemetry.addData("rgb@L", "red: %d blue: %d green: %d", red_L, blue_L, green_L);
            telemetry.addData("rgb@M", "red: %d blue: %d green: %d", red_M, blue_M, green_M);
            telemetry.addData("rgb@R", "red: %d blue: %d green: %d", red_R, blue_R, green_R);
        }

        if(red_L>100&&green_L>90) l = false;
        if(red_R>100&&green_R>90) r = false;
        if(red_M>100&&green_M>90) m = false;

        if(l)result=0;
        if(m)result=1;
        if(r)result=2;
        telemetry.addData("position: ", result);
        telemetry.update();
        frame.close();
        return result;
    }

    //------------------------------------------------------------------TFOD------------------------------------------------------------------------
    /*
    //TFOD
    protected TFObjectDetector tfod;

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


 */

    //-----------------------------------------------------------------Sensors----------------------------------------------------------------------
    protected void initSensors(){
        //rangeSensorSide = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "side");
       // rangeSensorFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front");
        //left = hardwareMap.get(Rev2mDistanceSensor.class,"left");
       // right = hardwareMap.get(Rev2mDistanceSensor.class,"right");
        //tower_top = hardwareMap.get(Rev2mDistanceSensor.class, "tower_top");
    }


    //-------------------------------------------------------------------IMU-----------------------------------------------------------------------

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
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES);
        angle = Double.parseDouble(String.format(Locale.getDefault(), "%.2f", AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle))));
        imuAbsolute = getError(angle,0);
        imuHeading = getError(angle,imuOffset);
        return imuHeading;
    }

    protected void setNewGyro0(){
        //looks dumb but crucial
        imuOffset = 0;
        imuOffset = getHeading();
    }

    protected void setNewGyro(double target){
        imuOffset = target;
    }

    protected double getError(double target, double cur) {
        double robotError =target-cur;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    protected double getError(double targetAngle) {
        return getError(targetAngle,getHeading());
    }

    protected boolean onHeading(double turnSpeed, double angle, double PCoeff, double threshold) {
        double   error = getError(angle), steer, speed;
        boolean  onTarget = false;
        if (Math.abs(error) <= threshold) {
            steer = 0.0;
            speed = 0.0;
            onTarget = true;
        }
        else {
            //Ie+=
            steer = Range.clip(error/180 * PCoeff, -1, 1);
            speed  = turnSpeed * steer;
            speed=(0<speed&&speed<.2)?.2:(0>speed&&speed>-.2)?-.2:speed;
        }
        setAllDrivePower(speed);
        if(showTelemetry)telemetry.update();
        return onTarget;
    }

    //-----------------------------------------------------------------Odometry-------------------------------------------------------------------

    protected void initOdometry(){
        //L2 is Y encoder
        //platform grabber is X encoder
        platform_grabber = hardwareMap.get(DcMotor.class, "platform");
        platform_grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        platform_grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        xOdometry = hardwareMap.get(DcMotor.class, "xOdo");
        xOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L2 = hardwareMap.get(DcMotor.class, "L2");
        L2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        xOdometryEnableServo = hardwareMap.get(Servo.class,"xOdoEnable");
        cooThread=new CooThread();
    }

    protected void updateCoo(){
        double heading=getError(getHeading()+imuOffset,0);
        double dtheta=heading-theta;
        tmpBulkData = hub4.getBulkInputData();
        double xcur=tmpBulkData.getMotorCurrentPosition(xOdometry)/odometryEncXPerInch,y1cur=-tmpBulkData.getMotorCurrentPosition(platform_grabber)/odometryEncYPerInch,y2cur=-tmpBulkData.getMotorCurrentPosition(L2)/odometryEncYPerInch,thetacur=heading/180*Math.PI;
        double dx=(near(dtheta,0,5))?xcur-xpre:0;
        double dy=(y1cur+y2cur-y1pre-y2pre)/2;//(near(dtheta,0,5))?y1cur-y1pre:0;
        n_pass[0] += (dx * Math.cos(thetacur) + dy * Math.sin(thetacur));
        n_pass[1] += (dx * Math.sin(thetacur) + dy * Math.cos(thetacur));
        xpre=xcur;
        y1pre=y1cur;
        y2pre=y2cur;
        theta=heading;
    }

    protected class CooThread extends Thread{
        volatile public boolean stop = false;
        @Override
        public void run() {
            //this.setPriority(4);
            this.setName("Coord Thread "+this.getId());
            Log.i("coordThread"+this.getId(),"Started running");
            while (!isInterrupted() && !stop) {
                try{
                    Thread.sleep(60);
                }
                catch(Exception e){}
                drive.updatePoseEstimate();
                //Pose2d pos=drive.getPoseEstimate();
                //telemetry.addLine("x,y "+pos.getX()+","+pos.getY());
                //telemetry.update();
            }
            Log.i("coordThread"+this.getId(), "thread finished");
        }
        public void stopThread(){
            stop = true;
        }
    }

    //-------------------------------------------------------------Movement methods---------------------------------------------------------------

    protected void lefty() throws InterruptedException {
        double target=90,kd=0.922,kp=0.028;
        acctarget=0;
        setNewGyro0();
        double e = target;
        ElapsedTime t = new ElapsedTime();
        int i=0;
        setAllDrivePower(1,1,-1,-1);
        Thread.sleep(40);
        while(i<5){
            double e2 = target-(getAdjustedHeading(target));
            double D = kd*(e2-e)/t.milliseconds();
            double P = e2*kp;
            if(Math.abs(P)>Math.abs(0.7))P=P>0?0.7:-0.7;
            double A=P+D;
            setAllDrivePower(A+0.3+0.1,A-0.3+0.1,A+0.3-0.1,A-0.3-0.1);
            e=e2;
            if(near(e2-e,0,0.2)&&near(e,0,4))
                i++;

            runSlide();
            handleRTState();
            t.reset();
        }
        setAllDrivePower(0);
        setNewGyro0();
    }
    protected void righty() throws InterruptedException {
        double target=-90;
        acctarget=0;
        setNewGyro0();
        double e = target;
        ElapsedTime t = new ElapsedTime();
        int i=0;
        setAllDrivePower(1,1,-1,-1);
        Thread.sleep(40);
        while(i<5){
            double e2 = target-(getAdjustedHeading(target));
            double D = 0.922*(e2-e)/t.milliseconds();
            double P = e2*0.028;
            if(Math.abs(P)>Math.abs(0.7))P=P>0?0.7:-0.7;
            double A=P+D;
            setAllDrivePower(A-0.3+0.1,A+0.3+0.1,A-0.3-0.1,A+0.3-0.1);
            e=e2;
            if(near(e2-e,0,0.2)&&near(e,0,4))
                i++;

            runSlide();
            handleRTState();
            t.reset();
        }
        setAllDrivePower(0);
        setNewGyro0();
    }
    protected void backy() throws InterruptedException {
        Log.i("backY","called");
        double target=180,kd=0.922,kp=0.028;
        double e = target;
        acctarget=0;
        setNewGyro0();
        ElapsedTime t = new ElapsedTime();
        int i=0;
        ElapsedTime n = new ElapsedTime();
        setAllDrivePower(1,1,-1,-1);
        Thread.sleep(150);
        Log.i("backY","starting main loop");
        while(i<5&&n.milliseconds()<5000){
            double e2 = target-(getAdjustedHeading(target));
            double D = kd*(e2-e)/t.milliseconds();
            double P = e2*kp;
            if(Math.abs(P)>Math.abs(1))P=P>0?1:-1;
            double A=P+D;
            setAllDrivePower(A);
            e=e2;
            if(near(e2-e,0,0.2)&&near(e,0,4)) {
                i++;
                Log.i("backY", "i is now "+i);
            }
            if(n.milliseconds() > 5000){
                Log.i("backY","5s passed");
            }

            runSlide();
            handleRTState();
            t.reset();
        }
        Log.i("backY","main loop ended");
        setAllDrivePower(0);
    }

    protected void turn(double angle, double speed, double threshold) {
        setMode_RUN_WITHOUT_ENCODER();
        setNewGyro(theta);
        double p_TURN = 6;
        //double Ie=0;

        double rangle = angle;
        if(angle>25)rangle-=2;
        else if(angle<-25)rangle+=1;

        while(!onHeading(speed, rangle, p_TURN, threshold));
        theta=getError(theta+angle,0);
        //setNewGyro(angle);
    }

    protected void PIDturn(double target, boolean resetOffset) throws InterruptedException {
        tunePIDturn(target,0.068,0.9,0.5,resetOffset);
    }

    protected void PIDturnfast(double target, boolean resetOffset) throws InterruptedException {
        tunePIDturn(target,0.029,2.291,1,false);
    }

    protected void align(double target) throws InterruptedException {
        PIDturnfast(-getError(imuAbsolute,target),false);
        setNewGyro(target);
    }

    protected void tunePIDturn(double target, double kp, double kd, double speed, boolean resetOffset) throws InterruptedException {
        if(resetOffset){
            acctarget=0;
            setNewGyro0();
        }
        double e = target;
        ElapsedTime t = new ElapsedTime();
        ElapsedTime n = new ElapsedTime();
        int i=0;
        while(i<5&&n.milliseconds()<((speed>0.5)?800:2000)){
            Thread.sleep(0);
            double e2 = target-(getAdjustedHeading(target));
            double D = kd*(e2-e)/t.milliseconds();
            t.reset();
            double P = e2*kp;
            if(Math.abs(P)>Math.abs(speed))P=P>0?speed:-speed;
            setAllDrivePower(P+D);
            e=e2;
            if(near(e2-e,0,0.3)&&near(e2,0,3))
                i++;
        }
        setAllDrivePower(0);
        acctarget+=target;
        if(resetOffset) {
            acctarget = 0;
            setNewGyro0();
        }
        else
            setNewGyro(acctarget);
    }

    private double getAdjustedHeading(double target){
        double i = getHeading();
        if(target>0)
            return i<-100?i+360:i;
        else
            return i>100?i-360:i;
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
            double highp = 0.03 / .18,cyclems=200;
            if(w!=0)setNewGyro0();
        try {
            Thread.sleep(0, (int) (cyclems * (1 - highp)));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        setAllDrivePowerG(-.2 * dir - .5 * x + .2 * w, -.2 * dir + .5 * x + .2 * w, .2 * dir - .5 * x + .2 * w, .2 * dir + .5 * x + .2 * w);
        try {
            Thread.sleep(0, (int) (cyclems * highp));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        setAllDrivePowerG(-.02 * dir - .05 * x + .02 * w, -.02 * dir + .05 * x + .02 * w, .02 * dir - 0.05 * x + .02 * w, .2 * dir + .05 * x + .2 * w);
    }

    protected void moveInchesG(double xInch, double yInch, double speed, double Kp) throws InterruptedException {
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
        tmpBulkData = hub2.getBulkInputData();
        boolean elf=Math.abs(-encoder_x-encoder_y)>Math.abs(-tmpBulkData.getMotorCurrentPosition(LF)),elb=Math.abs(encoder_x-encoder_y)>Math.abs(-tmpBulkData.getMotorCurrentPosition(LB)),erf=Math.abs(-encoder_x+encoder_y)>Math.abs(-tmpBulkData.getMotorCurrentPosition(RF)),erb=Math.abs(encoder_x+encoder_y)>Math.abs(-tmpBulkData.getMotorCurrentPosition(RB));
        while((elf|| elb|| erf|| erb)){
            tmpBulkData = hub2.getBulkInputData();
            elf=Math.abs(-encoder_x-encoder_y)>Math.abs(-tmpBulkData.getMotorCurrentPosition(LF));
            elb=Math.abs(encoder_x-encoder_y)>Math.abs(-tmpBulkData.getMotorCurrentPosition(LB));
            erf=Math.abs(-encoder_x+encoder_y)>Math.abs(-tmpBulkData.getMotorCurrentPosition(RF));
            erb=Math.abs(encoder_x+encoder_y)>Math.abs(-tmpBulkData.getMotorCurrentPosition(RB));
            setAllDrivePowerG(fgt*(-vx-vy),fgt*(vx-vy),fgt*(-vx+vy),fgt*(vx+vy),Kp);
        }
        //brake
        setAllDrivePower(-LF.getPower()/Math.abs(LF.getPower()),-LB.getPower()/Math.abs(LB.getPower()),-RF.getPower()/Math.abs(RF.getPower()),-RB.getPower()/Math.abs(RB.getPower()));
        Thread.sleep(75);
        setAllDrivePower(0);
        reset_ENCODER();
        setMode_RUN_WITHOUT_ENCODER();
    }

    protected void moveInchesG(double xInch, double yInch, double speed) throws InterruptedException {
        moveInchesG(xInch,yInch,speed,.8);
    }

    protected void moveInchesGOY(double yInch, double speed) throws InterruptedException {
        moveInchesGOY(yInch,speed,1);
    }

    protected void moveInchesGOY(double yInch, double speed,double kV) throws InterruptedException {//use 0.4 for short-dist
        yInch = -yInch;
        //setNewGyro0();
        double kP = 1, kD = 0.12;
        if(yInch == 0)return;
        if(Math.abs(speed) == 0.3){
            kP = 1;
            kD = 0.12;
        }else if(Math.abs(speed) == 0.4){
            kP = 0.27;
            kD = 0.03;
        }else if(Math.abs(speed) == 0.6){
            kP = 0.075;
            kD = 1.4E-2;
        }else if(Math.abs(speed) == 0.9){
            kP = 0.0325;
            kD = 7.3E-3;
        }
        ElapsedTime t = new ElapsedTime();
        int offsetY = getY1Odometry();
        speed=Math.abs(speed);
        double multiply_factor, prev_speed = 0;
        int odometryYGoal = offsetY + (int)(yInch * odometryEncYPerInch);
        double vy = speed;
        int previousPos = offsetY, currentOdometry, Dterm;
        double tpre = 0, tcur;
        int steadyCounter = 0;
        while(steadyCounter < 3 && !this.gamepad1.b){//b is there so we can break out of loop anytime
            Thread.sleep(0);
            currentOdometry = getY1Odometry();
            tcur=t.milliseconds();
            Dterm = (int)((currentOdometry - previousPos)/(tcur-tpre));
            multiply_factor = -Math.min(1, Math.max(-1, kV*((kP * (currentOdometry - odometryYGoal)/ odometryEncYPerInch) +  (near(Dterm,0,speed * 5000 / 0.3)?(kD * Dterm):0))));
            if(near(prev_speed, multiply_factor*vy,0.001) && near(prev_speed, 0, 0.1)){
                steadyCounter++;
            }else{
                steadyCounter = 0;
            }
            Log.d("GOY "+yInch,"steady"+steadyCounter+", position"+currentOdometry+", LF speed"+prev_speed+", OC speed="+Dterm+"bulkSpd="+hub4.getBulkInputData().getMotorVelocity(platform_grabber));
            previousPos = currentOdometry;
            tpre=tcur;
            setAllDrivePowerG(multiply_factor*vy,multiply_factor*vy,multiply_factor*-vy,multiply_factor*-vy);
            prev_speed = multiply_factor * vy;
        }
        setAllDrivePower(0);
    }

    protected void moveInchesGOY_XF(double yInch, double speed,double kV) throws InterruptedException {//use 0.4 for short-dist
        yInch = -yInch;
        //setNewGyro0();
        double kP = 1, kD = 0.12;
        if(yInch == 0)return;
        if(Math.abs(speed) == 0.3){
            kP = 1;
            kD = 0.12;
        }else if(Math.abs(speed) == 0.4){
            kP = 0.27;
            kD = 0.03;
        }else if(Math.abs(speed) == 0.6){
            kP = 0.075;
            kD = 1.4E-2;
        }else if(Math.abs(speed) == 0.9){
            kP = 0.0325;
            kD = 7.3E-3;
        }
        ElapsedTime t = new ElapsedTime();
        int offsetY = getY1Odometry();
        int offsetX = getXOdometry();
        double diff = 0;
        speed=Math.abs(speed);
        double multiply_factor, prev_speed = 0;
        int odometryYGoal = offsetY + (int)(yInch * odometryEncYPerInch);
        double vy = speed;
        int previousPos = offsetY, currentOdometry, Dterm;
        double tpre = 0, tcur;
        int steadyCounter = 0;
        while(steadyCounter < 5 && !this.gamepad1.b){//b is there so we can break out of loop anytime
            Thread.sleep(0);
            diff = (getXOdometry() - offsetX)/odometryEncXPerInch/10;
            currentOdometry = getY1Odometry();
            tcur=t.milliseconds();
            Dterm = (int)((currentOdometry - previousPos)/(tcur-tpre));
            multiply_factor = -Math.min(1, Math.max(-1, kV*((kP * (currentOdometry - odometryYGoal)/ odometryEncYPerInch) +  (near(Dterm,0,speed * 5000 / 0.3)?(kD * Dterm):0))));
            if(near(prev_speed, multiply_factor*vy,0.001) && near(prev_speed, 0, 0.1)){
                steadyCounter++;
            }else{
                steadyCounter = 0;
            }
            Log.d("GOY "+yInch,"steady"+steadyCounter+", position"+currentOdometry+", LF speed"+prev_speed+", OC speed="+Dterm+"bulkSpd="+hub4.getBulkInputData().getMotorVelocity(platform_grabber));
            previousPos = currentOdometry;
            tpre=tcur;
            setAllDrivePowerG(multiply_factor*vy + diff,multiply_factor*vy - diff,multiply_factor*-vy + diff,multiply_factor*-vy - diff,1.4);
            prev_speed = multiply_factor * vy;
        }
        setAllDrivePower(0);
    }

    protected void moveInchesGOY_XF_F(double yInch, double speed,double kV, int FixXOffset) throws InterruptedException {//use 0.4 for short-dist
        yInch = -yInch;
        //setNewGyro0();
        double kP = 1, kD = 0.12;
        if(yInch == 0)return;
        if(Math.abs(speed) == 0.3){
            kP = 1;
            kD = 0.12;
        }else if(Math.abs(speed) == 0.4){
            kP = 0.27;
            kD = 0.03;
        }else if(Math.abs(speed) == 0.6){
            kP = 0.075;
            kD = 1.4E-2;
        }else if(Math.abs(speed) == 0.9){
            kP = 0.0325;
            kD = 7.3E-3;
        }
        //-0.0025
        double kPx = 0.25, kDx = 0.005;
        ElapsedTime t = new ElapsedTime();
        int offsetY = getY1Odometry();
        int offsetX = FixXOffset;
        double diff = 0;
        speed=Math.abs(speed);
        double multiply_factor, prev_speed = 0;
        int odometryYGoal = offsetY + (int)(yInch * odometryEncYPerInch);
        double vy = speed;
        int previousPos = offsetY, previousPosX = offsetX, currentOdometry, currentOdometryX, Dterm, DtermX;
        double tpre = 0, tcur;
        int steadyCounter = 0;
        while(steadyCounter < 3 && !this.gamepad1.b){//b is there so we can break out of loop anytime
            //telemetry.addData("x",getXOdometry());
            //telemetry.addData("yL",getY1Odometry());
            //telemetry.addData("yR",getY2Odometry());
            //telemetry.update();
            Thread.sleep(0);
            currentOdometry = getY1Odometry();
            currentOdometryX = getXOdometry();
            tcur=t.milliseconds();
            Dterm = (int)((currentOdometry - previousPos)/(tcur-tpre));
            DtermX = (int)((currentOdometryX - previousPosX)/(tcur-tpre));
            diff = (currentOdometryX - offsetX)/odometryEncXPerInch*kPx + (near(DtermX,0,speed * 5000 / 0.3)?(kDx *DtermX):0);
            multiply_factor = -Math.min(1, Math.max(-1, kV*((kP * (currentOdometry - odometryYGoal)/ odometryEncYPerInch) +  (near(Dterm,0,speed * 5000 / 0.3)?(kD * Dterm):0))));
            if(near(prev_speed, multiply_factor*vy,0.001) && near(prev_speed, 0, 0.1)){
                steadyCounter++;
            }else{
                steadyCounter = 0;
            }
            Log.d("GOY "+yInch,"steady"+steadyCounter+", position"+currentOdometry+", LF speed"+prev_speed+", OC speed="+Dterm+"bulkSpd="+hub4.getBulkInputData().getMotorVelocity(platform_grabber));
            previousPos = currentOdometry;
            previousPosX = currentOdometryX;
            tpre=tcur;
            setAllDrivePowerG(multiply_factor*vy + diff,multiply_factor*vy - diff,multiply_factor*-vy + diff,multiply_factor*-vy - diff,1.4);
            prev_speed = multiply_factor * vy;
        }
        setAllDrivePower(0);
    }

    protected void moveInchesGOX(double xInch, double speed) throws InterruptedException {
        moveInchesGOX(xInch,speed,1);
    }

    protected void moveInchesGOX(double xInch, double speed,double kV) throws InterruptedException {//0.5 only
        if(xInch == 0)return;
        ElapsedTime t = new ElapsedTime();
        int offsetX = getXOdometry();
        speed=Math.abs(speed);
        double multiply_factor, prev_speed = 0;
        int odometryXGoal = offsetX + (int)(xInch * odometryEncXPerInch);
        double vx = speed;//(xInch/Math.abs(xInch)*speed);//
        int previousPos = offsetX, currentOdometry, Dterm;
        double tpre = 0, tcur;
        int steadyCounter = 0;
        while(steadyCounter < 5 && !this.gamepad1.b){
            Thread.sleep(0);
            currentOdometry = getXOdometry();
            tcur=t.milliseconds();
            Dterm = (int)((currentOdometry - previousPos)/(tcur-tpre));
            multiply_factor = -Math.min(1, Math.max(-1, kV*((0.5 * (currentOdometry - odometryXGoal)/odometryEncXPerInch) +  (near(Dterm,0,speed * 5000 / 0.3)?(0.05 * Dterm):0))));
            if(near(prev_speed, multiply_factor*vx,0.001) && near(prev_speed, 0, 0.1)){
                steadyCounter++;
            }else{
                steadyCounter = 0;
            }
            Log.d("GOX "+xInch,"steady"+steadyCounter+", position"+currentOdometry+", speed"+prev_speed);
            previousPos = currentOdometry;
            tpre=tcur;
            setAllDrivePowerG(multiply_factor*-vx,multiply_factor*vx,multiply_factor*-vx,multiply_factor*vx);
            prev_speed = multiply_factor * vx;
        }
        setAllDrivePower(0);
    }

    protected void moveInchesGOXT(double xInch, double speed,double kV, int timer) throws InterruptedException{//0.5 only
        if(xInch == 0)return;
        ElapsedTime t = new ElapsedTime();
        int offsetX = getXOdometry();
        speed=Math.abs(speed);
        double multiply_factor, prev_speed = 0;
        int odometryXGoal = offsetX + (int)(xInch * odometryEncXPerInch);
        double vx = speed;//(xInch/Math.abs(xInch)*speed);//
        int previousPos = offsetX, currentOdometry, Dterm;
        double tpre = 0, tcur;
        int steadyCounter = 0;
        t.reset();
        while(steadyCounter < 5 && !this.gamepad1.b){
            Thread.sleep(0);
            //telemetry.addData("d",t.milliseconds());
            //telemetry.update();
            currentOdometry = getXOdometry();
            tcur=t.milliseconds();
            Dterm = (int)((currentOdometry - previousPos)/(tcur-tpre));
            multiply_factor = -Math.min(1, Math.max(-1, kV*((0.5 * (currentOdometry - odometryXGoal)/odometryEncXPerInch) +  (near(Dterm,0,speed * 5000 / 0.3)?(0.05 * Dterm):0))));
            if(near(prev_speed, multiply_factor*vx,0.001) && near(prev_speed, 0, 0.1)){
                steadyCounter++;
            }
            else if(tcur>timer){
                Log.d("GOX "+xInch,"timeout");
                break;
            }
            else{
                steadyCounter = 0;
            }
            Log.d("GOX "+xInch,"steady"+steadyCounter+", position"+currentOdometry+", speed"+prev_speed);
            previousPos = currentOdometry;
            tpre=tcur;
            setAllDrivePowerG(multiply_factor*-vx,multiply_factor*vx,multiply_factor*-vx,multiply_factor*vx);
            prev_speed = multiply_factor * vx;
        }
        setAllDrivePower(0);
    }

    protected void moveInchesGOX_platform(double xInch, double speed) throws InterruptedException {
        moveInchesGOX_platform(xInch,speed,1);
    }

    protected void moveInchesGOX_platform(double xInch, double speed,double kV) throws InterruptedException {//0.8 only, for doing the platform
        double kP = 1, kD = 0.07;
        if(Math.abs(speed) == 0.8){
            kP = 1;
            kD = 0.07;
        }
        if(xInch == 0)return;
        ElapsedTime t = new ElapsedTime();
        int offsetX = getXOdometry();
        speed=Math.abs(speed);
        double multiply_factor, prev_speed = 0;
        int odometryXGoal = offsetX + (int)(xInch * odometryEncXPerInch);
        double vx = speed;//(xInch/Math.abs(xInch)*speed);//
        int previousPos = offsetX, currentOdometry, Dterm;
        double tpre = 0, tcur;
        int steadyCounter = 0;
        while(steadyCounter < 5 && !this.gamepad1.b){
            Thread.sleep(0);
            currentOdometry = getXOdometry();
            tcur=t.milliseconds();
            Dterm = (int)((currentOdometry - previousPos)/(tcur-tpre));
            multiply_factor = -Math.min(1, Math.max(-1, kV*((kP * (currentOdometry - odometryXGoal)/odometryEncXPerInch) +  (near(Dterm,0,speed * 5000 / 0.3)?(kD * Dterm):0))));
            if(near(prev_speed, multiply_factor*vx,0.001) && near(currentOdometry, odometryXGoal, odometryEncXPerInch)){
                steadyCounter++;
            }else{
                steadyCounter = 0;
            }
            Log.d("GOX "+xInch,"steady"+steadyCounter+", position"+currentOdometry+", speed"+prev_speed);
            previousPos = currentOdometry;
            tpre=tcur;
            setAllDrivePowerG(multiply_factor*-vx,multiply_factor*vx,multiply_factor*-vx,multiply_factor*vx);
            prev_speed = multiply_factor * vx;
        }
        setAllDrivePower(0);
    }

    protected void brake() throws InterruptedException {
        double speed = LF.getPower();
        setAllDrivePower(-speed,-speed,speed,speed);
        Thread.sleep(40+(int)(400*Math.abs(speed)));
    }

    //---------------------------------------------------------Autonomous methods-------------------------------------------------

    protected int Ultra_get_position() throws InterruptedException {
        int poss = 0;
        int[] resultcounter = {0,0,0};
        //find skystone
        for (int i = 0;i<4;++i) resultcounter[new_skystoneposition()]++;
        int curmax = -1;
        for (int i = 0;i<3;++i){ if(resultcounter[i]>curmax){poss = i;curmax=resultcounter[i];} }
        return poss;
    }

    protected int Ultra_get_positionR() throws InterruptedException {
        int poss = 0;
        int[] resultcounter = {0,0,0};
        //find skystone
        for (int i = 0;i<4;++i) resultcounter[new_skystonepositionR()]++;
        int curmax = -1;
        for (int i = 0;i<3;++i){ if(resultcounter[i]>curmax){poss = i;curmax=resultcounter[i];} }
        return poss;
    }

    protected void first_block() throws InterruptedException {
        //move forward to the skystone
        while(-getY1Odometry() < 20*odometryEncYPerInch){Thread.sleep(0);setAllDrivePowerG(-.3,-.3,.3,.3);}
        while(-getY1Odometry() < 30*odometryEncYPerInch){Thread.sleep(0);setAllDrivePowerG(-.2,-.2,.2,.2);}
        setAllDrivePowerG(-.1,-.1,.1,.1);
        grabber.setPosition(grabber_closed);
        Thread.sleep(100);
        servoThread.setExtTarget(grabberServoGrab);
        while(-getY1Odometry()> 29.5*odometryEncYPerInch){Thread.sleep(0);setAllDrivePowerG(.3,.3,-.3,-.3);}
        setAllDrivePower(0);
    }

    protected void before_start(){
        platform_grabber.setPower(1);
        servoThread.setExtTarget(0.88);
        if(showTelemetry)telemetry.clear();
        grabber.setPosition(grabber_open);
        platform_grabber.setPower(0.0);
    }
}
