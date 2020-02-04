package org.firstinspires.ftc.teamcode20;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous
public class AutonInitTest extends BaseAuto {
    @Override
    public void init() {
        initAutonomous();
        /*

        normal, linear, before: 6.188s
        2020-02-04 12:51:37.642 9232-9602/com.qualcomm.ftcrobotcontroller I/Auto init: 43229 start hub init
        2020-02-04 12:51:37.642 9232-9602/com.qualcomm.ftcrobotcontroller I/Auto init: 523021 start drivetrain init
        2020-02-04 12:51:37.824 9232-9602/com.qualcomm.ftcrobotcontroller I/Auto init: 182167883 start IMU
        2020-02-04 12:51:39.083 9232-9602/com.qualcomm.ftcrobotcontroller I/Auto init: 1440798321 start grabber
        2020-02-04 12:51:39.084 9232-9602/com.qualcomm.ftcrobotcontroller I/Auto init: 1441942175 start platform
        2020-02-04 12:51:39.121 9232-9602/com.qualcomm.ftcrobotcontroller I/Auto init: 1479144939 start sensors
        2020-02-04 12:51:39.121 9232-9602/com.qualcomm.ftcrobotcontroller I/Auto init: 1479623429 start odometry
        2020-02-04 12:51:39.222 9232-9602/com.qualcomm.ftcrobotcontroller I/Auto init: 1580515366 start vuforia
        2020-02-04 12:51:43.830 9232-9602/com.qualcomm.ftcrobotcontroller I/Auto init: 6188323012 done init (6 seconds)


        separate thread: 4.016s
        2020-02-04 13:12:41.827 14061-14451/com.qualcomm.ftcrobotcontroller I/Auto init: 1580839961827 start hub init
        2020-02-04 13:12:41.828 14061-14451/com.qualcomm.ftcrobotcontroller I/Auto init: 1580839961828 start drivetrain init
        2020-02-04 13:12:41.829 14061-14460/com.qualcomm.ftcrobotcontroller I/Auto init thread: started at 1580839961829
        2020-02-04 13:12:42.005 14061-14451/com.qualcomm.ftcrobotcontroller I/Auto init: 1580839962005 start IMU
        2020-02-04 13:12:43.237 14061-14451/com.qualcomm.ftcrobotcontroller I/Auto init: 1580839963237 start grabber
        2020-02-04 13:12:43.238 14061-14451/com.qualcomm.ftcrobotcontroller I/Auto init: 1580839963238 start platform
        2020-02-04 13:12:43.270 14061-14451/com.qualcomm.ftcrobotcontroller I/Auto init: 1580839963270 start sensors
        2020-02-04 13:12:43.271 14061-14451/com.qualcomm.ftcrobotcontroller I/Auto init: 1580839963271 start odometry
        2020-02-04 13:12:43.372 14061-14451/com.qualcomm.ftcrobotcontroller I/Auto init: 1580839963372 done init
        2020-02-04 13:12:45.843 14061-14460/com.qualcomm.ftcrobotcontroller I/Auto init thread: finished at 1580839965843
        2020-02-04 13:12:45.843 14061-14451/com.qualcomm.ftcrobotcontroller I/Auto init: initThread done



        without targetsSkystone - around 1.5s
        2020-02-04 13:16:08.610 15518-15652/com.qualcomm.ftcrobotcontroller I/Auto init: 1580840168610 start hub init
        2020-02-04 13:16:08.610 15518-15652/com.qualcomm.ftcrobotcontroller I/Auto init: 1580840168610 start drivetrain init
        2020-02-04 13:16:08.612 15518-15748/com.qualcomm.ftcrobotcontroller I/Auto init thread: started at 1580840168612
        2020-02-04 13:16:08.783 15518-15652/com.qualcomm.ftcrobotcontroller I/Auto init: 1580840168783 start IMU
        2020-02-04 13:16:09.829 15518-15652/com.qualcomm.ftcrobotcontroller I/Auto init: 1580840169829 start grabber
        2020-02-04 13:16:09.830 15518-15652/com.qualcomm.ftcrobotcontroller I/Auto init: 1580840169830 start platform
        2020-02-04 13:16:09.862 15518-15652/com.qualcomm.ftcrobotcontroller I/Auto init: 1580840169862 start sensors
        2020-02-04 13:16:09.862 15518-15652/com.qualcomm.ftcrobotcontroller I/Auto init: 1580840169862 start odometry
        2020-02-04 13:16:09.961 15518-15652/com.qualcomm.ftcrobotcontroller I/Auto init: 1580840169961 done init

        2020-02-04 13:20:22.822 17936-18125/com.qualcomm.ftcrobotcontroller I/Auto init: 1580840422821 start hub init
        2020-02-04 13:20:22.822 17936-18125/com.qualcomm.ftcrobotcontroller I/Auto init: 1580840422822 start drivetrain init
        2020-02-04 13:20:22.822 17936-18129/com.qualcomm.ftcrobotcontroller I/Auto init thread: started at 1580840422822
        2020-02-04 13:20:23.003 17936-18125/com.qualcomm.ftcrobotcontroller I/Auto init: 1580840423003 start IMU
        2020-02-04 13:20:24.205 17936-18125/com.qualcomm.ftcrobotcontroller I/Auto init: 1580840424205 start grabber
        2020-02-04 13:20:24.206 17936-18125/com.qualcomm.ftcrobotcontroller I/Auto init: 1580840424206 start platform
        2020-02-04 13:20:24.239 17936-18125/com.qualcomm.ftcrobotcontroller I/Auto init: 1580840424239 start sensors
        2020-02-04 13:20:24.240 17936-18125/com.qualcomm.ftcrobotcontroller I/Auto init: 1580840424240 start odometry
        2020-02-04 13:20:24.323 17936-18125/com.qualcomm.ftcrobotcontroller I/Auto init: 1580840424323 done init
         */




        //Without useless stuff: 3.89s (JUST vuforia)
        //2020-02-04 13:05:39.513 11748-12127/com.qualcomm.ftcrobotcontroller I/Auto init: 1557207759 start vuforia
        //2020-02-04 13:05:43.400 11748-12127/com.qualcomm.ftcrobotcontroller I/Auto init: 5444309396 done init
    }

    class AutonomousInitThread extends Thread{
        @Override
        public void run() {
            Log.i("Auto init thread","started at "+System.currentTimeMillis());
            initVuforia();
            Log.i("Auto init thread", "finished at "+ System.currentTimeMillis());
        }
    }



    protected void initVuforia(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AZlAJSf/////AAABmf5BgAyil0t8tK506wQNabJX0SH5ekkamom8UybSLKgtsYTY/0/AB5n0Db9/JRrUDLEhDRXJgx5osNHZt6kVKSIF5cdge/dE9OgOunoX6LWBqk8cHGwBlKCXl1eGuvBPwQa3OaJDC7neKLmlZf2/NJiJKMvi9VBqKEDsS74Dp0tFbJka5cJa8YpKyrJh8593SN8p2qcYxXRORCWzmdMdD2xHUJXw28foxuNOotp2onbDmpnfH7x4oegFalegxvQbJ3J0cFqOuP8pboEjoN0Zl64xFVu6ZCc2uvsnXECEgWtycA+bWmQZNG6BD4SLYN/LWVYBp6U5MrIHsNeOOQfwTAZNVDcLELke77iK1XuWnCzG";
        parameters.cameraDirection   = BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        //targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565,true);
        vuforia.setFrameQueueCapacity(6);

        /*
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
*/
    }
    @Override
    public void stop() {
        servoThread.stopThread();
        super.stop();
    }
}
