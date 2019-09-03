package org.firstinspires.ftc.teamcode.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BaseAuto;
@Disabled
@Autonomous
public class AAutoTest extends BaseAuto {
    private double landerAwayTurningAmount = 58;
    private double threshold = 3;
    private int TFOD_result;
    private boolean initLoopFlag = false;
    private double landerAndCraterOrientation = 44;
    private double landerAndWallOrientation = 86;
    double[] currOrientation;
    double currHeading;

    double turningAmount;

    @Override
    public void init() {
        msStuckDetectInit = 10000;
        msStuckDetectLoop = 30000;
        msStuckDetectInitLoop = 10000;
        super.init();
        initIMU();
        initVuforiaEngine();
        initVumark();
        super.useTFOD = false;
        super.useVuMark = true;

//        tfod.activate();
    }

    @Override
    public void init_loop(){
        if(!initLoopFlag){
            ElapsedTime t = new ElapsedTime();
            wait(1000);
//            TFOD_result = TFOD_double();
//            if(TFOD_result != -1){
//                initLoopFlag = true;
//                tfod.deactivate();
//                stopTFOD();
            TFOD_result = 0;
                telemetry.addData("TFOD result", TFOD_result);
                telemetry.addData("Time in init loop", t.milliseconds());
                telemetry.update();
            }
        }

    @Override
    public void loop() {
        ElapsedTime t = new ElapsedTime();
//        lowerRobot();
        turn(-landerAwayTurningAmount,0.2,threshold);
        moveInchesHighSpeedEncoder(-8, -8, 0.3,3,3,0.1,0.2,0.1);
        //        if (waitUntilVumarkRecognized() == -1)
//        {
            turningAmount = 52;
//        }
//        else
//        {
//        waitUntilVumarkRecognized();
//        refreshVumarkReading();
//        currOrientation = getOrientationFromVumark();
//        currHeading = currOrientation[2];
//        turningAmount = headingOffset(currHeading, landerAndCraterOrientation);
//
//        telemetry.addData("currHeading", currHeading);
//        telemetry.addData("landerAndWallOrientation", landerAndWallOrientation);
//        telemetry.addData("turningAmount", turningAmount);
//        telemetry.update();

        turn(turningAmount, 0.15, 1);



        moveInchesHighSpeedEncoder(-3.5, 0, 0.1,1,1,0.15,0.2,0.1);

        //0(Completed)
        moveInchesHighSpeedEncoder(0,-15.7,0.3,3,6,0.1,0.55,0.15);
        moveInchesHighSpeedEncoder(-11,0,0.2,2,4,0.1,0.3,0.1);
        moveInchesHighSpeedEncoder(11,0,0.2,2,4,0.1,0.3,0.1);
        moveInchesHighSpeedEncoder(0,-32,0.8,8,16,0.1,0.65,0.15);
        // 2(Completed)
//        moveInchesHighSpeedEncoder(0,15,0.3,3,6,0.1,0.55,0.15);
//        moveInchesHighSpeedEncoder(-11,0,0.2,2,4,0.1,0.3,0.1);
//        moveInchesHighSpeedEncoder(8,0,0.2,2,4,0.1,0.3,0.1);
//        moveInchesHighSpeedEncoder(0,-62.5,0.8,8,16,0.1,0.55,0.1);

        //1
//        moveInchesHighSpeedEncoder(-10,0,0.3,3,6,0.1,0.55,0.15);
//        moveInchesHighSpeedEncoder(10,0,0.3,2,4,0.1,0.55,0.15);
//        moveInchesHighSpeedEncoder(0,-47,0.8,8,16,0.1,0.55,0.1);
//        wait(1000);
        turn(10, 0.3, 2);
        wait(250);
        refreshVumarkReading();
        currOrientation = getOrientationFromVumark();
        currHeading = currOrientation[2];
//        Log.i("VMOri", "Heading: "+currHeading);
        turningAmount = headingOffset(currHeading, landerAndWallOrientation);
//        telemetry.addData("currHeading", currHeading);
//        telemetry.addData("landerAndWallOrientation", landerAndWallOrientation);
//        telemetry.addData("turningAmount", turningAmount);
//        telemetry.update();
//        wait(3000);
        turn(turningAmount, 0.15, 1);
//        // 2(Completed)
//        moveInchesHighSpeedEncoder(-4, 0, 0.15, 1,1,0.15,0.35,0.15);
//        moveInchesHighSpeedEncoder(0, -18.5, 0.4, 4,8,0.1,0.55,0.15);
//        moveInchesHighSpeedEncoder(13,0,0.2,2,4,0.1,0.3,0.1);
//        moveInchesHighSpeedEncoder(-13,0,0.2,2,4,0.1,0.3,0.1);
//        moveInchesHighSpeedEncoder(0, -17, 0.4, 4,8,0.1,0.55,0.15);
//
//        //0(Completed)
        moveInchesHighSpeedEncoder(-2.95, 0, 0.15, 1,1,0.15,0.35,0.15);
        moveInchesHighSpeedEncoder(0, -35.5, 0.4, 4,8,0.1,0.55,0.1);

        //1(Completed)
//        moveInchesHighSpeedEncoder(-4, 0, 0.15, 1,1,0.15,0.35,0.15);
//        moveInchesHighSpeedEncoder(0, -35.5, 0.4, 4,8,0.1,0.55,0.15);

//        wait(2000);
        dropMarker();
//        //move arm joint -6500 counts
//        grabber_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        grabber_shoulder.setTargetPosition(-6250);
//        grabber_shoulder.setPower(1);
        moveInchesHighSpeedEncoder(0,53, 1, 10,20,0.2,0.7,0.1);
        moveInchesHighSpeedEncoder(-3,0, 0.15, 1,1,0.1,0.1,0.1);
        requestOpModeStop();
    }

    @Override
    public void stop() {
        super.stop();
        grabber_shoulder.setPower(0);
    }
}
