package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode19.BaseAuto;

@Autonomous(group = "Test")
@Disabled
public class AutoRCVumarkDistance extends BaseAuto {
    private double landerAwayTurningAmount = 56;
    private double threshold = 3;
    private double landerAndCraterOrientation = -140;
    private double landerAndWallOrientation = -94;
    double[] currOrientation;
    double currHeading;
    double turningAmount;
    private boolean initLoopFlag = false;
    boolean tfodUsed = false;
    boolean firstVumarkUsed = false;
    boolean secondVumarkUsed= false;

    @Override
    public void init() {
        msStuckDetectInit = 10000;
        msStuckDetectLoop = 30000;
        msStuckDetectInitLoop = 10000;
        super.init();
        initVumark();
    }

    @Override
    public void loop() {
        stopTFOD();
        lowerRobot(); //Lower+MoveArm

        turn(-landerAwayTurningAmount,0.2,threshold);
        moveInchesHighSpeedEncoder(-8, -8, 0.3,3,3,0.1,0.1,0.1);

        if (waitUntilVumarkRecognized() == -1)
            turningAmount = 51;
        else
        {
            refreshVumarkReading();
            currOrientation = getOrientationFromVumark();
            currHeading = currOrientation[2];
            turningAmount = headingOffset(currHeading, landerAndCraterOrientation);
            if (Math.abs(turningAmount-51) >=15)
                turningAmount = 51;
            else
                firstVumarkUsed = true;
        }
        turn(turningAmount, 0.15, 1);

        if(firstVumarkUsed)
            light.setPosition(1);
        moveInchesHighSpeedEncoder(-3.5, 0, 0.1,1,1,0.15,0.2,0.1);
        light.setPosition(0.5);

//        switch(TFOD_result){
//            case 0:
//                moveInchesHighSpeedEncoder(0,-15.7,0.3,3,6,0.1,0.55,0.15);
//                moveInchesHighSpeedEncoder(-9.5,0,0.2,2,4,0.1,0.3,0.1);
//                moveInchesHighSpeedEncoder(10,0,0.2,2,4,0.1,0.3,0.1);
//                moveInchesHighSpeedEncoder(0,-28,0.8,8,16,0.1,0.65,0.15);
//                break;
//            case 1:
//                moveInchesHighSpeedEncoder(-10,0,0.3,3,6,0.1,0.55,0.15);
//                moveInchesHighSpeedEncoder(10,0,0.3,2,4,0.1,0.55,0.15);
//                moveInchesHighSpeedEncoder(0,-49,0.8,8,16,0.1,0.55,0.1);
//                break;
//            case 2:
//                moveInchesHighSpeedEncoder(0,15,0.3,3,6,0.1,0.55,0.15);
//                moveInchesHighSpeedEncoder(-10,0,0.2,2,4,0.1,0.3,0.1);
//                moveInchesHighSpeedEncoder(9,0,0.2,2,4,0.1,0.3,0.1);
//                moveInchesHighSpeedEncoder(0,-62.5,0.8,8,16,0.1,0.55,0.1);
//                break;
//            default:
                moveInchesHighSpeedEncoder(0,15,0.3,3,6,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(-11,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(9.5,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(0,-51,0.8,8,16,0.1,0.55,0.1);
//                break;
//        }


        turn(55, 0.3, 2);
        refreshVumarkReading();
        currOrientation = getOrientationFromVumark();
        currHeading = currOrientation[2];
        turningAmount = headingOffset(currHeading, landerAndWallOrientation);
        wait(150);
        if (vumarkIsVisible()) {
            turn(turningAmount, 0.15, 1);
            telemetry.addData("turningAmount", turningAmount);
        }
        else
            telemetry.addData("Cannot see Vumark", 0);

        telemetry.update();
        wait(5000);

//
//        switch(TFOD_result){
//            case -1:
//                throw new IllegalArgumentException("TFOD returned -1");
//            case 0:
//                if(firstVumarkUsed)
//                    light.setPosition(1);
//                moveInchesHighSpeedEncoder(-5.5, 0, 0.15, 1,1,0.15,0.35,0.15);
//                light.setPosition(0.5);
//                moveInchesHighSpeedEncoder(0, -35.5, 0.4, 4,8,0.1,0.55,0.1);
//                break;
//            case 1:
//                if(firstVumarkUsed)
//                    light.setPosition(1);
//                moveInchesHighSpeedEncoder(-5.5, 0, 0.15, 1,1,0.15,0.35,0.15);
//                light.setPosition(0.5);
//                moveInchesHighSpeedEncoder(0, -35.5, 0.4, 4,8,0.1,0.55,0.1);
//                break;
//            case 2:
//                if(firstVumarkUsed)
//                    light.setPosition(1);
//                moveInchesHighSpeedEncoder(-5.5, 0, 0.15, 1,1,0.15,0.35,0.15);
//                light.setPosition(0.5);
//                moveInchesHighSpeedEncoder(0, -35.5, 0.4, 4,8,0.1,0.55,0.1);
//                break;
//            default:
//                if(firstVumarkUsed)
//                    light.setPosition(1);
//                moveInchesHighSpeedEncoder(-4, 0, 0.15, 1,1,0.15,0.35,0.15);
//                light.setPosition(0.5);
//                moveInchesHighSpeedEncoder(0, -35.5, 0.4, 4,8,0.1,0.55,0.1);
//                break;
//        }
//
//        dropMarker();
//
//        grabber_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        grabber_shoulder.setTargetPosition(-5450);
//        grabber_shoulder.setPower(1);
//
//
//        moveInchesHighSpeedEncoder(0,56.5, 1, 15,30,0.1,0.7,0.2);
//
//        grabber_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        requestOpModeStop();
    }

    @Override
    public void stop() {
        super.stop();
        grabber_shoulder.setPower(0);
    }
}
