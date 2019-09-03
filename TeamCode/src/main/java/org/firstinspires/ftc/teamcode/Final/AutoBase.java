package org.firstinspires.ftc.teamcode.Final;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BaseAuto;

@Autonomous(group = "Final")
public class AutoBase extends BaseAuto {
    // Used to align robot to the crater before moving to the final position
    private int grabberShoulderFinalPos = -5000;
    // Used to extend the slide to the ideal position
    private int grabberSlideFinalPos = -500;
    private double landerAwayTurningAmount = 56;
    private double threshold = 3;
    private double[] dumpMarkerY = {0,-50,0,50};
    private double[] landerAndWallAllignX = {0,68.5,0,-68.5};
    // CraterBaseCorridor 0,1,2,3
    private double[] AllWOrientation = {-95,-175,95,5};
    // BaseLanderCorridor 0,1,2,3
    private double[] AllCOrientation = {-50,0,133,0};
    @Override
    public void loop() {
        stopTFOD();
        lowerRobot(); //Lower+MoveArm

        turn(-landerAwayTurningAmount,0.2,threshold);
        moveInchesHighSpeedEncoder(-8, -8, 0.3,3,3,0.1,0.1,0.3);

        int VUMarkResult = waitUntilVumarkRecognized();

        if(VUMarkResult != -1){
            refreshVumarkReading();
            light.setPosition(0.5);
            flash();
            currOrientation = getOrientationFromVumark();
            currHeading = currOrientation[2];
            turningAmount = headingOffset(currHeading, AllCOrientation[VUMarkResult]);
            if(Math.abs(turningAmount-51) >=15) turningAmount = 51;
        }
        else turningAmount = 51;
        turn(turningAmount, 0.15, 1);
        moveInchesHighSpeedEncoder(-3, 0, 0.15,1,1,0.15,0.3,0.1);

        switch(TFOD_result){
            case 0:
                moveInchesHighSpeedEncoder(0,-14,0.3,3,6,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(-9,0,0.3,3,6,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(9,0,0.3,2,4,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(0,-29,0.55,8,16,0.2,0.65,0.1);
                break;
            case 1:
                moveInchesHighSpeedEncoder(-10,0,0.3,3,6,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(8.5,0,0.3,2,4,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(0,-39,0.8,8,16,0.1,0.55,0.1);
                break;
            case 2:
                moveInchesHighSpeedEncoder(0,15,0.3,3,6,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(-11,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(9.5,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(0,-60,0.8,8,16,0.1,0.55,0.1);
                break;
            default:
                moveInchesHighSpeedEncoder(0,15,0.3,3,6,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(-11,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(9.5,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(0,-60,0.8,8,16,0.1,0.55,0.1);
                break;
        }
        turn(30,0.25,2);
        moveInches(-1.5,0,0.3);
        wait(100);
        //int VUMarkResult2 = waitUntilVumarkRecognized();

        if(vumarkIsVisible()){
            refreshVumarkReading();
            currOrientation = getOrientationFromVumark();
            currHeading = currOrientation[2];
            flash();
            currPosition = getPositionFromVumark();
            XOffset = landerAndWallAllignX[VUMarkResult+1] - currPosition[0];
            YOffset = dumpMarkerY[VUMarkResult+1] - currPosition[1];
            turningAmount = headingOffset(currHeading, AllWOrientation[VUMarkResult+1]);
            //telemetry.addData("", currOrientation[0]);
            //telemetry.addData("", currOrientation[1]);
            telemetry.addData("Using VUMark", currHeading);
            telemetry.addData("turning target: ",AllWOrientation[VUMarkResult+1]);
            telemetry.addData("turning angle: ", turningAmount);
            telemetry.addData("XOff", XOffset);
            telemetry.addData("YOff",YOffset);
            if(Math.abs(XOffset-4.3)>3) XOffset = 4.3;
            if(Math.abs(YOffset+47)>10) YOffset = -47;
            telemetry.update();
            if(Math.abs(turningAmount+165.0) > 15) turningAmount = -165;
        }
        else{
            turningAmount = -165;
            telemetry.addData("using hardcode", turningAmount);
            XOffset = 4.3;
            YOffset = -47;
            telemetry.update();
        }

        turn(turningAmount, 0.15, 1);

        //moveInchesHighSpeedEncoder(5.3, 0, 0.1, 1,1,0.15,0.35,0.1);
        //moveInchesHighSpeedEncoder(0, -47, 0.6, 4,8,0.1,0.55,0.15);
        moveInchesHighSpeedEncoder(XOffset, 0, 0.1, 1,1,0.15,0.35,0.1);
        moveInchesHighSpeedEncoder(0, YOffset, 0.4, 1,1,0.15,0.35,0.1);

        dropMarker();

        //move arm joint -6250 counts
        grabber_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber_shoulder.setTargetPosition(grabberShoulderFinalPos);
        grabber_slide.setTargetPosition(grabberSlideFinalPos);
        grabber_shoulder.setPower(1);
        grabber_slide.setPower(1);
        //moveInchesHighSpeedEncoder(2,0,0.1,1,1,0.15,0.3,0.2);
        //moveInchesHighSpeedEncoder(-2,0,0.1,1,1,0.15,0.35,0.1);
        moveInchesHighSpeedEncoder(0,49, 0.7, 15,30,0.1,0.7,0.2);

        grabber_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        requestOpModeStop();
    }
}
