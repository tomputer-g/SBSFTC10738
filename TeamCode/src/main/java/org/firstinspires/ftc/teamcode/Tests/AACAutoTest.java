package org.firstinspires.ftc.teamcode.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BaseAuto;
@Autonomous
@Deprecated
@Disabled
public class AACAutoTest extends BaseAuto {
    private double landerAwayTurningAmount = 59.6;
    private double landerAndCraterOrientation = -140;
    private double landerAndWallInitialTurningAmount = 30;
    private double landerAndWallOrientation = -90.4;
    private double landerAndWallAllignY = - 71;
    private double dumpMarkerX = 50;
    private double ASTwoX = 33.6;
    private double ASOneX = 45.6;
    private double grabberAndCraterOrientation = -54;


    @Override
    public void loop() {
        stopTFOD();
        lowerRobot(); //Lower+MoveArm

        turn(-landerAwayTurningAmount,0.2,threshold);
        moveInchesHighSpeedEncoder(-8, -8, 0.3,3,3,0.1,0.1,0.1);

        if (waitUntilVumarkRecognized() == -1)
            turningAmount = 54;
        else
        {
            refreshVumarkReading();
            currOrientation = getOrientationFromVumark();
            currHeading = currOrientation[2];
            turningAmount = headingOffset(currHeading, landerAndCraterOrientation);
            if (Math.abs(turningAmount-51) >=15)
                turningAmount = 54;
            else {
                Log.i("firstTurningAmount: ", "" + turningAmount);
                firstVumarkUsed = true;
            }
        }
        turn(turningAmount, 0.15, 1);

        if(firstVumarkUsed)
            light.setPosition(1);
        moveInchesHighSpeedEncoder(-3.5, 0, 0.1,1,1,0.15,0.2,0.1);
        light.setPosition(0.5);

        switch(TFOD_result){
            case 0:
                moveInchesHighSpeedEncoder(0,-15.7,0.3,3,6,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(-10,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(9,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(0,-18,0.4,4,4,0.1,0.55,0.15);
                break;
            case 1:
                moveInchesHighSpeedEncoder(0,-2.5,0.2,2,2,0.3,0.5,0.1);
                moveInchesHighSpeedEncoder(-8,0,0.3,3,6,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(7,0,0.3,2,4,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(0,-34,0.8,8,16,0.1,0.55,0.1);
                break;
            case 2:
                moveInchesHighSpeedEncoder(0,15,0.3,3,6,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(-10,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(9,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(0,-47.5,0.8,8,16,0.1,0.55,0.1);
                break;
            default:
                moveInchesHighSpeedEncoder(0,15,0.3,3,6,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(-10,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(9,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(0,-47.5,0.8,8,16,0.1,0.55,0.1);
                break;
        }

        turn(landerAndWallInitialTurningAmount, 0.3, 3);

        wait(1000); //magic, do not delete

        if (vumarkIsVisible()) {
            refreshVumarkReading();
            currOrientation = getOrientationFromVumark();
            currPosition = getPositionFromVumark();
            currHeading = currOrientation[2];
            turningAmount = headingOffset(currHeading, landerAndWallOrientation);
            if (Math.abs(turningAmount) > 15) {
                Log.i("Align_Overwrite", "True");
                turningAmount = 0;
            }
            else
                Log.i("Align_Overwrite", "False");
        }

        turn(turningAmount, 0.15, 1.5);

        Log.i("turningAmount: ", ""+turningAmount);
        Log.i("currHeading: ", ""+currHeading);
        Log.i("lWlOrientation: ", ""+landerAndWallOrientation);
        Log.i("x: ", ""+ currPosition[0]);
        Log.i("y: ", ""+ currPosition[1]);
        Log.i("z: ", ""+ currPosition[2]);

        YOffset = landerAndWallAllignY - currPosition[1];
        moveInchesHighSpeedEncoder(YOffset, 0, 0.2, 2,2,0.15,0.45,0.15);

        if (Math.abs(-15-YOffset) >=5) {
            Log.i("YOffset_Overwrite", "True");
            YOffset = -13;
        }
        else
            Log.i("YOffset_Overwrite", "False");

        switch(TFOD_result){
            case 0:
                XOffset = dumpMarkerX - currPosition[0];

                if (Math.abs(42.2-XOffset) >10) {
                Log.i("XOffset_Overwrite", "True");
                XOffset = 42.2;
                }
                else
                    Log.i("XOffset_Overwrite", "False");
                moveInchesHighSpeedEncoder(0, - XOffset, 0.4, 4,8,0.1,0.55,0.15);

                break;
            case 1:
                XOffset = ASOneX - currPosition[0];
                moveInchesHighSpeedEncoder(0, - XOffset, 0.4, 4,8,0.1,0.55,0.15);
//                moveInchesHighSpeedEncoder(21,0,0.2,2,4,0.1,0.3,0.1);
//                moveInchesHighSpeedEncoder(-21,0,0.2,2,4,0.1,0.3,0.1);
//                moveInchesHighSpeedEncoder(0, -4.4, 0.3, 2, 2, 0.1, 0.3, 0.1);
                break;
            case 2:
                XOffset = ASTwoX - currPosition[0];
                moveInchesHighSpeedEncoder(0, - XOffset, 0.4, 4,8,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(13,0,0.2,2,4,0.1,0.4,0.1);
                moveInchesHighSpeedEncoder(-13,0,0.2,2,4,0.2,0.4,0.1);
                moveInchesHighSpeedEncoder(0, -16.4, 0.4, 3, 6, 0.1, 0.55, 0.1);
                break;

        }


        Log.i("YOffset: ", ""+ YOffset);
        Log.i("XOffset: ", ""+ XOffset);
//
//        XOffset = dumpMarkerX - currPosition[0];
//
//        if (Math.abs(42.2-XOffset) >8) {
//            Log.i("XOffset_Overwrite", "True");
//            XOffset = 42.2;
//        }
//        else
//            Log.i("XOffset_Overwrite", "False");
//

//        moveInchesHighSpeedEncoder(0, - XOffset, 0.4, 4,8,0.1,0.55,0.15);
//
//        Log.i("YOffset: ", ""+ YOffset);
//        Log.i("XOffset: ", ""+ XOffset);
//
        dropMarker();

//        grabber_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        grabber_shoulder.setTargetPosition(-5450);
//        grabber_shoulder.setPower(1);

//        moveInchesHighSpeedEncoder(0,43 , 1, 5,5,0.3,0.5,0.08);
        moveInchesHighSpeedEncoder(0,XOffset , 0.7, 7,7,0.3,0.5,0.08);

        moveInchesHighSpeedEncoder(-YOffset, 0, 0.2, 2,2,0.15,0.45,0.15);
//        grabber_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wait(200);
        refreshVumarkReading();
        currOrientation = getOrientationFromVumark();
        currHeading = currOrientation[2];
        turningAmount = headingOffset(currHeading, grabberAndCraterOrientation);
        turn(turningAmount, 0.15, 1);
        moveInchesHighSpeedEncoder(35,0,0.6,6,12,0.1,0.55,0.1);

//        grabber_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        requestOpModeStop();
    }
}

