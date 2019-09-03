package org.firstinspires.ftc.teamcode.Final;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BaseAuto;

@Autonomous(group = "Final")
public class AutoRedCraterDS12_SF extends BaseAuto {
    private double landerAwayTurningAmount = 60;
    // The first turnning Angle of Auto, basically get off the lander

    private double landerAndCraterOrientation = -140;
    // Make the Robot parallel to crater

    private double landerAndWallInitialTurningAmount = 40;
    // The angle turned to see Vumark when it's close to the wall

    private double landerAndWallOrientation = -90.7;
    // The angle used to align robot to the wall (With Vumark)

    private double landerAndWallAllignY = -67.25;
    // The Y value of the robot position before going to the depot

    private double dumpMarkerX = -50;
    // The X value of the robot position before dumping the marker

    private double DSPosZeroX = 60;
    // The X value of the robot position before doing DS in Position Two

    private double DSPosOneX = 48.5;
    // The X value of the robot position before doing DS in Position One

    private double DSPosTwoX = 35.5;
    // The X value of the robot position before doing DS in Position Zero

    private double grabberAndCraterOrientation = -55.25;
    // Used to align robot to the crater before moving to the final position


    public void loop(){
        stopTFOD();
        lowerRobot();

        // Clear the lander
        turn(-landerAwayTurningAmount,0.2,threshold);
        moveInchesHighSpeedEncoder(-8, -8, 0.3,3,3,0.1,0.1,0.1);

        // Attempt to see Vumark in order to align robot with the crater
        if (waitUntilVumarkRecognized() == -1) {
            turningAmount = 55.5;
            Log.i("LanderAndCraterAlign: ", "Overwrite: True");
        }
        else
        {
            refreshVumarkReading();
            currOrientation = getOrientationFromVumark();
            currHeading = currOrientation[2];
            turningAmount = headingOffset(currHeading, landerAndCraterOrientation);
            if (Math.abs(turningAmount-55.5) >=15) {
                turningAmount = 55.5;
                Log.i("LanderAndCraterAlign: ", "Overwrite: True");
            }
            else {
                Log.i("LanderAndCraterAlign: ", "Overwrite: False");
                Log.i("LanderAndCraterAlign: ", "" + turningAmount);
            }
        }
        turn(turningAmount, 0.15, 1);

        moveInchesHighSpeedEncoder(-3.5, 0, 0.1,1,1,0.15,0.2,0.1);


        // Move to position in order to push mineral. Push it. Come Back. Move to the wall
        switch(TFOD_result) {
            case 0:
                moveInchesHighSpeedEncoder(0, -15.7, 0.3, 3, 6, 0.2, 0.4, 0.15);
                moveInchesHighSpeedEncoder(-10, 0, 0.3, 3, 3, 0.2, 0.3, 0.1);
                moveInchesHighSpeedEncoder(10, 0, 0.3, 3, 3, 0.2, 0.3, 0.1);
                moveInchesHighSpeedEncoder(0, -18, 0.8, 4, 8, 0.2, 0.4, 0.15);
                break;
            case 1:
                moveInchesHighSpeedEncoder(-10, 0, 0.3, 3, 6, 0.1, 0.55, 0.15);
                moveInchesHighSpeedEncoder(9, 0, 0.3, 2, 4, 0.1, 0.55, 0.15);
                moveInchesHighSpeedEncoder(0, -33.5, 0.8, 8, 16, 0.1, 0.55, 0.1);
                break;
            case 2:
                moveInchesHighSpeedEncoder(0, 14.5, 0.3, 3, 6, 0.1, 0.55, 0.15);
                moveInchesHighSpeedEncoder(-10, 0, 0.2, 2, 4, 0.1, 0.3, 0.1);
                moveInchesHighSpeedEncoder(9, 0, 0.2, 2, 4, 0.1, 0.3, 0.1);
                moveInchesHighSpeedEncoder(0, -49, 0.8, 8, 16, 0.1, 0.55, 0.1);
                break;
            default:
                moveInchesHighSpeedEncoder(0, 13, 0.4, 3, 6, 0.1, 0.55, 0.15);
                moveInchesHighSpeedEncoder(-9, 0, 0.4, 2, 4, 0.1, 0.3, 0.1);
                moveInchesHighSpeedEncoder(9, 0, 0.4, 2, 4, 0.1, 0.3, 0.1);
                moveInchesHighSpeedEncoder(0, -47.5, 0.8, 8, 16, 0.1, 0.55, 0.1);
        }

        // Turn in order to see Vumark
        turn(landerAndWallInitialTurningAmount, 0.3, 3);
        wait(1000); //magic, do not delete

        // Calibration. After this block of code, the robot should be exactly parallel to the wall.
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
        else
        {
            Log.i("Vumark_Visibility", "False");
            Log.i("Align_Overwrite", "True");
            turningAmount = 0;
        }
        turn(turningAmount, 0.1, 1);

        Log.i("turningAmount: ", ""+turningAmount);
        Log.i("currHeading: ", ""+currHeading);
        Log.i("lWlOrientation: ", ""+landerAndWallOrientation);
        Log.i("x: ", ""+ currPosition[0]);
        Log.i("y: ", ""+ currPosition[1]);

        // Calculating the route to the depot.
        switch(TFOD_result) {
            case 0:
                // Calculating the route to the depot.
                if (vumarkIsVisible()) {
                    refreshVumarkReading();
                    currPosition = getPositionFromVumark();
                    XOffset = dumpMarkerX - currPosition[0];
                    YOffset = landerAndWallAllignY - currPosition[1];
                    Log.i("Calculated XOffset", "" + XOffset);
                    Log.i("Calculated YOffset", "" + YOffset);
                    if (Math.abs(XOffset-47) >= 10) {
                        XOffset = 47;
                        Log.i("XOffset_Overwrite", "True");
                    }
                    else
                        Log.i("XOffset_Overwrite", "False");

                    if (Math.abs(YOffset+10.5) >= 5) {
                        YOffset = -10.5;
                        Log.i("YOffset_Overwrite", "True");
                    }
                    else
                        Log.i("YOffset_Overwrite", "False");
                }
                else{
                    XOffset = 47;
                    YOffset = -10.5;
                    Log.i("XOffset_Overwrite", "True");
                    Log.i("YOffset_Overwrite", "True");
                }
                break;
            case 1:
                if (vumarkIsVisible()) {
                    refreshVumarkReading();
                    currPosition = getPositionFromVumark();
                    XOffset = DSPosOneX - currPosition[0];
                    YOffset = landerAndWallAllignY - currPosition[1];
                    Log.i("Calculated XOffset", "" + XOffset);
                    Log.i("Calculated YOffset", "" + YOffset);
                    if (Math.abs(XOffset - 41) > 10)
                    {
                        XOffset = 41;
                        Log.i("XOffset_Overwrite", "True");
                    }
                    else
                        Log.i("XOffset_Overwrite", "False");
                    if (Math.abs(YOffset +11.3) > 7)
                    {
                        YOffset = -11.3;
                        Log.i("YOffset_Overwrite", "True");
                    }
                    else
                        Log.i("YOffset_Overwrite", "False");
                }
                else
                {
                    Log.i("Vumark_Visibility", "False");
                    XOffset = 41;
                    YOffset = -11.3;
                    Log.i("XOffset_Overwrite", "True");
                    Log.i("YOffset_Overwrite", "True");
                }
                break;
            case 2:
                if (vumarkIsVisible()) {
                    refreshVumarkReading();
                    currPosition = getPositionFromVumark();
                    XOffset = DSPosTwoX - currPosition[0];
                    YOffset = landerAndWallAllignY - currPosition[1];
                    Log.i("Calculated XOffset", "" + XOffset);
                    Log.i("Calculated YOffset", "" + YOffset);
                    if (Math.abs(XOffset - 26.7) > 10)
                    {
                        XOffset = 26.7;
                        Log.i("XOffset_Overwrite", "True");
                    }
                    else
                        Log.i("XOffset_Overwrite", "False");
                    if (Math.abs(YOffset +13) > 10)
                    {
                        YOffset = -13;
                        Log.i("YOffset_Overwrite", "True");
                    }
                    else
                        Log.i("YOffset_Overwrite", "False");
                }
                else
                {
                    Log.i("Vumark_Visibility", "False");
                    XOffset = 26.7;
                    YOffset = -15;
                    Log.i("XOffset_Overwrite", "True");
                    Log.i("YOffset_Overwrite", "True");
                }
                break;
            default:
                refreshVumarkReading();
                currPosition = getPositionFromVumark();
                XOffset = dumpMarkerX - currPosition[0];
                YOffset = landerAndWallAllignY - currPosition[1];
                break;
        }
        Log.i("Moving to depot","Moving to depot");
        Log.i("x: ", ""+ currPosition[0]);
        Log.i("y: ", ""+ currPosition[1]);
        Log.i("z: ", ""+ currPosition[2]);
        Log.i("XOffset: ", ""+ XOffset);
        Log.i("YOffset: ", ""+ YOffset);

        // Move closer to the wall, but not hitting it.(If hitting, increase landerAndWallAllignY)
        moveInchesHighSpeedEncoder(YOffset, 0, 0.2, 2,2,0.15,0.45,0.15);

        // Move to the depot, without going too far. (If going too far, decrease dumpMarkerX)
        moveInchesHighSpeedEncoder(0, - XOffset, 0.4, 4,8,0.1,0.55,0.15);

        switch(TFOD_result) {
            case 0:

                // Only dropping the marker into depot
                dropMarker();
                break;
            case 1:
                // Push alliance mineral and come back
                moveInchesHighSpeedEncoder(20,0,0.3,4,8,0.1,0.55,0.1);
                moveInchesHighSpeedEncoder(-20,0,0.3,4,8,0.1,0.55,0.1);
                // Move to depot
                moveInchesHighSpeedEncoder(0, -1.5, 0.2, 2,2,0.2,0.55,0.15);
                // Drop marker into depot
                dropMarker();
                break;
            case 2:
                // Push alliance mineral and come back
                moveInchesHighSpeedEncoder(12,0,0.3,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(-12,0,0.3,2,4,0.1,0.3,0.1);
                // Move to depot
                moveInchesHighSpeedEncoder(0, -14.5, 0.4, 4,8,0.2,0.55,0.15);
                // Drop marker into depot
                dropMarker();
                break;
            default:
                dropMarker();
                break;
        }


        // Move Arm and slide to grabbing position
        grabber_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber_shoulder.setTargetPosition(grabberShoulderFinalPos);
        grabber_slide.setTargetPosition(grabberSlideFinalPos);
        grabber_shoulder.setPower(1);
        grabber_slide.setPower(1);

        if (TFOD_result == 0)
            moveInchesHighSpeedEncoder(0,63, 1, 10,20,0.3,0.5,0.2);
        else
            moveInchesHighSpeedEncoder(0,57, 1, 10,20,0.3,0.6,0.2);

        // Change grabber shoulder motor mode. Get ready for Teleop.
        grabber_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        requestOpModeStop();
        }
    }
