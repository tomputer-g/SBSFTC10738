package org.firstinspires.ftc.teamcode.Final;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BaseAuto;

@Autonomous(group = "Final")
public class AutoBlueCraterSS_MF extends BaseAuto {
    private double landerAwayTurningAmount = 58;
    // The first turnning Angle of Auto, basically get off the lander

    private double landerAndCraterOrientation = 44;
    // Make the Robot parallel to crater

    private double landerAndWallInitialTurningAmount = 40;
    // The angle turned to see Vumark when it's close to the wall

    private double landerAndWallOrientation = 90;
    // The angle used to align robot to the wall (With Vumark)

    private double landerAndWallAllignY = 69.5;
    // The Y value of the robot position before going to the depot

    private double dumpMarkerX = -50;
    // The X value of the robot position defore dumping the marker

    private double grabberAndCraterOrientation = 129.5;
    // The heading of the robot before moving back between the lander and crater

    public void loop() {
        stopTFOD();
        lowerRobot();

        // Clear the lander
        turn(-landerAwayTurningAmount, 0.2, threshold);
        moveInchesHighSpeedEncoder(-8, -8, 0.3, 3, 3, 0.1,
                0.1, 0.1);

        // Attempt to see Vumark in order to align robot with the crater
        // Wait exists here:
        if (waitUntilVumarkRecognized() == -1) {
            Log.i("Vumark_Visibility", "False");
            turningAmount = 55.5;
            Log.i("LanderAndCraterAlign: ", "Overwrite: True");
        } else {
            Log.i("Vumark_Visibility", "True");
            refreshVumarkReading();
            currOrientation = getOrientationFromVumark();
            currHeading = currOrientation[2];
            turningAmount = headingOffset(currHeading, landerAndCraterOrientation);
            if (Math.abs(turningAmount - 51) >= 15) {
                turningAmount = 55.5;
                Log.i("LanderAndCraterAlign: ", "Overwrite: True");
            } else {
                Log.i("LanderAndCraterAlign: ", "Overwrite: False");
                Log.i("LanderAndCraterAlign: ", "" + turningAmount);
            }
        }
        Log.i("LanderAndCraterAlign: ", "" + turningAmount);
        turn(turningAmount, 0.15, 1);

        moveInchesHighSpeedEncoder(-3.5, 0, 0.1, 1, 1, 0.15,
                0.2, 0.1);


        // Move to position in order to push mineral. Push it. Come Back. Move to the wall
        switch (TFOD_result) {
            case 0:
                moveInchesHighSpeedEncoder(0, -15.7, 0.3, 3, 6,
                        0.1, 0.55, 0.15);
                moveInchesHighSpeedEncoder(-10, 0, 0.2, 2, 4,
                        0.1, 0.3, 0.1);
                moveInchesHighSpeedEncoder(9, 0, 0.2, 2, 4,
                        0.1, 0.3, 0.1);
                moveInchesHighSpeedEncoder(0, -18, 0.4, 4, 4,
                        0.1, 0.55, 0.15);
                break;
            case 1:
//                moveInchesHighSpeedEncoder(0, -1.5, 0.2, 2, 2,
//                        0.3, 0.5, 0.1);
                moveInchesHighSpeedEncoder(-10, 0, 0.3, 3, 6,
                        0.3, 0.4, 0.15);
                moveInchesHighSpeedEncoder(9, 0, 0.3, 2, 4,
                        0.3, 0.4, 0.15);
                moveInchesHighSpeedEncoder(0, -33.5, 0.8, 4, 16,
                        0.2, 0.5, 0.1);
                break;
            case 2:
                moveInchesHighSpeedEncoder(0, 15, 0.3, 3, 6,
                        0.1, 0.55, 0.15);
                moveInchesHighSpeedEncoder(-10, 0, 0.2, 2, 4,
                        0.1, 0.3, 0.1);
                moveInchesHighSpeedEncoder(9, 0, 0.2, 2, 4,
                        0.1, 0.3, 0.1);
                moveInchesHighSpeedEncoder(0, -47.5, 0.8, 8, 16,
                        0.1, 0.55, 0.1);
                break;
            default:
                moveInchesHighSpeedEncoder(0, 13, 0.3, 3, 6,
                        0.1, 0.55, 0.15);
                moveInchesHighSpeedEncoder(-9, 0, 0.2, 2, 4,
                        0.1, 0.3, 0.1);
                moveInchesHighSpeedEncoder(9, 0, 0.2, 2, 4,
                        0.1, 0.3, 0.1);
                moveInchesHighSpeedEncoder(0, -47.5, 0.8, 8, 16,
                        0.1, 0.55, 0.1);
                break;
        }

        // Turn in order to see Vumark
        turn(landerAndWallInitialTurningAmount, 0.3, 3);
        wait(1000); //magic, do not delete

        // Calibration. After this block of code, the robot should be exactly parallel to the wall.
        if (vumarkIsVisible()) {
            Log.i("Vumark_Visibility", "True");
            refreshVumarkReading();
            currOrientation = getOrientationFromVumark();
            currPosition = getPositionFromVumark();
            currHeading = currOrientation[2];
            turningAmount = headingOffset(currHeading, landerAndWallOrientation);
            if (Math.abs(turningAmount) > 15) {
                Log.i("Align_Overwrite", "True");
                turningAmount = 0;
            } else {
                Log.i("Align_Overwrite", "False");
                Log.i("Vumark_Visibility", "False");
            }
        }
        else
        {
            Log.i("Vumark_Visibility", "False");
            Log.i("Align_Overwrite", "True");
            turningAmount = 0;
        }
        turn(turningAmount, 0.1, 1);
        if (!(currPosition == null)) {
            Log.i("turningAmount: ", "" + turningAmount);
            Log.i("currHeading: ", "" + currHeading);
            Log.i("lWlOrientation: ", "" + landerAndWallOrientation);
            Log.i("x: ", "" + currPosition[0]);
            Log.i("y: ", "" + currPosition[1]);
            Log.i("z: ", "" + currPosition[2]);
        }
        else
        {
            Log.i("currPosition: ", "null");
        }

        // Calculating the route to the depot.
        if (vumarkIsVisible()) {
            Log.i("Vumark_Visibility", "True");
            refreshVumarkReading();
            currPosition = getPositionFromVumark();
            XOffset = dumpMarkerX - currPosition[0];
            YOffset = landerAndWallAllignY - currPosition[1];
            Log.i("Calculated XOffset", "" + XOffset);
            Log.i("Calculated YOffset", "" + YOffset);
            if (Math.abs(XOffset + 41.6) >= 10) {
                XOffset = -41.6;
                Log.i("XOffset_Overwrite", "True");
            } else
                Log.i("XOffset_Overwrite", "False");

            if (Math.abs(YOffset - 16.6) >= 7) {
                YOffset = 16.6;
                Log.i("YOffset_Overwrite", "True");
            } else
                Log.i("YOffset_Overwrite", "False");
        } else {
            Log.i("Vumark_Visibility", "False");
            XOffset = -41.6;
            YOffset = 16.6;
            Log.i("XOffset_Overwrite", "True");
            Log.i("YOffset_Overwrite", "True");
        }
        if (!(currPosition == null)) {
            Log.i("x: ", "" + currPosition[0]);
            Log.i("y: ", "" + currPosition[1]);
            Log.i("z: ", "" + currPosition[2]);
            Log.i("XOffset: ", "" + XOffset);
            Log.i("YOffset: ", "" + YOffset);
        }
        else
        {
            Log.i("currPosition: ", "null");
        }
        // Move closer to the wall, but not hitting it.(If hitting, decrease landerAndWallAllignY)
        moveInchesHighSpeedEncoder(-YOffset, 0, 0.2, 2, 2, 0.15,
                0.45, 0.15);

        // Move to the depot, without going too far. (If going too far, increase dumpMarkerX)
        moveInchesHighSpeedEncoder(0, XOffset, 0.4, 4, 8, 0.1,
                0.55, 0.15);

        // Drop marker into depot
        dropMarker();

        // Move back to Vumark.
        moveInchesHighSpeedEncoder(0, -XOffset -3.5, 0.75, 5, 4,
                0.3, 0.4, 0.08);
        moveInchesHighSpeedEncoder(YOffset-4, 0, 0.2, 2, 2, 0.15,
                0.45, 0.15);

        // Calculating how grabber can be parallel to the crater
        if (vumarkIsVisible()) {

            
            Log.i("Vumark_Visibility", "True");
            refreshVumarkReading();
            currOrientation = getOrientationFromVumark();
            currHeading = currOrientation[2];
            turningAmount = headingOffset(currHeading, grabberAndCraterOrientation);
            if (Math.abs(turningAmount-37.7) > 15)
            {
                turningAmount =37.7;
                Log.i("Last_Turning_Overwrite", "True");
            }
            else
                Log.i("Last_Turning_Overwrite", "False");
        }
        else{
            turningAmount = 37.7;
            Log.i("Vumark_Visibility", "False");
            Log.i("Last_Turning_Overwrite", "True");
        }

            Log.i("turningAmount: ", "" + turningAmount);
            Log.i("currHeading: ", "" + currHeading);
            Log.i("g&cOrientation: ", "" + grabberAndCraterOrientation);

            // Perform the last movement. Reach optimal position to start Teleop
            turn(turningAmount, 0.15, 1);

            // Move Arm to grabbing position
            grabber_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            grabber_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            grabber_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            grabber_shoulder.setTargetPosition(grabberShoulderFinalPos);
            grabber_slide.setTargetPosition(grabberSlideFinalPos);
            grabber_shoulder.setPower(1);
            grabber_slide.setPower(1);

            moveInchesHighSpeedEncoder(25, 0, 0.6, 6, 12, 0.1,
                    0.55, 0.1);

            // Change grabber shoulder and arm motor mode. Get ready for Teleop.
            grabber_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            grabber_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            grabber_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            requestOpModeStop();
        }
    }
