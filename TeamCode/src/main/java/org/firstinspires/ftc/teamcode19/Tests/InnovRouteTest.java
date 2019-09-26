package org.firstinspires.ftc.teamcode19.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode19.BaseAuto;

@Autonomous(group = "Test")
@Disabled
public class InnovRouteTest extends BaseAuto {
    private double landerAwayTurningAmount = 59.6;
    // The first turnning Angle of Auto, basically get off the lander
    private double landerAndCraterOrientation = -140;
    // Make the Robot parallel to crater
    private double landerAndWallInitialTurningAmount = 40;
    // The angle turned to see Vumark when it's close to the wall
    private double landerAndWallOrientation = -45;
    // The angle used to align robot to the wall (With Vumark)
    private double landerAndWallAllignY = - 69.5;
    // The Y value of the robot position before going to the depot
    private double dumpMarkerX = 50;
    // The X value of the robot position defore dumping the marker
    private double grabberAndCraterOrientation = -55.25;
    // Used to align robot to the crater before moving to the final position
    private int grabberShoulderFinalPos = -5450;
    // Used to extend the slide to the ideal position
    private int grabberSlideFinalPos = -675;

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
            if (Math.abs(turningAmount-51) >=15) {
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
        moveInchesHighSpeedEncoder(0, -36, 0.8, 8, 16, 0.1, 0.55, 0.1);


        // Turn in order to see Vumark
        turn(landerAndWallInitialTurningAmount, 0.3, 3);
        wait(1000); //magic, do not delete

        refreshVumarkReading();
        currOrientation = getOrientationFromVumark();
        currPosition = getPositionFromVumark();
        currHeading = currOrientation[2];
        turningAmount = headingOffset(currHeading, landerAndWallOrientation);
        turn(turningAmount, 0.3, 2);

        moveInchesHighSpeedEncoder(0, -38, 0.8, 8, 16, 0.1, 0.55, 0.1);


        requestOpModeStop();
        }
    }
