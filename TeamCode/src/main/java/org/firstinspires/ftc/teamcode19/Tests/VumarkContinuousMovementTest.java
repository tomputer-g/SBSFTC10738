package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode19.BaseAuto;

@Autonomous
@Disabled
public class VumarkContinuousMovementTest extends BaseAuto {
    double firstAngle = -140;
    double targetY = -45;
    double secondAngle = -90;
    @Override
    public void init() {
        super.init();
        initIMU();
        initVuforiaEngine();
        initVumark();
    }

    @Override
    public void loop() {
        targetVisible = false;
        while(!targetVisible) {
            refreshVumarkReading();
        }

        // Provide feedback as to where the robot is located (if we know).
        // express position (translation) of robot in inches.
        double[] currPos = getPositionFromVumark();
        double[] currOrientation = getOrientationFromVumark();
        double currX = currPos[0];
        double currY = currPos[1];
        double currZ = currPos[2];
        double currRow = currOrientation[0];
        double currPitch = currOrientation[1];
        double currHeading = currOrientation[2];

        double turningAmount = headingOffset(currHeading, firstAngle);
        turn(turningAmount, 0.2, 2);

//        currY = -1000;
//
//        setAllDrivePower(-0.2, 0);
//
//        while (Math.abs(targetY-currY) > 1.0 || currY == -1000)
//        {
//            refreshVumarkReading();
//            currPos = getPositionFromVumark();
//            currX = currPos[0];
//            currY = currPos[1];
//            currZ = currPos[2];
//            currRow = currOrientation[0];
//            currPitch = currOrientation[1];
//            currHeading = currOrientation[2];
//            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                              currX, currY, currZ);
//            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", currRow, currPitch, currHeading);
//            telemetry.update();
//        }
//        setAllDrivePower(0,0);
////        turn(-50, 0.2, 3);
//
//        refreshVumarkReading();
//        currPos = getPositionFromVumark();
//        currX = currPos[0];
//        currY = currPos[1];
//        currZ = currPos[2];
//        currRow = currOrientation[0];
//        currPitch = currOrientation[1];
//        currHeading = currOrientation[2];
//
//        turningAmount = headingOffset(currHeading, secondAngle);
//        turn(turningAmount, 0.2, 2);

        requestOpModeStop();
    }
    private int getVumarkOrientation(String VuMarkName){
        if(VuMarkName.equals("Back-Space"))
            return 0;
        if(VuMarkName.equals("Blue-Rover"))
            return 90;
        if(VuMarkName.equals("Red-Footprint"))
            return 180;
        if(VuMarkName.equals("Front-Craters"))
            return 270;
        return -1;
    }
}
