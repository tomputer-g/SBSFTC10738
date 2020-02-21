package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.revextensions2.ExpansionHubEx;

@Autonomous
public class BlueAuto_NoF extends BaseAuto {
    int pos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initAutonomous();

        while(!isStarted() && !isStopRequested()){
            pos = new_skystoneposition();
            wait(200);
        }
        //go forward 1 floor mat (24")w
        //vuforia - recognize block & move to pick up
        //after pickup: turn 90 deg. move to platform, drop off
        //move to platform, drag into position, release
        //repeat until run out of time; first on other skystones

        //initialization
        before_start();

        //shift to align to skystone
        int shift;
        if(pos == 1){ shift = 0; }
        else if (pos == 0){ moveInchesGOXT(-8,0.8,1,1200);shift=-8; }
        else { moveInchesGOXT(8,0.8,1,1200);shift=8; }

        //move forward to the skystone
        ElapsedTime p = new ElapsedTime();

        first_block();

        //move forward & approach foundation
        align(90);
        p.reset();
        resetXOdometry();
        moveInchesGOY_XF((85.25+shift),0.9,1);
        p.reset();

        double curX;
        double info[] = {78.75,78.75+8,78.75+16,78.75+24,78.75+24,78.75+24};
        double origin[] = {0, 41}, dd[] = adjustToViewMark(true);
        for (int i = 0; i < 1; ++i) {
            setAllDrivePower(0);
            curX = getXOdometry();
            if (i > 0) servoThread.setTarget(0.75);
            grabber.setPosition(grabber_open);
            align(90);
            moveInchesGOY_XF_F(-info[pos+2], 0.6, 1, (int) (curX - (origin[1] - dd[1]) * odometryEncXPerInch));
            servoThread.setTarget(0.98);
            align(0);

            double yorigin = getY1Odometry();
            while ((getY1Odometry() - yorigin) * -1 < odometryEncYPerInch * 4) {
                setAllDrivePowerG(-.3, -.3, .3, .3);
            }
            while ((getY1Odometry() - yorigin) * -1 < odometryEncYPerInch * 8) {
                setAllDrivePowerG(-.1, -.1, .1, .1);
            }
            grabber.setPosition(grabber_closed);
            wait(300);
            servoThread.setTarget(0.85);
            while ((getY1Odometry() - yorigin) * -1 > odometryEncYPerInch * 2) {
                setAllDrivePowerG(.3, .3, -.3, -.3);
            }
            setAllDrivePower(0);
            align(90);
            servoThread.setTarget(0.65);
            moveInchesGOY_XF_F(info[pos+2]-1, 0.6, 1, (int) (curX - (origin[1] - dd[1]) * odometryEncXPerInch));
        }

        grabber.setPosition(grabber_open);
        dd = adjustToViewMark(true);

        double diss=78.75+8;
        if(pos==0)diss=78.75;
        for (int i = 0; i < 1; ++i) {
            setAllDrivePower(0);
            curX = getXOdometry();
            if (i > 0) servoThread.setTarget(0.75);
            grabber.setPosition(grabber_open);
            align(90);
            moveInchesGOY_XF_F(-diss, 0.6, 1, (int) (curX - (origin[1] - dd[1]) * odometryEncXPerInch));
            servoThread.setTarget(0.98);
            align(0);

            double yorigin = getY1Odometry();
            while ((getY1Odometry() - yorigin) * -1 < odometryEncYPerInch * 4) {
                setAllDrivePowerG(-.3, -.3, .3, .3);
            }
            while ((getY1Odometry() - yorigin) * -1 < odometryEncYPerInch * 8) {
                setAllDrivePowerG(-.1, -.1, .1, .1);
            }
            grabber.setPosition(grabber_closed);
            wait(300);
            servoThread.setTarget(0.85);
            while ((getY1Odometry() - yorigin) * -1 > odometryEncYPerInch * 2) {
                setAllDrivePowerG(.3, .3, -.3, -.3);
            }
            setAllDrivePower(0);
            align(90);
            servoThread.setTarget(0.65);
            moveInchesGOY_XF_F(diss-1, 0.6, 1, (int) (curX - (origin[1] - dd[1]) * odometryEncXPerInch));
        }
        grabber.setPosition(grabber_open);

        moveInchesGOY_XF_F(-44,0.6,1,(int) (getXOdometry() - (41 - adjustToViewMark(true)[1]) * odometryEncXPerInch));
        requestOpModeStop();
    }
}

