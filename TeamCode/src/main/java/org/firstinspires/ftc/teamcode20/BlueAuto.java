package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.Roadrunner.drive.mecanum.SampleMecanumDriveREV;
import org.openftc.revextensions2.ExpansionHubEx;

@Autonomous
public class BlueAuto extends BaseAuto {
    int pos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //new StopHandlerThread(Thread.currentThread());
        initAutonomous();
        drive=new SampleMecanumDriveREV(hardwareMap);
        //cooThread.start();
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
        if (pos == 1) {
            shift = 0;
        } else if (pos == 0) {
            moveInchesGOXT(-8, 0.8, 1, 1200);
            shift = -8;
        } else {
            moveInchesGOXT(8, 0.8, 1, 1200);
            shift = 8;
        }
        //move forward to the skystone
        first_block();

        //move forward & approach foundation
        align(90);
        resetXOdometry();
        moveInchesGOY_XF((85.25 + shift), 0.9, 1);
        moveInchesGOXT(13, .65, 1, 3000); //drag +errordistance

        platform_grabber.setPower(-1);
        wait(300);
        moveInchesGOX_platform(-16, 0.8, 1 + (13.65 - hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)) / 13.65);
        int steps = 20;
        double basespeed = 0.3;
        for (int i = 10; i <= steps; ++i) {
            RF.setPower(i * basespeed / steps);
            LB.setPower(2 * i * basespeed / steps);
            LF.setPower(3 * i * basespeed / steps);
            wait(20);
            //LB.setPower(0);
        }

        while (imuAbsolute < 160) {
            getHeading();
        }
        ElapsedTime p = new ElapsedTime();
        while (imuAbsolute < 170 && p.milliseconds() < 3000) {
            getHeading();
            RF.setPower(RF.getPower() * getError(180, imuAbsolute) / 20);
            LB.setPower(LB.getPower() * getError(180, imuAbsolute) / 20);
            LF.setPower(LF.getPower() * getError(180, imuAbsolute) / 20);
        }
        setNewGyro(180);
        setAllDrivePower(0);
        after_dragged_foundation_B();

        second_and_more_B(pos, 1);
        moveInchesGOY_XF_F(-44, 0.6, 1, (int) (getXOdometry() - (41 - adjustToViewMark(true)[1]) * odometryEncXPerInch));

        requestOpModeStop();
    }
}
