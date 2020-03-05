package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.Roadrunner.drive.mecanum.SampleMecanumDriveREV;
import org.openftc.revextensions2.ExpansionHubEx;

@Autonomous
public class BlueAuto extends BaseAuto {
    protected void second_and_more_B(int result, int times) throws InterruptedException {
        ElapsedTime p = new ElapsedTime();
        platform_grabber.setPower(1);
        servoThread.setExtTarget(0.2);
        Thread.sleep(300);
        //p.reset();
        ///while (p.milliseconds()<300);
        PIDturnfast(-90,true);
        //setAllDrivePower(0);
        //turn(-90-getHeading(),0.5,1);
        setNewGyro(90);
        //setAllDrivePowerG(-.2,-.2,.2,.2);
        //moveInchesGOY(5,0.4);

        p.reset();
        while (p.milliseconds()<1200){
            setAllDrivePowerG(-.4,-.4,.4,.4);
            Thread.sleep(0);
        }
        setAllDrivePower(0);
        double origin[] = {0, 41}, dd[] = adjustToViewMark(true);
        telemetry.addData("Y",dd[1]);
        telemetry.update();
        servoThread.setExtTarget(0.6);
        p.reset();
        while (p.milliseconds()<600){Thread.sleep(0);}
        platform_grabber.setPower(0);
        grabber.setPosition(grabber_open);

        double curX;
        double info[] = {78.75,78.75+8,78.75+16,78.75+24,78.75+24,78.75+24};
        for (int i = 0; i < times; ++i) {
            setAllDrivePower(0);
            curX = getXOdometry();
            if (i > 0) servoThread.setExtTarget(0.75);
            grabber.setPosition(grabber_open);
            align(90);
            moveInchesGOY_XF_F(-info[result+2], 0.6, 1, (int) (curX - (origin[1] - dd[1]) * odometryEncXPerInch));
            servoThread.setExtTarget(0.98);
            align(0);
            if(result==2||result==1)moveInchesGOX(-4,1);
            double yorigin = getY1Odometry();
            while ((getY1Odometry() - yorigin) * -1 < odometryEncYPerInch * 4) {
                Thread.sleep(0);
                setAllDrivePowerG(-.3, -.3, .3, .3);
            }
            while ((getY1Odometry() - yorigin) * -1 < odometryEncYPerInch * 9) {
                Thread.sleep(0);
                setAllDrivePowerG(-.1, -.1, .1, .1);
            }
            grabber.setPosition(grabber_closed);
            Thread.sleep(300);
            servoThread.setExtTarget(0.85);
            while ((getY1Odometry() - yorigin) * -1 > odometryEncYPerInch * 2) {
                setAllDrivePowerG(.3, .3, -.3, -.3);
            }
            setAllDrivePower(0);
            align(90);
            servoThread.setExtTarget(0.4);
            moveInchesGOY_XF_F(info[result+2]-1, 0.6, 1, (int) (curX - (origin[1] - dd[1]) * odometryEncXPerInch));
        }
        grabber.setPosition(grabber_open);
    }
    @Override public void runOpMode() throws InterruptedException {

            initAutonomous();
            hub4.setLedColor(255,20,147);
            drive = new SampleMecanumDriveREV(hardwareMap);
            cooThread.start();
            int pos = 0;
            while (!isStarted()) {
                pos = new_skystoneposition();
                Thread.sleep(200);
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
            moveInchesGOXT(12, .45, 1, 2000); //magic, do not touch
            platform_grabber.setPower(-1);
            Thread.sleep(300);
            moveInchesGOX_platform(-16, 0.8, 1 + (13.65 - hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)) / 13.65);
            int steps = 20;
            double basespeed = 0.3;
            for (int i = 10; i <= steps; ++i) {
                RF.setPower(i * basespeed / steps);
                LB.setPower(2 * i * basespeed / steps);
                LF.setPower(3 * i * basespeed / steps);
                Thread.sleep(20);
                //LB.setPower(0);
            }

            while (imuAbsolute < 160) { Thread.sleep(0);getHeading(); }
            ElapsedTime p = new ElapsedTime();
            while (imuAbsolute < 170 && p.milliseconds() < 3000) {
                Thread.sleep(0);
                getHeading();
                RF.setPower(RF.getPower() * getError(180, imuAbsolute) / 20);
                LB.setPower(LB.getPower() * getError(180, imuAbsolute) / 20);
                LF.setPower(LF.getPower() * getError(180, imuAbsolute) / 20);
            }
            setAllDrivePower(0);
            setNewGyro(180);
            second_and_more_B(pos, 1);
            hub4.setLedColor(255,20,147);
            moveInchesGOY_XF_F(-44, 0.6, 1, (int) (getXOdometry() - (41 - adjustToViewMark(true)[1]) * odometryEncXPerInch));
        requestOpModeStop();
    }
}
