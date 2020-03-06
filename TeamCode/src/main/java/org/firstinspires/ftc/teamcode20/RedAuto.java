package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.revextensions2.ExpansionHubEx;


@Autonomous
public class RedAuto extends BaseAuto {
    int pos = 0;
    protected void second_and_more_R(int result, int times) throws InterruptedException {

        platform_grabber.setPower(1);
        servoThread.setExtTarget(0.2);
        Thread.sleep(300);

        PIDturnfast(-90,true);
        //setAllDrivePower(0);
        //turn(-90-getHeading(),0.5,1);
        setNewGyro(-90);
        //setAllDrivePowerG(-.2,-.2,.2,.2);
        //moveInchesGOY(5,0.4);

        ElapsedTime p = new ElapsedTime();
        while (p.milliseconds()<1200){
            Thread.sleep(0);
            setAllDrivePowerG(-.4,-.4,.4,.4);
        }
        setAllDrivePower(0);
        double origin[] = {0, -40}, dd[] = adjustToViewMark(false);
        telemetry.addData("Y",dd[1]);
        telemetry.update();
        servoThread.setExtTarget(0.6);
        Thread.sleep(600);
        platform_grabber.setPower(0);
        grabber.setPosition(grabber_open);

        double curX;
        double info[] = {78.75,78.75+8,78.75+20,78.75+28,78.75+16,78.75+24};
        for (int i = 0; i < times; ++i) {
            Thread.sleep(0);
            setAllDrivePower(0);
            curX = getXOdometry();
            if (i > 0) servoThread.setExtTarget(0.75);
            grabber.setPosition(grabber_open);
            align(-91);
            moveInchesGOY_XF_F(-info[result+2], 0.6, 1, (int) (curX - (origin[1] - dd[1]) * odometryEncXPerInch));
            servoThread.setExtTarget(0.98);
            align(0);
            if(pos==0||pos==1)moveInchesGOXT(-4,1,1,1500);
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
            servoThread.setExtTarget(0.72);
            while ((getY1Odometry() - yorigin) * -1 > odometryEncYPerInch * 2) {
                Thread.sleep(0);
                setAllDrivePowerG(.3, .3, -.3, -.3);
            }
            setAllDrivePower(0);
            align(-91);
            servoThread.setExtTarget(0.4);
            moveInchesGOY_XF_F(info[result+2]-1, 0.6, 1, (int) (curX - (origin[1] - dd[1]) * odometryEncXPerInch));
        }
        grabber.setPosition(grabber_open);
    }
    @Override
    public void runOpMode() throws InterruptedException {
            initAutonomous();
            while (!isStarted() && !isStopRequested()) {
                pos = new_skystonepositionR();
                Thread.sleep(200);
            }

            before_start();

            //shift to align to skystone
            int shift;
            if (pos == 1) {
                shift = 0;
            } else if (pos == 0) {
                moveInchesGOXT(-8, 0.8, 1, 1200);
                shift = 8;
            } else {
                moveInchesGOXT(8, 0.8, 1, 1200);
                shift = -8;
            }

            //move forward to the skystone

            first_block();

            //move forward & approach foundation
            align(91);
            resetXOdometry();
            moveInchesGOY(-(85.25 + shift), 0.6, 0.915);
        int pre, cur = getXOdometry(), origin = cur;
        boolean flag = false;
        while (!flag){
            setAllDrivePowerG(-0.5,0.5,-0.5,0.5);
            pre = cur;
            cur = getXOdometry();
            if((cur-origin)/odometryEncXPerInch > 8){
                //telemetry.addData("diff", cur-pre);
                //       telemetry.update();
                if(cur-pre <4000){
                    flag = true;
                }
            }
            Thread.sleep(100);
        }
        platform_grabber.setPower(-1);
        Thread.sleep(300);
        moveInchesGOX_platform(pos==2?-21:-18, 0.8, 1 + (13.65 - hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)) / 13.65);
        setAllDrivePower(0);

        int steps = 20;
            double basespeed = 0.33;
            for (int i = 10; i <= steps; ++i) {
                RB.setPower(-i * basespeed / steps);
                LF.setPower(-2 * i * basespeed / steps);
                LB.setPower(-3 * i * basespeed / steps);
                Thread.sleep(20);
                //rf.setPower(0);
            }
            while (imuAbsolute > 20) {
                Thread.sleep(0);
                getHeading();
            }
            ElapsedTime p = new ElapsedTime();
            while (imuAbsolute > 10 && p.milliseconds() < 3000) {
                Thread.sleep(0);
                getHeading();
                RB.setPower(RF.getPower() * getError(imuAbsolute, 0) / 20);
                LF.setPower(LB.getPower() * getError(imuAbsolute, 0) / 20);
                LB.setPower(LF.getPower() * getError(imuAbsolute, 0) / 20);
            }
            setNewGyro(0);
            setAllDrivePower(0);

            second_and_more_R(pos, 1);
            hub4.setLedColor(255,20,147);
            //moveInchesGOY_XF_F(-44,0.6,1,(int) (getXOdometry() - (-38.5 - adjustToViewMark(false)[1]) * odometryEncXPerInch));
            moveInchesGOY(-44, 0.6);
    }
}
