package org.firstinspires.ftc.teamcode20;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.openftc.revextensions2.ExpansionHubEx;

import java.nio.ByteBuffer;


@Autonomous
public class RedAuto extends BaseAuto {


    int pos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initAutonomous();
        initViewMarks();
        while(!isStarted()){
            pos = new_skystoneposition();
            wait(200);
        }
        //shutdownVuforia();
        servoThread.setTarget(0.98);
        platform_grabber.setPower(1);
        platform_grabber.setPower(0.0);
        if(showTelemetry)telemetry.clear();
        grabber.setPosition(grabber_open);
        //wait(500);
        //shift to align to skystone
        int shift;
        if(pos == 1){
            shift = 0;
        }
        else if (pos == 0){
            moveInchesGOXT(-8,0.8,1,1200);
            shift=8;
        }
        else {
            moveInchesGOXT(8,0.8,1,1200);
            shift=-8;
        }

        //move forward to the skystone
        ElapsedTime p = new ElapsedTime();
        //moveInchesGOY(30.5,0.6,(1+(13.65-hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS))/13.65));
        //grab 1st block
        while(-getY1Odometry() < 25*odometryEncYPerInch)setAllDrivePowerG(-.3,-.3,.3,.3);
        setAllDrivePowerG(-.1,-.1,.1,.1);
        grabber.setPosition(grabber_closed);
        wait(300);
        servoThread.setTarget(0.85);

        while(-getY1Odometry()> 26*odometryEncYPerInch)setAllDrivePowerG(.3,.3,-.3,-.3);
        setAllDrivePower(0);
        //move forward & approach foundation
        //turn(90, 0.5,1);
        align(90);
        p.reset();
        resetXOdometry();
        moveInchesGOY(-(85.25+shift),0.6);
        p.reset();
        //while (p.milliseconds()<900)setAllDrivePowerG(-.5,.5,-.5,.5);


        moveInchesGOXT(18-getXOdometry()/odometryEncXPerInch,.5,1,2000); //drag +errordistance

        platform_grabber.setPower(-1);
        wait(300);
        moveInchesGOX_platform(-16,1,1+(13.65-hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS))/13.65);
        int steps = 20;
        double basespeed = 0.33;
        for(int i = 10;i<=steps;++i){
            RB.setPower(  -i*basespeed/steps);
            LF.setPower(-2*i*basespeed/steps);
            LB.setPower(-3*i*basespeed/steps);
            wait(20);
            //rf.setPower(0);
        }
        while (imuAbsolute>20){ getHeading(); }
        p.reset();
        while (imuAbsolute>10 && p.milliseconds()<3000){
            getHeading();
            RB.setPower(RF.getPower()*getError(imuAbsolute,0)/20);
            LF.setPower(LB.getPower()*getError(imuAbsolute,0)/20);
            LB.setPower(LF.getPower()*getError(imuAbsolute,0)/20);
        }
        setNewGyro(0);
        setAllDrivePower(0);
        after_dragged_foundation_R();
        setNewGyro(-90);

        second_and_more_R(pos);//???????


        align(0);
        if(pos==1 || pos==0)moveInchesGOXT(-4,0.8,1,1000);
        else moveInchesGOXT(4,0.8,1,1000);
        moveInchesGOY(14,0.3);
        //moveInchesGOY((right.getDistance(DistanceUnit.INCH)-2.6)*.69,.4);
        grabber.setPosition(grabber_closed);
        wait(300);
        setAllDrivePower(0);
        servoThread.setTarget(0.85);
        //setAllDrivePower(0.0);
        moveInchesG(0,-9,0.4);
        PIDturnfast(-90,false);
        setNewGyro(-90);
        int sfi = 0;
        if(pos==2)sfi = -9;
        moveInchesGOY(74+sfi,0.9);
        grabber.setPosition(grabber_open);
        moveInchesG(0,-9,0.5);
        moveInchesGOY_XF_F(-44,0.6,1,(int) (getXOdometry() - (-38.5 - adjustToViewMark(false)[1]) * odometryEncXPerInch));
    }
}
