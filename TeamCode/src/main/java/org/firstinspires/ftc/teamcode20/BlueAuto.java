package org.firstinspires.ftc.teamcode20;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode20.Roadrunner.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode20.Roadrunner.drive.mecanum.SampleMecanumDriveREV;
import org.openftc.revextensions2.ExpansionHubEx;

import java.nio.ByteBuffer;

@Autonomous
public class BlueAuto extends BaseAuto {
    private int pos = 0;
    private CThread cthread;
    private SampleMecanumDriveREV drive;
    @Override
    public void init() {
        initAutonomous();
        initViewMarks();
        drive=new SampleMecanumDriveREV(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-36,72,-Math.PI/2));
        cthread=new CThread();
	}
    @Override
    public void init_loop(){
        pos = new_skystoneposition();
        wait(200);
    }

    @Override
    public void start(){
    }

    @Override
    public void loop() {
        //go forward 1 floor mat (24")w
        //vuforia - recognize block & move to pick up
        //after pickup: turn 90 deg. move to platform, drop off
        //move to platform, drag into position, release
        //repeat until run out of time; first on other skystones

        //initialization
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
            shift=-8;
        }
        else {
            moveInchesGOXT(8,0.8,1,1200);
            shift=8;
        }

        //move forward to the skystone
        ElapsedTime p = new ElapsedTime();
        moveInchesGOY(30.5,0.6,(1+(13.65-hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS))/13.65));
        //grab 1st block
        grabber.setPosition(grabber_closed);
        wait(300);
        servoThread.setTarget(0.85);
        //setAllDrivePower(0.0);
        moveInchesG(0,-6,0.4);

        //move forward & approach foundation
        align(90);
        p.reset();
        resetXOdometry();
        moveInchesGOY_XF((85.25+shift),0.9,1);
        p.reset();

        moveInchesGOXT(13.5-getXOdometry()/odometryEncXPerInch,.5,1,1300); //drag +errordistance

        platform_grabber.setPower(-1);
        wait(300);
        moveInchesGOX_platform(-19,0.8,1+(13.65-hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS))/13.65);
        int steps = 20;
        double basespeed = 0.3;
        for(int i = 10;i<=steps;++i){
            RF.setPower(  i*basespeed/steps);
            LB.setPower(2*i*basespeed/steps);
            LF.setPower(3*i*basespeed/steps);
            wait(20);
            //LB.setPower(0);
        }
        double curAng = getHeading();
        while (curAng<70){
            curAng = getHeading();
        }
        while (curAng<88){
            curAng = getHeading();
            RF.setPower(RF.getPower()*getError(90,curAng)/20);
            LB.setPower(LB.getPower()*getError(90,curAng)/20);
            LF.setPower(LF.getPower()*getError(90,curAng)/20);

        }
        setNewGyro(180);
        setAllDrivePower(0);
        after_dragged_foundation_B();
        setNewGyro(90);

        second_and_more_B(pos);
        /*
        moveInchesGOXT(-6,0.5,1,1000);

        int shiftt = 0;
        if(pos == 1 || pos == 2) shiftt = -8;
        moveInchesGOY_XF(-96.5+shiftt,0.6,1);
        servoThread.setTarget(0.95);
        PIDturnfast(-90,false);
        setNewGyro(0);

        align(0);
        if(pos==1||pos==2)moveInchesGOXT(4,0.8,1,1000);
        moveInchesGOY_XF(10,0.3,1);
        //moveInchesGOY((right.getDistance(DistanceUnit.INCH)-2.6)*.69,.4);
        grabber.setPosition(grabber_closed);
        wait(300);
        setAllDrivePower(0);
        servoThread.setTarget(0.85);
        //setAllDrivePower(0.0);
        moveInchesG(0,-9.5,0.4);
        PIDturnfast(90,false);
        setNewGyro(90);
        int sfi = 0;
        if(pos==0)sfi = -9;
        moveInchesGOY_XF(77+sfi,0.9,1);
        grabber.setPosition(grabber_open);
        moveInchesG(0,-13,0.5);


         */
        moveInchesGOY_XF_F(-44,0.6,1,(int) (getXOdometry() - (41 - adjustToViewMark(true)[1]) * odometryEncXPerInch));
        requestOpModeStop();
    }

    @Override
    public void stop(){
        cthread.stopThread();
    }

    private class CThread extends Thread{
        volatile boolean stop = false;
        @Override
        public void run() {
            while(!isInterrupted()&&!stop){
                drive.update();
                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
                telemetry.update();
            }
        }
        public void stopThread(){
            stop = true;
        }
    }
}
