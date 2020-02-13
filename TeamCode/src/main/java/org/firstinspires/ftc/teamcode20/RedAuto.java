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
int pos = 0, result;
@Override
    public void init() {
        initAutonomous();

    }
    @Override
    public void init_loop(){
        pos = new_skystoneposition();
        wait(200);
    }
    @Override
    public void loop(){
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
        moveInchesGOY(30.5,0.6,(1+(13.65-hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS))/13.65));
        //grab 1st block
        grabber.setPosition(grabber_closed);
        wait(300);
        servoThread.setTarget(0.85);
        //setAllDrivePower(0.0);
        moveInchesG(0,-6,0.4);

        //move forward & approach foundation
        //turn(90, 0.5,1);
        PIDturnfast(90,false);
        setNewGyro(90);
        p.reset();
        resetXOdometry();
        moveInchesGOY(-(85.25+shift),0.6);
        p.reset();
        //while (p.milliseconds()<900)setAllDrivePowerG(-.5,.5,-.5,.5);


        moveInchesGOXT(13.5-getXOdometry()/odometryEncXPerInch,.5,1,1200); //drag +errordistance

        platform_grabber.setPower(-1);
        wait(300);
        moveInchesGOX_platform(-19,1,1+(13.65-hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS))/13.65);
        int steps = 20;
        double basespeed = 0.33;
        for(int i = 10;i<=steps;++i){
            RB.setPower(  -i*basespeed/steps);
            LF.setPower(-2*i*basespeed/steps);
            LB.setPower(-3*i*basespeed/steps);
            wait(20);
            //rf.setPower(0);
        }
        double curAng = getHeading();
        while (curAng>-70){
            curAng = getHeading();
        }
        while (curAng>-81){
            curAng = getHeading();
            RB.setPower(RF.getPower()*getError(90,curAng)/20);
            LF.setPower(LB.getPower()*getError(90,curAng)/20);
            LB.setPower(LF.getPower()*getError(90,curAng)/20);

        }
        setNewGyro(0);
        setAllDrivePower(0);
        after_dragged_foundation_R();

        setNewGyro(-90);
        moveInchesGOXT(8,0.8,1,1500);
        /*
        p.reset();
        while (p.milliseconds()<1100)setAllDrivePowerG(.7,.7,-.7,-.7);
        p.reset();
        while (p.milliseconds()<600)setAllDrivePowerG(.2,.2,-.2,-.2);

         */
        int shiftt = 0;
        if(pos == 0 || pos == 1) shiftt = -8;
        moveInchesGOY(-96.5+shiftt,0.6);
        servoThread.setTarget(0.95);
        PIDturnfast(90,false);
        setNewGyro(0);

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

        requestOpModeStop();
    }
}
