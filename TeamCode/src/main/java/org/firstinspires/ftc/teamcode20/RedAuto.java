package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.revextensions2.ExpansionHubEx;


@Autonomous
public class RedAuto extends BaseAuto {
    int pos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initAutonomous();

        while(!isStarted() && !isStopRequested()){
            pos = new_skystonepositionR();
            wait(200);
        }

        before_start();

        //shift to align to skystone
        int shift;
        if(pos == 1){ shift = 0; }
        else if (pos == 0){ moveInchesGOXT(-8,0.8,1,1200);shift=8; }
        else { moveInchesGOXT(8,0.8,1,1200);shift=-8; }

        //move forward to the skystone
        ElapsedTime p = new ElapsedTime();
        first_block();

        //move forward & approach foundation
        align(90);
        p.reset();
        resetXOdometry();
        int xx = getXOdometry();
        moveInchesGOY(-(85.25+shift),0.6,1);
        p.reset();

        moveInchesGOXT(10,.5,1,2500); //drag +errordistance

        platform_grabber.setPower(-1);
        wait(300);
        moveInchesGOX_platform(-15,1,1+(13.65-hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS))/13.65);
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

        second_and_more_R(pos, 1);

        //moveInchesGOY_XF_F(-44,0.6,1,(int) (getXOdometry() - (-38.5 - adjustToViewMark(false)[1]) * odometryEncXPerInch));
        moveInchesGOY(-44,0.6);
        requestOpModeStop();
    }
}
