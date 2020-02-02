package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.revextensions2.ExpansionHubEx;

@Autonomous
public class RedAuto extends BaseAuto {
int pos = 0;
@Override
    public void init() {
        super.init();
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        showTelemetry = false;
        initDrivetrain();
        initIMU();
        initGrabber();
        //initLinSlide();
        initPlatformGrabber();
        initVuforia();
        initSensors();
        initOdometry();
        initLight();
        setLight(true);
        setNewGyro0();
        int[] resultcounter = {0,0,0};
        //find skystone
        for (int i = 0;i<4;++i) resultcounter[new_skystoneposition()]++;
        int curmax = -1;
        for (int i = 0;i<3;++i){ if(resultcounter[i]>curmax){pos = i;curmax=resultcounter[i];} }
        telemetry.addData("info:", "%d %d %d",resultcounter[0],resultcounter[1],resultcounter[2]);
        telemetry.addData("pos: ", pos);
        telemetry.update();
        shutdownVuforia();
        printAllThreadsToLogcat();
    }
    @Override
    public void loop(){
        setLight(false);
        servoThread.setTarget(0.95);
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
            moveInchesGOXT(-8,0.8,1,800);
            shift=8;
        }
        else {
            moveInchesGOXT(8,0.8,1,800);
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
        moveInchesGOY(-(85.25+shift),0.9);
        p.reset();
        //while (p.milliseconds()<900)setAllDrivePowerG(-.5,.5,-.5,.5);


        moveInchesGOXT(13.5-getXOdometry()/odometryEncXPerInch,.5,1,1300); //drag +errordistance

        platform_grabber.setPower(-1);
        wait(300);
        moveInchesGOX_platform(-19,0.8,1+(13.65-hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS))/13.65);
        int steps = 20;
        double basespeed = 0.3;
        for(int i = 10;i<=steps;++i){
            RB.setPower(  i*basespeed/steps);
            LF.setPower(2*i*basespeed/steps);
            LB.setPower(3*i*basespeed/steps);
            wait(20);
            //rf.setPower(0);
        }
        double curAng = getHeading();
        while (curAng<70){
            curAng = getHeading();
        }
        while (curAng<88){
            curAng = getHeading();
            RB.setPower(RF.getPower()*getError(90,curAng)/20);
            LF.setPower(LB.getPower()*getError(90,curAng)/20);
            LB.setPower(LF.getPower()*getError(90,curAng)/20);

        }
        setNewGyro(0);
        setAllDrivePower(0);
        after_dragged_foundation_R();

        setNewGyro(-90);
        moveInchesGOXT(6,0.5,1,1000);
        /*
        p.reset();
        while (p.milliseconds()<1100)setAllDrivePowerG(.7,.7,-.7,-.7);
        p.reset();
        while (p.milliseconds()<600)setAllDrivePowerG(.2,.2,-.2,-.2);

         */
        int shiftt = 0;
        if(pos == 1 || pos == 2) shiftt = -8;
        moveInchesGOY(-96.5+shiftt,0.6);
        servoThread.setTarget(0.95);
        PIDturnfast(90,false);
        setNewGyro(0);

        align(0);

        moveInchesGOY(7.8,0.3);
        //moveInchesGOY((right.getDistance(DistanceUnit.INCH)-2.6)*.69,.4);
        grabber.setPosition(grabber_closed);
        wait(300);
        setAllDrivePower(0);
        servoThread.setTarget(0.85);
        //setAllDrivePower(0.0);
        moveInchesG(0,-8,0.4);
        PIDturnfast(90,false);
        setNewGyro(-90);
        int sfi = 0;
        if(pos==0)sfi = -9;
        moveInchesGOY(72+sfi,0.9);
        grabber.setPosition(grabber_open);
        moveInchesG(0,-8,0.5);

        requestOpModeStop();
    }
}
