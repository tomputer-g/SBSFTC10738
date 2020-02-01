package org.firstinspires.ftc.teamcode20;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.ExpansionHubEx;

import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;

@Autonomous
public class BlueAuto extends BaseAuto {
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
        //cooThread.start();
    }
    @Override
    public void loop() {
        //go forward 1 floor mat (24")w
        //vuforia - recognize block & move to pick up
        //after pickup: turn 90 deg. move to platform, drop off
        //move to platform, drag into position, release
        //repeat until run out of time; first on other skystones

        //initialization
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
            moveInchesG(-8,0,0.5);
            shift=-8;
        }
        else {
            moveInchesG(8, 0,0.5);
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
        //turn(90, 0.5,1);
        PIDturnfast(90,false);
        setNewGyro(90);
        p.reset();
        resetXOdometry();
        moveInchesGOY((86.75+shift),0.9);
        p.reset();
        //while (p.milliseconds()<900)setAllDrivePowerG(-.5,.5,-.5,.5);


        moveInchesGOX(13.5-getXOdometry()/odometryEncXPerInch,.5); //drag +errordistance

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
        after_dragged_foundation();

        setNewGyro(90);
        moveInchesGOX(-6,0.5);
        p.reset();
        while (p.milliseconds()<1100)setAllDrivePowerG(.7,.7,-.7,-.7);
        p.reset();
        while (p.milliseconds()<600)setAllDrivePowerG(.2,.2,-.2,-.2);
        //moveInchesGOY(-104,0.9);
        servoThread.setTarget(0.95);
        PIDturnfast(-90,false);
        setNewGyro(0);

        p.reset();
        while (p.milliseconds()<500)setAllDrivePowerG(-.5,.5,-.5,.5);

        int shiftt = -4;
        if(pos==0)shiftt=-12;
        moveInchesGOX(shiftt,0.8);

        //setAllDrivePowerG(-.5,-.5,.5,.5);
        //wait(300);
        moveInchesGOY(9,0.3);
        //moveInchesGOY((right.getDistance(DistanceUnit.INCH)-2.6)*.69,.4);
        grabber.setPosition(grabber_closed);
        wait(300);
        setAllDrivePower(0);
        servoThread.setTarget(0.85);
        //setAllDrivePower(0.0);
        moveInchesG(0,-13,0.4);
        PIDturnfast(90,false);
        setNewGyro(90);
        moveInchesGOY(72,0.6);
        grabber.setPosition(grabber_open);
        moveInchesG(0,-8,0.5);

        requestOpModeStop();
        //telemetry.addData()
        /*


        //turn foundation
        platform_grabber.setPower(-.8);
        wait(200);
        setAllDrivePowerG(0.55,-0.55,0.55,-0.55);
        wait(1000);
        setAllDrivePower(0);
        turn(90, 0.8, 4);
        //drag foundation forward
        setNewGyro(180);

        //double koe=0.75;
        //p.reset();
        //while(10<rangeSensorFront.getDistance(DistanceUnit.INCH) || p.milliseconds() < 3400){
        //    setAllDrivePowerG(koe*(0.25-0.55+0.37),koe*(0.25-0.55-0.37),koe*(0.25+0.55+0.37),koe*(0.22+0.5-0.37)); //turn+f0rwrd+side
        //}

        setAllDrivePower(0);
        double tempY = getY1Odometry();
        double targetdist = getY1Odometry()-15*1316;
        p = new ElapsedTime();
        while(getY1Odometry()>targetdist&&p.milliseconds()<2000)
            setAllDrivePowerG(-0.5,-0.5,0.5,0.5,2);
        setAllDrivePower(0);

        //push it in
        setAllDrivePowerG(-.7,.7,-.7,.7,2);
        wait(1000);

        //release grabber
        platform_grabber.setPower(1);
        wait(300);
        setAllDrivePower(0);
        platform_grabber.setPower(0.0);

        //strafe left to put the block

        servoThread.setTarget(0.5);
        moveInchesG(-6.5,0,0.5);
        turn(-90,0.4,1);
        setNewGyro(90);
        //holdSlide((int)slideEncoderPerInch/10);
        //wait(1000);
        //moveInchesG(0,4,0.43);
        setAllDrivePowerG(-0.4,-0.4,0.4,0.4);
        //wait(200);
        servoThread.setTarget(.8);
        wait(1300);
        setAllDrivePower(0.0);

        grabber.setPosition(grabber_open);
        platform_grabber.setPower(1);
        wait(300);
        //park
        moveInchesG(0,-38,0.3);

        setAllDrivePower(0.0);
        servoThread.stopThread();
        setNewGyro0();
        platform_grabber.setPower(0);

        */
    }
}