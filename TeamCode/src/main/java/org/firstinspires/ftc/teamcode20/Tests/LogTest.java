package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;

public class LogTest extends BaseAuto {
    ModernRoboticsI2cRangeSensor rangeSensorFront, rangeSensorSide;
    Rev2mDistanceSensor left,right;
    ElapsedTime t;
    double distFront, distSide,distLeft,distRight,speed;
    int started = 0;
    String logName = "LogTestLog"+System.currentTimeMillis()+".csv";
    boolean dpadUPrimed =false, dpadDPrimed = false, leftBPrimed = false;

    public void init() {
        initLogger(logName);
        writeLogHeader("time,LF_count,LB_count,RF_count,RB_count,LF_power,LB_power,RF_power,RB_power,front_UltS,left_UltS,right_UltS,front_left_REV,front_right_REV");
        initDrivetrain();
        t = new ElapsedTime();
        rangeSensorSide = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "side");
        rangeSensorFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front");
        left = hardwareMap.get(Rev2mDistanceSensor.class,"left");
        right = hardwareMap.get(Rev2mDistanceSensor.class,"right");
        initIMU();
        speed = 0.3;
    }

    @Override
    public void init_loop() {
        if(this.gamepad1.dpad_up){
            dpadUPrimed = true;
            speed+=0.05;
        }
        if(!this.gamepad1.dpad_up && dpadUPrimed)dpadUPrimed=false;
        if(this.gamepad1.dpad_down){
            dpadDPrimed = true;
            speed-=0.05;
        }
        if(!this.gamepad1.dpad_down && dpadDPrimed)dpadDPrimed=false;
        telemetry.addLine("vr: "+speed);
        distLeft = left.getDistance(DistanceUnit.INCH);
        telemetry.addData("inch ahead",distFront);
    }

    @Override
    public void loop() {
        if(this.gamepad1.left_bumper){
            leftBPrimed = true;
            started++;
        }
        if(!this.gamepad1.left_bumper && leftBPrimed)leftBPrimed=false;
        if(started==1)setAllDrivePower(-speed,-speed,speed,speed);
        else if(started >=2){
            setAllDrivePower(0);
            brake();
        }
        distFront = rangeSensorFront.getDistance(DistanceUnit.INCH);
        distSide = rangeSensorSide.getDistance(DistanceUnit.INCH);
        distLeft = left.getDistance(DistanceUnit.INCH);
        distRight = right.getDistance(DistanceUnit.INCH);
        telemetry.addData("inch ahead",distFront);
        telemetry.addData("inch ahead",distSide);
        telemetry.addData("inch ahead",distLeft);
        telemetry.addData("inch ahead",distRight);
        writeLog(t.milliseconds()+",NA,NA,NA,NA,"+LF.getPower()+","+LB.getPower()+","+RF.getPower()+","+RB.getPower()+","+distFront+","+",No_Sensor,"+distSide+","+distLeft+","+distRight);
    }


    @Override public void stop() {
        setAllDrivePower(0);
        stopLog();
    }
}
