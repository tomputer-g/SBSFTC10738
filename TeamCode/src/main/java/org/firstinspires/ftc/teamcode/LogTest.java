package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp

public class LogTest extends BaseAuto {
    ModernRoboticsI2cRangeSensor rangeSensorFront, rangeSensorSide;
    Rev2mDistanceSensor left,right;
    ElapsedTime t;
    double distFront, distSide,distLeft,distRight,speed;
    boolean started = false;
    String logName = "LogTestLog"+System.currentTimeMillis()+".csv";
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
            while (this.gamepad1.dpad_up);
            speed+=0.05;
        }
        if(this.gamepad1.dpad_down){
            while (this.gamepad1.dpad_down);
            speed-=0.05;
        }
        telemetry.addLine("speed: "+speed);
        distLeft = left.getDistance(DistanceUnit.INCH);
        telemetry.addData("inch ahead",distFront);
    }

    @Override
    public void loop() {
        if(this.gamepad1.left_bumper)started = !started;
        if(started)setAllDrivePower(-speed,-speed,speed,speed);
        else setAllDrivePower(0);
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
