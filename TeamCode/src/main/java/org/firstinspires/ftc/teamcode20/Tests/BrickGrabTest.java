package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;

@TeleOp

public class BrickGrabTest extends BaseAuto {
    Rev2mDistanceSensor left,right;
    ElapsedTime t;
    double speed,dl,dr;
    //double indicator;
    //String logName = "FaceWallLog"+System.currentTimeMillis()+".csv";
    public void init() {
        //initLogger(logName);
        //writeLogHeader("time,LF_count,LB_count,RF_count,RB_count,LF_power,LB_power,RF_power,RB_power,front_UltS,left_UltS,right_UltS,front_left_REV,front_right_REV");
        initDrivetrain();
        t = new ElapsedTime();
        left = hardwareMap.get(Rev2mDistanceSensor.class,"left");
        right = hardwareMap.get(Rev2mDistanceSensor.class,"right");
        initIMU();
        speed = 0.15;
    }
    @Override
    public void loop() {
        if(gamepad1.right_bumper){
            while(gamepad1.right_bumper){}
            brake();
            while(!near(dl,dr,.8)){
                dl=left.getDistance(DistanceUnit.INCH);
                dr=right.getDistance(DistanceUnit.INCH);
                if(dl<dr)setAllDrivePower(-speed,-speed,-speed,-speed);
                else setAllDrivePower(speed,speed,speed,speed);
            }
            setAllDrivePower(0);
            while(dr<20){
                dl=left.getDistance(DistanceUnit.INCH);
                dr=right.getDistance(DistanceUnit.INCH);
                setAllPDrivePower1(-speed,speed,-speed,speed);
            }
            setAllDrivePower(0);
        }
    }
    @Override
    public void stop() {
        setAllDrivePower(0);
        stopLog();
    }
}
