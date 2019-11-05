package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;

@TeleOp(name = "闲话终日有 干就自然无", group = "Tests")

public class BrickGrabTest extends BaseAuto {
    Rev2mDistanceSensor left,right;
    ElapsedTime t;
    double speed,dl,dr;
    boolean rBprimed=false;
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
        if(gamepad1.dpad_up){
            while (gamepad1.dpad_up);
            speed+=0.05;
        }
        if(gamepad1.dpad_down){
            while (gamepad1.dpad_down);
            speed-=0.05;
        }
        telemetry.addData("speed",speed);
        telemetry.addData("WAITING FOR ACTIONS",0);
        if(this.gamepad1.right_bumper){
            rBprimed = true;
            setAllDrivePower(0);
            telemetry.addData("开整",0);
            dl=left.getDistance(DistanceUnit.INCH);
            dr=right.getDistance(DistanceUnit.INCH);
            while(!near(dl,dr,.5)){
                dl=left.getDistance(DistanceUnit.INCH);
                dr=right.getDistance(DistanceUnit.INCH);
                if(dl<dr)setAllDrivePower(speed-0.2,speed+0.2,speed-0.2,speed+0.2);
                else setAllDrivePower(-speed+0.2,-speed-0.2,-speed+0.2,-speed-0.2);
            }
            setAllDrivePower(0);
        }
        if(!this.gamepad1.right_bumper && rBprimed)rBprimed=false;
        /**
        if(this.gamepad1.right_bumper){
            while(gamepad1.right_bumper);
            setAllDrivePower(0);
            telemetry.addData("开整",0);
            dl=left.getDistance(DistanceUnit.INCH);
            dr=right.getDistance(DistanceUnit.INCH);
            while(!near(dl,dr,.5)){
                dl=left.getDistance(DistanceUnit.INCH);
                dr=right.getDistance(DistanceUnit.INCH);
                if(dl<dr)setAllDrivePower(speed+0.5,speed-0.5,speed+0.5,speed-0.5);
                else setAllDrivePower(-speed-0.5,-speed+0.5,-speed-0.5,-speed+0.5);
            }
            setAllDrivePower(0);
            /*
            double a=left.getDistance(DistanceUnit.INCH);
            dr=right.getDistance(DistanceUnit.INCH);
            while(dr>a){
                dl=left.getDistance(DistanceUnit.INCH);
                dr=right.getDistance(DistanceUnit.INCH);
                setAllPDrivePower1(-speed,speed,-speed,speed);
            }
            setAllDrivePower(0);
            */
            telemetry.update();
        //}

        telemetry.update();
    }
    @Override
    public void stop() {
        setAllDrivePower(0);
    }
}
