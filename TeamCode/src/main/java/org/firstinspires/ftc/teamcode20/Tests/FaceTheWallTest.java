package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;

@TeleOp

public class FaceTheWallTest extends BaseAuto {
    ModernRoboticsI2cRangeSensor rangeSensorFront, rangeSensorSide;
    Rev2mDistanceSensor left,right;
    ElapsedTime t;
    double distFront, distSide, diff,distLeft,distRight,speed;
    double indicator;
    String logName = "FaceWallLog"+System.currentTimeMillis()+".csv";
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
        telemetry.addLine("vr: "+speed);
        telemetry.update();
        }

    @Override
    public void loop() {

        //distLeft = left.getDistance(DistanceUnit.INCH);
        //telemetry.addData("inch ahead",distLeft);


        if(this.gamepad1.left_bumper){
            while(this.gamepad1.left_bumper);
            distFront = rangeSensorFront.getDistance(DistanceUnit.INCH);
            distSide = rangeSensorSide.getDistance(DistanceUnit.INCH);
            distLeft = left.getDistance(DistanceUnit.INCH);
            distRight = right.getDistance(DistanceUnit.INCH);
            while(distFront > 16.5){
                //indicator = ((1/(1+Math.pow(Math.E,-(distLeft-18))))-0.5)*2;
                indicator = 1;
                setAllDrivePower(indicator*(-speed),indicator*(-speed),indicator*speed,indicator*speed);
                distFront = rangeSensorFront.getDistance(DistanceUnit.INCH);
                distSide = rangeSensorSide.getDistance(DistanceUnit.INCH);
                distLeft = left.getDistance(DistanceUnit.INCH);
                distRight = right.getDistance(DistanceUnit.INCH);
                telemetry.addData("inch",distFront);
                telemetry.update();
                writeLog(t.milliseconds()+",NA,NA,NA,NA,"+LF.getPower()+","+LB.getPower()+","+RF.getPower()+","+RB.getPower()+","+distFront+","+",No_Sensor,"+distSide+","+distLeft+","+distRight);
            }
            setAllDrivePower(0);
            brake();
              //wait(200);

            //imuHeading=0;
            //turn(90,0.05,3);

            //imuHeading=0;
            //while(!near(imuHeading,90,2)){
            //    setAllDrivePower(0.5,0.5,-0.5,-0.5);
            //}


            //while(!(near(distLeft,18,1))){
            //    distFront = rangeSensorFront.getDistance(DistanceUnit.INCH);
            //    distSide = rangeSensorSide.getDistance(DistanceUnit.INCH);
            //   diff = Math.min(0.2,Math.max((distSide-12)/10,-0.2))/2;
            //   distLeft = left.getDistance(DistanceUnit.INCH);
            //   setAllDrivePower(-0.1,-0.1,0.1,0.1);
            //}
        }
        if(this.gamepad1.right_bumper){
            while(this.gamepad1.right_bumper);
            setAllDrivePower(-speed,-speed,speed,speed);
            wait(1500);
            brake();
        }

    }
    @Override public void stop() {
        setAllDrivePower(0);
        stopLog();
    }
}
