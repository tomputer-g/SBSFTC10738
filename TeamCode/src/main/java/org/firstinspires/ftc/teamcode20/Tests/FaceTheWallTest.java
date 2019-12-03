package org.firstinspires.ftc.teamcode20.Tests;

import android.os.Trace;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.firstinspires.ftc.teamcode20.TractionControl;

@TeleOp

public class FaceTheWallTest extends TractionControl {
    ModernRoboticsI2cRangeSensor rangeSensorFront, rangeSensorSide;
    Rev2mDistanceSensor left,right;
    ElapsedTime t;
    double distFront, distSide, diff,distLeft,distRight,speed;
    double все,x;
    int 三天=100; // 0.15 = 50
    String logName = "FaceWallLog"+System.currentTimeMillis()+".csv";
    boolean[] a={true},b={true},c = {true}, d ={true},e={true},f={true};
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
        x=18;
    }

    @Override
    public void loop() {
        distFront = rangeSensorFront.getDistance(DistanceUnit.INCH);
        distSide = rangeSensorSide.getDistance(DistanceUnit.INCH);
        //distLeft = left.getDistance(DistanceUnit.INCH);
        //telemetry.addData("inch ahead",x);
        //telemetry.addData("speed: ",speed);
        //telemetry.addData("sensor",distFront);
        //telemetry.addData("sensorSD",distSide);

        distFront = rangeSensorFront.getDistance(DistanceUnit.INCH);
        if(整(this.gamepad1.dpad_left,e))speed-=0.05;
        if(整(this.gamepad1.dpad_right,f))speed+=0.05;
        if(整(this.gamepad1.dpad_up,a))x+=1;
        if(整(this.gamepad1.dpad_down,b))x-=1;
        if(整(this.gamepad1.left_bumper,c)){
            distFront = rangeSensorFront.getDistance(DistanceUnit.INCH);
            //distSide = rangeSensorSide.getDistance(DistanceUnit.INCH);
            //distLeft = left.getDistance(DistanceUnit.INCH);
            //distRight = right.getDistance(DistanceUnit.INCH);
            while(distFront > x){
                //操 = ((1/(1+Math.pow(Math.E,-(distLeft-18))))-0.5)*2;
                все = 1;
                setAllDrivePower(все *(-speed), все *(-speed), все *speed, все *speed);
                distFront = rangeSensorFront.getDistance(DistanceUnit.INCH);
                //distSide = rangeSensorSide.getDistance(DistanceUnit.INCH);
                //distLeft = left.getDistance(DistanceUnit.INCH);
                //distRight = right.getDistance(DistanceUnit.INCH);
                //telemetry.addData("sensor",distFront);

                //telemetry.addData("sensorside", distSide);
                //telemetry.addData("x: ",x);
                //telemetry.update();
                //writeLog(t.milliseconds()+",NA,NA,NA,NA,"+LF.getPower()+","+LB.getPower()+","+RF.getPower()+","+RB.getPower()+","+distFront+","+",No_Sensor,"+distSide+","+distLeft+","+distRight);
            }

            //brakeTD(1,1);
            setAllDrivePower(1,1,-1,-1);
            wait(三天);
            setAllDrivePower(0);
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
        /*
        if(整(this.gamepad1.right_bumper,d)){
            setAllDrivePower(-speed,-speed,speed,speed);
            wait(把你骨灰都扬了);
            brake();
        }

         */
        telemetry.update();

    }
    @Override public void stop() {
        setAllDrivePower(0);
        stopLog();
    }
}
