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
    double 操,x;
    int 把你骨灰都扬了 = 1500;
    String logName = "FaceWallLog"+System.currentTimeMillis()+".csv";
    boolean[] a={true},b={true},c = {true}, d ={true};
    public void init() {
        initLogger(logName);
        writeLogHeader("time,LF_count,LB_count,RF_count,RB_count,LF_power,LB_power,RF_power,RB_power,front_UltS,left_UltS,right_UltS,front_left_REV,front_right_REV");
        initDrivetrain();
        t = new ElapsedTime();
        //rangeSensorSide = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "side");
        rangeSensorFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front");
        left = hardwareMap.get(Rev2mDistanceSensor.class,"left");
        right = hardwareMap.get(Rev2mDistanceSensor.class,"right");
        扬();
        speed = 0.3;
        x=18;
    }

    @Override
    public void loop() {

        //distLeft = left.getDistance(DistanceUnit.INCH);
        telemetry.addData("inch ahead",x);

        if(整(this.gamepad1.dpad_up,a))x+=1;
        if(整(this.gamepad1.dpad_down,b))x-=1;
        if(整(this.gamepad1.left_bumper,c)){
            distFront = rangeSensorFront.getDistance(DistanceUnit.INCH);
            //distSide = rangeSensorSide.getDistance(DistanceUnit.INCH);
            distLeft = left.getDistance(DistanceUnit.INCH);
            distRight = right.getDistance(DistanceUnit.INCH);
            while(distFront > x){
                //操 = ((1/(1+Math.pow(Math.E,-(distLeft-18))))-0.5)*2;
                操 = 1;
                setAllDrivePower(操 *(-speed), 操 *(-speed), 操 *speed, 操 *speed);
                distFront = rangeSensorFront.getDistance(DistanceUnit.INCH);
                //distSide = rangeSensorSide.getDistance(DistanceUnit.INCH);
                distLeft = left.getDistance(DistanceUnit.INCH);
                distRight = right.getDistance(DistanceUnit.INCH);
                telemetry.addData("inch",distFront);
                telemetry.addData("x: ",x);
                telemetry.update();
                writeLog(t.milliseconds()+",NA,NA,NA,NA,"+LF.getPower()+","+LB.getPower()+","+RF.getPower()+","+RB.getPower()+","+distFront+","+",No_Sensor,"+distSide+","+distLeft+","+distRight);
            }
            setAllDrivePower(0);
            brakeTD(1,1);
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
        if(整(this.gamepad1.right_bumper,d)){
            setAllDrivePower(-speed,-speed,speed,speed);
            wait(把你骨灰都扬了);
            brake();
        }
        telemetry.update();

    }
    @Override public void stop() {
        setAllDrivePower(0);
        stopLog();
    }
}
