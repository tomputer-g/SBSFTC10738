package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp

public class FaceTheWallTest extends BaseAuto {
    ModernRoboticsI2cRangeSensor rangeSensorFront, rangeSensorSide;
    Rev2mDistanceSensor left,right;
    ElapsedTime t;
    double distFront, distSide, diff,distLeft,distRight,speed;
    int indicator;
    @Override
    public void init() {

        initDrivetrain();
        t = new ElapsedTime();
        //rangeSensorFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front");
        //rangeSensorSide = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "side");
        left = hardwareMap.get(Rev2mDistanceSensor.class,"left");
        right = hardwareMap.get(Rev2mDistanceSensor.class,"right");
        initIMU();
        speed = 0.3;
    }

    @Override
    public void loop() {
        distLeft = left.getDistance(DistanceUnit.INCH);
            telemetry.addData("inch",distLeft);
            telemetry.update();
            if(this.gamepad1.dpad_up){
                while (this.gamepad1.dpad_up);
                speed+=0.05;
                telemetry.addData("speed",speed);
                telemetry.update();
            }
        if(this.gamepad1.dpad_down){
            while (this.gamepad1.dpad_down);
            speed-=0.05;
            telemetry.addData("speed",speed);
            telemetry.update();
        }
        if(this.gamepad1.left_bumper){
            while(this.gamepad1.left_bumper);
            //distFront = rangeSensorFront.getDistance(DistanceUnit.INCH);
            //distSide = rangeSensorSide.getDistance(DistanceUnit.INCH);
            distLeft = left.getDistance(DistanceUnit.INCH);
            //while(!(near(distLeft,12,0.3))){
              while(distLeft > 15.5){
                if(distLeft > 15.5) indicator = 1;
                else indicator = -1;
                setAllDrivePower(indicator*(-speed),indicator*(-speed),indicator*speed,indicator*speed);
                //distFront = rangeSensorFront.getDistance(DistanceUnit.INCH);
                //distSide = rangeSensorSide.getDistance(DistanceUnit.INCH);
                telemetry.addData("inch",distLeft);
                telemetry.update();
                distLeft = left.getDistance(DistanceUnit.INCH);
            }
              setAllDrivePower(0);
            //turn(90,0.1,3);
            /*
            imuHeading=0;
            while(!near(imuHeading,90,2)){
                setAllDrivePower(0.5,0.5,-0.5,-0.5);
            }
            */
/*
            while(!(near(distLeft,18,0.3))){
                //distFront = rangeSensorFront.getDistance(DistanceUnit.INCH);
                //distSide = rangeSensorSide.getDistance(DistanceUnit.INCH);
//                diff = Math.min(0.2,Math.max((distSide-12)/10,-0.2))/2;
                distLeft = left.getDistance(DistanceUnit.INCH);
                setAllDrivePower(-0.1,-0.1,0.1,0.1);
            }
*/
        }

    }
}
