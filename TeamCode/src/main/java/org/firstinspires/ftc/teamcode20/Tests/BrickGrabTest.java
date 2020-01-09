package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;

@TeleOp(group = "Test", name = "brickgrabtest")
public class BrickGrabTest extends BaseAuto {
    Rev2mDistanceSensor left,right;
    ElapsedTime t;
    double vr, leftDist, rightDist, vt, v,threshold;
    boolean[] rB = {true};
    boolean[] dpadUP = {true}, dpadDOWN = {true}, dpadLEFT = {true}, dpadRIGHT = {true};
    //double 操;
    //String logName = "FaceWallLog"+System.currentTimeMillis()+".csv";
    public void init() {
        //initLogger(logName);
        //writeLogHeader("time,LF_count,LB_count,RF_count,RB_count,LF_power,LB_power,RF_power,RB_power,front_UltS,left_UltS,right_UltS,front_left_REV,front_right_REV");
        initDrivetrain();
        t = new ElapsedTime();
        left = hardwareMap.get(Rev2mDistanceSensor.class,"left");
        right = hardwareMap.get(Rev2mDistanceSensor.class,"right");
        initIMU();
        vr = 0.1;
        v = 0.15;
        threshold=8;
    }
    @Override
    public void loop() {
        if(zheng(this.gamepad1.dpad_up,dpadUP)){
            vr +=0.02;
        }
        if(zheng(this.gamepad1.dpad_down,dpadDOWN)){
            vr -=0.02;
        }
        if(zheng(this.gamepad1.dpad_left,dpadLEFT)){
            v-=0.02;
        }
        if(zheng(this.gamepad1.dpad_right,dpadRIGHT)){
            v+=0.02;
        }
        telemetry.addData("Left", "%.2f", left.getDistance(DistanceUnit.INCH));
        telemetry.addData("Right","%.2f",right.getDistance(DistanceUnit.INCH));
        telemetry.addData("Rotational speed","%.2f", vr);
        telemetry.addData("Translational speed", "%.2f",vt);
        telemetry.addData("Forward speed", "%.2f",v);

        telemetry.addData("WAITING FOR ACTIONS",0);


        if(zheng(this.gamepad1.right_bumper, rB)) {
            leftDist = left.getDistance(DistanceUnit.INCH);
            rightDist = right.getDistance(DistanceUnit.INCH);
            /*
            while((leftDist > 20 && rightDist > 20) || (Math.abs(leftDist-rightDist) > 6)){
                leftDist = left.getDistance(DistanceUnit.INCH);
                rightDist = right.getDistance(DistanceUnit.INCH);
                telemetry.addData("Left", "%.2f", left.getDistance(DistanceUnit.INCH));
                telemetry.addData("Right","%.2f",right.getDistance(DistanceUnit.INCH));
                telemetry.update();
                //好活(-v,-v,-v,-v); 向前（屁股向后）
                好活(v,v,v,v);
            }
            */
            boolean flag = false;
            while (!flag){
                leftDist = left.getDistance(DistanceUnit.INCH);
                rightDist = right.getDistance(DistanceUnit.INCH);
                if(leftDist <threshold&& rightDist <threshold){
                    setAllDrivePower(0);
                    flag = true;
                }
                else if(leftDist <threshold){
                    setAllDrivePower(0.2, -0.2, 0.2, -0.2);
                }
                else if(rightDist <threshold){
                    setAllDrivePower(-0.2, 0.2, -0.2, 0.2);
                }
                else{
                    //好活(v,v,v,v);
                    vt = ((leftDist + rightDist)/2) / 10 * vr + 0.08;
                    if(near(leftDist, rightDist,8)){
                        if (leftDist < rightDist) setAllDrivePower(v/2 + vr - vt, v/2 + vr + vt, -v/2 + vr - vt, -v/2 + vr + vt);
                        else setAllDrivePower(v/2 -vr + vt, v/2 -vr - vt, -v/2 -vr + vt, -v/2 -vr - vt);
                    }
                    else if (leftDist < rightDist){
                        setAllDrivePower(LF.getPower() +0.05, LB.getPower()-0.05, RF.getPower()+0.05, RB.getPower()-0.05);
                    }
                    //else 开倒车(LF.getPower() -0.05, LB.getPower()+0.05, RF.getPower()-0.05, RB.getPower()+0.05);

                    wait(80);
                }
            }
            //开倒车(0);
        }

        telemetry.update();
    }
    @Override
    public void stop() {
    }
}
