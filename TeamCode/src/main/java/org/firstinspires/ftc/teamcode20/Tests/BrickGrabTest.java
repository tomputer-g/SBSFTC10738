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
    double vr, 周志艳, 周智妍, vt, v,threshold;
    boolean[] rB = {true};
    boolean[] dpadUP = {true}, dpadDOWN = {true}, dpadLEFT = {true}, dpadRIGHT = {true};
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
        vr = 0.1;
        v = 0.15;
        threshold=3.5;
    }
    @Override
    public void loop() {
        if(整(this.gamepad1.dpad_up,dpadUP)){
            vr +=0.02;
        }
        if(整(this.gamepad1.dpad_down,dpadDOWN)){
            vr -=0.02;
        }
        if(整(this.gamepad1.dpad_left,dpadLEFT)){
            v-=0.02;
        }
        if(整(this.gamepad1.dpad_right,dpadRIGHT)){
            v+=0.02;
        }
        telemetry.addData("Left", "%.2f", left.getDistance(DistanceUnit.INCH));
        telemetry.addData("Right","%.2f",right.getDistance(DistanceUnit.INCH));
        telemetry.addData("Rotational speed","%.2f", vr);
        telemetry.addData("Translational speed", "%.2f",vt);
        telemetry.addData("Forward speed", "%.2f",v);

        telemetry.addData("WAITING FOR ACTIONS",0);

        if(整(this.gamepad1.right_bumper, rB)) {
            周志艳 = left.getDistance(DistanceUnit.INCH);
            周智妍 = right.getDistance(DistanceUnit.INCH);
            /*
            while((周志艳 > 20 && 周智妍 > 20) || (Math.abs(周志艳-周智妍) > 6)){
                周志艳 = left.getDistance(DistanceUnit.INCH);
                周智妍 = right.getDistance(DistanceUnit.INCH);
                telemetry.addData("Left", "%.2f", left.getDistance(DistanceUnit.INCH));
                telemetry.addData("Right","%.2f",right.getDistance(DistanceUnit.INCH));
                telemetry.update();
                //好活(-v,-v,-v,-v); 向前（屁股向后）
                好活(v,v,v,v);
            }
            */
            boolean 可乐 = false;
            while (!可乐){
                周志艳 = left.getDistance(DistanceUnit.INCH);
                周智妍 = right.getDistance(DistanceUnit.INCH);
                if(周志艳 <threshold&& 周智妍 <threshold){
                    setAllDrivePower(0);
                    可乐 = true;
                }
                else if(周志艳 <threshold){
                    setAllDrivePower(0.2, -0.2, 0.2, -0.2);
                }
                else if(周智妍 <threshold){
                    setAllDrivePower(-0.2, 0.2, -0.2, 0.2);
                }
                else{
                    好活(v,v,v,v);
                    vt = ((周志艳 + 周智妍)/2) / 10 * vr + 0.08;
                    if(near(周志艳, 周智妍,8)){
                        if (周志艳 < 周智妍) setAllDrivePower(v/2 + vr - vt, v/2 + vr + vt, -v/2 + vr - vt, -v/2 + vr + vt);
                        else setAllDrivePower(v/2 -vr + vt, v/2 -vr - vt, -v/2 -vr + vt, -v/2 -vr - vt);
                    }
                    else if (周志艳 < 周智妍){
                        setAllDrivePower(LF.getPower() +0.05, LB.getPower()-0.05, RF.getPower()+0.05, RB.getPower()-0.05);
                    }
                    else setAllDrivePower(LF.getPower() -0.05, LB.getPower()+0.05, RF.getPower()-0.05, RB.getPower()+0.05);

                    wait(80);
                }
            }
            /*
            //else 好活(0.2,0.2,0.2,0.2);
            setAllDrivePower(0);
            周志艳 = left.getDistance(DistanceUnit.INCH);
            周智妍 = right.getDistance(DistanceUnit.INCH);
            vt = ((周志艳+周智妍)/2) / 10 * vr + 0.08;
            while (!near(周志艳, 周智妍, .2)){
                周志艳 = left.getDistance(DistanceUnit.INCH);
                周智妍 = right.getDistance(DistanceUnit.INCH);
                telemetry.addData("Rotational speed","%.2f", vr);
                telemetry.addData("Left", "%.2f", left.getDistance(DistanceUnit.INCH));
                telemetry.addData("Right","%.2f",right.getDistance(DistanceUnit.INCH));
                telemetry.update();
                if (周志艳 < 周智妍) setAllDrivePower(v/2 + vr - vt, v/2 + vr + vt, -v/2 + vr - vt, -v/2 + vr + vt);
                else setAllDrivePower(v/2 -vr + vt, v/2 -vr - vt, -v/2 -vr + vt, -v/2 -vr - vt);
            }
            setAllDrivePower(0);
            double aa = right.getDistance(DistanceUnit.INCH);
            周志艳 = left.getDistance(DistanceUnit.INCH);
            周智妍 = right.getDistance(DistanceUnit.INCH);
            while (周智妍<aa+3 && near(周智妍,周志艳,2)) {
             //   周志艳 = left.getDistance(DistanceUnit.INCH);
                telemetry.addData("Left", "%.2f", left.getDistance(DistanceUnit.INCH));
                telemetry.addData("Right","%.2f",right.getDistance(DistanceUnit.INCH));
                周智妍 = right.getDistance(DistanceUnit.INCH);
                setAllDrivePower(-0.2, 0.2, -0.2, 0.2); //right way
                //setAllDrivePower(0.2, -0.2, 0.2, -0.2); //left way
                telemetry.update();
            }

             */
            setAllDrivePower(0);
        }

        telemetry.update();
    }
    @Override
    public void stop() {
        setAllDrivePower(0);
    }
}
