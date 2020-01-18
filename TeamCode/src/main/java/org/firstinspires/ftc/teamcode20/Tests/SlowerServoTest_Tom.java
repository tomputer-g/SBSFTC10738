package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseOpMode;
@Disabled

@TeleOp
public class SlowerServoTest_Tom extends BaseOpMode {
    private Servo servo1, servo2;
    private boolean xP = false, yP = false, mxbRunning = false;
    private int delayMS = 1000;
    private double m, b;//m is actual change, b is initial pos
    private ElapsedTime t;

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        setDoubleServoPosition(0.5);
        t = new ElapsedTime();
    }

    @Override public void loop() {
        if(this.gamepad1.x){xP = true;}if(xP && !this.gamepad1.x){xP = false;
            //from 0 to 90
            b = 1;
            m = -0.5;
            t.reset();
            mxbRunning = true;
        }
        if(this.gamepad1.y){yP = true;}if(yP && !this.gamepad1.y){yP = false;
            b = 0.5;
            m = 0.5;
            t.reset();
            mxbRunning = true;
        }
        if(this.gamepad1.a){
            mxbRunning = false;
            setDoubleServoPosition(1);
        }
        if(this.gamepad1.b){
            mxbRunning = false;
            setDoubleServoPosition(0.5);
        }
        runServomxb();
    }

    private void runServomxb(){
        if(mxbRunning){
            setDoubleServoPosition(b + m * (Math.min(1, t.milliseconds() / delayMS)));
        }
        if(t.milliseconds() / delayMS >= 1.0){
            mxbRunning = false;
        }
    }


    ElapsedTime ServoRunTime;
    private void runExtenderServos(double to, double timePerStep){
        if(to != grabber_extend1.getPosition()){
            //move by 0.01 until reached
            if(to > grabber_extend1.getPosition()){
                //down
                setDoubleServoPosition(grabber_extend1.getPosition() + 0.01);
                
            }
        }
    }

    private void setDoubleServoPosition(double position){
        servo1.setPosition(position);
        servo2.setPosition(1-position);
    }


/*

    @Override
    public void loop() {
        if(this.gamepad1.x){xP = true;}if(xP && !this.gamepad1.x) {xP = false;
            for(double pos = 0.02; pos <= 0.5; pos += 0.02){
                try {
                    wait((int)(delayMS/(0.5/0.02)));
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                servo.setPosition(pos);
            }
        }
        if(this.gamepad1.y){yP = true;}if(yP && !this.gamepad1.y){yP = false;
            for(double pos = 0.48; pos >= 0; pos -= 0.02){
                try {
                    wait((int)(delayMS/(0.5/0.02)));
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                servo.setPosition(pos);
            }
        }
    }

 */

}
