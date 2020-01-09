package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Ziming Gao on 1/16/2018.
 */
@TeleOp
public class ServoTest extends OpMode {
    boolean x,l,r;
    boolean[] X = {true}, L = {true}, R = {true};
    double p=0.5;
    private Servo servo;
    protected static boolean checkButton(boolean b, boolean[] f){
        //chzch butt on press
        //淦 --yeah
        //微笑着面对它
        if(b||!f[0]){
            if(b)f[0]=false;
            else f[0]=true;
            if(f[0])return true;
        }
        return false;
    }
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class,"servo");
        servo.setPosition(0.5);
    }

    @Override
    public void loop() {

        if(checkButton(this.gamepad1.x,X)){
            p+=1/60;
            if(p>1)p=1;
            servo.setPosition(p);
        }
        if(checkButton(this.gamepad1.y,R)){
            p-=1/60;
            if(p<0)p=0;
            servo.setPosition(p);
        }


        telemetry.addData("servo: ",servo.getPosition());
        telemetry.update();
    }
}
