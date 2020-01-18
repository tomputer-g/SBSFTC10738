package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode20.BaseOpMode;

@TeleOp
public class OldDummyBot extends BaseOpMode {
    Servo servo;
    boolean[] B = {false};
    @Override
    public void init() {
        initDrivetrain();
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(.4);
    }

    @Override
    public void loop() {
        if(checkButton(this.gamepad1.b,B)){
            if(servo.getPosition() < 0.2){
                servo.setPosition(.4);
            }else{
                servo.setPosition(0);
            }
        }
        move(-this.gamepad1.left_stick_x, this.gamepad1.left_stick_y, -this.gamepad1.right_stick_x);
    }


    protected static boolean checkButton(boolean b, boolean[] f){
        if(b||!f[0]){
            if(b)f[0]=false;
            else f[0]=true;
            if(f[0])return true;
        }
        return false;
    }
}
