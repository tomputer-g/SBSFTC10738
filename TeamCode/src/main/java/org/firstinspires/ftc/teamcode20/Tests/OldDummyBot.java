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
    public void runOpMode() throws InterruptedException {
        initDrivetrain();
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(.4);
        waitForStart();
        while(opModeIsActive()){
            if(checkButton(this.gamepad1.b,B)){
                if(servo.getPosition() < 0.2){
                    servo.setPosition(.4);
                }else{
                    servo.setPosition(0);
                }
            }
            move(-this.gamepad1.left_stick_x, this.gamepad1.left_stick_y, -this.gamepad1.right_stick_x);
        }
    }


    protected static boolean checkButton(boolean b, boolean[] f){
        if(b||!f[0]){
            f[0]= !b;
            return f[0];
        }
        return false;
    }
}
