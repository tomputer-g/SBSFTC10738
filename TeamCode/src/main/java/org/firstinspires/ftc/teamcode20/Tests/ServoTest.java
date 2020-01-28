package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Ziming Gao on 1/16/2018.
 */

@TeleOp
public class ServoTest extends OpMode{
    private boolean lP, rP;
    private Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class,"capstone");
        servo.setPosition(0.5);
    }

    @Override
    public void loop() {
        if(this.gamepad1.dpad_left){lP = true;}if(lP && !this.gamepad1.dpad_left){
            lP = false;
            servo.setPosition(Math.max(servo.getPosition() - 0.05, 0));
        }
        if(this.gamepad1.dpad_right){rP = true;}if(rP && !this.gamepad1.dpad_right) {
            rP = false;
            servo.setPosition(Math.min(servo.getPosition() + 0.05, 1));
        }
        telemetry.addData("servo: ",servo.getPosition());
        telemetry.update();
    }
}
