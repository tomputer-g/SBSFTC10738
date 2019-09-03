package org.firstinspires.ftc.teamcode18.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Ziming Gao on 1/16/2018.
 */
@TeleOp(name = "servo test", group = "test")
@Disabled
public class ServoTest extends OpMode {
    Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class,"csarm");
        servo.setPosition(0.5);
    }

    @Override
    public void loop() {
        if(this.gamepad1.dpad_left){
            while(this.gamepad1.dpad_left);
            if(servo.getPosition() >= 0.05)
                servo.setPosition(servo.getPosition() - 0.05);
            if(servo.getPosition() > 0 && servo.getPosition() < 0.05)
                servo.setPosition(0);
        }
        if(this.gamepad1.dpad_right){
            while(this.gamepad1.dpad_right);
            if(servo.getPosition() <= 0.95)
                servo.setPosition(servo.getPosition() + 0.05);
            if(servo.getPosition() > 0.95 && servo.getPosition() < 1)
                servo.setPosition(1);
        }
        telemetry.addData("servo position", servo.getPosition());
        telemetry.update();
    }
}
