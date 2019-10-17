package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Ziming Gao on 1/16/2018.
 */
@TeleOp
public class ServoTest extends OpMode {
    private Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class,"servo");
        servo.setPosition(0.5);
    }

    @Override
    public void loop() {

        if(this.gamepad1.x){
            servo.setPosition(0.5);
        }

        if(this.gamepad1.dpad_left){
            while(this.gamepad1.dpad_left);
            if (servo.getPosition() >= 0.05)
                servo.setPosition(servo.getPosition() - 0.05);
            if (servo.getPosition() < 0.05)
                servo.setPosition(0);

        }
        else if(this.gamepad1.dpad_right){
            while(this.gamepad1.dpad_right);
            if (servo.getPosition() <= 0.95)
                servo.setPosition(servo.getPosition() + 0.05);
            if (servo.getPosition() > 0.95)
                servo.setPosition(1);
        }
        telemetry.addData("servo: ",servo.getPosition());
        telemetry.update();
    }
}
