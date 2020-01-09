package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DoubleServoTest extends OpMode {
    private Servo servo1, servo2;

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo1.setPosition(0.5);
        servo2.setPosition(0.5);
    }

    @Override
    public void loop() {
        servo1.setPosition((1.0-this.gamepad1.left_stick_y)/2.0);
        servo2.setPosition(1.0-((1.0-this.gamepad1.left_stick_y)/2.0));
    }
}
