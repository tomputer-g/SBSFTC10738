package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Yan_BlockGrabberTest extends OpMode {
    private Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(0.5);
    }

    @Override
    public void loop() {
        if(this.gamepad1.a){
            servo.setPosition(1);
        }else if(this.gamepad1.b){
            servo.setPosition(0.5);
        }
    }
}
