package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BaseOpMode;
@Autonomous(group = "Test")
public class LightTest extends BaseOpMode {
    Servo light;
    @Override
    public void init() {
        light = hardwareMap.get(Servo.class,"light");
        light.setPosition(1);
    }
}
