package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode19.BaseOpMode;
@Autonomous(group = "Test")
@Disabled
public class LightTest extends BaseOpMode {
    Servo light;
    @Override
    public void init() {
        light = hardwareMap.get(Servo.class,"light");
        light.setPosition(1);
    }
}
