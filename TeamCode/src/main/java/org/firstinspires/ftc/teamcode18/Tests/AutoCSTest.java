package org.firstinspires.ftc.teamcode18.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode18.BaseAuto;

/**
 * Created by Ziming Gao on 1/23/2018.
 */
@Autonomous(name = "test red")
@Disabled
public class AutoCSTest extends BaseAuto {
    @Override
    public void init() {
        CSArm = hardwareMap.get(Servo.class,"csarm");
        cs = hardwareMap.get(ColorSensor.class, "cs");
        cs.enableLed(true);
        CSArm.setPosition(ARM_HOME);
    }

    @Override
    public void loop() {
        if(!autonomousDone){
            CSArm.setPosition(ARM_GOAL-0.1);
            wait(800);
            CSArm.setPosition(ARM_GOAL);
            wait(500);
            CSDetect();
            autonomousDone = true;
        }
    }

    @Override
    public void stop() {

    }
}
