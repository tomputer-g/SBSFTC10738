package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode20.BaseOpMode;

@Autonomous
public class MultithreadTest extends BaseOpMode {
    Thread testThread;
    Servo servo1, servo2;

    //moves Servo1 from main and Servo2 from stick. Thanks to https://stemrobotics.cs.pdx.edu/node/5184

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        testThread = new TestThread();
    }

    @Override
    public void stop() {
        testThread.interrupt();
    }

    @Override
    public void start() {
        testThread.start();
    }

    @Override
    public void loop() {
        wait(2000);
        servo1.setPosition(0);
        wait(2000);
        servo1.setPosition(1);
    }

    private class TestThread extends Thread{
        @Override
        public void run() {
            while(!isInterrupted()) {
                servo2.setPosition((-gamepad1.left_stick_y+1)/2.0);
            }
        }
    }
}
