package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode20.BaseOpMode;
@Disabled
@Autonomous
public class MultithreadTest extends BaseOpMode {
    Thread testThread;
    Servo servo1, servo2;

    //moves Servo1 from main and Servo2 from stick. Thanks to https://stemrobotics.cs.pdx.edu/node/5184



    @Override
    public void stop() {
        testThread.interrupt();
    }

    @Override
    public void start() {
        testThread.start();
    }

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        testThread = new TestThread();
    }

    @Override
    public void loop() {
        wait(1000);
        servo1.setPosition(0);
        wait(1000);
        servo1.setPosition(1);
        telemetry.addData("Test thread is alive?",testThread.isAlive());
        telemetry.addData("Test thread is interrupted?", testThread.isInterrupted());
        telemetry.addLine("State: "+testThread.getState());
        telemetry.update();
    }

    private class TestThread extends Thread{
        volatile Boolean bStop = false;//volatile means the program always reads it from the main memory, as opposed to cache
        @Override
        public void run() {
            while(!isInterrupted() && !bStop) {
                servo2.setPosition((-gamepad1.left_stick_y+1)/2.0);
                if(gamepad1.b){
                    bStop = true;//makes thread alive = false & Thread.State TERMINATED
                }
            }
        }
        public void stopThread(){
            bStop = true;
        }
    }
}
