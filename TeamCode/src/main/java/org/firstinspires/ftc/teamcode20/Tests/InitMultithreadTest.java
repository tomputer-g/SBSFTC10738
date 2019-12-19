package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;

@Autonomous
public class InitMultithreadTest extends BaseAuto {

    @Override
    public void init() {
        ElapsedTime t = new ElapsedTime();
        VuforiaInitThread vuforiaInit = new VuforiaInitThread();
        vuforiaInit.start();
        initIMU();
        initDrivetrain();
        while(vuforiaInit.isAlive());
        telemetry.addData("Initialization time",t.milliseconds());
        telemetry.update();
    }

    private class VuforiaInitThread extends Thread{
        @Override public void run() {
            initVuforiaWebcam();
        }
    }


    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        shutdownVuforia();
    }
}
