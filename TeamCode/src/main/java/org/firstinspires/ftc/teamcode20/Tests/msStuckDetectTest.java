package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode20.BaseOpMode;

public class msStuckDetectTest extends BaseOpMode {

    private boolean flag;

    @Override
    public void internalPreInit() {
        msStuckDetectInit = 30000;
        telemetry.setAutoClear(false);
        telemetry.addLine("preInit()");
        telemetry.update();
    }


    @Override
    public void init() {
        wait(7000);
        telemetry.setAutoClear(false);
        telemetry.addLine("7s has passed (init)");
        telemetry.update();
    }

}
