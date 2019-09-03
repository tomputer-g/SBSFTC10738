package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.BaseOpMode;
@Autonomous
@Disabled
public class RandomTest extends BaseOpMode {
    @Override
    public void start(){
        telemetry.addLine("Start");
    }

    @Override
    public void init() {
        telemetry.setAutoClear(false);
        telemetry.addLine("Init");
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Init loop");
        wait(250);
    }

    @Override
    public void loop() {
        telemetry.update();
    }
}
