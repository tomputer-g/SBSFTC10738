package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode19.BaseOpMode;
@Autonomous
public class LoopTime extends BaseOpMode {
    private ElapsedTime t;

    @Override
    public void init() {
        t = new ElapsedTime();
    }

    @Override
    public void loop() {
        telemetry.addData("ms per loop",t.milliseconds());
        t.reset();

    }
}
