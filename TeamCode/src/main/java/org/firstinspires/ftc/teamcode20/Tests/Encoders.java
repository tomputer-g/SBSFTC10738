package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode19.BaseOpMode;

@Autonomous(group = "Test")
public class Encoders extends BaseOpMode {
    @Override
    public void init() {
        initDrivetrain();
    }

    @Override
    public void loop() {
        telemetry.addData("Drivetrain","( "+LF.getCurrentPosition()+", "+LB.getCurrentPosition()+", "+RF.getCurrentPosition()+", "+RB.getCurrentPosition()+")");
        telemetry.update();
    }


}