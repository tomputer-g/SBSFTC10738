package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode19.BaseOpMode;
@Autonomous(group = "Test")
public class AutoResetLifter extends BaseOpMode {
    @Override
    public void init() {
        msStuckDetectLoop = 10000;
        initLifter();
    }

    @Override
    public void loop() {
        ElapsedTime t = new ElapsedTime();
        lander_lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lander_lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lander_lifter.setPower(1);
        lander_lifter.setTargetPosition(-15000);
        while(lander_lifter.isBusy() && t.milliseconds() < 7000);
        requestOpModeStop();

    }


}
