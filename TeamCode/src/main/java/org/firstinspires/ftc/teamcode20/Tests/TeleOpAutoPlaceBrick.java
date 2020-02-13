package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode20.BaseOpMode;

public class TeleOpAutoPlaceBrick extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initDrivetrain();
        initGrabber();
        initLinSlide();
        L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabber.setPosition(0.35);
        waitForStart();
        while(opModeIsActive()){}
    }
}
