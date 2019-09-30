package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode19.BaseOpMode;
@Autonomous(group = "Test")
@Disabled
public class Encoders extends BaseOpMode {
    @Override
    public void init() {
        initDrivetrain();
        initGrabber();
        initLifter();
    }

    @Override
    public void loop() {
        telemetry.addData("Drivetrain","( "+LF.getCurrentPosition()+", "+LB.getCurrentPosition()+", "+RF.getCurrentPosition()+", "+RB.getCurrentPosition()+")");
        telemetry.addData("shoulder", grabber_shoulder.getCurrentPosition());
        telemetry.addData("slide", grabber_slide.getCurrentPosition());
        telemetry.addData("lifter", lander_lifter.getCurrentPosition());
        telemetry.update();
    }
    protected void initGrabber(){
        grabber_shoulder = hardwareMap.get(DcMotor.class,"grabber_shoulder");
        grabber_shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabber_shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabber_slide = hardwareMap.get(DcMotor.class, "grabber_slide");
        grabber_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabber_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabber = hardwareMap.get(DcMotor.class, "grabber");
        slideLimitSW = hardwareMap.get(RevTouchSensor.class, "slideLimitSW");
    }
}
