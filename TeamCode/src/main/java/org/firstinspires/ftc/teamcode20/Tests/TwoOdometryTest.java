package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode20.BaseAuto;

@Autonomous(group = "Test")
public class TwoOdometryTest extends BaseAuto {

    /*
            +-------------------------------------------------------------------+
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |  +------+                                                         |
            |  |      |  1                                                      |
            |  +------+                                                         |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                                                   |
            |                                 2                                 |
            |                                +--+                               |
            |                                |  |                               |
            |                                |  |                               |
            |                                |  |                               |
            |                                +--+                               |
            +-------------------------------------------------------------------+

     */


    private DcMotor fakeEnc1, fakeEnc2;
    private final double odometryEncPerInch = 2048.0/Math.PI;

    @Override
    public void init() {
        fakeEnc1 = hardwareMap.get(DcMotor.class, "motor1");
        fakeEnc2 = hardwareMap.get(DcMotor.class, "motor2");
        fakeEnc1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fakeEnc2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fakeEnc1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fakeEnc2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        initIMU();
    }

    @Override
    public void loop() {
        telemetry.addLine("2 wheel odometry mode");
        telemetry.addData("Enc 1", fakeEnc1.getCurrentPosition());
        telemetry.addData("Enc 2", fakeEnc2.getCurrentPosition());
        telemetry.addLine("-----------------------------------------");
        telemetry.addData("dX (in)",fakeEnc1.getCurrentPosition() / odometryEncPerInch);
        telemetry.addData("dY (in)",fakeEnc2.getCurrentPosition() / odometryEncPerInch);
        telemetry.addData("dR (deg)", getHeading());
        telemetry.update();
    }
}
