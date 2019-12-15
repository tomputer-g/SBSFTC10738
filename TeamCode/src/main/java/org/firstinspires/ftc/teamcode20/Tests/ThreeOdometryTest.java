package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode20.BaseAuto;

@Autonomous
public class ThreeOdometryTest extends BaseAuto {

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
            |                                                             +--+  |
            |  +------+                                                   |  |  |
            |  |      |  1                                              3 |  |  |
            |  +------+                                                   |  |  |
            |                                                             +--+  |
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

    private DcMotor fakeEnc1, fakeEnc2, fakeEnc3;
    private final double enc3_dist_from_center_inch = 7.0;
    private final double odometryEncPerInch = 2048.0/Math.PI;

    @Override
    public void init() {
        fakeEnc1 = hardwareMap.get(DcMotor.class, "motor1");
        fakeEnc2 = hardwareMap.get(DcMotor.class, "motor2");
        fakeEnc3 = hardwareMap.get(DcMotor.class, "motor3");
        fakeEnc1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fakeEnc2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fakeEnc3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fakeEnc1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fakeEnc2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fakeEnc3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        initIMU();
    }

    @Override
    public void loop() {
        telemetry.addLine("3 wheel odometry mode");
        telemetry.addData("Enc 1", fakeEnc1.getCurrentPosition());
        telemetry.addData("Enc 2", fakeEnc2.getCurrentPosition());
        telemetry.addData("Enc 3", fakeEnc3.getCurrentPosition());
        telemetry.addLine("-----------------------------------------");
        double dR_3 = fakeEnc3.getCurrentPosition() - fakeEnc2.getCurrentPosition();
        double theta = ((180 * dR_3)/(Math.PI * enc3_dist_from_center_inch))%360.0;
        telemetry.addData("dR actual(deg)", getHeading());
        telemetry.addData("dR derive(deg)", theta);
        telemetry.update();
    }
}
