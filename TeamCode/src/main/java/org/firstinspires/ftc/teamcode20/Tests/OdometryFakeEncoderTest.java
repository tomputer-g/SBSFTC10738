package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode20.BaseOpMode;

public class OdometryFakeEncoderTest extends OpMode {
    private final double odometryEncPerInch = 2048.0/Math.PI;
    private DigitalChannel encA, encB;
    private DcMotor fakeEncMotor;
    @Override
    public void init() {
        fakeEncMotor = hardwareMap.get(DcMotor.class, "motor");
        fakeEncMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fakeEncMotor.setPower(0.2);
    }

    @Override
    public void loop() {
        telemetry.addData("Enc", fakeEncMotor.getCurrentPosition());
        telemetry.update();
    }
}

