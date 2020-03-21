package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode20.Roadrunner.drive.GoBildaMotor1150;

@Autonomous
public class DrivetrainPIDFTuner extends LinearOpMode {
    DcMotorEx motor;
    double currentVelocity;
    double maxVelocity = 0.0;
    double currentPower = 0;
    boolean lb, rb;

    @Override public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "LF");

        motor.setMotorType(MotorConfigurationType.getMotorType(GoBildaMotor1150.class));
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        motor.setPower(1);
        while (opModeIsActive()) {
            if(this.gamepad1.left_bumper){lb = true;}if(lb && !this.gamepad1.left_bumper){
                lb = false;
                currentPower -= 0.1;
                currentPower = Math.max(currentPower,0);
                motor.setPower(currentPower);
            }
            if(this.gamepad1.right_bumper){rb = true;}if(rb && !this.gamepad1.right_bumper){
                rb = false;
                currentPower += 0.1;
                currentPower = Math.min(currentPower,1);
                motor.setPower(currentPower);
            }
            currentVelocity = motor.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }
            telemetry.addData("current power", currentPower);
            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
        motor.setPower(0);
    }
}
