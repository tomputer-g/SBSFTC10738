package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode20.Roadrunner.drive.GoBildaMotor1150;

@Autonomous
public class DrivetrainPIDFTuner extends LinearOpMode {
    DcMotorEx LF, LB, RF, RB;
    double currentVelocity;
    double maxVelocity = 0.0;
    double currentPower = 0.5;
    boolean lb, rb;

    @Override public void runOpMode() {
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        RB = hardwareMap.get(DcMotorEx.class, "RB");
        LF.setMotorType(MotorConfigurationType.getMotorType(GoBildaMotor1150.class));
        LB.setMotorType(MotorConfigurationType.getMotorType(GoBildaMotor1150.class));
        RF.setMotorType(MotorConfigurationType.getMotorType(GoBildaMotor1150.class));
        RB.setMotorType(MotorConfigurationType.getMotorType(GoBildaMotor1150.class));
        waitForStart();
        LF.setPower(-currentPower);
        LB.setPower(-currentPower);
        RF.setPower(currentPower);
        RB.setPower(currentPower);
        while (opModeIsActive()) {
            if(this.gamepad1.left_bumper){lb = true;}if(lb && !this.gamepad1.left_bumper){
                lb = false;
                currentPower -= 0.1;
                currentPower = Math.max(currentPower,0);
                LF.setPower(-currentPower);
                LB.setPower(-currentPower);
                RF.setPower(currentPower);
                RB.setPower(currentPower);
            }
            if(this.gamepad1.right_bumper){rb = true;}if(rb && !this.gamepad1.right_bumper){
                rb = false;
                currentPower += 0.1;
                currentPower = Math.min(currentPower,1);
                LF.setPower(-currentPower);
                LB.setPower(-currentPower);
                RF.setPower(currentPower);
                RB.setPower(currentPower);
            }
            currentVelocity = (-LF.getVelocity() - LB.getVelocity() + RF.getVelocity() + RB.getVelocity())/4.0;

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }
            if(this.gamepad1.b){
                maxVelocity = 0;
            }
            telemetry.addData("current power", currentPower);
            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum avg velocity", maxVelocity);
            telemetry.update();
        }
        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
    }
}
