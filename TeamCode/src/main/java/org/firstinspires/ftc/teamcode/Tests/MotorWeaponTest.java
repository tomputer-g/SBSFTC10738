package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class MotorWeaponTest extends LinearOpMode {
    private DcMotor motorTest;
    @Override
    public void runOpMode() {
        motorTest = hardwareMap.get(DcMotor.class, "motorTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        motorTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double tgtPower = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(this.gamepad1.left_bumper){
                while(this.gamepad1.left_bumper);
                tgtPower += 0.05;
            }
            else if(this.gamepad1.right_bumper){
                while(this.gamepad1.right_bumper);
                tgtPower -= 0.05;
            }
            else if(this.gamepad1.x){
                tgtPower = 0;
            }
            motorTest.setPower(tgtPower);
            telemetry.addData("Current Power: ", tgtPower);
            telemetry.update();
        }
    }
}