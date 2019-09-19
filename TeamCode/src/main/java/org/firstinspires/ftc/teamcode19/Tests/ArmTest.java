package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name ="ArmTest", group = "test")
@Disabled
public class ArmTest extends LinearOpMode {
    private DcMotor motorTest;
    public void runOpMode() {
        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // motorTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double tgtPower = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (this.gamepad1.dpad_up){
                tgtPower = 0.2;
                motorTest.setPower(tgtPower);
            }
            else if (this.gamepad1.dpad_down){
                tgtPower = -0.21;
                motorTest.setPower(tgtPower);
            }
            else{
                tgtPower = 0;
                motorTest.setPower(tgtPower);
            }
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
