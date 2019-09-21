package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.Thread.sleep;

/**
 * Created by Lucien Liu & Frank Zhou on 10/31/2018, testing on motors.
 */
@Disabled
@TeleOp
public class RevMotorTest extends LinearOpMode {
    private DcMotor motorTest;
    @Override
    public void runOpMode() throws InterruptedException {
        motorTest = hardwareMap.get(DcMotor.class, "motorTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        motorTest.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTest.setPower(1);
        int position = 10;
        int delay = 1;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            motorTest.setTargetPosition(position);
            wait(delay);
            motorTest.setTargetPosition(0);
            wait(delay);

            if(this.gamepad1.dpad_up) {
                while (this.gamepad1.dpad_up);
                position++;
            }else if(this.gamepad1.dpad_down){
                while(this.gamepad1.dpad_up);
                position--;
            }
            if(this.gamepad1.dpad_left && delay > 0){
                while(this.gamepad1.dpad_left);
                delay--;
            }else if(this.gamepad1.dpad_right){
                while(this.gamepad1.dpad_right);
                delay++;
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Power", motorTest.getPower());
            telemetry.addData("currentPosition" , motorTest.getCurrentPosition());
            telemetry.addData("targetPosition", position);
            telemetry.addData("delay", delay);
            telemetry.update();
        }
    }
    protected void wait(int time){
        sleep(time);
    }
}
