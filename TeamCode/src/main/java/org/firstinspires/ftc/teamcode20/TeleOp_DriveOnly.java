package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode19.BaseTeleOp;

public class TeleOp_DriveOnly extends BaseTeleOp {

    @Override
    public void init() {
        initDrivetrain();
    }

    @Override
    public void loop() {
        move(custom_linear(this.gamepad1.left_trigger > 0.5,this.gamepad1.left_stick_x), custom_linear(this.gamepad1.left_trigger > 0.5,-this.gamepad1.left_stick_y), custom_linear(this.gamepad1.left_trigger > 0.5,-this.gamepad1.right_stick_x));
    }
}
