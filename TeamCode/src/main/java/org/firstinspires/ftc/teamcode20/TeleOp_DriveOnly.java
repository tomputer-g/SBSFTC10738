package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode19.BaseTeleOp;
@TeleOp
public class TeleOp_DriveOnly extends BaseTeleOp {

    @Override
    public void init() {
        initDrivetrain();
        LF.setPower(1);
    }

    @Override
    public void loop() {
        //for old bot only
        move(-this.gamepad1.left_stick_x, this.gamepad1.left_stick_y, -this.gamepad1.right_stick_x);
    }

    @Override
    protected void initDrivetrain() {
        super.initDrivetrain();
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
