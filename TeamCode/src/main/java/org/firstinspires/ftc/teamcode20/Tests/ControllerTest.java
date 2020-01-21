package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode19.BaseOpMode;

/**
 * Created by Ziming Gao on 12/11/2017.
 */
public class ControllerTest extends BaseOpMode {

    @Override public void init() {}
    @Override public void loop() {
        telemetry.addData("L stick", to3d(this.gamepad1.left_stick_x) + " / " + to3d(this.gamepad1.left_stick_y));
        telemetry.addData("R stick", to3d(this.gamepad1.right_stick_x) + " / " + to3d(this.gamepad1.right_stick_y));
        telemetry.addData("LT / RT", to3d(this.gamepad1.left_trigger) + " / " + to3d(this.gamepad1.right_trigger));
        telemetry.addData("LB / RB", this.gamepad1.left_bumper + " / " + this.gamepad1.right_bumper);
        telemetry.addData("DPAD",(this.gamepad1.dpad_left?" L ":"")+(this.gamepad1.dpad_right?" R ":"")+(this.gamepad1.dpad_up?" U ":"")+(this.gamepad1.dpad_down?" D ":""));
        telemetry.addData("XYAB",(this.gamepad1.a ? " A ": "")+(this.gamepad1.b?" B ":"")+(this.gamepad1.x?" X ":"")+(this.gamepad1.y?" Y ":""));
        telemetry.addData("Other", (this.gamepad1.back?"BACK ":"")+(this.gamepad1.guide?"GUIDE ":"")+(this.gamepad1.start?"START ":""));
        telemetry.update();
    }
}
