package org.firstinspires.ftc.teamcode18.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Ziming Gao on 12/11/2017.
 */
@TeleOp(name = "Ctrler test", group = "test")
@Disabled
public class ControllerTest extends OpMode {
    @Override public void init() {}
    @Override public void loop() {
        telemetry.addData("L stick", this.gamepad1.left_stick_x + " / " + this.gamepad1.left_stick_y);
        telemetry.addData("R stick", this.gamepad1.right_stick_x + " / " + this.gamepad1.right_stick_y);
        telemetry.addData("LT / RT", this.gamepad1.left_trigger + " / " + this.gamepad1.right_trigger);
        telemetry.addData("LB / RB", this.gamepad1.left_bumper + " / " + this.gamepad1.right_bumper);
        telemetry.addData("DPAD",(this.gamepad1.dpad_left?" L ":"")+(this.gamepad1.dpad_right?" R ":"")+(this.gamepad1.dpad_up?" U ":"")+(this.gamepad1.dpad_down?" D ":""));
        telemetry.addData("XYAB",(this.gamepad1.a ? " A ": "")+(this.gamepad1.b?" B ":"")+(this.gamepad1.x?" X ":"")+(this.gamepad1.y?" Y ":""));
        telemetry.update();
    }
}
