package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode19.BaseAuto;

@TeleOp
@Disabled
public class VCalibrationTest extends BaseAuto {
    @Override
    public void init() {
        super.init();
        initVumark();
        waitUntilVumarkRecognized();
    }

    @Override
    public void loop() {
        telemetry.addData("HDG",getOrientationFromVumark()[2]);
        telemetry.update();
        if(this.gamepad1.left_bumper){
            while (this.gamepad1.left_bumper);
            waitUntilVumarkRecognized();

            moveInches(-5,5,0.2);
        }
        if(this.gamepad1.right_bumper){
            while (this.gamepad1.right_bumper);
            waitUntilVumarkRecognized();

            double headingOffset = headingOffset(getOrientationFromVumark()[2], 180);
            turn(-headingOffset, 0.1,1);
        }
    }
}
