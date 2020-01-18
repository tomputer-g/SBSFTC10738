package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode20.BaseOpMode;
@Disabled

@TeleOp
public class LinSlideManualTest extends BaseOpMode {
    @Override
    public void init() {
        initLinSlide();
    }

    @Override
    public void loop() {
        if (this.gamepad1.dpad_up){
            L1.setPower(1);
            L2.setPower(-1);
        }else if(this.gamepad1.dpad_down){
            L1.setPower(-1);
            L2.setPower(1);
        }else{
            L1.setPower(0);
            L2.setPower(0);
        }
        telemetry.addData("enc 1", L1.getCurrentPosition());
        telemetry.addData("enc 2",L2.getCurrentPosition());
        telemetry.update();
    }
}
