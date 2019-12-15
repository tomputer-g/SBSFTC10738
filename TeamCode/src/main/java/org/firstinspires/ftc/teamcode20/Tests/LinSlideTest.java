package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode20.BaseOpMode;

@TeleOp(group = "Test")
public class LinSlideTest extends BaseOpMode {

    private int hold = 0;
    private boolean holdSet;
    private double a = 45;
    private boolean LP, RP;


    //travel 58 inch in 2000

    @Override
    public void init() {
        telemetryOn = true;
        initLinSlide();//0-2000
        L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {
        runSlide();
        if(this.gamepad1.dpad_left){LP = true;}if(LP && !this.gamepad1.dpad_left){LP = false;
            a -= 5;
        }
        if(this.gamepad1.dpad_right){RP = true;}if(RP && !this.gamepad1.dpad_right){RP = false;
            a += 5;
        }
        if(holdSet){telemetry.addData("Hold pos", hold);}
        telemetry.addData("actual 1",L1.getCurrentPosition());
        telemetry.addData("actual 2",L2.getCurrentPosition());
        telemetry.addData("a", a);
        telemetry.update();
    }
}
