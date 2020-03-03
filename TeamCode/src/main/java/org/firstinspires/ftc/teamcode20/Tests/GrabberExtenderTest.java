package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode20.BaseAuto;
@TeleOp
public class GrabberExtenderTest extends BaseAuto {
    boolean[] a = {true};
    boolean[] b = {true};
    double pos = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        initGrabber();
        pos = 1;
        servoThread.setExtTarget(pos);
        waitForStart();
        while(opModeIsActive()){
            if(zheng(this.gamepad1.dpad_down, a)){
                pos+=0.01;
            }
            if(zheng(this.gamepad1.dpad_up, b)){
                pos-=0.01;
            }
            servoThread.setExtTarget(pos);
            telemetry.addData("pos", pos);
            telemetry.update();
        }
    }
}
