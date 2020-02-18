package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode20.BaseOpMode;

@TeleOp
public class TeleOpAutoPlaceBrick extends BaseOpMode {
    boolean a= false,a_rt=false,a_lt=false;
    int placeLevel=4;
    double groundHeightEnc=2.25;
    @Override

    public void runOpMode() throws InterruptedException {
        initDrivetrain();
        initGrabber();
        initLinSlide();
        L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabber.setPosition(0.35);
        waitForStart();
        while(opModeIsActive()){
            if(this.gamepad1.a&&!a) {
                a = true;
                if (this.gamepad1.left_trigger > 0.5) {
                    a_lt = true;
                    telemetry.addLine("LT");
                }
                if (this.gamepad1.right_trigger > 0.5) {
                    a_rt = true;
                    telemetry.addLine("RT");
                }
                if (a_lt) {
                    //remain the same level
                } else if (a_rt) {
                    //reset
                    placeLevel = 2;
                } else {
                    placeLevel++;
                }

                a_lt = false;
                a_rt = false;
                autoPlaceLevel();
            }
                if (!this.gamepad1.a && a) {
                    a = false;
                }
            telemetry.addData("placeLevel: ",placeLevel);
            telemetry.update();
        }
    }

    private void autoPlaceLevel () {
        int goalEnc = (int) (slideEncoderPerInch * 4 * placeLevel - 2.25 * slideEncoderPerInch);//per inch is already neg.
        holdSet = false;
        holdSlide(goalEnc);
    }
}
