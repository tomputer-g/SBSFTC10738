package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode20.BaseOpMode;

@TeleOp
public class TeleOpAutoPlaceBrick extends BaseOpMode {
    boolean a= false,a_rt=false,a_lt=false;
    boolean[] aa={true};
    int placeLevel=0;
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

                } else if (a_rt) {
                    //reset
                    placeLevel = 0;
                } else {
                    placeLevel++;
                }

                a_lt = false;
                a_rt = false;
                runSlidetoBlock(placeLevel);
            }
                if (!this.gamepad1.a && a) {
                    a = false;
                }
            runSlide();
            telemetry.addData("placeLevel: ",placeLevel);
            telemetry.addData("L1: ",L1.getCurrentPosition());
            telemetry.update();
        }
    }

    private void autoPlaceLevel () {
        int goalEnc = (int) (slideEncoderPerInch * 4 * placeLevel + 2.25 * slideEncoderPerInch);//per inch is already neg.
        holdSet = false;
        holdSlide(goalEnc);
    }

    private void runSlidetoBlock(int block){
        int goalEnc = (int) (slideEncoderPerInch * 4 * placeLevel + 2.25 * slideEncoderPerInch);
        while(L1.getCurrentPosition()>goalEnc){
            L1.setPower(-1);
            L2.setPower(1);
        }
        L1.setPower(0);
        L2.setPower(0);
        holdSet = false;
        holdSlide(goalEnc);
    }

}

