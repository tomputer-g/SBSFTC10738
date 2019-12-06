package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode20.BaseOpMode;

public class LinSlideTest extends BaseOpMode {

    private int hold = 0;
    private boolean holdSet;
    private double a = 45;
    private boolean LP, RP;

    //travel 58 inch in 2000

    @Override
    public void init() {
        initLinSlide();//0-2000
        L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        telemetry.addData("actual",L1.getCurrentPosition());
        telemetry.addData("a", a);
        telemetry.update();
    }

    private double joystick_quad(double input){
        if(input < 0)
            return - (input * input);
        return input * input;
    }

    private void runSlide(){
        if(this.gamepad1.left_bumper && !near(this.gamepad1.right_stick_y, 0, 0.05)) {//long-dist
            if (this.gamepad1.right_stick_y < 0 && L1.getCurrentPosition() < 2000) {//up
                holdSet = false;
                telemetry.addLine("CHANGING SLIDE");
                L1.setPower(-this.gamepad1.right_stick_y);
                L2.setPower(this.gamepad1.right_stick_y);
            } else if (this.gamepad1.right_stick_y > 0 && L1.getCurrentPosition() > 0) {
                holdSet = false;
                telemetry.addLine("CHANGING SLIDE");
                L1.setPower(-0.5 * this.gamepad1.right_stick_y);
                L2.setPower(0.5 * this.gamepad1.right_stick_y);
            } else {
                holdSlide();
            }
        }else{
            holdSlide();
        }
    }

    private void holdSlide(){
        if (!holdSet) {
            holdSet = true;
            hold = Math.max(0,Math.min(2000,L1.getCurrentPosition()));
        }
        int error = hold - L1.getCurrentPosition();
        double power = Math.min(1, Math.max(0, error/a));
        if(hold == 0){power = 0;}
        telemetry.addLine("error: "+hold+" - "+(hold-error) + " = "+error);
        telemetry.addData("PWR", power);
        L1.setPower(power);
        L2.setPower(-power);
    }
}
