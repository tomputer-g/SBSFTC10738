package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseOpMode;

@TeleOp
public class LinSlideTest extends BaseOpMode {

    private double slideP = 0.5;
    private int hold = 0;
    private boolean holdSet;

    @Override
    public void init() {
        initLinSlide();//0-2000
    }

    @Override
    public void loop() {
        runPos();
        telemetry.addData("actual",L1.getCurrentPosition());
        telemetry.update();
    }

    private double joystick_quad(double input){
        if(input < 0)
            return - (input * input);
        return input * input;
    }

    private void runPos(){
        if(near(this.gamepad1.right_stick_y,0,0.05) || !this.gamepad1.left_bumper){//keep position

            if(!holdSet){
                holdSet = true;
                L1.setTargetPosition(L1.getCurrentPosition());
                telemetry.addData("set to",L1.getCurrentPosition());
                L2.setTargetPosition(-L1.getCurrentPosition());
                L1.setPower(1);
                L2.setPower(1);
                L1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                L2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


        }else if(this.gamepad1.left_bumper){//long-dist
            holdSet = false;
            telemetry.addLine("CHANGING SLIDE");
            L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if(-this.gamepad1.right_stick_y > 0){//up
                L1.setPower(-this.gamepad1.right_stick_y);
                L2.setPower(this.gamepad1.right_stick_y);
            }else{
                L1.setPower(-0.6 * this.gamepad1.right_stick_y);
                L2.setPower(0.6 * this.gamepad1.right_stick_y);
            }
        }
    }
}
