package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode20.BaseAuto;

//@TeleOp
public class RevMotorTest extends BaseAuto {
    private boolean bPrimed;
    @Override
    public void init() {
        grabber_extender = hardwareMap.get(DcMotor.class, "grabber_extender");
        grabber_extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if(this.gamepad1.dpad_up){
            grabber_extender.setPower(0.4);
        }else if(this.gamepad1.dpad_down){
            grabber_extender.setPower(-0.4);
        }else{
            grabber_extender.setPower(0);
        }
        if(this.gamepad1.a){
            grabber_extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            grabber_extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(this.gamepad1.b){
            bPrimed = true;
        }
        if(!this.gamepad1.b && bPrimed) {
            bPrimed = false;
            grabber_extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            grabber_extender.setTargetPosition(-330);
            grabber_extender.setPower(.7);
            grabber_extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            grabber_extender.setPower(0);
            grabber_extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        telemetry.addData("encoder", grabber_extender.getCurrentPosition());
        telemetry.update();
    }
}
