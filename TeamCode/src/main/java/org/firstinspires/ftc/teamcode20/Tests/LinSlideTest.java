package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseOpMode;

@TeleOp
public class LinSlideTest extends BaseOpMode {
    private boolean DPDPrimed, DPUPrimed;
    private ElapsedTime t;

    private double value, a = 1;

    @Override
    public void init() {
        value = 0;
        initLinSlide();
        L1.setTargetPosition(0);
        L2.setTargetPosition(0);
        L1.setPower(1);
        L2.setPower(1);
        L1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        if(t == null){
            t = new ElapsedTime();
        }
        if(this.gamepad1.left_bumper){
            telemetry.addLine("CHANGING SLIDE");
            value += a * (joystick_quad(-this.gamepad1.right_stick_y)) * t.milliseconds();
        }
        t.reset();
        if(this.gamepad1.dpad_up){
            DPUPrimed = true;
        }
        if(!this.gamepad1.dpad_up && DPUPrimed){
            DPUPrimed = false;
            a += 0.05;
        }

        if(this.gamepad1.dpad_down){
            DPDPrimed = true;
        }
        if(!this.gamepad1.dpad_down && DPDPrimed){
            DPDPrimed = false;
            a -= 0.05;
        }

        L1.setTargetPosition((int)value);
        L2.setTargetPosition(-(int)value);
        telemetry.addData("target",value);
        telemetry.addData("actual",L1.getCurrentPosition());
        telemetry.addData("a", a);
        telemetry.addData("input", to3d(-this.gamepad1.right_stick_y) + " -> " + to3d(joystick_quad(-this.gamepad1.right_stick_y)));
        telemetry.addData("position", value);
        telemetry.update();
    }

    private double joystick_quad(double input){
        if(input < 0)
            return - (input * input);
        return input * input;
    }
}
