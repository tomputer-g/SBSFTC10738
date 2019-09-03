package org.firstinspires.ftc.teamcode18.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Ziming Gao on 12/5/2017.
 */
@TeleOp(name = "grabber test", group = "test")
@Disabled
public class GrabberTest extends OpMode {
    DcMotor motor;
    int count;
    @Override
    public void init() {
        count = 0;
        motor = hardwareMap.get(DcMotor.class, "ls1");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.3);
    }

    @Override
    public void loop() {
        if(this.gamepad1.dpad_right){
            while(this.gamepad1.dpad_right);
            count++;
            motor.setTargetPosition(motor.getCurrentPosition()+10);
        }//position += 10
        if(this.gamepad1.dpad_left){
            while(this.gamepad1.dpad_left);
            count--;
            motor.setTargetPosition(motor.getCurrentPosition()-10);
        }//position -= 10
        if(this.gamepad1.dpad_up){
            while(this.gamepad1.dpad_up);
            if(motor.getPower() < 0.95)
                motor.setPower(motor.getPower() + 0.05);
            else
                motor.setPower(1);
        }//power+=0.05
        if(this.gamepad1.dpad_down){
            while(this.gamepad1.dpad_down);
            if(motor.getPower() > 0.05)
                motor.setPower(motor.getPower() - 0.05);
            else
                motor.setPower(0);
        }//power-=0.05
        if(this.gamepad1.b){
            motor.setPower(0.1);
            motor.setTargetPosition(0);
            count = 0;
        }//0.1 speed RTH
        if(this.gamepad1.y){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }//set current pos as 0
        telemetry.addData("count", count);
        telemetry.addData("motor power",motor.getPower());
        telemetry.addData("motor Pos", motor.getCurrentPosition());
        telemetry.addData("motor set Pos",  10*count);
        telemetry.update();
    }
}
