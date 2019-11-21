package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseOpMode;

public class JoystickAdjustTest extends LinearOpMode {
    private double a = 1;
    private double value = 0, time = 0, pos = 0;
    private boolean DPDPrimed, DPUPrimed;


    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime t = new ElapsedTime();
        ElapsedTime t2 = new ElapsedTime();
        boolean t2resetted = false, timed = false;
        DPDPrimed = false;
        DPUPrimed = false;
        waitForStart();
        t.reset();
        while(opModeIsActive()){


            if(this.gamepad1.a){
                value = 0;
            }

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
            if(this.gamepad1.left_stick_y == -1 && !t2resetted){
                t2.reset();
                timed = false;
                t2resetted = true;
            }else if(this.gamepad1.left_stick_y != -1){
                t2resetted = false;

            }
            if(value > 1000){
                if(!timed) {
                    timed = true;
                    time = t2.milliseconds();
                    pos = value;
                }
                telemetry.addData("1000 time", time);
                telemetry.addData("position", pos);
            }else{
                telemetry.addData("1000 time",t2.milliseconds());
            }


            telemetry.addData("a", a);
            telemetry.addData("input", -this.gamepad1.left_stick_y);
            telemetry.addData("position", value);
            telemetry.addData("cycle time", t.milliseconds()+"ms");
            telemetry.update();
            value += a * -this.gamepad1.left_stick_y * t.milliseconds();
            t.reset();
        }
    }
}
