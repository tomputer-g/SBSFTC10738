package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode19.BaseAuto;

@Disabled
@TeleOp
public class TurningCalibrate extends BaseAuto {
    double turningAmount = 45.0;
    double speed = 0.2;
    double threshold = 2;

    public void init(){
        super.init();
        turningAmount = 45.0;
        speed = 0.2;
        threshold = 2;
    }

    public void loop(){
        super.loop();
        if (this.gamepad1.dpad_left)
        {
            while (this.gamepad1.dpad_left);
            turningAmount -= 1;
        }
        else if (this.gamepad1.dpad_right)
        {
            while(this.gamepad1.dpad_right);
            turningAmount += 1;
        }
        else if (this.gamepad1.dpad_up)
        {
            while (this.gamepad1.dpad_up);
            speed += 0.1;
        }
        else if (this.gamepad1.dpad_down)
        {
            while(this.gamepad1.dpad_down);
            speed -= 0.1;
        }
        else if (this.gamepad1.left_bumper)
        {
            while(this.gamepad1.left_bumper);
            threshold-=1;
        }
        else if (this.gamepad1.right_bumper)
        {
            while(this.gamepad1.right_bumper);
            threshold+=1;
        }
        else if (this.gamepad1.x)
        {
            while(this.gamepad1.x);
            turningAmount = -turningAmount;
        }
        else if (this.gamepad1.y)
        {
            while(this.gamepad1.y);
            turn(turningAmount, speed, threshold);
        }
        telemetry.addData("Current speed: ","%.2f", speed);
        telemetry.addData("Current turning amount: ", turningAmount);
        telemetry.addData("Current threshold: ", threshold);
        telemetry.update();
    }
}
