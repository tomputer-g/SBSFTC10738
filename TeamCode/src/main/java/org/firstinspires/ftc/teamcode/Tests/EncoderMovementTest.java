package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BaseAuto;

import static java.lang.Math.sqrt;
@Disabled
@TeleOp
public class EncoderMovementTest extends BaseAuto {
    @Override
    public void init() {
        initDrivetrain();
    }


    @Override
    public void loop() {
        if(this.gamepad1.a){
            while(this.gamepad1.a);
            moveInches(0,-24,0.3);
        }else if(this.gamepad1.b){
            while(this.gamepad1.b);
            moveInches(24,0,0.3);
        } else if (this.gamepad1.x) {
            while (this.gamepad1.x);
            moveInches(-24,0,0.3);
        }else if (this.gamepad1.y) {
            while (this.gamepad1.y);
            moveInches(0,24,0.3);
        }
        else if(this.gamepad1.right_bumper){
            while(this.gamepad1.right_bumper);
            moveInches(24,24,0.3);
        }
        else if(this.gamepad1.left_bumper){
            while(this.gamepad1.left_bumper);
            moveInches(-24,-24,0.3);
        }

    }
}
