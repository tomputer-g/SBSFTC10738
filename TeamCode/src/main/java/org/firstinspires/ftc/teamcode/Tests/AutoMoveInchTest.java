package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseAuto;
@Disabled
@TeleOp
public class AutoMoveInchTest extends BaseAuto {

    @Override public void init() {
        initDrivetrain();
        initIMU();
    }

    @Override
    public void loop() {
        if(this.gamepad1.a){
            while(this.gamepad1.a);
            moveInches(48,24,0.3);
        }else if(this.gamepad1.y){
            while(this.gamepad1.y);
            moveInches(0,24,0.3);
        }else if(this.gamepad1.x){
            while(this.gamepad1.x);
            moveInches(-24, 0,0.3);
        }else if(this.gamepad1.b){
            while(this.gamepad1.b);
            moveInches(24,0,0.3);
        }

    }
}
