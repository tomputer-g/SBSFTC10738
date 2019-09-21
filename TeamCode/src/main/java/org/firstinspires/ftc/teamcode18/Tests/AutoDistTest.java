package org.firstinspires.ftc.teamcode18.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode18.BaseAuto;

/**
 * Created by Ziming Gao on 1/21/2018.
 */
@TeleOp(name = "distance", group = "test")
@Disabled
public class AutoDistTest extends BaseAuto {
    private int f=0,r=0,l=0;
    @Override
    public void loop() {
        if(this.gamepad1.dpad_up){
            while (this.gamepad1.dpad_up);
            moveForwardDistance(100,0.2);
            f+=100;
        }
        if(this.gamepad1.dpad_left) {
            while (this.gamepad1.dpad_left) ;
            moveLeftDistance(100, 0.2);
            l += 100;
            r -= 100;
        }
        if(this.gamepad1.dpad_right){
            while(this.gamepad1.dpad_right);
            moveRightDistance(100,0.2);
            l-=100;
            r+=100;
        }
        telemetry.addData("L",l);
        telemetry.addData("F",f);
        telemetry.addData("R",r);
        telemetry.update();
    }
}
