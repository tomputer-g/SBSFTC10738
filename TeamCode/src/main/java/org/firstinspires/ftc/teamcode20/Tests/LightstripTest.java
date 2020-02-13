package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode20.BaseOpMode;


public class LightstripTest extends BaseOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initLight();
        waitForStart();
        while(opModeIsActive()){
            if(this.gamepad1.a){
                setLight(true);
            }
            if(this.gamepad1.b){
                setLight(false);
            }
        }
    }

}
