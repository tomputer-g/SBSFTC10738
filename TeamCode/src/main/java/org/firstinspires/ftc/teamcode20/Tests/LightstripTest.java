package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode20.BaseOpMode;


@TeleOp
public class LightstripTest extends BaseOpMode {
    @Override
    public void init() {
        initLight();
    }

    @Override
    public void loop() {
        if(this.gamepad1.a){
            setLight(true);
        }
        if(this.gamepad1.b){
            setLight(false);
        }
    }
}
