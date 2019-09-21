package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode19.BaseAuto;
@Autonomous
@Disabled
public class LastResortTest extends BaseAuto {
    @Override
    public void init(){
        super.init();
    }

    @Override
    public void loop() {
        moveInches(-30,-40,0.3);
        turn(-40,0.3,1);
        moveInches(-30,60,0.5);
    }
}
