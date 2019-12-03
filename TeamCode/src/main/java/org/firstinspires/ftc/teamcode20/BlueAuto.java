package org.firstinspires.ftc.teamcode20;

import org.firstinspires.ftc.teamcode20.BaseAuto;

public class BlueAuto extends BaseAuto {


    @Override
    public void init() {
        initDrivetrain();
        initIMU();
        initGrabber();
        initLinSlide();
        initVuforiaWebcam();
    }

    @Override
    public void loop() {
        //go forward 1 floor mat (24")
        //vuforia - recognize block & move to pick up
        //after pickup: turn 90 deg. move to platform, drop off
        //move to platform, drag into position, release
        //repeat until run out of time; first on other skystones

    }
}
