package org.firstinspires.ftc.teamcode19.Tests;

import org.firstinspires.ftc.teamcode19.BaseAuto;

public class LowerTest extends BaseAuto {
    @Override
    public void init() {
        msStuckDetectLoop = 30000;
        msStuckDetectInit = 10000;
        initDrivetrain();
        initGrabber();
        initLifter();
    }

    @Override
    public void loop(){
        lowerRobot();
        requestOpModeStop();
    }

    @Override
    public void stop() {

    }
}
