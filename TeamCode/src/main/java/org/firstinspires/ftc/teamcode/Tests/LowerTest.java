package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BaseAuto;

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
