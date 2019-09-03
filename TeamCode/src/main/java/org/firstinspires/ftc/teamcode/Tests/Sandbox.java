package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BaseAuto;

//@TeleOp
public class Sandbox extends BaseAuto {

    @Override
    public void init() {
        initLifter();
        initGrabber();
        lowerRobot();
    }

    @Override
    public void loop() {

    }
}
