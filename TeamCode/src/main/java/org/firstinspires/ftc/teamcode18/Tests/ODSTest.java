package org.firstinspires.ftc.teamcode18.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Created by Ziming Gao on 1/20/2018.
 */
@Autonomous(name = "ODS test", group = "test")
@Disabled
public class ODSTest extends OpMode {
    private AnalogInput homeSlide;


    @Override
    public void init() {
        homeSlide = hardwareMap.analogInput.get("home");
    }

    @Override
    public void loop() {
        telemetry.addData("ODS voltage",homeSlide.getVoltage());
        telemetry.update();
    }
}
