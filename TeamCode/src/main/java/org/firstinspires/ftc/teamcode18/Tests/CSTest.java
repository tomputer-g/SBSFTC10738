package org.firstinspires.ftc.teamcode18.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Ziming Gao on 1/22/2018.
 */
@Autonomous(name = "CSTest", group = "test")
@Disabled
public class CSTest extends OpMode {
    private ColorSensor cs;
    @Override
    public void init() {
        cs = hardwareMap.get(ColorSensor.class, "cs");
        cs.enableLed(true);
    }

    @Override
    public void loop() {
        telemetry.addData("red",cs.red());
        telemetry.addData("blue",cs.blue());
        telemetry.update();
    }
}
