package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode20.BaseAuto;

@TeleOp
public class TouchSensorTest extends BaseAuto {
    ElapsedTime t;
    String logName = "TouchSensorLog"+System.currentTimeMillis()+".csv";
    TouchSensor touch;
    public void init() {
        initLogger(logName);
        writeLogHeader("time,touched");
        t = new ElapsedTime();
        touch = hardwareMap.get(TouchSensor.class, "touch");
    }
    @Override
    public void loop() {
        telemetry.addData("Digital Touch", touch.isPressed()?"Is Pressed":"Is Not Pressed");
        writeLog(t.milliseconds()+","+touch.isPressed());

    }


    @Override public void stop() {
        setAllDrivePower(0);
        stopLog();
    }
}
