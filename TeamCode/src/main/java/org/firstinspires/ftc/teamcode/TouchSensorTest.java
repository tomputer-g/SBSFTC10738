package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class TouchSensorTest extends BaseAuto{
    ElapsedTime t;
    String logName = "TouchSensorLog"+System.currentTimeMillis()+".csv";
    DigitalChannel touch;
    public void init() {
        initLogger(logName);
        writeLogHeader("time,touched");
        t = new ElapsedTime();
        touch = hardwareMap.get(DigitalChannel.class, "touch");
        touch.setMode(DigitalChannel.Mode.INPUT);
    }
    @Override
    public void loop() {
        telemetry.addData("Digital Touch", touch.getState()?"Is Pressed":"Is Not Pressed");
        writeLog(t.milliseconds()+","+touch.getState());
    }


    @Override public void stop() {
        setAllDrivePower(0);
        stopLog();
    }
}
