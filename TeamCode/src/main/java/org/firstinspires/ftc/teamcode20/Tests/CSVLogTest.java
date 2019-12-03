package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseOpMode;

public class CSVLogTest extends BaseOpMode{


    String filename = "CSVLogTest_"+System.currentTimeMillis()+".csv";

    ElapsedTime t;

    @Override public void init() {
        //try RobotLog
        initLogger(filename);
        writeLogHeader("Timestamp, data1, data2, data3");
        telemetry.update();
        t = new ElapsedTime();
    }

    @Override public void loop() {
        writeLog(t.milliseconds()+", test, test2, blah");
    }

    @Override public void stop() {
        stopLog();
    }
}
