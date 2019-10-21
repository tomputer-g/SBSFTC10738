package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.BaseOpMode;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.Buffer;
import java.sql.Date;

@Autonomous
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
