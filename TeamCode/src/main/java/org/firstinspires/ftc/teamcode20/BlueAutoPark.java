package org.firstinspires.ftc.teamcode20;


import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.Roadrunner.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode20.Tests.StopAnytimeTest;
import org.openftc.revextensions2.ExpansionHubEx;

@Autonomous
public class BlueAutoPark extends BaseAuto{
    @Override public void runOpMode() throws InterruptedException {
        showTelemetry = false;
        initDrivetrain();//181.64ms
        initIMU();//!!!!!1.259s : stops if thread has Interrupt flag
        initGrabber();//1.14ms
        initPlatformGrabber();//34.20ms
        initOdometry();//100.89ms
        xOdometryEnableServo.setPosition(xOdoEnable);
        setNewGyro0();
        initHubs();
        //initVuforia();
        //initViewMarks();
        Log.i("Auto init", "done");
        while (!isStarted()) {
            Thread.sleep(200);
        }
        //before_start();
        moveInchesGOXT(-10,0.8,1,1500);
        requestOpModeStop();
    }
}
