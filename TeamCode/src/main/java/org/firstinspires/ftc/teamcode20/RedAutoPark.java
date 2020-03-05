package org.firstinspires.ftc.teamcode20;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.Roadrunner.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode20.Tests.StopAnytimeTest;
import org.openftc.revextensions2.ExpansionHubEx;

@Autonomous
public class RedAutoPark extends BaseAuto{
    @Override public void runOpMode() throws InterruptedException {
        initAutonomous();
        while (!isStarted()) {
            Thread.sleep(200);
        }
        before_start();
        moveInchesGOXT(8,0.8,1,1200);
        requestOpModeStop();
    }
}