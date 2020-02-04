package org.firstinspires.ftc.teamcode20.Tests;

import com.acmerobotics.dashboard.config.Config;

i
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;



/*

 * This is a simple routine to test translational drive capabilities.

 */

@Config
@Autonomous(group = "drive")
public class Straighttest extends LinearOpMode {
    public static double DISTANCE = 60;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(DISTANCE)
                .build();
        waitForStart();
        if (isStopRequested()) return;
        drive.followTrajectorySync(trajectory);
    }
}
