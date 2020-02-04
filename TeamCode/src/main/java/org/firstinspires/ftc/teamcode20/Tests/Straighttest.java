package org.firstinspires.ftc.teamcode20.Tests;
import com.acmerobotics.dashboard.*;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
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