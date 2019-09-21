package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode19.BaseAuto;

@Autonomous
@Disabled
public class IMUCollisionTest extends BaseAuto {
    private double maxAcc;
    @Override
    public void init() {
        initIMU();
    }

    @Override
    public void loop() {
        Acceleration acc = imu.getLinearAcceleration();
        telemetry.addData("Acceleration", "("+to3d(acc.xAccel) + ", " + to3d(acc.yAccel) + ", " + to3d(acc.zAccel) + ")");
        double magnitude = Math.sqrt(Math.pow(acc.xAccel, 2) + Math.pow(acc.yAccel, 2) + Math.pow(acc.zAccel, 2));
        telemetry.addData("Magnitude", to3d(magnitude));
        maxAcc = Math.max(maxAcc,magnitude);
        telemetry.addLine("----------------------------------");
        telemetry.addData("Max magnitude", to3d(maxAcc));
        telemetry.update();
        if(this.gamepad1.a){
            maxAcc = 0;
        }
    }
}
