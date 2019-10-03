package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseAuto;

@TeleOp(name = "BNO055 IMU", group = "Sensor")
public class REVIMUTest extends BaseAuto{
    @Override
    public void init() {
        initIMU();
    }

    @Override
    public void loop() {
        if(this.gamepad1.a){
            while(this.gamepad1.a);
            setNewGyro0();
        }
        getHeading();
        telemetry.addData("Gyro heading", imuHeading);
        telemetry.addData("Gyro offset", imuOffset);
        telemetry.update();
    }
}
