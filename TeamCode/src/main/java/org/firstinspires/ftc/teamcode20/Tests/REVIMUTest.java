package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;

//@TeleOp(name = "BNO055 IMU", group = "Sensor")
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
        telemetry.addData("Heading", imuHeading);
        telemetry.update();
    }
}
