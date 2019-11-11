package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;

@TeleOp(name = "BNO055 IMU", group = "Sensor")
public class REVIMUTest extends BaseAuto{
    private ModernRoboticsI2cRangeSensor rangeSensor;
    @Override
    public void init() {
        扬();
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
    }

    @Override
    public void loop() {
        if(this.gamepad1.a){
            while(this.gamepad1.a);
            setNewGyro0();
        }
        getHeading();
        telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
        telemetry.addData("raw optical", rangeSensor.rawOptical());
        telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Gyro heading", imuHeading);
        telemetry.addData("Gyro offset", imuOffset);
        telemetry.update();
    }
}
