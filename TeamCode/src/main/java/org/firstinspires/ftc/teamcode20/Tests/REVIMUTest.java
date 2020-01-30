package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;

public class REVIMUTest extends BaseAuto{
    boolean a;
    @Override
    public void init() {
        initIMU();
    }

    @Override
    public void loop() {
        if(this.gamepad1.a){a = true;}if(!this.gamepad1.a && a){
            a = false;
            setNewGyro0();
        }
        getHeading();
        telemetry.addData("Heading", imuHeading);
        telemetry.addData("offset",imuOffset);
        telemetry.update();
    }
}
