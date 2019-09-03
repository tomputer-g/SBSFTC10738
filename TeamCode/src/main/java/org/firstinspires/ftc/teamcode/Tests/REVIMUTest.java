package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.BaseAuto;

import java.util.Locale;
@Disabled
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
