package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseOpMode;


@Deprecated
public class AutoGrabberTest extends BaseOpMode {

    Servo servo;
    Rev2mDistanceSensor dist;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
        dist = hardwareMap.get(Rev2mDistanceSensor.class, "dist");
        servo.setPosition(0.5);
    }

    @Override
    public void loop() {
        if(dist.getDistance(DistanceUnit.CM) < 2.8){
            telemetry.addLine("Autograbbing");
            servo.setPosition(1);
        }

        telemetry.addData("Dist",dist.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}
