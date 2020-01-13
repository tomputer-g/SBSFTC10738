package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode19.BaseOpMode;
import org.firstinspires.ftc.teamcode20.BaseAuto;

@Autonomous(group = "Test")
public class Encoders extends OpMode {

    private DcMotor LF, LB, RF, RB, a,b,c,d;
    @Override
    public void init() {
        LF = hardwareMap.get(DcMotor.class,"LF");
        LB = hardwareMap.get(DcMotor.class,"LB");
        RF = hardwareMap.get(DcMotor.class,"RF");
        RB = hardwareMap.get(DcMotor.class,"RB");
        a = hardwareMap.get(DcMotor.class,"a");
        b = hardwareMap.get(DcMotor.class,"b");
        c = hardwareMap.get(DcMotor.class,"c");
        d = hardwareMap.get(DcMotor.class,"d");
    }

    @Override
    public void loop() {
        telemetry.addData("Hub 2","( "+LF.getCurrentPosition()+", "+LB.getCurrentPosition()+", "+RF.getCurrentPosition()+", "+RB.getCurrentPosition()+")");
        telemetry.addData("Hub 4", "( "+a.getCurrentPosition()+", "+b.getCurrentPosition()+", "+c.getCurrentPosition()+","+d.getCurrentPosition()+")");
        telemetry.update();
    }


}
