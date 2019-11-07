package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;

@TeleOp(name = "闲话终日有 干就自然无", group = "Tests")

public class BrickGrabTest extends BaseAuto {
    Rev2mDistanceSensor left,right;
    ElapsedTime t;
    double vr,dl,dr, vt;
    boolean[] rB = {true};
    boolean[] dpadUP = {true};
    boolean[] dpadDOWN = {true};
    //double indicator;
    //String logName = "FaceWallLog"+System.currentTimeMillis()+".csv";
    public void init() {
        //initLogger(logName);
        //writeLogHeader("time,LF_count,LB_count,RF_count,RB_count,LF_power,LB_power,RF_power,RB_power,front_UltS,left_UltS,right_UltS,front_left_REV,front_right_REV");
        initDrivetrain();
        t = new ElapsedTime();
        left = hardwareMap.get(Rev2mDistanceSensor.class,"left");
        right = hardwareMap.get(Rev2mDistanceSensor.class,"right");
        initIMU();
        vr = 0.12;
    }
    @Override
    public void loop() {
        if(cBP(this.gamepad1.dpad_up,dpadUP)){
            vr +=0.05;
        }
        if(cBP(this.gamepad1.dpad_down,dpadDOWN)){
            vr -=0.05;
        }
        telemetry.addData("Left", "%.2f", dl);
        telemetry.addData("Right","%.2f",dr);
        telemetry.addData("Rotational speed","%.2f", vr);
        telemetry.addData("Translational speed", "%.2f",vt);
        telemetry.addData("WAITING FOR ACTIONS",0);

        if(cBP(this.gamepad1.right_bumper, rB)) {
            setAllDrivePower(0);
            dl = left.getDistance(DistanceUnit.INCH);
            dr = right.getDistance(DistanceUnit.INCH);
            vt = (dl+dr)/(2*11.25)*vr;
            while (!near(dl, dr, .5)) {
                dl = left.getDistance(DistanceUnit.INCH);
                dr = right.getDistance(DistanceUnit.INCH);
                telemetry.update();
                if(dl > 30 || dr > 30);
                else if (dl < dr) setAllDrivePower(vr - vt, vr + vt, vr - vt, vr + vt);
                else setAllDrivePower(-vr + vt, -vr - vt, -vr + vt, -vr - vt);
            }
            setAllDrivePower(0);
            while (dr<20) {
             //   dl = left.getDistance(DistanceUnit.INCH);
                dr = right.getDistance(DistanceUnit.INCH);
                setAllDrivePower1(vr, vr, vr, vr);
                telemetry.update();
            }
        }

        telemetry.update();
    }
    @Override
    public void stop() {
        setAllDrivePower(0);
    }
}
