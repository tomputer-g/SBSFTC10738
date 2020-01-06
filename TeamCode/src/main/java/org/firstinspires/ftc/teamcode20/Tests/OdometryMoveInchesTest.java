package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.firstinspires.ftc.teamcode20.BaseOpMode;

@TeleOp
public class OdometryMoveInchesTest extends BaseAuto {
    private boolean APrimed = false;

    @Override
    public void init() {
        initDrivetrain();
        initIMU();
        initOdometry();
        //L2 is Y
    }

    @Override
    public void loop() {
        if(this.gamepad1.a){APrimed = true;}if(APrimed && !this.gamepad1.a){APrimed = false;
            moveInchesGO(0,12,0.2);
        }
        telemetry.addData("enc", L2.getCurrentPosition());
        telemetry.update();

    }

    protected final double odometryEncPerInch = 4096.0/Math.PI;
    protected int offsetX = 0, offsetY = 0;

    protected void moveInchesGO(double xInch, double yInch, double speed, double Kp){
        offsetX = 0;//stub
        offsetY = L2.getCurrentPosition();

        speed=Math.abs(speed);
        double fgt=1;

        int odometryXGoal = 0, odometryYGoal = offsetY + (int)(yInch * odometryEncPerInch);
        double theta=(yInch==0)?90:Math.abs(Math.atan(xInch/yInch));
        double vx=(xInch==0)?0:xInch/Math.abs(xInch)*Math.sin(theta)*speed;
        double vy=(yInch==0)?0:(yInch/Math.abs(yInch)*Math.cos(theta)*speed);
        while(!near(odometryYGoal, L2.getCurrentPosition(), 0.5*odometryEncPerInch)){
            //TODO: PID tune it
            setAllDrivePowerG(fgt*(-vx-vy),fgt*(vx-vy),fgt*(-vx+vy),fgt*(vx+vy),Kp);
            telemetry.addData("current",L2.getCurrentPosition());
            telemetry.addData("Y goal",odometryYGoal);
            telemetry.update();
        }
        telemetry.addLine("final Y error: "+(L2.getCurrentPosition() - odometryYGoal)+"/"+to3d((L2.getCurrentPosition() - odometryYGoal)/odometryEncPerInch)+"in.");
        telemetry.update();
        //brake
        setAllDrivePower(-LF.getPower()/Math.abs(LF.getPower()),-LB.getPower()/Math.abs(LB.getPower()),-RF.getPower()/Math.abs(RF.getPower()),-RB.getPower()/Math.abs(RB.getPower()));
        wait(75);
        setAllDrivePower(0);
    }
    protected void moveInchesGO(double xInch, double yInch, double speed){
        moveInchesGO(xInch, yInch, speed, 0.8);
    }
}
