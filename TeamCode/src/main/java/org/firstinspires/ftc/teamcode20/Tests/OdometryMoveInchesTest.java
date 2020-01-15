package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode20.BaseAuto;

@TeleOp
public class OdometryMoveInchesTest extends BaseAuto {
    private boolean APrimed = false, upP = false, downP = false, leftP = false, rightP = false, x = false;
    private double speed = 0.4, kP = 0.5, kI = 0, kD = 0.0025;

    //FOR 0.3: use P = 0.7, D = 0.003

    @Override
    public void init() {
        initDrivetrain();
        initOdometry();
        initIMU();
    }
    @Override
    public void loop() {
        if(this.gamepad1.a){APrimed = true;}if(APrimed && !this.gamepad1.a){ APrimed = false;
            moveInchesGO(-72,speed);
        }
        if(this.gamepad1.dpad_left){leftP = true;}if(leftP && !this.gamepad1.dpad_left){ leftP = false;
            speed -= 0.05;
        }
        if(this.gamepad1.dpad_right){rightP = true;}if(rightP && !this.gamepad1.dpad_right){ rightP = false;
            speed += 0.05;
        }
        if(this.gamepad1.dpad_down){downP = true;}if(downP && !this.gamepad1.dpad_down){ downP = false;

            if(this.gamepad1.left_bumper){
                kP -= 0.01;
            }else if(this.gamepad1.right_bumper){
                kI -= 0.00001;
            }else{
                kD -= 0.0001;
            }
        }
        if(this.gamepad1.dpad_up){upP = true;}if(upP && !this.gamepad1.dpad_up){ upP = false;
            if(this.gamepad1.left_bumper){
                kP += 0.01;
            }else if(this.gamepad1.right_bumper){
                kI += 0.00001;
            }else{
                kD += 0.0001;
            }
        }
        kP = Math.round(kP * 1000) / 1000.0;
        kI = Math.round(kI * 100000) / 100000.0;
        kD = Math.round(kD * 10000) / 10000.0;
        telemetry.addData("enc Y", getYOdometry());
        telemetry.addData("speed", speed);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.update();
    }
    protected final double odometryEncPerInch = 1320;//4096.0/Math.PI;
    protected int offsetY = 0;

    protected void moveInchesGO(double yInch, double speed){
        offsetY = getYOdometry();
        speed=Math.abs(speed);
        double multiply_factor=1;
        int odometryYGoal = offsetY + (int)(yInch * odometryEncPerInch);
        double vx = 0;
        double vy=(yInch==0)?0:(yInch/Math.abs(yInch)*speed);
        long IError = 0;
        setAllDrivePower((vy),(vy),(-vy),(-vy));
        int previousPos = getYOdometry();
        int Dterm;
        while(!this.gamepad1.b){
            multiply_factor = -Math.min(1, Math.max(-1, (kP * (getYOdometry() - odometryYGoal)/odometryEncPerInch) + (kI * IError ) + (kD * (getYOdometry() - previousPos))));
            Dterm = getYOdometry() - previousPos;
            previousPos = getYOdometry();
            IError += (getYOdometry() - odometryYGoal)/odometryEncPerInch;
            setAllDrivePowerG(multiply_factor*(-vx-vy),multiply_factor*(vx-vy),multiply_factor*(-vx+vy),multiply_factor*(vx+vy));

            telemetry.addData("kP", kP);
            telemetry.addData("P term", (getYOdometry() - odometryYGoal)/odometryEncPerInch);
            telemetry.addData("kI", kI);
            telemetry.addData("I term", IError);
            telemetry.addData("kD", kD);
            telemetry.addData("D term",Dterm);
            telemetry.addData("current",getYOdometry());
            telemetry.addData("Y goal",odometryYGoal);
            telemetry.update();
        }
        setAllDrivePower(0);
    }
}
