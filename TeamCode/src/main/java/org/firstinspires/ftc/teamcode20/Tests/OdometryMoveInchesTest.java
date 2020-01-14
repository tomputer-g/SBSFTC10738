package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.firstinspires.ftc.teamcode20.BaseOpMode;

@TeleOp
public class OdometryMoveInchesTest extends BaseAuto {
    private boolean APrimed = false, upP = false, downP = false, leftP = false, rightP = false, x = false;
    private double speed = 0.3, moveInches_kP = 0.11, moveInches_kI = 0.000003;

    @Override
    public void init() {
        initDrivetrain();
        initOdometry();
        initIMU();
    }
    @Override
    public void loop() {
        if(this.gamepad1.a){APrimed = true;}if(APrimed && !this.gamepad1.a){ APrimed = false;
            moveInchesGO(-48,speed);
        }
        if(this.gamepad1.dpad_left){leftP = true;}if(leftP && !this.gamepad1.dpad_left){ leftP = false;
            speed -= 0.05;
        }
        if(this.gamepad1.dpad_right){rightP = true;}if(rightP && !this.gamepad1.dpad_right){ rightP = false;
            speed += 0.05;
        }
        if(this.gamepad1.dpad_down){downP = true;}if(downP && !this.gamepad1.dpad_down){ downP = false;

            if(this.gamepad1.left_bumper){
                moveInches_kP -= 0.001;
            }else if(this.gamepad1.right_bumper){
                moveInches_kI -= 0.000001;
            }
        }
        if(this.gamepad1.dpad_up){upP = true;}if(upP && !this.gamepad1.dpad_up){ upP = false;
            if(this.gamepad1.left_bumper){
                moveInches_kP += 0.001;
            }else if(this.gamepad1.right_bumper){
                moveInches_kI += 0.000001;
            }
        }
        moveInches_kP = Math.round(moveInches_kP * 1000) / 1000.0;
        moveInches_kI = Math.round(moveInches_kI * 1000000) / 1000000.0;
        telemetry.addData("enc Y", getYOdometry());
        telemetry.addData("speed", speed);
        telemetry.addData("kP", moveInches_kP);
        telemetry.addData("kI", moveInches_kI);
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
        while(Math.abs((getYOdometry() - odometryYGoal)/odometryEncPerInch) > 6){
            IError += (getYOdometry() - odometryYGoal)/odometryEncPerInch;
        }
        while(!this.gamepad1.b){
            IError += (getYOdometry() - odometryYGoal)/odometryEncPerInch;
            multiply_factor = -Math.min(1, Math.max(-1, (-moveInches_kP * (getYOdometry() - odometryYGoal)/odometryEncPerInch) + (moveInches_kI * IError )));

            setAllDrivePowerG(multiply_factor*(-vx-vy),multiply_factor*(vx-vy),multiply_factor*(-vx+vy),multiply_factor*(vx+vy));

            telemetry.addData("kP", moveInches_kP);
            telemetry.addData("P term", (getYOdometry() - odometryYGoal)/odometryEncPerInch);
            telemetry.addData("kI", moveInches_kI);
            telemetry.addData("I term", IError);
            telemetry.addData("current",getYOdometry());
            telemetry.addData("Y goal",odometryYGoal);
            telemetry.update();
        }
        setAllDrivePower(0);
    }
}
