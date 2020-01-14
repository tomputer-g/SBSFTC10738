package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.firstinspires.ftc.teamcode20.BaseOpMode;

@TeleOp
public class OdometryMoveInchesTest extends BaseAuto {
    private boolean APrimed = false,BPrimed = false, upP = false, downP = false, leftP = false, rightP = false, x = false;
    private double speed = 0.4, moveInches_kP = 0.1, moveInches_kI = 0;

    @Override
    public void init() {
        initDrivetrain();
        initOdometry();
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
                moveInches_kI -= 0.00001;
            }
        }
        if(this.gamepad1.dpad_up){upP = true;}if(upP && !this.gamepad1.dpad_up){ upP = false;
            if(this.gamepad1.left_bumper){
                moveInches_kP += 0.001;
            }else if(this.gamepad1.right_bumper){
                moveInches_kI += 0.00001;
            }
        }
        moveInches_kP = Math.round(moveInches_kP * 1000) / 1000.0;
        moveInches_kI = Math.round(moveInches_kI * 100000) / 100000.0;
        telemetry.addData("enc Y", getYOdometry());
        telemetry.addData("speed", speed);
        telemetry.addData("kP", moveInches_kP);
        telemetry.addData("kI", moveInches_kI);
        telemetry.update();
    }
    protected final double odometryEncPerInch = 1316;//4096.0/Math.PI;
    protected int offsetY = 0;

    protected void moveInchesGO(double yInch, double speed){
        offsetY = getYOdometry();
        speed=Math.abs(speed);
        double multiply_factor=1;
        ElapsedTime stable_timer = null;
        int stable_timer_time = 5000;
        int odometryYGoal = offsetY + (int)(yInch * odometryEncPerInch);
        double vx = 0;
        double vy=(yInch==0)?0:(yInch/Math.abs(yInch)*speed);
        long IError = 0;
        setAllDrivePower((vy),(vy),(-vy),(-vy));
        while(Math.abs((getYOdometry() - odometryYGoal)/odometryEncPerInch) > 6);
        while(!this.gamepad1.b){
            IError += (getYOdometry() - odometryYGoal)/odometryEncPerInch;
            multiply_factor = -Math.min(1, Math.max(-1, (-moveInches_kP * (getYOdometry() - odometryYGoal)/odometryEncPerInch) + (moveInches_kI * IError )));

            setAllDrivePower(multiply_factor*(-vx-vy),multiply_factor*(vx-vy),multiply_factor*(-vx+vy),multiply_factor*(vx+vy));
            /*if(near(multiply_factor, 0, 0.1) && stable_timer == null){
                stable_timer = new ElapsedTime();
            }
            if(stable_timer != null)telemetry.addData("stable timer", stable_timer.milliseconds());

             */
            telemetry.addData("speed", speed);
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
