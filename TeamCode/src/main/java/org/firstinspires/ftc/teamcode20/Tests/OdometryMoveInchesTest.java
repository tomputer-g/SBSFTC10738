package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.firstinspires.ftc.teamcode20.BaseOpMode;

@TeleOp
public class OdometryMoveInchesTest extends BaseAuto {
    private boolean APrimed = false,BPrimed = false, upP = false, downP = false, leftP = false, rightP = false;
    private double speed = 0.4, moveInches_kP = 0.3;

    @Override
    public void init() {
        initDrivetrain();
        initOdometry();
    }

    @Override
    public void loop() {
        if(this.gamepad1.a){APrimed = true;}if(APrimed && !this.gamepad1.a){ APrimed = false;
            moveInchesGO(0,12,speed);
        }
        if(this.gamepad1.b){BPrimed = true;}if(BPrimed && !this.gamepad1.b){ BPrimed = false;
            moveInchesGO(12, 0, speed);
        }
        if(this.gamepad1.dpad_left){leftP = true;}if(leftP && !this.gamepad1.dpad_left){ leftP = false;
            speed -= 0.05;
        }
        if(this.gamepad1.dpad_right){rightP = true;}if(rightP && !this.gamepad1.dpad_right){ rightP = false;
            speed += 0.05;2
        }
        if(this.gamepad1.dpad_down){downP = true;}if(downP && !this.gamepad1.dpad_down){ downP = false;
            moveInches_kP -= 0.01;
        }
        if(this.gamepad1.dpad_up){upP = true;}if(upP && !this.gamepad1.dpad_up){ upP = false;
            moveInches_kP += 0.01;
        }
        //telemetry.addData("enc X", platform_grabber.getCurrentPosition());
        telemetry.addData("enc Y", L2.getCurrentPosition());
        telemetry.addData("speed", speed);
        telemetry.addData("kP", moveInches_kP);
        telemetry.update();
    }

    protected final double odometryEncPerInch = 4096.0/Math.PI;
    protected int offsetX = 0, offsetY = 0;

    protected void moveInchesGO(double xInch, double yInch, double speed){
        offsetX = platform_grabber.getCurrentPosition();
        offsetY = L2.getCurrentPosition();
        speed=Math.abs(speed);
        double multiply_factor=1;
        ElapsedTime stable_timer = null;
        int stable_timer_time = 1500;
        int odometryXGoal = offsetX + (int)(xInch * odometryEncPerInch), odometryYGoal = offsetY + (int)(yInch * odometryEncPerInch);
        double theta=(yInch==0)?90:Math.abs(Math.atan(xInch/yInch));
        double vx=(xInch==0)?0:(xInch/Math.abs(xInch)*Math.sin(theta)*speed);
        double vy=(yInch==0)?0:(yInch/Math.abs(yInch)*Math.cos(theta)*speed);
        while( stable_timer == null || stable_timer.milliseconds() < stable_timer_time){//!near(odometryYGoal, L2.getCurrentPosition(), 0.5*odometryEncPerInch) && !near(odometryXGoal, platform_grabber.getCurrentPosition(), 0.5*odometryEncPerInch)
            multiply_factor = Math.min(1, Math.max(-1, moveInches_kP * (L2.getCurrentPosition() - odometryYGoal)/odometryEncPerInch));
            setAllDrivePowerG(multiply_factor*(-vx+vy),multiply_factor*(vx+vy),multiply_factor*(-vx-vy),multiply_factor*(vx-vy),0.8);
            if(near(multiply_factor, 0, 0.1) && stable_timer == null){
                stable_timer = new ElapsedTime();
            }
            if(stable_timer != null)telemetry.addData("stable timer", stable_timer.milliseconds());
            telemetry.addData("current",L2.getCurrentPosition());
            telemetry.addData("Y goal",odometryYGoal);
            telemetry.update();
        }
        setAllDrivePower(0);
    }

}
