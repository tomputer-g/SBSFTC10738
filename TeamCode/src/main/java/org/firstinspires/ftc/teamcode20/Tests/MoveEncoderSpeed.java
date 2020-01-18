package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseOpMode;

import static java.lang.Thread.sleep;

@Disabled
@Autonomous(group="Test")
public class MoveEncoderSpeed extends BaseOpMode {

    private final double xEncPerInch = 1430.5/72, yEncPerInch =1305.25/72;// 996.75/72;
    private double moveEncSpeed = 0.05;
    private ElapsedTime encoderTimer, cycleTime, lowCycleTimer;
    private double moveEncP = 1, moveEncD = 0.5;
    private boolean upP, downP, leftP, rightP;
    private double kickstartSpeed = 0.18, lowSpeed = 0.03;
    private double cycleTimeMS = 20;
    private double encoderTmp = 0, encoderDisplayTmp = 0;
    private double lowCycleTimeMS = 10;

    @Override
    public void init() {
        initDrivetrain();
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        if(encoderTimer.milliseconds() >= 500){
            encoderDisplayTmp = ( (encoderTmp - LF.getCurrentPosition()) / (encoderTimer.nanoseconds()/1000000.0) ) / yEncPerInch;
            encoderTimer.reset();
            encoderTmp = LF.getCurrentPosition();
        }
        if(this.gamepad1.dpad_up){upP = true;}if(upP && !this.gamepad1.dpad_up){upP = false;
            moveEncSpeed += 0.01;
        }
        if(this.gamepad1.dpad_down){downP = true;}if(downP && !this.gamepad1.dpad_down) {downP = false;
            moveEncSpeed -= 0.01;
        }
        if(this.gamepad1.dpad_left){leftP = true;}if(leftP && !this.gamepad1.dpad_left) {leftP = false;
            if(this.gamepad1.left_bumper){
                lowSpeed -= 0.01;
            }else if(this.gamepad1.right_bumper){
                kickstartSpeed -= 0.01;
            }else{
                cycleTimeMS -= 1;
            }
        }
        if(this.gamepad1.dpad_right){rightP = true;}if(rightP && !this.gamepad1.dpad_right) {rightP = false;
            if(this.gamepad1.left_bumper){
                lowSpeed += 0.01;
            }else if(this.gamepad1.right_bumper){
                kickstartSpeed += 0.01;
            }else{
                cycleTimeMS += 1;
            }
        }

        telemetry.addData("loop time", cycleTime.milliseconds());//0.05ms
        try {
            movePWM(moveEncSpeed);

        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        cycleTime.reset();
        telemetry.addLine("speeds: ("+lowSpeed+", "+kickstartSpeed+")");
        telemetry.addData("enc timer", encoderTimer.nanoseconds());
        telemetry.addData("PWM cycle time", cycleTimeMS);
        telemetry.addData("speed", moveEncSpeed);
        telemetry.addData("inch/s",encoderDisplayTmp);
        telemetry.update();
    }

    @Override
    public void start() {
        encoderTimer = new ElapsedTime();
        cycleTime = new ElapsedTime();
        lowCycleTimer = new ElapsedTime();
    }

    private void moveYByEncoder(double vy){//1 inch per second
        encoderTimer.reset();
        setAllDrivePower(-moveEncSpeed, -moveEncSpeed, moveEncSpeed, moveEncSpeed);
        int a = LF.getCurrentPosition();
        double vyGoal = vy * yEncPerInch;
        while(LF.getCurrentPosition() == a);
        double msTime = encoderTimer.milliseconds();
        double vyActual = 1000.0/msTime;
        telemetry.addLine("Enc: took "+msTime+"ms");
        telemetry.addLine("Enc: Goal = "+vy+", Actual = "+vyActual/yEncPerInch);
    }

    private void movePWM(double vy) throws InterruptedException {//3,9,18
        if(vy > kickstartSpeed){
            setAllDrivePower(-vy, -vy, vy, vy);
        }else if(vy >= lowSpeed){
            telemetry.addLine("PWM move active");//set 20; change delay between 20 and 0
            double dutyPercent = (vy - lowSpeed) / (kickstartSpeed-lowSpeed);
            telemetry.addLine("Duty "+(int)(100*dutyPercent)+"%; high "+to3d(dutyPercent * cycleTimeMS)+", low "+to3d((1-dutyPercent) * cycleTimeMS));
            sleep(0, (int)lowCycleTimeMS*1000);
            setAllDrivePower(-kickstartSpeed, -kickstartSpeed, kickstartSpeed, kickstartSpeed);
            sleep(0,(int)(1000 * dutyPercent * cycleTimeMS));
            setAllDrivePower(-lowSpeed, -lowSpeed, lowSpeed, lowSpeed);
            lowCycleTimer.reset();
            lowCycleTimeMS = (1-dutyPercent) * cycleTimeMS;
        }else{
            setAllDrivePower(0);
            telemetry.addLine("speed is below PWM minimum!");
        }
    }
}
