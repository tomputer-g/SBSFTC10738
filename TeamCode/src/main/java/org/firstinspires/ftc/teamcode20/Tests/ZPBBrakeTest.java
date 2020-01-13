package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseOpMode;

import static java.lang.Thread.sleep;

@TeleOp
public class ZPBBrakeTest extends OpMode {
    private boolean upP = false, downP = false, leftP = false, rightP = false;
    private double brakePwr = -0.03, start = 0.25;
    private int brakeDriftOdo = 0;

    protected DcMotor LF, LB, RF, RB, L2;
    private int brakeStart,brakeEnd;
    @Override
    public void init() {
        LF = hardwareMap.get(DcMotor.class,"LF");
        LB = hardwareMap.get(DcMotor.class,"LB");
        RF = hardwareMap.get(DcMotor.class,"RF");
        RB = hardwareMap.get(DcMotor.class,"RB");
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        initOdometry();
    }

    //30,700 is brake arming speed (0.25 at 13.1V)
    //6000 is brake distance at arming speed

    @Override
    public void loop() {
        if(this.gamepad1.a){
            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            setAllDrivePower(0,start);
            wait(2000);
            brakeStart = L2.getCurrentPosition();
        }
        if(this.gamepad1.b){
            moveInches025(110);
        }
        /*if(this.gamepad1.dpad_left){leftP = true;}if(leftP && !this.gamepad1.dpad_left){ leftP = false;
            brakePwr -= 0.01;
        }
        if(this.gamepad1.dpad_right){rightP = true;}if(rightP && !this.gamepad1.dpad_right){ rightP = false;
            brakePwr += 0.01;
        }
        if(this.gamepad1.dpad_down){downP = true;}if(downP && !this.gamepad1.dpad_down){ downP = false;
            start -= 0.01;
        }
        if(this.gamepad1.dpad_up){upP = true;}if(upP && !this.gamepad1.dpad_up){ upP = false;
            start += 0.01;
        }

         */
        telemetry.addData("start",start);
        telemetry.addData("brake",brakePwr);
        telemetry.addData("Y enc", L2.getCurrentPosition());
        telemetry.addData("brake dist", L2.getCurrentPosition() - brakeStart);
        telemetry.update();
    }

    protected void moveInches025(double yInch){
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int offsetY = L2.getCurrentPosition(), odometryYGoal = offsetY + (int)(yInch * 1313);
        setAllDrivePower(0,0.25);
        while(L2.getCurrentPosition() < (odometryYGoal-6000));
        brakeStart = L2.getCurrentPosition() - offsetY;
        LF.setTargetPosition(LF.getCurrentPosition());
        LF.setPower(.5);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setTargetPosition(LB.getCurrentPosition());
        LB.setPower(.5);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setTargetPosition(RF.getCurrentPosition());
        RF.setPower(.5);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setTargetPosition(RB.getCurrentPosition());
        RB.setPower(.5);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    protected void setAllDrivePower(double pX, double pY){
        if(Math.abs(pX)+Math.abs(pY) > 1)
            throw new IllegalArgumentException("setAllDrivePower(px,py) sets a power beyond 1");
        LF.setPower(pX-pY);
        LB.setPower(-pX-pY);
        RF.setPower(pX+pY);
        RB.setPower(-pX+pY);
    }

    protected void initOdometry(){
        //L2 is Y encoder
        L2 = hardwareMap.get(DcMotor.class, "L2");
        L2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected void wait(int time){
        try {sleep(time);} catch (InterruptedException e) {e.printStackTrace();}
    }
}
