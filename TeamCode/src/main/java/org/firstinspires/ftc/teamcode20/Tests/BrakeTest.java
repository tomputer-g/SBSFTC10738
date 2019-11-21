package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.TractionControl;

@TeleOp
public class BrakeTest extends TractionControl {
    private boolean[] a={true},b={true},c={true},d={true};
    private double runSpeed =0.5,brakeSpeed=0.2,start,end;
    int phase=0;
    ElapsedTime t;
    @Override
    public void init(){
        initDrivetrain();
        setMode_RUN_WITHOUT_ENCODER();
        t= new ElapsedTime();
    }
    @Override
    public void init_loop(){
        if(整(this.gamepad1.dpad_up,a))
            runSpeed +=0.05;
        if(整(this.gamepad1.dpad_down,b))
            runSpeed -=0.05;
        if(整(this.gamepad1.left_bumper,d))
            brakeSpeed -=0.05;
        if(整(this.gamepad1.right_bumper,c))
            brakeSpeed +=0.05;
        telemetry.addData("RUN SPEED: ", runSpeed);
        telemetry.addData("BRAKE SPEED: ", brakeSpeed);
        telemetry.update();
    }

    @Override
    public void loop(){
        if(整(this.gamepad1.right_bumper,c))phase++;
        if(整(this.gamepad1.left_bumper,d)) phase--;
        if(phase==0) {
            telemetry.addLine("IDLE");
            setAllDrivePower(0);
        }
        else if(phase==1) {
            setAllDrivePower(-runSpeed, -runSpeed, runSpeed, runSpeed);
            telemetry.addData("MOVING AT ",runSpeed);
            telemetry.addData("actual", LF.getPower());
            telemetry.addData("encoder", LF.getCurrentPosition());
        }
        else if(phase>1){
            telemetry.addData("BRAKING AT ",brakeSpeed);
            start = t.milliseconds();
            brakeTD(brakeSpeed,0);
            end=t.milliseconds();
            phase = 0;
        }
        telemetry.addData("PHASE ",phase);
        telemetry.addData("Brake Time:",end-start);
    }
}