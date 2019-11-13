package org.firstinspires.ftc.teamcode20.Tests;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode20.*;


@TeleOp(name = "MotorCountTest", group = "Tests")
public class MotorCountTest extends TractionControl {
    ElapsedTime t;
    double speed,brakeSpeed;
    int actionIndex = 0;
    double icLF=0,icLB=0,icRF=0,icRB=0;
    boolean[] primed = {true};boolean[] primedr = {true};boolean[] primedu = {true};boolean[] primedd = {true};boolean[] primedlb = {true};boolean[] primedrb = {true};
    //String logName = "MotorCountTest"+System.currentTimeMillis()+".csv";
    public void init() {
        //initLogger(logName);
        //writeLogHeader("time,LF_count,LB_count,RF_count,RB_count,LF_power,LB_power,RF_power,RB_power");
        initDrivetrain();
        t = new ElapsedTime();
        initIMU();
        speed = 0.3;
        brakeSpeed = 1;
        actionIndex = 1;
    }
    @Override
    public void init_loop() {
        if(checkButton(this.gamepad1.dpad_up,primed))
            speed+=0.05;
        if(checkButton(this.gamepad1.dpad_down,primed))
            speed-=0.05;
        if(checkButton(this.gamepad1.dpad_left,primed))
            brakeSpeed-=0.05;
        if(checkButton(this.gamepad1.dpad_right,primed))
            brakeSpeed+=0.05;
        if(checkButton(this.gamepad1.left_bumper,primed))
            actionIndex = 1;
        if(checkButton(this.gamepad1.right_bumper,primed))
            actionIndex = 2;
        icLF=getMC(LF);icLB=getMC(LB);icRF=getMC(RF);icRB=getMC(RB);
        telemetry.addLine("vr: "+speed);
        telemetry.addLine("brake vr: "+brakeSpeed);
        telemetry.addLine("actionIndex: "+actionIndex);
        telemetry.addLine("MC: "+getMC(LF)+","+getMC(LB)+","+getMC(RF)+","+getMC(RB));
        telemetry.update();

    }

    @Override
    public void loop() {
        if(checkButton(this.gamepad1.left_bumper,primed)) actionIndex = 0;
        if(actionIndex == 1)
            setAllDrivePower(-speed,-speed,speed,speed);
        else{
            brakeTD(brakeSpeed,10);
            stop();
        }
        telemetry.addLine("MC: "+getMC(LF)+","+getMC(LB)+","+getMC(RF)+","+getMC(RB));
    }
    @Override public void stop() {
        setAllDrivePower(0);
        //stopLog();
    }
}