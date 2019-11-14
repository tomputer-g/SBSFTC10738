package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;

import static java.lang.Math.sqrt;

@TeleOp
public class MiscTest extends BaseAuto {
    double speed,x,y;
    boolean[] bF={true}, lF = {true}, e = {true}, f = {true}, ee = {true}, ff = {true};

    private void 三天之内刹了你(){
        setAllDrivePower(1,1,-1,-1);
        wait(200);
        setAllDrivePower(0);
    }

    @Override
    public void init(){
        initDrivetrain();
        speed=0.25;
        y = 0;
        x = 12;
       // 三天之内刹了你();
    }

    @Override
    public void loop(){
        //x+ left x- right y+ forward y- backward
        if(整(this.gamepad1.dpad_left,e))speed-=0.05;
        if(整(this.gamepad1.dpad_right,f))speed+=0.05;
        if(整(this.gamepad1.dpad_up,ee))y++;
        if(整(this.gamepad1.dpad_down,ff))y--;
        telemetry.addData("speed: ",speed);
        telemetry.addData("x:", x);
        telemetry.addData("y: ", y);
        telemetry.addData("LF",LF.getCurrentPosition());
        telemetry.addData("LB",LB.getCurrentPosition());
        telemetry.addData("RF",RF.getCurrentPosition());
        telemetry.addData("RB",RB.getCurrentPosition());
        if(整(this.gamepad1.right_bumper,bF))
            moveInches(0,y,speed);
        if(整(this.gamepad1.left_bumper,lF))
            moveInches(x,0,speed);
        telemetry.update();
    }
}
