package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;

import static java.lang.Math.sqrt;

@TeleOp
public class MiscTest extends BaseAuto {
    double speed;
    boolean[] bF={true}, e = {true}, f = {true};

    private void 三天之内刹了你(){
        setAllDrivePower(1,1,-1,-1);
        wait(200);
        setAllDrivePower(0);
    }

    @Override
    public void init(){
        initDrivetrain();
        speed=0.1;
       // 三天之内刹了你();
    }

    @Override
    public void loop(){
        //x+ left x- right y+ forward y- backward
        if(整(this.gamepad1.dpad_left,e))speed-=0.05;
        if(整(this.gamepad1.dpad_right,f))speed+=0.05;
        telemetry.addData("",speed);
        if(整(this.gamepad1.right_bumper,bF)) //moveInches(0,24,speed);
            //setAllDrivePower(speed);
            moveInches(0,12,speed);
        telemetry.update();
    }
}
