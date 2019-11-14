package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode20.BaseAuto;

@TeleOp
public class MiscTest extends BaseAuto {
    double speed;
    boolean[] bF={true};

    private void 三天之内刹了你(){
        setAllDrivePower(1,1,-1,-1);
        wait(200);
        setAllDrivePower(0);
    }

    @Override
    public void init(){
        initDrivetrain();
        speed=0.07;
       // 三天之内刹了你();
    }

    @Override
    public void loop(){
        //x+ left x- right y+ forward y- backward
        if(整(this.gamepad1.right_bumper,bF)) //moveInches(0,24,speed);
            //setAllDrivePower(speed);
            moveInchesHighSpeed(0,12,0.07,1,1,0.4,0.4,0);
    }
}
