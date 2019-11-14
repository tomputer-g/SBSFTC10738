package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.firstinspires.ftc.teamcode20.TractionControl;

@TeleOp
public class BrakeTest extends TractionControl {
    private boolean[] a={true},b={true},c={true};
    double speed=0.5;
    int phase=0;
    @Override
    public void init(){
        if(整(this.gamepad1.dpad_up,a)) speed+=0.1;
        if(整(this.gamepad1.dpad_down,c)) speed+=0.1;
    }

    @Override
    public void loop(){
        if(整(this.gamepad1.right_bumper,c))
            phase++;
        if(phase==1)
            setAllDrivePower(-speed,-speed,speed,speed);
        if(phase>=2){
            brakeTD(1,0);
            phase = 0;
        }
    }
}
