package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.firstinspires.ftc.teamcode20.TractionControl;

@TeleOp
public class BrakeTest extends TractionControl {
    private boolean[] a={true},b={true},c={true};
    double speed=0.1,brakeSpeed=0.1,start,end;
    int phase=0;
    ElapsedTime t;
    @Override
    public void init(){
        initDrivetrain();
        t= new ElapsedTime();
    }
    @Override
    public void init_loop(){
        if(整(this.gamepad1.dpad_up,a))
            speed+=0.1;
        if(整(this.gamepad1.dpad_down,b))
            speed-=0.1;
        telemetry.addData("speed: ", speed);
        telemetry.update();
    }

    @Override
    public void loop(){
        setMode_RUN_WITH_ENCODER();
        if(整(this.gamepad1.right_bumper,c)){
            phase++;
            end=0;start=0;
        }
        if(phase==0)
            telemetry.addLine("IDLE");
        if(phase==1) {
            setAllDrivePower(-speed, -speed, speed, speed);
            telemetry.addLine("MOVING");
        }
        else if(phase>1){
            telemetry.addLine("BRAKING");
            start = t.milliseconds();
            brakeTD(brakeSpeed,0);
            end=t.milliseconds();
            phase = 0;
        }
        if(end-start!=0)telemetry.addData("Brake Time:",end-start);
    }
}
