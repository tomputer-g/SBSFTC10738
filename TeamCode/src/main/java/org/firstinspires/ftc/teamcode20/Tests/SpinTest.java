package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;

@TeleOp
public class SpinTest extends BaseAuto {
    private boolean a[]={true}, b[]={true};
    private double last_imu, d, speed;
    ElapsedTime p  =new ElapsedTime();

    @Override
    public void init() {
        initDrivetrain();
        initIMU();
        initGrabber();
        speed = 0.3;
    }

    @Override
    public void loop() {
        setAllDrivePower(speed);
        d = (p.milliseconds()/500);
        if(d - (int) d  < 0.01){
            if(!near(last_imu,getHeading(),10)){
                grabber_extend1.setPosition(1);
                grabber_extend2.setPosition(0);
            }
        }
        if(zheng(this.gamepad1.x, a)){
            speed +=0.05;
        }
        if (zheng(this.gamepad1.y,b)){
            speed-=0.05;
        }
        last_imu = getHeading();
    }
}
