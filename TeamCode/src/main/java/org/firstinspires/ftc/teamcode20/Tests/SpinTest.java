package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;
@Disabled

@TeleOp
public class SpinTest extends BaseAuto {
    private boolean a[]={true}, b[]={true}, aa[]={true};
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
        /*
        setAllDrivePower(speed);

        if(p.milliseconds() >1000){
            p.reset();
            if(!near(last_imu,getHeading(),10)){
                servoThread.setTarget(0.99);
            }
            last_imu = getHeading();
        }
         */
        telemetry.addData("speed", "%.3f", speed);
        if(zheng(this.gamepad1.x, a)){
            speed +=0.05;
        }
        if(zheng(this.gamepad1.y,b)){
            speed-=0.05;
        }
        if(zheng(this.gamepad1.a,aa)){
            setAllDrivePowerG(-speed,-speed,speed,speed);
            LF.setTargetPosition(-(int) xmult*48);
            LB.setTargetPosition(-(int) xmult*48);
            RF.setTargetPosition((int) xmult*48);
            RB.setTargetPosition((int) xmult*48);
            LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (LF.isBusy() || LB.isBusy() || RF.isBusy() || RB.isBusy());
            setAllDrivePower(0);
        }
        telemetry.update();
    }
}
