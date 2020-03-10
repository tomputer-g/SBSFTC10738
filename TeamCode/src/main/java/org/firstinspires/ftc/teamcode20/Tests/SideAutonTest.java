package org.firstinspires.ftc.teamcode20.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.firstinspires.ftc.teamcode20.BlueAuto;

@TeleOp
public class SideAutonTest extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        //servoThread.setExtTarget(0.99);
        //grabber.setPosition(grabber_open);
        //platform_grabber.setPower(1);
        //platform_grabber.setPower(0.0);
        //pwmThread = new PWMThread();
        initDrivetrain();
        initOdometry();
        initIMU();
        setNewGyro0();
        waitForStart();
        while (opModeIsActive()) {
            int yOdoOffset = getY1Odometry();
            setAllDrivePowerG(-1,-1,1,1);
            while(Math.abs(getY1Odometry()-yOdoOffset)<Math.abs(10*odometryEncXPerInch)){
                //
            }
            PIDturnfast(90,false);
            double power = 0.5;
            setAllDrivePowerG(power,-power,power,-power);
            Thread.sleep(50);
            setAllDrivePower(0);
            requestOpModeStop();
        }
    }
    private void forleft(){
        double target=90;
        setAllDrivePower(-1,-1,1,1);
        setAllDrivePower(1,1,1,1);
        setAllDrivePower(-1,1,-1,1);
    }
}