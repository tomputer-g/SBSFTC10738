package org.firstinspires.ftc.teamcode20.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;
@TeleOp(name = "DataScienceTest")
public class DataScienceTest extends BaseAuto {

    protected void moveInchesGOX_TT(double xInch, double speed,double kV, int timer) throws InterruptedException {//0.5 only
        if(xInch == 0)return;
        ElapsedTime t = new ElapsedTime();
        int offsetX = getXOdometry();
        speed=Math.abs(speed);
        double multiply_factor, prev_speed = 0;
        int odometryXGoal = offsetX + (int)(xInch * odometryEncXPerInch);
        double vx = speed;//(xInch/Math.abs(xInch)*speed);//
        int previousPos = offsetX, currentOdometry, Dterm;
        double tpre = 0, tcur;
        int steadyCounter = 0;
        int steps = 20;
        for(int i = 7;i<=steps;++i){
            setAllDrivePower(-i*speed/steps, i*speed/steps,-i*speed/steps,i*speed/steps);
            Thread.sleep(40);
        }
        while(steadyCounter < 5 && !this.gamepad1.b){
            Thread.sleep(0);
            currentOdometry = getXOdometry();
            tcur=t.milliseconds();
            Dterm = (int)((currentOdometry - previousPos)/(tcur-tpre));
            multiply_factor = -Math.min(1, Math.max(-1, kV*(0.19 * (currentOdometry - odometryXGoal)/odometryEncXPerInch) +  (near(Dterm,0,speed * 5000 / 0.3)?(0.05 * Dterm):0)));
            if(near(prev_speed, multiply_factor*vx,0.001) && near(prev_speed, 0, 0.1)){
                steadyCounter++;
            }
            else if(t.milliseconds() > timer) break;
            else{
                steadyCounter = 0;
            }
            Log.d("GOX "+xInch,"steady"+steadyCounter+", position"+currentOdometry+", speed"+prev_speed);
            previousPos = currentOdometry;
            tpre=tcur;
            setAllDrivePowerG(multiply_factor*-vx,multiply_factor*vx,multiply_factor*-vx,multiply_factor*vx, 1.5);
            prev_speed = multiply_factor * vx;
        }
        setAllDrivePower(0);
    }
    public void runOpMode() throws InterruptedException {
        initAutonomous();
        waitForStart();
        grabber.setPosition(0);
        ElapsedTime y = new ElapsedTime();
        moveInchesGOX_TT(15,0.9,1,2000);
        int offset = getXOdometry();

        moveInchesGOY_XF_F(97,0.9,1,offset);

        moveInchesGOY_XF_F(-24-97,0.6,1,offset);
        moveInchesGOY_XF_F(24+97,0.9,1,offset);

        moveInchesGOY_XF_F(-97+8-8,0.6,1,offset);
        moveInchesGOY_XF_F(97-8+8,0.9,1,offset);

        moveInchesGOY_XF_F(-97-8-16,0.6,1,offset);
        moveInchesGOY_XF_F(97+8+16,0.9,1,offset);

        moveInchesGOY_XF_F(-97-8-8-16,0.6,1,offset);
        moveInchesGOY_XF_F(97+8+8+16,0.9,1,offset);
telemetry.addData("t",y.milliseconds());
        telemetry.update();

        Thread.sleep(3000);

    }
}
