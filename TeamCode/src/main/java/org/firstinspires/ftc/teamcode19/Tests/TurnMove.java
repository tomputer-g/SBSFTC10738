package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode19.BaseAuto;

import static java.lang.Math.sqrt;
@Disabled
public class TurnMove extends BaseAuto {
    @Override
    public void init() {
        initDrivetrain();
        initIMU();
    }

    @Override
    public void loop() {
        if(this.gamepad1.a){
            while(this.gamepad1.a);
            double[] start = {0,0}, end = {5,5};
            moveKeepHdg(start, end, 45);
        }
    }

    protected void moveKeepHdg(double[] start, double[] end, double deltaTheta){
        if(start.length != 2 || end.length != 2){
            throw new IllegalArgumentException();
        }
        setNewGyro0();
        double dx = end[0] - start[0];
        double dy = end[1] - start[1];
        double r = Math.sqrt(dx*dx+dy*dy);
        double moveTheta = Math.atan(1/(dx/dy));



    }

    protected void moveInches(double xInch, double yInch, double speed){
        setMode_RESET_AND_RUN_TO_POSITION();
        int xmult = 60, ymult = 56, p_mult = 80;
        int p_time = (int) (sqrt(xInch*xInch + yInch*yInch)*p_mult);
        ElapsedTime t = new ElapsedTime();
        int encoder_x = (int)(xInch * xmult), encoder_y = (int)(yInch * ymult);
        int encoder_1 = Math.abs(encoder_x + encoder_y); // LB, RF
        int encoder_2 = Math.abs(encoder_x - encoder_y); // LF, RB
        double conversion_fct = speed/((encoder_1 + encoder_2)/2);
        double speed_1 = conversion_fct * encoder_1, speed_2 = conversion_fct * encoder_2;
        setAllDrivePower(speed_2,speed_1,speed_1,speed_2);
        LF.setTargetPosition(encoder_x - encoder_y);
        LB.setTargetPosition(-encoder_x - encoder_y);
        RF.setTargetPosition(encoder_x + encoder_y);
        RB.setTargetPosition(-encoder_x + encoder_y);
        while((LF.isBusy()||LB.isBusy()||RF.isBusy()||RB.isBusy()) && t.milliseconds() < p_time);
    }
}
