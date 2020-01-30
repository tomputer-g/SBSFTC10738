package org.firstinspires.ftc.teamcode20.Tests;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseOpMode;
@TeleOp
public class CompositeMovementTest extends BaseAuto{

    private double kP=0.028,kD=0.922,speed=0.5,ssas=0.2;
    private int magnitude=-2;
    private boolean[] du ={true}, dd={true}, dl={true},dr={true},rb={true},y={true},a={true},x={true},b={true},lb={true};
    private double imuinitvalue=0, target=90, result=0, resuuu=0;
    private double acctarget=0;

    @Override
    public void init() {
        initDrivetrain();
        initIMU();
        initOdometry();
        wait(200);
        setNewGyro0();
    }
    @Override
    public void loop() {
        if(zheng(this.gamepad1.dpad_up,du)){kP+=Math.pow(10,magnitude);}
        if(zheng(this.gamepad1.dpad_down,dd)){kP-=Math.pow(10,magnitude);}
        if(zheng(this.gamepad1.y,y)){kD+=Math.pow(10,magnitude);}
        if(zheng(this.gamepad1.a,a)){kD-=Math.pow(10,magnitude);}
        if(zheng(this.gamepad1.x,x)){ssas+=Math.pow(10,magnitude);}
        if(zheng(this.gamepad1.b,b)){ssas-=Math.pow(10,magnitude);}
        if(zheng(this.gamepad1.dpad_left,dl)){magnitude++;}
        if(zheng(this.gamepad1.dpad_right,dr)){magnitude--;}
        telemetry.addData("magnitude: ",Math.pow(10,magnitude));
        telemetry.addData("kP: ",kP);
        telemetry.addData("kD: ",kD);
        telemetry.addData("imu: ",getHeading());
        telemetry.addData("ssas: ",ssas);
        if(zheng(this.gamepad1.left_bumper,lb)){
            lefty(false);
        }
        if(zheng(this.gamepad1.right_bumper,rb)){
            righty(false);
        }
    }
    private void taunePIDturn(double target, double kp, double kd, double spe, boolean resetOffset){
        if(resetOffset){
            acctarget=0;
            setNewGyro0();
        }
        double e = target;
        ElapsedTime t = new ElapsedTime();
        int i=0;
        setAllDrivePower(1,1,-1,-1);
        while(i<5&&!zheng(this.gamepad1.right_bumper, rb)){
            double e2 = target-(getAdjustedHeading(target));
            double D = kd*(e2-e)/t.milliseconds();
            double P = e2*kp;
            if(Math.abs(P)>Math.abs(speed))P=P>0?speed:-speed;
            double A=P+D;
            setAllDrivePower(A+0.5+0.2,A-0.5+0.2,A+0.5-0.2,A-0.2);
            e=e2;
            if(near(e2-e,0,0.2)&&near(e,0,4))
                i++;
            t.reset();
        }
        setAllDrivePower(0);
        acctarget+=target;
        if(resetOffset) {
            acctarget = 0;
            setNewGyro0();
        }
        else
            setNewGyro(acctarget);
    }

    private void lefty(boolean resetOffset){
        double target=90,kd=0.922,kp=0.028;
        if(resetOffset){
            acctarget=0;
            setNewGyro0();
        }
        double e = target;
        ElapsedTime t = new ElapsedTime();
        int i=0;
        setAllDrivePower(1,1,-1,-1);
        while(i<5&&!zheng(this.gamepad1.right_bumper, rb)){
            double e2 = target-(getAdjustedHeading(target));
            double D = kd*(e2-e)/t.milliseconds();
            double P = e2*kp;
            if(Math.abs(P)>Math.abs(0.6))P=P>0?0.7:-0.7;
            double A=P+D;
            setAllDrivePower(A+0.3+0.2,A-0.3+0.2,A+0.3-0.2,A-0.3-0.2);
            e=e2;
            if(near(e2-e,0,0.2)&&near(e,0,4))
                i++;
            t.reset();
        }
        setAllDrivePower(0);
        acctarget+=target;
        if(resetOffset) {
            acctarget = 0;
            setNewGyro0();
        }
        else
            setNewGyro(acctarget);
    }

    private void righty(boolean resetOffset){
        double target=-90,kd=0.922,kp=0.028;
        if(resetOffset){
            acctarget=0;
            setNewGyro0();
        }
        double e = target;
        ElapsedTime t = new ElapsedTime();
        int i=0;
        setAllDrivePower(1,1,-1,-1);
        while(i<5&&!zheng(this.gamepad1.right_bumper, rb)){
            double e2 = target-(getAdjustedHeading(target));
            double D = kd*(e2-e)/t.milliseconds();
            double P = e2*kp;
            if(Math.abs(P)>Math.abs(0.7))P=P>0?0.7:-0.7;
            double A=P+D;
            setAllDrivePower(A-0.3+0.2,A+0.3+0.2,A-0.3-0.2,A+0.3-0.2);
            e=e2;
            if(near(e2-e,0,0.2)&&near(e,0,4))
                i++;
            t.reset();
        }
        setAllDrivePower(0);
        acctarget+=target;
        if(resetOffset) {
            acctarget = 0;
            setNewGyro0();
        }
        else
            setNewGyro(acctarget);
    }



    private double getAdjustedHeading(double target){
        double i = getHeading();
        if(target>0)
            return i<-100?i+360:i;
        else
            return i>100?i-360:i;
    }
}
