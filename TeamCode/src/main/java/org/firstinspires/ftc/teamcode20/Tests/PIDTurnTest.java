
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

public class PIDTurnTest extends BaseAuto {
    private double[] params =       {0.17,  0,     0.015,       0.9,        -6,           };
    private String[] paramNames =   {"P",   "I",    "D",    "speed",    "targetInches"};
    private double kP=0.027,kD=0.922;
    private int magnitude=-2;
    private boolean[] du ={true}, dd={true}, dl={true},dr={true},rb={true},y={true},a={true},x={true},b={true},lb={true};
    private double imuinitvalue=0, target=90, result=0, resuuu=0, speed=1.0;
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
        if(zheng(this.gamepad1.x,x)){target+=5;}
        if(zheng(this.gamepad1.b,b)){target-=5;}
        if(zheng(this.gamepad1.dpad_left,dl)){magnitude++;}
        if(zheng(this.gamepad1.dpad_right,dr)){magnitude--;}
        if(this.gamepad1.left_trigger>0.5){speed+=0.1;}
        if(this.gamepad1.right_trigger>0.5){speed-=0.1;}
        if(zheng(this.gamepad1.dpad_right,dr)){magnitude--;}
        telemetry.addData("magnitude: ",Math.pow(10,magnitude));
        telemetry.addData("kP: ",kP);
        telemetry.addData("kD: ",kD);
        telemetry.addData("imu: ",getHeading());
        telemetry.addData("target:",target);
        telemetry.addData("speed:", speed);
        //telemetry.addData("result: ",result);
        //telemetry.addLine("LF: "+LF.getCurrentPosition()+" LB: "+LB.getCurrentPosition()+" RF: "+RF.getCurrentPosition()+" RB:"+RB.getCurrentPosition());
        if(zheng(this.gamepad1.left_bumper,lb)){
            tunePIDturn(target,kP,kD,speed,false);
        }
        if(zheng(this.gamepad1.right_bumper,rb)){
            acctarget=0;
            setNewGyro0();
        }
    }
    //0.8, kp 0.033,kd 0.8
    //1 kp 0.027 kd 0.922,90
    private void taunePIDturn(double target, double kp, double kd, double speed, boolean resetOffset){
        if(resetOffset){
            acctarget=0;
            setNewGyro0();
        }
        double e = target;
        ElapsedTime t = new ElapsedTime();
        int i=0;
        while(i<5&&!zheng(this.gamepad1.right_bumper, rb)){
            double e2 = target-(getAdjustedHeading(target));
            double D = kd*(e2-e)/t.milliseconds();
            double P = e2*kp;
            if(Math.abs(P)>Math.abs(speed))P=P>0?speed:-speed;
            setAllDrivePower(P+D);
            e=e2;
            if(near(e2-e,0,0.1)&&near(e,0,2))
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

    protected void moveShortInchesGOY(double yInch, double speed,double kP,double kD){//use 0.4 for short-dist
        yInch = -yInch;
        setNewGyro0();
        ElapsedTime t = new ElapsedTime();
        int offsetY = getY1Odometry();
        speed=Math.abs(speed);
        double multiply_factor, prev_speed = 0;
        int odometryYGoal = offsetY + (int)(yInch * odometryEncYPerInch);
        double vy = speed;
        int previousPos = offsetY, currentOdometry, Dterm;
        double tpre = 0, tcur;
        int steadyCounter = 0;
        while(steadyCounter < 5 && !this.gamepad1.b){//b is there so we can break out of loop anytime
            currentOdometry = getY1Odometry();
            tcur=t.milliseconds();
            Dterm = (int)((currentOdometry - previousPos)/(tcur-tpre));
            multiply_factor = -Math.min(1, Math.max(-1, ((kP * (currentOdometry - odometryYGoal)/ odometryEncYPerInch) +  (near(Dterm,0,speed * 5000 / 0.3)?(kD * Dterm):0))));
            if(near(prev_speed, multiply_factor*vy,0.001) && near(currentOdometry, odometryYGoal, odometryEncYPerInch)){
                steadyCounter++;
            }else{
                steadyCounter = 0;
            }
            //Log.d("GOY "+yInch,"steady"+steadyCounter+", position"+currentOdometry+", speed"+prev_speed);
            previousPos = currentOdometry;
            tpre=tcur;
            setAllDrivePowerG(multiply_factor*vy,multiply_factor*vy,multiply_factor*-vy,multiply_factor*-vy, params[0]);
            prev_speed = multiply_factor * vy;
        }
        setAllDrivePower(0);
    }

    public void testPIDturn(double target, double kd, double kp,double speed){
        double e = target;
        ElapsedTime t = new ElapsedTime();
        ElapsedTime n = new ElapsedTime();
        int i=0;
        while(i<5){
            double e2 = target-(getAdjustedHeading(target));
            double D = kd*(e2-e)/t.milliseconds();
            double P = e2*kp;
            if(Math.abs(P)>Math.abs(speed))P=P>0?speed:-speed;
            double power = P+D;
            setAllDrivePower(power);
            e=e2;
            if(near(e2-e,0,0.1)&&near(e,0,2))
                i++;
            t.reset();
            //telemetry.addLine("imua: "+getAdjustedHeading(target)+"\nimu"+getHeading());
            //telemetry.update();
        }
        setAllDrivePower(0.0);
        result=getHeading();
        acctarget+=target;
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
