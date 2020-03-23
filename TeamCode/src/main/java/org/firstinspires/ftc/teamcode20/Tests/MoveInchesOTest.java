package org.firstinspires.ftc.teamcode20.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;

@TeleOp
public class MoveInchesOTest extends BaseAuto {
    private double[] params =       {2.5E-3,  2.5E-4,      0,       0.99307,             90}; //TODO: change to better values
    private String[] paramNames =   {"kP",  "kD",          "kI",    "proportional",     "dist"};
    private int currentSelectParamIndex = 0;
    private boolean l, r, u, d, lb, rb,a;
    double iError = 0;
    double previous_odo = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initDrivetrain();
        initOdometry();
        initIMU();
        waitForStart();
        while(opModeIsActive()){
            if(this.gamepad1.left_bumper){lb = true;}if(!this.gamepad1.left_bumper && lb){
                lb = false;
                currentSelectParamIndex--;
                if(currentSelectParamIndex < 0){
                    currentSelectParamIndex = params.length - 1;
                }
            }
            if(this.gamepad1.right_bumper){rb = true;}if(!this.gamepad1.right_bumper && rb){
                rb = false;
                currentSelectParamIndex++;
                if(currentSelectParamIndex >= params.length){
                    currentSelectParamIndex = 0;
                }
            }
            if(this.gamepad1.dpad_left){l = true;}if(!this.gamepad1.dpad_left && l){
                l = false;
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] - 1) * 1E10) / 1E10;

            }
            if(this.gamepad1.dpad_right){r = true;}if(!this.gamepad1.dpad_right && r){
                r = false;
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] + 1) * 1E10) / 1E10;

            }
            if(this.gamepad1.dpad_up){u = true;}if(!this.gamepad1.dpad_up && u){
                u = false;
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] * 10.0) * 1E10) / 1E10;

            }
            if(this.gamepad1.dpad_down){d = true;}if(!this.gamepad1.dpad_down && d){
                d = false;
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] / 10.0) * 1E10) / 1E10;

            }

            if(this.gamepad1.a){a = true;}if(a && !this.gamepad1.a){
                a = false;
                double deltaTime = 0;
                resetY1Odometry();
                resetY2Odometry();
                previous_odo = 0;
                setNewGyro0();
                iError = 0;
                while(-getY1Odometry()/odometryEncYPerInch < params[4] && (!this.gamepad1.b)){
                    deltaTime = setAllDrivePowerO(-.4,-.4,.4,.4, deltaTime,params[0], params[1], params[2]);
                }//0.9945 //1.0025
                setAllDrivePower(0);
            }

            telemetry.addData("parameters",params[0]+", "+params[1]+", "+params[2]+", "+params[3]+","+params[4]);
            telemetry.addData("now changing", paramNames[currentSelectParamIndex]);
            telemetry.update();
        }
    }

    protected double setAllDrivePowerO(double a, double b, double c, double d,double dT,double Kp, double Kd, double kI){
        ElapsedTime t = new ElapsedTime();
        double current_heading = getHeading();
        double current_odo = getY1Odometry();
        //double proportion_L, sign;
        /*if(a<0) {
            proportion_L = 0.9945;
            sign = 1;
        }
        else {
            proportion_L = 1.0025;
            sign = -1;
        }

         */
        //proportion_L = params[3];
        current_error = -(current_odo-previous_odo) * Math.sin(Math.toRadians(current_heading));//proportion_L * getY1Odometry() - getY2Odometry();
        Log.i("err",""+current_error);
        iError += (current_error * dT);
        double p_term = Kp * current_error;
        double d_term = (dT == 0 ? 0 : Kd * (current_error - previous_error)/dT * 1);//sign);
        double i_term = kI * iError;
        previous_error = current_error;
        previous_odo = current_odo;
        setAllDrivePower(a-p_term-d_term-i_term,b-p_term-d_term-i_term,c-p_term-d_term-i_term,d-p_term-d_term-i_term);
        return t.milliseconds();
    }
}
