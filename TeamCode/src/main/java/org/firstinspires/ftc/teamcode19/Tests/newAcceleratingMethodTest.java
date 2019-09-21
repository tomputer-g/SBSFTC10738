package org.firstinspires.ftc.teamcode19.Tests;
/*
created by Lucien Liu in Dec 2018
 */
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode19.BaseOpMode;
@TeleOp
@Disabled
public class newAcceleratingMethodTest extends BaseOpMode {
    protected double acc_rate = 0.0375;
    //protected double dec_rate = 0.005;
    //protected double dec_rate = 0.94;
    //protected double tolerance = 0.002;
    protected double p = 2;
    protected double test_speed = 0;
    boolean myFlag = false;
    //protected int acc_t = 1000;
    protected int dec_t = 2000;
    protected int step = 2;
    protected double actual_dec_t;
    @Override
    public void loop() {
        super.loop();
        if(this.gamepad1.x){
            while (this.gamepad1.x);
            setAllDrivePower(0);
            test_speed = 0;
            telemetry.addData("The robot is reset, ","Aha!");
            telemetry.update();
        }
        if(this.gamepad1.a){
            while (this.gamepad1.a);
            test_speed += 0.005;
            telemetry.addData("Current speed: ", test_speed);
            telemetry.update();
            setForwardSpeed(test_speed,1);
        }

        if(this.gamepad1.dpad_up){
            while (this.gamepad1.dpad_up);
            myMove(1,0.3,1);
        }
        else if(this.gamepad1.dpad_down){
            while (this.gamepad1.dpad_down);
            myMove(1,0.3,-1);
        }
        else if(this.gamepad1.left_bumper){
            while(this.gamepad1.left_bumper);
            p-=0.1;
            telemetry.addData("Current power raised: ", "%.3f",p);
            telemetry.update();
        }
        else if(this.gamepad1.right_bumper){
            while(this.gamepad1.right_bumper);
            p+=0.1;
            telemetry.addData("Current power raised: ", "%.3f",p);
            telemetry.update();
        }
    }
    public void myMove(double desired_speed, double time_travelled, int direction) {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        actual_dec_t = dec_t * desired_speed;
        double cur_speed;
        double cur_time;
        ElapsedTime t = new ElapsedTime();

        //accelerating
        /*
        while(t.milliseconds() <= acc_t){
            cur_time = t.milliseconds();
            cur_speed = cur_time / acc_t * desired_speed;
            setForwardSpeed(cur_speed,direction);
        }
        */
        for(double i = 0;i<desired_speed;i+=(1/(double)step)*desired_speed){
            t.reset();
            while(t.milliseconds() <= actual_dec_t/2/step){
                cur_time = t.milliseconds();
                cur_speed = getNextSpeed(cur_time,i,i+(1/(double)step)*desired_speed,1);
                setForwardSpeed(cur_speed,direction);
            }

            while(t.milliseconds() <= actual_dec_t/step){
                cur_time = t.milliseconds();
                cur_speed = getNextSpeed(cur_time,i,i+(1/(double)step)*desired_speed,2);
                setForwardSpeed(cur_speed,direction);
            }
        }
        //waiting for the button to be pressed
        //while(!this.gamepad1.y);

        //running a given period of time
        /*
        t.reset();
        while(t.milliseconds() < time_travelled*1000);
        */

        //decelerating
        for(double i = desired_speed;i>0;i-=(1/(double)step)*desired_speed){
            t.reset();
            while(t.milliseconds() <= actual_dec_t/2/step){
                cur_time = t.milliseconds();
                cur_speed = getNextSpeed(cur_time,i,i-(1/(double)step)*desired_speed,1);
                setForwardSpeed(cur_speed,direction);
            }

            while(t.milliseconds() <= actual_dec_t/step){
                cur_time = t.milliseconds();
                cur_speed = getNextSpeed(cur_time,i,i-(1/(double)step)*desired_speed,2);
                setForwardSpeed(cur_speed,direction);
                //if(myFlag)break;
            }
        }


        //t.reset();
       // while (t.milliseconds() <= 500);
        //stop
        setAllDrivePower(0);
    }

    public void setForwardSpeed(double speed, int direction){
        LF.setPower(-speed*direction);
        LB.setPower(-speed*direction);
        RF.setPower(speed*direction);
        RB.setPower(speed*direction);
    }

    public double calculateSpeed(double t, double v0, double vf){
        return v0 + Math.pow(t,p) * (vf-v0) / (2 * Math.pow((actual_dec_t/2) , p));
    }

    public double getNextSpeed(double t, double v0, double vf, int state){
        if(state == 1) return calculateSpeed(t,v0,vf);
        else {
            double nextSpeed = 2*calculateSpeed(actual_dec_t/2,v0, vf) - calculateSpeed(actual_dec_t - t,v0,vf);
            //if(nextSpeed <= 0.1) myFlag = true;
            return nextSpeed;
        }
    }
}