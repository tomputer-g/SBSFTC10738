package org.firstinspires.ftc.teamcode20.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;
@TeleOp
public class BackAndForthTest extends BaseAuto {
    double kdx = 4, kdxx = 1;
    private double[] params = {1,0};
    private String[] paramNames = {"kT","ops"};
    private int currentSelectParamIndex = 0;
    private boolean[] aa = {true}, l ={true}, r = {true}, u, d, lb={true}, rb;
    private boolean a, b;
    private double speed = 0.9;

    protected void moveInchesGOY_XF_F_T(double yInch, double speed,double kV, int FixXOffset){//use 0.4 for short-dist
        yInch = -yInch;
        //setNexwGyro0();
        double kP = 1, kD = 0.12;
        if(yInch == 0)return;
        if(Math.abs(speed) == 0.3){
            kP = 1;
            kD = 0.12;
        }else if(Math.abs(speed) == 0.4){
            kP = 0.27;
            kD = 0.03;
        }else if(Math.abs(speed) == 0.6){
            kP = 0.075;
            kD = 1.4E-2;
        }else if(Math.abs(speed) == 0.9){
            kP = 0.0325;
            kD = 7.3E-3;
        }
        double kPx = 0.2, kDx = kdxx*Math.pow(10,-kdx);
        ElapsedTime t = new ElapsedTime();
        int offsetY = getY1Odometry();
        int offsetX = FixXOffset;
        double diff = 0;
        speed=Math.abs(speed);
        double multiply_factor, prev_speed = 0;
        int odometryYGoal = offsetY + (int)(yInch * odometryEncYPerInch);
        double vy = speed;
        int previousPos = offsetY, previousPosX = offsetX, currentOdometry, currentOdometryX, Dterm, DtermX;
        double tpre = 0, tcur, D;
        int steadyCounter = 0;
        while(steadyCounter < 5 && !this.gamepad1.b){//b is there so we can break out of loop anytime
            //telemetry.addData("x",getXOdometry());
            //telemetry.addData("yL",getY1Odometry());
            //telemetry.addData("yR",getY2Odometry());
            //telemetry.update();
            currentOdometry = getY1Odometry();
            currentOdometryX = getXOdometry();
            tcur=t.milliseconds();
            Dterm = (int)((currentOdometry - previousPos)/(tcur-tpre));
            DtermX = (int)((currentOdometryX - previousPosX)/(tcur-tpre));
            D = (near(DtermX,0,speed * 5000 / 0.3)?(kDx *DtermX):0);
            diff = (currentOdometryX - offsetX)/odometryEncXPerInch*kPx + D;
            multiply_factor = -Math.min(1, Math.max(-1, kV*((kP * (currentOdometry - odometryYGoal)/ odometryEncYPerInch) +  (near(Dterm,0,speed * 5000 / 0.3)?(kD * Dterm):0))));
            if(near(prev_speed, multiply_factor*vy,0.001) && near(prev_speed, 0, 0.1)){
                steadyCounter++;
            }else{
                steadyCounter = 0;
            }
            //Log.d("GOY "+yInch,"steady"+steadyCounter+", position"+currentOdometry+", LF speed"+prev_speed+", OC speed="+Dterm+"bulkSpd="+hub4.getBulkInputData().getMotorVelocity(platform_grabber));
            previousPos = currentOdometry;
            previousPosX = currentOdometryX;
            tpre=tcur;
            setAllDrivePowerG(multiply_factor*vy + diff,multiply_factor*vy - diff,multiply_factor*-vy + diff,multiply_factor*-vy - diff,1.4);
            prev_speed = multiply_factor * vy;
        }
        setAllDrivePower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initAutonomous();
        //initViewMarks();
        //initVuMarksFull();
        waitForStart();
        while(opModeIsActive())
        {
            if(this.gamepad1.b){b = true;}if(!this.gamepad1.b && b) {
            b = false;
            ElapsedTime p = new ElapsedTime();
            p.reset();
            int aa = getXOdometry();
            moveInchesGOY_XF_F_T(96,speed,1,aa);
            for(int i = 0;i<4;++i){
                moveInchesGOY_XF_F_T(-96,speed,1,aa);
                moveInchesGOY_XF_F_T(96,speed,1,aa);
            }
                telemetry.addData("time", p.milliseconds());
            telemetry.update();

        }
            if(this.gamepad1.a){a = true;}if(!this.gamepad1.a && a) {
                a = false;
                grabber.setPosition(grabber_open);
                double curX;
                //params[1] = getXOdometry();
                double origin[] = {0,-41}, dd[]=adjustToViewMark(false);
                telemetry.addData("posY", "%.2f",dd[1]);
                telemetry.update();
                wait(500);

                for(int i = 0;i<2;++i){
                    setAllDrivePower(0);
                    curX = getXOdometry();
                    //if(i>0)servoThread.setTarget(0.75);
                    //grabber.setPosition(grabber_open);
                    align(0);
                    telemetry.addData("O",getY1Odometry());
                    telemetry.update();
                    moveInchesGOY_XF_F_T(-50,0.6,1,(int) (curX-(origin[1]-dd[1])*odometryEncXPerInch));
                    //servoThread.setTarget(0.98);
                    /*
                    align(-90);

                    double yorigin = getY1Odometry();
                    while((getY1Odometry()-yorigin)*-1 < odometryEncYPerInch*4){
                        setAllDrivePowerG(-.3,-.3,.3,.3);
                    }
                    while((getY1Odometry()-yorigin)*-1 < odometryEncYPerInch*8){
                        setAllDrivePowerG(-speed,-speed,speed,speed);
                    }
                    grabber.setPosition(grabber_closed);
                    wait(300);
                    servoThread.setTarget(0.85);
                    while((getY1Odometry()-yorigin)*-1 > odometryEncYPerInch*2){
                        setAllDrivePowerG(.3,.3,-.3,-.3);
                    }
                    setAllDrivePower(0);
                    align(0);
                    servoThread.setTarget(0.6);
                     */
                    telemetry.addData("O",getY1Odometry());
                    telemetry.update();
                    moveInchesGOY_XF_F_T(50,0.6,1,(int) (curX-(origin[1]-dd[1])*odometryEncXPerInch));
                    dd=adjustToViewMark(false);
                    //telemetry.addData("original", "%.2f",origin[1]);
                    //telemetry.addData("current", "%.2f",dd[1]);
                    //telemetry.update();
                }

            }
            //if(zheng(this.gamepad1.dpad_up, aa))kdx++;
            //if(zheng(this.gamepad1.dpad_down, lb))kdx--;
            if(zheng(this.gamepad1.dpad_left, l))speed-=0.1;
            if(zheng(this.gamepad1.dpad_right, r))speed+=0.1;
            //telemetry.addData("num","%.2f",kdxx);
            //telemetry.addData("power","%.2f",kdx);
            //telemetry.addData("parameters",params[0]+", "+params[1]);
            //telemetry.addData("now changing", paramNames[currentSelectParamIndex]);
            //telemetry.update();
            //telemetry.addData("x",getXOdometry());
        }
    }


}
