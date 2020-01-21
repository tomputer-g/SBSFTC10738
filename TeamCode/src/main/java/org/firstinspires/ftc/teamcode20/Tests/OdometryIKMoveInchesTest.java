package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;
@Disabled
@TeleOp
public class OdometryIKMoveInchesTest extends BaseAuto {
    private boolean APrimed = false, upP = false, downP = false, leftP = false, rightP = false, x = false;
    private double speed = 0.3, kP = 0.1, kI = 0, kD = 0, k = 1, brakeDist = 7.5,brakeSpeed = 0;//kP = 0.035, kI = 1E-5, kD = 5E-6, k = 0.5, brakeDist = 7.5, brakeSpeed = 0.1;
    private int targetInches = -48;

    //FOR 0.3: use P = 0.7, D = 0.003

    @Override
    public void init() {
        initDrivetrain();
        initOdometry();
        initIMU();
        initLogger("PIDtest"+System.currentTimeMillis()+".csv");

    }
    @Override
    public void loop() {
        if(this.gamepad1.a){APrimed = true;}if(APrimed && !this.gamepad1.a){ APrimed = false;
            resetYOdometry();
            moveInchesGO(targetInches,speed);
        }
        if(this.gamepad1.dpad_left){leftP = true;}if(leftP && !this.gamepad1.dpad_left){ leftP = false;
            //speed -= 0.05;
            kP -= 1E-3;
        }
        if(this.gamepad1.dpad_right){rightP = true;}if(rightP && !this.gamepad1.dpad_right){ rightP = false;
            //speed += 0.05;
            kP += 1E-3;
        }
        if(this.gamepad1.dpad_down){downP = true;}if(downP && !this.gamepad1.dpad_down){ downP = false;
            kP /= 10.0;
            /*if(this.gamepad1.left_bumper){
                //kP -= 0.001;
                //kD /= 10.0;
                //kI /= 10.0;
            }else if(this.gamepad1.right_bumper){
                kI -= 1.0E-6;
            }else{
                //kD -= 1E-9;
                //k-=0.05;
            }

             */
        }
        if(this.gamepad1.dpad_up){upP = true;}if(upP && !this.gamepad1.dpad_up){ upP = false;
            kP *= 10.0;
            /*if(this.gamepad1.left_bumper){
                //kP += 0.001;
                //kD *= 10.0;
                kI *= 10.0;
            }else if(this.gamepad1.right_bumper){
                kI += 1.0E-6;
            }else{
                //kD += 1E-9;
                k+=0.05;
            }

             */
        }
        brakeSpeed = Math.round(brakeSpeed * 100) / 100.0;
        kP = Math.round(kP * 10000) / 10000.0;
        kI = Math.round(kI * 1E9) / 1.0E9;
        kD = Math.round(kD * 1E9) / 1.0E9;
        k = Math.round(k * 100) / 100.0;
        telemetry.addData("enc Y", getYOdometry());
        telemetry.addData("target", targetInches * odometryEncPerInch);
        telemetry.addData("speed", speed);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.addData("k",k);
        telemetry.addData("braking dist",brakeDist);
        telemetry.update();
    }
    protected final double odometryEncPerInch = 1316;//4096.0/Math.PI;
    protected int offsetY = 0;

    @Override
    public void stop() {
        stopLog();
        super.stop();

    }

    protected void moveInchesGO(double yInch, double speed){

        writeLogHeader("P="+kP+", I="+kI+", D="+kD+",k="+k+",speed="+speed+",target="+targetInches * odometryEncPerInch+", brakeDist="+brakeDist);
        writeLogHeader("time,position,P,I,D,LF speed");
        ElapsedTime t = new ElapsedTime();
        offsetY = getYOdometry();
        speed=Math.abs(speed);
        double multiply_factor=1;
        int odometryYGoal = offsetY + (int)(yInch * odometryEncPerInch);
        double vx = 0;
        double vy=(yInch==0)?0:(yInch/Math.abs(yInch)*speed);
        long IError = 0;
        setAllDrivePower((vy),(vy),(-vy),(-vy));
        int previousPos = getYOdometry();
        int Dterm;
        double tpre=0;
        while(Math.abs((getYOdometry() - odometryYGoal)/odometryEncPerInch) > brakeDist);
        while(!this.gamepad1.b && !near(getYOdometry(),odometryYGoal,100)){
            double tcur=t.milliseconds();
            Dterm = (int)((getYOdometry() - previousPos)/(tcur-tpre));
            multiply_factor = -Math.min(1, Math.max(-1, k *((kP * -(getYOdometry() - odometryYGoal)/odometryEncPerInch) +  (near(Dterm,0,5000)?(kD * Dterm):0)) + (kI * IError )));
            tpre=tcur;
            previousPos = getYOdometry();
            IError += (getYOdometry() - odometryYGoal);
            setAllDrivePowerG(multiply_factor*(-vx-vy),multiply_factor*(vx-vy),multiply_factor*(-vx+vy),multiply_factor*(vx+vy));

            writeLog(t.milliseconds()+", "+getYOdometry()+", "+((getYOdometry() - odometryYGoal)/odometryEncPerInch)+", "+IError+", "+Dterm+(near(Dterm,0,5000)?"":"clipped")+", "+multiply_factor*(-vx-vy));
            telemetry.addData("power",LF.getPower());
            telemetry.update();
            /*telemetry.addData("kP", kP);
            telemetry.addData("P term", (getYOdometry() - odometryYGoal)/odometryEncPerInch);
            telemetry.addData("kI", kI);
            telemetry.addData("I term", IError);
            telemetry.addData("kD", kD);
            telemetry.addData("D term",Dterm);
            telemetry.addData("current",getYOdometry());
            telemetry.addData("Y goal",odometryYGoal);
            telemetry.update();
             */
        }
        setAllDrivePower(0);
    }
}
