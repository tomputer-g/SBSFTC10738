package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.lang.Thread.sleep;

/*
BaseOpMode should have:

Mecanum driving & encoders (LF, RF, LB, RB)
Lander-grabber set (definitely 1 motor + maybe 1 motor/servo)
Vacuum set (2 motors)

Make sure TeleOp2019Trident and BaseAuto can inherit needed stuff by setting them to *protected*!
 */
public class BaseOpMode extends OpMode {

    protected DcMotor LF, LB, RF, RB;
    private final String logPrefix = "/sdcard/";
    private BufferedWriter logWriter;


    @Override public void init() {
        msStuckDetectInit = 10000;
        msStuckDetectLoop = 30000;
        initDrivetrain();
    }

    @Override public void loop() {

    }

    protected void setMode_RUN_WITH_ENCODER(){
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void setMode_RESET_AND_RUN_TO_POSITION(){
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    protected void setAllDrivePower(double power){
        LF.setPower(power);
        LB.setPower(power);
        RF.setPower(power);
        RB.setPower(power);
    }

    protected void setAllDrivePower(double pLF, double pLB, double pRF, double pRB){
        LF.setPower(pLF);
        LB.setPower(pLB);
        RF.setPower(pRF);
        RB.setPower(pRB);
    }

    protected void setAllDrivePower(double pX, double pY){
        if(Math.abs(pX)+Math.abs(pY) > 1)
            throw new IllegalArgumentException("setAllDrivePower(px,py) sets a power beyond 1");
        setMode_RUN_WITH_ENCODER();
        LF.setPower(pX-pY);
        LB.setPower(-pX-pY);
        RF.setPower(pX+pY);
        RB.setPower(-pX+pY);
    }

    protected void wait(int time){
        try {sleep(time);} catch (InterruptedException e) {e.printStackTrace();}
    }

    protected boolean near(double value, double target, double tolerance){
        return Math.abs(value-target) <= tolerance;
    }

    protected void initDrivetrain(){
        LF = hardwareMap.get(DcMotor.class,"LF");
        LB = hardwareMap.get(DcMotor.class,"LB");
        RF = hardwareMap.get(DcMotor.class,"RF");
        RB = hardwareMap.get(DcMotor.class,"RB");
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    protected void initOdometry(){
        telemetry.addLine("BaseOpMode -> initOdometry() still a stub!");
    }

    protected boolean cBP(boolean b,boolean[] f){
        //chzch butt on press
        //淦 --yeah
        //微笑着面对它
        if(b||!f[0]){
            if(b)f[0]=false;
            else f[0]=true;
            if(f[0])return true;
        }
        return false;
    }

    protected void move(double vx, double vy, double vr){
        LF.setPower(0.5 * (vx - vy + vr));
        LB.setPower(0.5 * (-vy - vx + vr));
        RF.setPower(0.5 * (vx + vy + vr));
        RB.setPower(0.5 * (-vx + vy + vr));
    }

    protected void 好活(double a, double b, double c, double d){
        setAllDrivePower(a,b,-c,-d);
    }

    private double sigmoid_brake(double power){
        return (1.8/(1+pow(Math.E,5*power)))-0.9;
    }

    protected void displayMotorPowers(double LF, double LB, double RF, double RB){
        telemetry.addLine();
        telemetry.addLine(""+to3dstr(LF)+"  |  "+to3dstr(RF));
        telemetry.addLine("-----------------------");
        telemetry.addLine(""+to3dstr(LB)+"  |  "+to3dstr(RB));
    }

    protected String to3dstr(double d){
        DecimalFormat df = new DecimalFormat("##0.000");
        return df.format(d);
    }

    protected double to3d(double d){
        DecimalFormat df = new DecimalFormat("##0.000");
        return Double.parseDouble(df.format(d));
    }
    //----------------------------------------Movement Code here-----------------------------------------
    //for phone: phone camera facing x-, extended grabber is y+
    protected void brake(){
        double lf=LF.getPower(),lb=LB.getPower(),rf=RF.getPower(),rb=RB.getPower();
        for(int i=0;i<4;i++){
            setAllDrivePower(lf-0.2*lf*i,lb-0.2*lb*i,rf-rf*0.2*i,rb-rb*0.2*i);
            if(Math.abs(lf-0.2*lf*i)<0.2) break;
        }
        setAllDrivePower(0);
        for(int i=0;i<10;i++){
            setAllDrivePower(.3,.3,-.3,-.3);
            wait(50);
            setAllDrivePower(0);
            wait(5);
        }
        //setAllDrivePower(sigmoid_brake(lf),sigmoid_brake(lb),sigmoid_brake(rf),sigmoid_brake(rb));
        //wait(300);
        setAllDrivePower(0);
    }
    
    protected void moveInches(double xInch, double yInch, double speed){
        setMode_RESET_AND_RUN_TO_POSITION();
        double xmult = 60, ymult = 57.174, p_mult = 80;
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

    protected void moveInchesHighSpeed(double xInch, double yInch, double speed, int acc_s, int dec_s, double acc_p, double dec_p, double initial_speed){
        setMode_RESET_AND_RUN_TO_POSITION();

        double xmult = 60, ymult = 57.174;
        int encoder_x = (int)(xInch * xmult), encoder_y = (int)(yInch * ymult);

        int encoder_1 = Math.abs(encoder_x + encoder_y); // LB, RF

        int encoder_2 = Math.abs(encoder_x - encoder_y); // LF, RB

        double conversion_fct = speed/((encoder_1 + encoder_2)/2);

        double incre_1 = (1.0-(initial_speed))/acc_s*conversion_fct * encoder_1;
        double incre_2 = (1.0-(initial_speed))/acc_s*conversion_fct * encoder_2;

        double decre_1 = (1.0)/dec_s*conversion_fct * encoder_1;
        double decre_2 = (1.0)/dec_s*conversion_fct * encoder_2;

        double speed_1 = incre_1+initial_speed, speed_2 = incre_2+initial_speed;

        setAllDrivePower(speed_2,speed_1,speed_1,speed_2);
        LF.setTargetPosition(encoder_x - encoder_y);
        LB.setTargetPosition(-encoder_x - encoder_y);
        RF.setTargetPosition(encoder_x + encoder_y);
        RB.setTargetPosition(-encoder_x + encoder_y);
        telemetry.addData("target: ", LF.getTargetPosition());
        telemetry.addData("initial: ", LF.getCurrentPosition());
        telemetry.update();

        for(int i = 1;i<acc_s;++i){
            while(((double)LF.getCurrentPosition() / LF.getTargetPosition()) < (acc_p/(acc_s-1)*i));
            speed_1+=incre_1;
            speed_2+=incre_2;
            setAllDrivePower(speed_2,speed_1,speed_1,speed_2);
        }

        for(int i = 0;i<dec_s-1;++i){
            while(((double)LF.getCurrentPosition() / LF.getTargetPosition()) < (1-dec_p) + (dec_p/(dec_s-1)*i));
            speed_1-=decre_1;
            speed_2-=decre_2;
            setAllDrivePower(speed_2,speed_1,speed_1,speed_2);
        }
        while((LF.isBusy()||LB.isBusy()||RF.isBusy()||RB.isBusy()));
    }

    protected void moveInchesHighSpeedWithoutWait(double xInch, double yInch, double speed, int acc_s, int dec_s, double acc_p, double dec_p, double initial_speed){
        setMode_RESET_AND_RUN_TO_POSITION();

        double xmult = 60, ymult = 57.174;
        int encoder_x = (int)(xInch * xmult), encoder_y = (int)(yInch * ymult);

        int encoder_1 = Math.abs(encoder_x + encoder_y); // LB, RF

        int encoder_2 = Math.abs(encoder_x - encoder_y); // LF, RB

        double conversion_fct = speed/((encoder_1 + encoder_2)/2);

        double incre_1 = (1.0-(initial_speed))/acc_s*conversion_fct * encoder_1;
        double incre_2 = (1.0-(initial_speed))/acc_s*conversion_fct * encoder_2;

        double decre_1 = (1.0)/dec_s*conversion_fct * encoder_1;
        double decre_2 = (1.0)/dec_s*conversion_fct * encoder_2;

        double speed_1 = incre_1+initial_speed, speed_2 = incre_2+initial_speed;

        setAllDrivePower(speed_2,speed_1,speed_1,speed_2);
        LF.setTargetPosition(encoder_x - encoder_y);
        LB.setTargetPosition(-encoder_x - encoder_y);
        RF.setTargetPosition(encoder_x + encoder_y);
        RB.setTargetPosition(-encoder_x + encoder_y);
        //telemetry.addData("target: ", LF.getTargetPosition());
        //telemetry.addData("initial: ", LF.getCurrentPosition());
        //telemetry.update();

        for(int i = 1;i<acc_s;++i){
            while(((double)LF.getCurrentPosition() / LF.getTargetPosition()) < (acc_p/(acc_s-1)*i));
            speed_1+=incre_1;
            speed_2+=incre_2;
            setAllDrivePower(speed_2,speed_1,speed_1,speed_2);
        }

        for(int i = 0;i<dec_s-1;++i){
            while(((double)LF.getCurrentPosition() / LF.getTargetPosition()) < (1-dec_p) + (dec_p/(dec_s-1)*i));
            speed_1-=decre_1;
            speed_2-=decre_2;
            setAllDrivePower(speed_2,speed_1,speed_1,speed_2);
        }
        //while((LF.isBusy()||LB.isBusy()||RF.isBusy()||RB.isBusy()));
    }

    protected void moveInchesHighSpeedEncoder(double xInch, double yInch, double speed, int acc_s, int dec_s, double acc_p, double dec_p, double initial_speed){
        setMode_RESET_AND_RUN_TO_POSITION();

        double xmult = 60, ymult = 57.174;
        int encoder_x = (int)(xInch * xmult), encoder_y = (int)(yInch * ymult);

        int encoder_1 = Math.abs(encoder_x + encoder_y); // LB, RF

        int encoder_2 = Math.abs(encoder_x - encoder_y); // LF, RB

        double conversion_fct = speed/((encoder_1 + encoder_2)/2);

        double incre_1 = (1.0-(initial_speed))/acc_s*conversion_fct * encoder_1;
        double incre_2 = (1.0-(initial_speed))/acc_s*conversion_fct * encoder_2;

        double decre_1 = (1.0)/dec_s*conversion_fct * encoder_1;
        double decre_2 = (1.0)/dec_s*conversion_fct * encoder_2;

        double speed_1 = incre_1+initial_speed, speed_2 = incre_2+initial_speed;

        setAllDrivePower(speed_2,speed_1,speed_1,speed_2);
        LF.setTargetPosition(encoder_x - encoder_y);
        LB.setTargetPosition(-encoder_x - encoder_y);
        RF.setTargetPosition(encoder_x + encoder_y);
        RB.setTargetPosition(-encoder_x + encoder_y);

        for(int i = 1;i<acc_s;++i){
            while(((double)LF.getCurrentPosition() / LF.getTargetPosition()) < (acc_p/(acc_s-1)*i));
            speed_1+=incre_1;
            speed_2+=incre_2;
            setAllDrivePower(speed_2,speed_1,speed_1,speed_2);
        }

        for(int i = 0;i<dec_s-1;++i){
            while(((double)LF.getCurrentPosition() / LF.getTargetPosition()) < (1-dec_p) + (dec_p/(dec_s-1)*i));
            speed_1-=decre_1;
            speed_2-=decre_2;
            setAllDrivePower(speed_2,speed_1,speed_1,speed_2);
        }

        while(!near(LF.getCurrentPosition(), LF.getTargetPosition(), 50) || !near(RF.getCurrentPosition(), RF.getTargetPosition(), 50) || !near(RB.getCurrentPosition(), RB.getTargetPosition(), 50) || !near(LB.getCurrentPosition(), LB.getTargetPosition(), 50));
    }

    //---------------------------------------------------Logging stuff-------------------------------------------

    protected void initLogger(String filename){
        String path = logPrefix + filename;
        telemetry.addLine("Writing log to "+path);
        try {
            logWriter = new BufferedWriter(new FileWriter(path));
            //writer = new FileWriter(filename);
            telemetry.addLine("writer create success");
        } catch (IOException e) {
            telemetry.addLine(e.toString());
        }
    }

    protected void writeLogHeader(String headers){
        writeLog(headers);
    }

    protected void writeLog(String message){
        try{
            logWriter.write(message+"\n");
        }catch (Exception e){
            telemetry.addLine(e.toString());
        }
    }

    protected void stopLog(){
        if(logWriter != null) {
            try {
                logWriter.close();
            } catch (IOException e) {
                telemetry.addLine(e.toString());
            }
        }
    }

}
