package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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
    protected Servo grabber;
    protected DcMotor grabber_extender;
    protected DcMotor L1, L2;
    private final String logPrefix = "/sdcard/";
    private BufferedWriter logWriter;
    private boolean[] bF={};

    @Override public void init() {
        msStuckDetectInit = 10000;
        msStuckDetectLoop = 30000;
        initDrivetrain();
    }

    @Override public void loop() {

    }

    protected void initLinSlide(){
        L1 = hardwareMap.get(DcMotor.class, "L1");
        L2 = hardwareMap.get(DcMotor.class, "L2");
        L1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    protected void moveLinSlide(double speed){
        L1.setPower(speed);
        L2.setPower(-speed);
    }

    protected void initGrabber(){
        grabber = hardwareMap.get(Servo.class, "grabber");
        grabber_extender = hardwareMap.get(DcMotor.class, "grabber_extender");
        grabber_extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected void reset_ENCODER(){
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    protected void setMode_RUN_WITH_ENCODER(){
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void setMode_RUN_WITHOUT_ENCODER(){//DO NOT USE ANY OTHER RUNMODES. GoBilda 5202 series motors have weird encoders
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    protected void 开倒车(double pLF, double pLB, double pRF, double pRB){
        LF.setPower(-pLF);
        LB.setPower(-pLB);
        RF.setPower(-pRF);
        RB.setPower(-pRB);
    }
    protected void 开倒车(double power){
        LF.setPower(-power);
        LB.setPower(-power);
        RF.setPower(-power);
        RB.setPower(-power);
    }
    protected void setAllDrivePower(double pX, double pY){
        if(Math.abs(pX)+Math.abs(pY) > 1)
            throw new IllegalArgumentException("setAllDrivePower(px,py) sets a power beyond 1");
        setMode_RUN_WITHOUT_ENCODER();
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

    protected boolean 整(boolean b, boolean[] f){
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

    private double sigmoid(double x){
        return (1.8/(1+pow(Math.E,5*x)))-0.9;
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

    protected void setAllDrivePower1(double a, double b, double c, double d){
        setAllDrivePower(-a,-b,c,d);
    }

    protected void 好活(double a,double b,double c,double d){
        setAllDrivePower(a,b,-c,-d);
    }

    protected void moveInches(double xInch, double yInch, double speed){
        /*
        double xmult = 14./2, ymult = 14./2, p_mult = 80;
        int encoder_x = (int)(xInch * xmult), encoder_y = (int)(yInch * ymult);
        int encoder_1 = Math.abs(encoder_x + encoder_y); // LB, RF
        int encoder_2 = Math.abs(encoder_x - encoder_y); // LF, RB
        double conversion_fct = speed/((encoder_1 + encoder_2)/2);
        double speed_1 = conversion_fct * encoder_1, speed_2 = conversion_fct * encoder_2;
        setAllDrivePower(speed_2,speed_1,speed_1,speed_2);
        setAllDrivePower(-speed,-speed,speed,speed);
        telemetry.addData("speed",speed_1+" "+speed_2);
        telemetry.addData("position",encoder_x+" "+encoder_y);
        telemetry.update();
        LF.setTargetPosition(encoder_x - encoder_y);
        LB.setTargetPosition(-encoder_x - encoder_y);
        RF.setTargetPosition(encoder_x + encoder_y);
        RB.setTargetPosition(-encoder_x + encoder_y);
        setMode_RESET_AND_RUN_TO_POSITION();
        while((LF.isBusy()||LB.isBusy()||RF.isBusy()||RB.isBusy()) && t.milliseconds() < p_time){
            telemetry.addData("Power", LF.getPower());
            telemetry.update();
        };
        setAllDrivePower(0);
        ();
        sup fuckers
        69
        cole wdnmd-p'
        */
        reset_ENCODER();
        setMode_RUN_WITHOUT_ENCODER();
        ElapsedTime t = new ElapsedTime();
        int p_time = (int) (sqrt(xInch*xInch + yInch*yInch)*100);
        double xmult = 133.5088/12, ymult = 133.7551/12;
        int encoder_x=(int)(xInch*xmult),encoder_y=(int)(yInch*ymult);
        double theta=Math.atan(xInch/yInch);
        double vy=Math.cos(theta)*speed,vx=Math.sin(theta)*speed;
        double coe=1;
        while(Math.abs(-encoder_x-encoder_y)>Math.abs(-LF.getCurrentPosition())||Math.abs(encoder_x-encoder_y)>Math.abs(-LB.getCurrentPosition())||Math.abs(-encoder_x+encoder_y)>Math.abs(-RF.getCurrentPosition())||Math.abs(encoder_x+encoder_y)>Math.abs(-RB.getCurrentPosition())){
            telemetry.addData("LF",-LF.getCurrentPosition());
            telemetry.addData("target",encoder_x-encoder_y);
            telemetry.addData("LB",-LB.getCurrentPosition());
            telemetry.addData("target",-encoder_x-encoder_y);
            telemetry.addData("RF",-RF.getCurrentPosition());
            telemetry.addData("target",encoder_x+encoder_y);
            telemetry.addData("RB",-RB.getCurrentPosition());
            telemetry.addData("target",-encoder_x+encoder_y);
            telemetry.update();
            //if (p_time < t.milliseconds()) break;
            setAllDrivePower(coe*(vx-vy),coe*(-vx-vy),coe*(vx+vy),coe*(-vx+vy));
            coe+=.1;
            coe=Math.max(coe,1);
        }
        setAllDrivePower(-LF.getPower()/Math.abs(LF.getPower()),-LB.getPower()/Math.abs(LB.getPower()),-RF.getPower()/Math.abs(RF.getPower()),-RB.getPower()/Math.abs(RB.getPower()));
        wait(75);
        setAllDrivePower(0);
        reset_ENCODER();
    }

    protected void moveInchesHighSpeed(double xInch, double yInch, double speed, int acc_s, int dec_s, double acc_p, double dec_p, double initial_speed)
    {

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
        setMode_RESET_AND_RUN_TO_POSITION();
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
