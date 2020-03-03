package org.firstinspires.ftc.teamcode20;

import android.util.Log;

import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.Set;

import static java.lang.Math.pow;
import static java.lang.Math.round;
import static java.lang.Math.sqrt;
import static java.lang.Thread.sleep;

public class BaseOpMode extends LinearOpMode {

    //Motors
    protected ExpansionHubMotor LF, LB, RF, RB;


    protected boolean showTelemetry = false;
    protected ExpansionHubEx hub2, hub4;
    protected ServoThread servoThread;

    protected Servo grabber, capstone;
    protected Servo grabber_extend1, grabber_extend2;
    protected DcMotor platform_grabber, xOdometry;
    protected DcMotor L1, L2;
    protected final double grabber_open = 0.35, grabber_closed = 0.7;
    private final String logPrefix = "/sdcard/";
    private BufferedWriter logWriter;

    protected final double odometryEncYPerInch = 1324.28, odometryEncXPerInch = 1314.42;


    protected RevBulkData tmpBulkData;//use this for all bulk reads


/*    protected void kill(String message){
        //Drivetrain
        if(LF != null && LB != null && RF != null && RB != null)setAllDrivePower(0);
        //Slide Motors TODO: better stopping method?
        if(L1 != null)L1.setPower(0);
        if(L2 != null)L2.setPower(0);
        //On slide: Servos (+ kill servo thread)
        if(servoThread != null && servoThread.isAlive())servoThread.stopThread();
        if(grabber_extend1 != null)grabber_extend1.setPosition(0.99);//TODO: Better stopping method?
        if(grabber_extend2 != null)grabber_extend2.setPosition(0.01);
        if(france != null)france.setPosition(0);
        if(grabber != null)grabber.setPosition(0.01);
        //tape
        if(xOdometry != null)xOdometry.setPower(0);
        //platform grabber
        if(platform_grabber != null)platform_grabber.setPower(0);

        //Non-physical dependencies:
        //Logger
        if(logWriter != null)stopLog();

        Log.w("Kill()","Message: "+message+". Printing stack:\n");
        dumpStack();
        stop();
    }
 */

    @Override
    public void runOpMode() throws InterruptedException {}

    protected void initPlatformGrabber(){
        platform_grabber = hardwareMap.get(DcMotor.class, "platform");
        platform_grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected void initLinSlide(){
        L1 = hardwareMap.get(DcMotor.class, "L1");
        L2 = hardwareMap.get(DcMotor.class, "L2");
        L1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    protected void initGrabber(){
        grabber = hardwareMap.get(Servo.class, "grabber");
        grabber_extend1 = hardwareMap.get(Servo.class, "servo1");
        grabber_extend2 = hardwareMap.get(Servo.class, "servo2");
        servoThread = new ServoThread();
        servoThread.start();
    }

    protected void reset_ENCODER(){
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    protected void setMode_RUN_WITH_ENCODER(){
        reset_ENCODER();
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void setMode_RUN_WITHOUT_ENCODER(){//DO NOT USE ANY OTHER RUNMODES. GoBilda 5202 series motors have weird encoders
        reset_ENCODER();
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected void setMode_RESET_AND_RUN_TO_POSITION(){
        reset_ENCODER();
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    protected void RUN_TO_POSITION(){
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    protected void setAllDrivePower(double power){
        setAllDrivePower(power,power,power,power);
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
        LF.setPower(-pX+pY);
        LB.setPower(pX+pY);
        RF.setPower(-pX-pY);
        RB.setPower(pX-pY);
    }

    protected void wait(int time){
        sleep(time);
    }

    protected boolean near(double value, double target, double tolerance){
        return Math.abs(value-target) <= tolerance;
    }

    protected void initDrivetrain(){
        LF = (ExpansionHubMotor) hardwareMap.get(DcMotor.class,"LF");
        LB = (ExpansionHubMotor) hardwareMap.get(DcMotor.class,"LB");
        RF = (ExpansionHubMotor) hardwareMap.get(DcMotor.class,"RF");
        RB = (ExpansionHubMotor) hardwareMap.get(DcMotor.class,"RB");
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setMotorType(MotorConfigurationType.getMotorType(GoBILDA5202Series.class));
        LB.setMotorType(MotorConfigurationType.getMotorType(GoBILDA5202Series.class));
        RF.setMotorType(MotorConfigurationType.getMotorType(GoBILDA5202Series.class));
        RB.setMotorType(MotorConfigurationType.getMotorType(GoBILDA5202Series.class));
        setMode_RUN_WITHOUT_ENCODER();
    }

    protected int getXOdometry(){
        return xOdometry.getCurrentPosition();
    }

    protected int getY1Odometry(){
        return platform_grabber.getCurrentPosition();
    }

    protected int getY2Odometry(){
        return L2.getCurrentPosition();
    }

    protected int[] bulkReadOdometry(){
        RevBulkData bulk = hub4.getBulkInputData();
        return new int[] {bulk.getMotorCurrentPosition(xOdometry),bulk.getMotorCurrentPosition(platform_grabber),bulk.getMotorCurrentPosition(L2)};
    }

    protected void resetY1Odometry(){
        platform_grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        platform_grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected void resetY2Odometry(){
        L2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected void resetXOdometry(){
        xOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected boolean zheng(boolean b, boolean[] f){
        if(b||!f[0]){
            f[0]= !b;
            return f[0];
        }
        return false;
    }

    protected void move(double vx, double vy, double vr){
        LF.setPower(-0.5 * (vx - vy + vr));
        LB.setPower(-0.5 * (-vy - vx + vr));
        RF.setPower(-0.5 * (vx + vy + vr));
        RB.setPower(-0.5 * (-vx + vy + vr));
    }

    /*
    private double sigmoid(double x){
        return (1.8/(1+pow(Math.E,5*x)))-0.9;
    }

    protected void displayMotorPowers(double LF, double LB, double RF, double RB){
        if(showTelemetry)telemetry.addLine();
        if(showTelemetry)telemetry.addLine(""+to3dstr(LF)+"  |  "+to3dstr(RF));
        if(showTelemetry)telemetry.addLine("-----------------------");
        if(showTelemetry)telemetry.addLine(""+to3dstr(LB)+"  |  "+to3dstr(RB));
    }

     */

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

    /*protected void setAllDrivePower1(double a, double b, double c, double d){
        setAllDrivePower(-a,-b,c,d);
    }

     */

    protected void moveInches(double xInch, double yInch, double speed){
        /*
        sup fuckers
        69
        by cole wdnmd-p'
        */
        reset_ENCODER();
        setMode_RUN_WITHOUT_ENCODER();
        double xmult = 232.5088/12, ymult = 232.7551/12;
        int encoder_x=(int)(xInch*xmult),encoder_y=(int)(yInch*ymult);
        double theta=Math.atan(xInch/yInch);
        double vy=Math.cos(theta)*speed,vx=Math.sin(theta)*speed;
        double coe=1;
        while(Math.abs(-encoder_x-encoder_y)>Math.abs(-tmpBulkData.getMotorCurrentPosition(LF))||Math.abs(encoder_x-encoder_y)>Math.abs(-tmpBulkData.getMotorCurrentPosition(LB))||Math.abs(-encoder_x+encoder_y)>Math.abs(-tmpBulkData.getMotorCurrentPosition(RF))||Math.abs(encoder_x+encoder_y)>Math.abs(-tmpBulkData.getMotorCurrentPosition(RB))){
            //if (p_time < t.milliseconds()) break;
            setAllDrivePower(coe*(-vx-vy),coe*(vx-vy),coe*(-vx+vy),coe*(vx+vy));
            //coe+=.1;
            //coe=Math.max(coe,1);
        }
        setAllDrivePower(-LF.getPower()/Math.abs(LF.getPower()),-LB.getPower()/Math.abs(LB.getPower()),-RF.getPower()/Math.abs(RF.getPower()),-RB.getPower()/Math.abs(RB.getPower()));
        wait(120);
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
        if(showTelemetry){
            telemetry.addData("target: ", LF.getTargetPosition());
            telemetry.addData("initial: ", LF.getCurrentPosition());
            telemetry.update();
        }
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
        //if(showTelemetry)telemetry.addData("target: ", LF.getTargetPosition());
        //if(showTelemetry)telemetry.addData("initial: ", LF.getCurrentPosition());
        //if(showTelemetry)telemetry.update();

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
        tmpBulkData = hub2.getBulkInputData();
        while(!near(tmpBulkData.getMotorCurrentPosition(LF), LF.getTargetPosition(), 50) || !near(tmpBulkData.getMotorCurrentPosition(RF), RF.getTargetPosition(), 50) || !near(tmpBulkData.getMotorCurrentPosition(RB), RB.getTargetPosition(), 50) || !near(tmpBulkData.getMotorCurrentPosition(LB), LB.getTargetPosition(), 50)){
            tmpBulkData = hub2.getBulkInputData();
        }
    }

    //---------------------------------------------------Logging stuff-------------------------------------------

    protected void initLogger(String filename){
        String path = logPrefix + filename;
        if(showTelemetry)telemetry.addLine("Writing log to "+path);
        try {
            logWriter = new BufferedWriter(new FileWriter(path));
            //writer = new FileWriter(filename);
            if(showTelemetry)telemetry.addLine("writer create success");
        } catch (IOException e) {
            if(showTelemetry)telemetry.addLine(e.toString());
        }
    }

    protected void writeLogHeader(String headers){
        writeLog(headers);
    }

    protected void writeLog(String message){
        try{
            logWriter.write(message+"\n");
        }catch (Exception e){
            if(showTelemetry)telemetry.addLine(e.toString());
        }
    }

    protected void stopLog(){
        if(logWriter != null) {
            try {
                logWriter.close();
            } catch (IOException e) {
                if(showTelemetry)telemetry.addLine(e.toString());
            }
        }
    }

    //----------------------------------------------------TeleOp--------------------------------------

    protected void scaledMove(double vx, double vy, double vr){
        if(showTelemetry)telemetry.addLine("vX: "+to3d(vx)+", vY: "+to3d(vy)+", vR: "+to3d(vr));
        double[] speeds = {vx - vy + vr, -vy - vx + vr, vx + vy + vr, -vx + vy + vr};
        double absMax = 0;
        for(double d : speeds)
            absMax = Math.max(Math.abs(d),absMax);
        if(absMax <= 1){
            setAllDrivePower(speeds[0], speeds[1], speeds[2], speeds[3]);
        }else{
            if(showTelemetry)telemetry.addLine("SCALED power: max was "+absMax);
            setAllDrivePower(speeds[0]/absMax, speeds[1]/absMax, speeds[2]/absMax,speeds[3]/absMax);
        }
    }


    protected int hold = 0;
    protected boolean holdSet;
    //protected double a = 0.2;

    protected final int slideEncoderTravel = -5156;//IMPORTANT: if up is negative, this is negative
    protected final double slideInchTravel = 72.0;//inch of slide travel

    //value changed
    protected final double slideEncoderPerInch = -85.2204;
    protected int RTState = -1;
    protected final double ctrl_deadzone = 0.2;
    protected int slow = 0;

    protected int autoPlaceState = -1;

    protected void setExtenderServoPosition(double position){
        grabber_extend1.setPosition(roundTo2Dec(position));
        grabber_extend2.setPosition(roundTo2Dec(1-position));
    }
    //---------------slide-----------------
    protected void runSlide(){
        int L1CurrentPos = L1.getCurrentPosition();
        if(this.gamepad1.left_bumper && !near(this.gamepad1.right_stick_y, 0, 0.05)) {//long-dist
            if(servoThread.extLastPosition < 0.75){//very slow?
                holdSet = false;
                telemetry.addLine("slide is very slow");
                if(-this.gamepad1.right_stick_y > 0){//asc
                    L1.setPower(0.5*this.gamepad1.right_stick_y);
                    L2.setPower(-0.5*this.gamepad1.right_stick_y);
                }else{//dec
                    L1.setPower(0.15*this.gamepad1.right_stick_y);
                    L2.setPower(-0.15*this.gamepad1.right_stick_y);
                }

            }else if (this.gamepad1.right_stick_y < 0 && (slideEncoderTravel > 0? L1CurrentPos < slideEncoderTravel-50 : L1CurrentPos > slideEncoderTravel+50)) {
                L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                holdSet = false;//up
                L1.setPower(this.gamepad1.right_stick_y);
                L2.setPower(-this.gamepad1.right_stick_y);
                telemetry.addData("L1 power",this.gamepad1.right_stick_y);
            } else if (this.gamepad1.right_stick_y > 0 && L1CurrentPos < 0) {
                holdSet = false;
                L1.setPower(0.15*this.gamepad1.right_stick_y);
                L2.setPower(-0.15*this.gamepad1.right_stick_y);
            } else {
                holdSlide(L1CurrentPos);
            }
        }else if(RTState == -1 && autoPlaceState == -1){
            holdSlide(L1CurrentPos);
        }
    }

    protected void holdSlide(int position){
        if (!holdSet) {
            holdSet = true;
            hold = (slideEncoderTravel > 0? Math.max(0,Math.min(slideEncoderTravel, position)) : Math.min(0,Math.max(slideEncoderTravel,position)));
        }
        int error = hold - L1.getCurrentPosition();//this doesn't change for pos/neg directions
        double power = (slideEncoderTravel > 0? Math.max(0,Math.min(1,error/60.0)) : Math.min(0, Math.max(-1, error/60.0)));
        if(hold == 0){power = 0;}
        if(showTelemetry)telemetry.addData("holding",hold);
        if(showTelemetry)telemetry.addData("error",error);
        if(L1.getCurrentPosition() < 0) {
            L1.setPower(power);
            L2.setPower(-power);
        }else{
            L1.setPower(0);
            L2.setPower(0);
        }
    }

    //private int descendTarget = 0, ascendTarget = 0;
    //private double inchApproachTarget = 10.6, approachSpeed = 0.2;
    protected Rev2mDistanceSensor tower_top;

    protected final double grabberServoOut = 0.6, grabberServoIn = 0.99, grabberServoGrab = 0.76;

    /*
    protected void autoPlace(){
        switch(autoPlaceState){
            case -1:
                break;
            case 0:
                servoThread.setExtTarget(0.3);
                if(near(servoThread.extLastPosition, 0.3, 0.02)){
                    autoPlaceState++;
                }
            case 1://approach
                if(tower_top.getDistance(DistanceUnit.INCH) > inchApproachTarget + 0.5){
                    setAllDrivePower(-approachSpeed,-approachSpeed,approachSpeed,approachSpeed);
                }else if(tower_top.getDistance(DistanceUnit.INCH) < inchApproachTarget - 0.5){
                    setAllDrivePower(approachSpeed, approachSpeed, -approachSpeed, -approachSpeed);
                }else{
                    setAllDrivePower(0);
                    autoPlaceState++;
                }
                break;
            case 2://just started. rise to top of tower
                L1.setPower(-1);
                L2.setPower(1);
                int L1CurrentPos = L1.getCurrentPosition();
                if(tower_top.getDistance(DistanceUnit.INCH) > 20.0 || (slideEncoderTravel > 0? L1CurrentPos > slideEncoderTravel : L1CurrentPos < slideEncoderTravel)){
                    ascendTarget = L1CurrentPos + (int)(10*slideEncoderPerInch);
                    L1.setPower(-.7);
                    L2.setPower(.7);
                    servoThread.setExtTarget(grabberServoOut);
                    autoPlaceState++;
                }
                break;
            case 3: //rise a bit more and hold position
                if(ascendTarget + 50 > L1.getCurrentPosition()){
                    L1.setPower(0);
                    L2.setPower(0);
                    holdSet = false;
                    holdSlide(L1.getCurrentPosition());
                    autoPlaceState++;
                }
                break;
            case 4: //extend

                if(near(servoThread.extLastPosition, grabberServoOut, 0.05)){
                    autoPlaceState++;
                    holdSet = false;
                    descendTarget = L1.getCurrentPosition() - (int)( 17 * slideEncoderPerInch);
                    L1.setPower(0.3);
                    L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    L2.setPower(0);
                }
                break;
            case 5: //drop & hold to correct level (descend 1200) & drop
                if(L1.getCurrentPosition() > descendTarget - 50){
                    //autoPlaceState++;
                    holdSet = false;
                    L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    holdSlide(L1.getCurrentPosition());
                    autoPlaceState = -1;
                    //grabber.setPosition(grabber_open);
                }
                break;
        }
    }
     */

    protected void handleRTState(){//call in loop; non-blocking
        switch (RTState) {
            case -1: //none
                break;
            case 0:
                grabber.setPosition(0.1);
                servoThread.setExtTarget(grabberServoIn);
                if (near(servoThread.extLastPosition, grabberServoIn, 0.05)){
                    RTState = 1;
                }
                break;
            case 1://drop to ground
                holdSet = false;
                L1.setPower(0.8);
                L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                L2.setPower(0);
                int L1CurrentPos = L1.getCurrentPosition();
                if((slideEncoderTravel > 0? L1CurrentPos < 50: L1CurrentPos > -50)){
                    RTState = -1;
                    L1.setPower(0);
                    L2.setPower(0);
                    L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    holdSet = false;
                    holdSlide(0);
                    grabber.setPosition(grabber_open);
                    servoThread.setExtTarget(grabberServoGrab);
                }
                servoThread.setExtTarget(grabberServoGrab);
                break;
        }
    }

    private void 三天之内刹了你(){
        setAllDrivePower(1,1,-1,-1);
        wait(200);
        setAllDrivePower(0);
    }

    protected class ServoThread extends Thread{
        public double extTargetPosition = 1, extLastPosition = 1;
        public double grabTargetPosition, grabLastPosition;
        public int grabDelayCount = 0;
        volatile public int delayStep = 10;
        volatile public boolean stop = false;
        private boolean upWasHeld, downWasHeld, grabberDelayMove = false;
        @Override
        public void run() {
            this.setName("Servo Thread "+this.getId());
            Log.i("servoThread"+this.getId(),"Started running");
            while(!isInterrupted() && !stop){
                upWasHeld = gamepad1.dpad_up;
                downWasHeld = gamepad1.dpad_down;
                try {
                    sleep(delayStep);
                } catch (InterruptedException e) {
                    stop = true;
                }

                //set target if manual input

                if(gamepad1.dpad_up && upWasHeld){
                    setExtTarget(extLastPosition - 0.01);
                    autoPlaceState = -1;
                    RTState = -1;
                }else if(gamepad1.dpad_down && downWasHeld){
                    setExtTarget(extLastPosition + 0.01);
                    autoPlaceState = -1;
                    RTState = -1;
                }

                //execute target
                if (extLastPosition < extTargetPosition) {
                    setExtenderServoPosition(extLastPosition + 0.01);
                    extLastPosition += 0.01;
                } else if(extLastPosition > extTargetPosition){
                    setExtenderServoPosition(extLastPosition - 0.01);
                    extLastPosition -= 0.01;
                }

                if(grabberDelayMove && grabDelayCount++ == 4){
                    grabDelayCount = 0;
                    if(grabLastPosition < grabTargetPosition){
                        grabber.setPosition(grabLastPosition + 0.01);
                        grabLastPosition += 0.01;
                    }else if(grabLastPosition > grabTargetPosition){
                        grabber.setPosition(grabLastPosition - 0.01);
                        grabLastPosition -= 0.01;
                    }
                }

                extLastPosition = roundTo2Dec(extLastPosition);
                extTargetPosition = roundTo2Dec(extTargetPosition);
                grabLastPosition = roundTo2Dec(grabLastPosition);
                grabTargetPosition = roundTo2Dec(grabTargetPosition);

            }
            Log.i("servoThread"+this.getId(), "thread finished");
        }

        public void directSetGrabTarget(double target){
            target = roundTo2Dec(target);
            grabLastPosition = target;
            grabTargetPosition = target;
            grabber.setPosition(target);
            grabberDelayMove = false;
        }

        public void delaySetGrabTarget(double target){
            target = roundTo2Dec(target);
            grabTargetPosition = target;
            grabberDelayMove = true;
        }

        public void setExtTarget(double target){
            if(target > 0.99){target = 0.99;}
            if(target < 0.2){target = 0.2;}
            Log.i("servoThread"+this.getId(),"setting target "+target);
            extTargetPosition = roundTo2Dec(target);
        }

        public void setDelay(int delayPerStep){
            delayStep = delayPerStep;
        }
        public void servoWait(int milisec){try {Thread.sleep(milisec);} catch (Exception e) {} }
        public void stopThread(){
            stop = true;
        }

    }

    private double roundTo2Dec(double d){
        return (Math.round(100*d) / 100.0);
    }

    protected void printAllThreadsToLogcat(){
        Set<Thread> keys = Thread.getAllStackTraces().keySet();
        Log.d("All threads log start","-------------------- "+keys.size()+"Threads -----------------------");
        for(Thread t : keys){
            Log.d("All threads: #"+t.getId(),t.getName()+" Priority is "+t.getPriority()+", in thread group "+t.getThreadGroup().getName());
        }
        Log.d("All threads log end","-------------------------------------------");
    }
}
