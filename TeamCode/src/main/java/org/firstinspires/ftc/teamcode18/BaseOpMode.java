package org.firstinspires.ftc.teamcode18;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.text.DecimalFormat;

import static java.lang.Thread.sleep;

/**
 * Created by Ziming Gao on 12/5/2017.
 */

public class BaseOpMode extends OpMode {

    private final int oneInch = 224, angleACCturnBuffer = 30, posACCmoveBuffer = 1000;

    private final double SLIDE_ODS_THRESHOLD = 0.15, minPwr = 0.1;
    protected Servo CSArm;
    protected AnalogInput homeSlide;
    protected DcMotor LF,LB,RF,RB,slide, grabber;//sensors & actuators
    protected double angle = 0.0;
    protected BNO055IMU imu;
    protected Orientation lastAngles = new Orientation();//imu

    //---------------------------------@Overrides--------------------------------
    @Override public void init() {
        msStuckDetectInit = 10000;
        msStuckDetectLoop = 10000;
        LF = hardwareMap.get(DcMotor.class,"LF");
        LB = hardwareMap.get(DcMotor.class,"LB");
        RF = hardwareMap.get(DcMotor.class,"RF");
        RB = hardwareMap.get(DcMotor.class,"RB");
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode_RUN_WITH_ENCODER();
        CSArm = hardwareMap.get(Servo.class,"csarm");
        grabber = hardwareMap.get(DcMotor.class,"ls1");
        grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber.setPower(0.5);

        slide = hardwareMap.get(DcMotor.class,"slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //INIT ACTUATORS: MOTORS

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters BNO055parameters = new BNO055IMU.Parameters();
        BNO055parameters.mode = BNO055IMU.SensorMode.IMU;
        BNO055parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        BNO055parameters.loggingEnabled = false;
        imu.initialize(BNO055parameters);
        while(!imu.isGyroCalibrated()){wait(50);}//INIT IMU

        homeSlide = hardwareMap.analogInput.get("home");//INIT SENSORS


    }

    @Override public void loop() {}

    //-----------------------------software only methods-------------------------
    protected double to3d(double d){
        DecimalFormat df = new DecimalFormat("##0.000");
        return Double.parseDouble(df.format(d));
    }

    protected boolean near(double a, double b, double tolerance){
        return Math.abs(a-b) <= tolerance;
    }

    protected double getAngle() {//20ms
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        angle += deltaAngle;
        lastAngles = angles;
        return angle;
    }
    protected void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angle = 0;
    }
    //------------------------------actuator methods-----------------------------
        protected void turn(double deltaDegree){//deltaDegree: CW--, CCW++,
        angle = getAngle();
        double startAngle = angle, endAngle = angle + deltaDegree +(deltaDegree > 0 ? -8 : 8);
        setMode_RUN_WITH_ENCODER();
        setAllDrivePower((deltaDegree > 0)?-0.1:0.1);
        while (angle <= endAngle) {
            angle = getAngle();
            /*if(deltaDegree > 0) {
                setAllDrivePower(-Range.clip(Math.min(0.02 * (angle - startAngle) + 0.1, 0.002 * (endAngle - angle) * (endAngle - angle) + 0.1), 0.1, 0.3));
            }else{
                setAllDrivePower(Range.clip(Math.min(0.02 * (startAngle - angle) + 0.1, 0.002 * (angle - endAngle) * (angle - endAngle) + 0.1), 0.1, 0.3));
            }*/
        }
        setAllDrivePower(0);
    }
    protected void adjustLinSlide(Boolean raise){
        final int start = 30,lowBuffer = oneInch * 2 + 30, stop1 = oneInch * 7 + 30, highBuffer = oneInch * 12 , end = oneInch * 13 + 10;
        if(raise){
            if(near(start, slide.getCurrentPosition(),300)){
                slide.setPower(1);
                slide.setTargetPosition(stop1);
            }else if(near(stop1,slide.getCurrentPosition(),300)){
                slide.setPower(1);
                slide.setTargetPosition(highBuffer);
                while(slide.isBusy());
                slide.setPower(0.2);
                slide.setTargetPosition(end);
            }
        }else{
            if(near(end, slide.getCurrentPosition(),300)){
                slide.setPower(1);
                slide.setTargetPosition(stop1);
            }else if(near(stop1, slide.getCurrentPosition(),300)){
                slide.setPower(1);
                slide.setTargetPosition(lowBuffer);
                while(slide.isBusy());
                slide.setPower(0.2);
                slide.setTargetPosition(start);
            }
        }
    }

    protected void homeSlide(){//works!!!
        if(!near(slide.getCurrentPosition(),0,500)) {
            setAllDrivePower(0);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setPower(-0.3);
            double baseVolt = homeSlide.getVoltage();
            while (homeSlide.getVoltage() <= baseVolt + SLIDE_ODS_THRESHOLD) ;
            slide.setPower(0);
            int oldPosition, newPosition;
            do {
                oldPosition = slide.getCurrentPosition();
                wait(100);
                newPosition = slide.getCurrentPosition();
            } while (!near(oldPosition, newPosition, 3));
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    //----------------------------Lazy methods-----------------------------------
    void setMode_RESET_ENCODER(){
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void setMode_RUN_WITH_ENCODER(){
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void setMode_RUN_TO_POSITION(){
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
    protected void wait(int time){
        try {sleep(time);} catch (InterruptedException e) {e.printStackTrace();}
    }
}
