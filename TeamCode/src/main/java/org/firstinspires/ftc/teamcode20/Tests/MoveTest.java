package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode20.BaseAuto;

import static java.lang.Math.sqrt;

@TeleOp
public class MoveTest extends BaseAuto {
    private double speeed, speed,x,y, GYRO_kp, side_distance, kp,kd,moveInches_kP = 0.5,odometryEncPerInch =1316;
    private int offsetX = 0, offsetY = 0;
    private boolean[] qq = {true}, bF={true}, lF = {true}, e = {true}, f = {true}, ee = {true}, ff = {true}, eee = {true}, fff = {true}, m = {true},mm={true},mmm={true},jk={true};
    private ElapsedTime t=new ElapsedTime();
    private double speedLF=0,speedLB=0,speedRF=0,speedRB=0;
    private double  kP = 0.5, kI = 0, kD = 0.0025;
    int WaitingTime = 300;
    int steps = 20;
    double basespeed = 0.23;


    private PG pg=new PG();
    private Thread uc=new UC();
    int dir;
    private void 三天之内刹了你(){
        setAllDrivePower(1,1,-1,-1);
        wait(200);
        setAllDrivePower(0);
    }

    @Override
    public void init(){
        initIMU();
        initDrivetrain();
        initOdometry();
        initLinSlide();
        initGrabber();
        initVuforia();
        //initVuforiaWebcam();
        setNewGyro0();
        rangeSensorSide = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "side");
        speed=0.3;
        speeed = 0.03;
        dir=1;
        y = -90;
        x = 0;
        // 三天之内刹了你();
    }

    @Override
    public void start(){
        pg.start();
        uc.start();
    }

    @Override
    public void stop(){
    }

    @Override
    public void loop(){
        if(zheng(this.gamepad1.dpad_left,eee))x-=2;
        if(zheng(this.gamepad1.dpad_right,fff))x+=2;
        if(zheng(this.gamepad1.dpad_up,ee))y+=0.01;
        if(zheng(this.gamepad1.dpad_down,ff))y+=0.01;
        if(zheng(this.gamepad1.y,m))speed+=1;
        if(zheng(this.gamepad1.a,mm))speed-=.01;
        if(zheng(this.gamepad1.b,f))setNewGyro0();
        /*
        if(zheng(this.gamepad1.left_bumper,bF)){
            ElapsedTime t=new ElapsedTime();
            targetsSkyStone.activate();
            VuforiaTrackable trackable = allTrackables.get(11);
            while(t.milliseconds()<50000) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    if (trackable.getName().equals("Rear Perimeter 1")) {
                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        telemetry.addLine("Turn " + (int) Math.abs(rotation.thirdAngle - 90) + (rotation.thirdAngle - 90 > 0 ? "deg. CW" : "deg. CCW"));
                        VectorF translation = lastLocation.getTranslation();
                        double disty = translation.get(1)/mmPerInch;
                        double distx = translation.get(0)/mmPerInch;
                        double distz = translation.get(2)/mmPerInch;
                        telemetry.addData("x: ",distx);
                        telemetry.addData("y: ",disty);
                        telemetry.addData("z: ",distz);
                    }
                    telemetry.update();
                }
            }
            shutdownVuforia();
        }
        */

        if(zheng(this.gamepad1.left_bumper,lF)) {
            //setP(-speed,-speed,speed,speed);
        }
        if(zheng(this.gamepad1.right_bumper,bF)) {
            turn(x,speed,2);
        }
        telemetry.addData("x: ",x);
        telemetry.addData("y: ",y);
        telemetry.addData("Imu: ","%.2f",getHeading());
        telemetry.addData("target: ",acctarget);
        telemetry.addData("Speed: ","%.2f" ,speed);;
        //telemetry.addData("[x]: ","%.2f",n_pass[0]);
        //telemetry.addData("[y]: ","%.2f" ,n_pass[1]);;
        telemetry.update();


    }

    private double getError(double target, double cur) {
        double robotError =target-cur;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    private double getError(double targetAngle) {
        return getError(targetAngle,getHeading());
    }
    private boolean onHeading(double turnSpeed, double angel,double epre, double Kp, double Kd, double threshold) {
 q
        boolean  onTarget = false;
        if (Math.abs(error) <= threshold) {
            steer = 0.0;
            speed = 0.0;
            onTarget = true;
        }
        else {
            steer = Range.clip(Kp*epre/180+Kd*(error-epre), -turnSpeed, turnSpeed);
            speed  = steer;//(turnSpeed<0)?Range.clip(steer,turnSpeed,0):Range.clip(steer,0,turnSpeed);
        }
        setAllDrivePower(speed);
        return onTarget;
    }
    protected void turn(double angle, double speed, double threshold) {
        setNewGyro(acctarget);
        double p_TURN = 6,d_turn=0,epre=getError(angle);
        while(!onHeading(speed, angle,epre, p_TURN,d_turn, threshold)){
            epre = getError(angle);
        }
        acctarget=getError(acctarget+angle,0);
    }

    private class UC extends Thread{
        volatile boolean stop = false,run=false;
        @Override
        public void run() {
            while(!isInterrupted()&&!stop){
                updateCoo();
            }
        }
        public void stopThread(){
            stop = true;
        }
    }

    private class PG extends Thread{
        volatile boolean stop = false,run=false;
        private double a,b,c,d,Kp;

        public void PG(){ a=0;b=0;c=0;d=0;Kp=.8; }
        public void setAllPower(double w,double x,double y,double z){ a=w;b=x;c=y;d=z; }

        @Override
        public void run() {
            double p = 0;
            while (!isInterrupted() && !stop) {
                if (a == 0 && b == 0 && c == 0 && d == 0) {
                } else {
                    p = Kp * (getHeading() * 0.1 / 9);
                    setAllDrivePower(a - p, b - p, c - p, d - p);
                }
            }


            }


    protected void slowModeMove(double vx, double vy, double vr){
        double[] speeds = {vx - vy + vr, -vy - vx + vr, vx + vy + vr, -vx + vy + vr};
        double absMax = 0;
        for(double d : speeds)
            absMax = Math.max(Math.abs(d),absMax);
        if(absMax <= 1 && Math.abs(vr) < 0.01){
            setAllDrivePowerG(speeds[0], speeds[1], speeds[2], speeds[3]);
        }else if(Math.abs(vr) < 0.01){
            if(showTelemetry)telemetry.addLine("SCALED power: max was "+absMax);
            setAllDrivePowerG(speeds[0]/absMax, speeds[1]/absMax, speeds[2]/absMax,speeds[3]/absMax);
        }else if(absMax <= 1){
            setNewGyro0();
            setAllDrivePower(speeds[0], speeds[1], speeds[2], speeds[3]);
        }else{
            setNewGyro0();
            setAllDrivePower(speeds[0]/absMax, speeds[1]/absMax, speeds[2]/absMax,speeds[3]/absMax);
        }
        if(Math.abs(vx) < 0.01 && Math.abs(vy) < 0.01 && Math.abs(vr) < 0.01){
            setNewGyro0();
            setAllDrivePower(0);
        }
    }
    //move
    protected void moveInchesGOY(double yInch, double speed) {
        offsetY = getY1Odometry();
        speed = Math.abs(speed);
        double multiply_factor = 1;
        int odometryYGoal = offsetY + (int) (yInch * odometryEncPerInch);
        double vx = 0;
        double vy = (yInch == 0) ? 0 : (yInch / Math.abs(yInch) * speed);
        long IError = 0;
        setAllDrivePowerG((vy), (vy), (-vy), (-vy));
        int previousPos = getY1Odometry();
        int Dterm;
        //platform_grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (multiply_factor>0.1) {
            multiply_factor = -Math.min(1, Math.max(-1, (kP * (getY1Odometry() - odometryYGoal) / odometryEncPerInch) + (kI * IError) + (kD * (getY1Odometry() - previousPos))));
            Dterm = getY1Odometry() - previousPos;
            previousPos = getY1Odometry();
            IError += (getY1Odometry() - odometryYGoal) / odometryEncPerInch;
            setAllDrivePowerG(multiply_factor * (-vx - vy), multiply_factor * (vx - vy), multiply_factor * (-vx + vy), multiply_factor * (vx + vy));
            /*
            telemetry.addData("kP", kP);
            telemetry.addData("P term", (getY1Odometry() - odometryYGoal) / odometryEncYPerInch);
            telemetry.addData("kI", kI);
            telemetry.addData("I term", IError);
            telemetry.addData("kD", kD);
            telemetry.addData("D term", Dterm);
            telemetry.addData("current", getY1Odometry());
            telemetry.addData("Y goal", odometryYGoal);
            telemetry.update();
            */
                }
                setAllDrivePower(0);
            }
        }

}
