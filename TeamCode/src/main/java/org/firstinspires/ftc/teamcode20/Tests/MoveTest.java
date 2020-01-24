package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.w3c.dom.Element.*;
import com.google.ftcresearch.tfod.tracking.ObjectTracker;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.firstinspires.ftc.teamcode20.TractionControl;

import static java.lang.Math.sqrt;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@TeleOp
public class MoveTest extends BaseAuto {
    private double speeed, speed,x,y, GYRO_kp, side_distance, kp,kd,moveInches_kP = 0.5,odometryEncPerInch =1316;
    private int offsetX = 0, offsetY = 0;
    private boolean[] qq = {true}, bF={true}, lF = {true}, e = {true}, f = {true}, ee = {true}, ff = {true}, eee = {true}, fff = {true}, m = {true},mm={true},mmm={true},jk={true};
    private ElapsedTime t=new ElapsedTime();
    private double speedLF=0,speedLB=0,speedRF=0,speedRB=0;
    private double  kP = 0.5, kI = 0, kD = 0.0025;
    int WaitingTime = 300;

    //ModernRoboticsI2cRangeSensor rangeSensorSide;
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
    public void loop(){
        if(zheng(this.gamepad1.dpad_left,eee))speedLB+=0.1*dir;
        if(zheng(this.gamepad1.dpad_right,fff))speedLF+=0.1*dir;
        if(zheng(this.gamepad1.dpad_up,ee))speedRF+=0.1*dir;
        if(zheng(this.gamepad1.dpad_down,ff))speedRB+=0.1*dir;
        if(zheng(this.gamepad1.y,m))dir*=-1;
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
            setAllDrivePower(speedLF,speedLB,speedRF,speedRB);
            //wait(1000);
            //setAllDrivePowerG(-speed,-speed,speed,speed);
        }
            /*
            ElapsedTime t=new ElapsedTime();
            setAllDrivePower(-speed,-speed,speed,speed);
            wait(1200);
            t.reset();
            //setAllDrivePower(-.25,-.25,.25,.25);
            setAllDrivePower(0);
            int a=1000;
            while(!near(a,0,50)){
                a=L2.getCurrentPosition();
                wait(10);
                a=L2.getCurrentPosition()-a;
                if(a<307){
                    x=t.milliseconds();
                    setAllDrivePower(0);
                    setAllDrivePower(.03,.03,-.03,-.03);
                    t.reset();
                    break;
                }
            }
            while(!near(a,0,50)){
                a=L2.getCurrentPosition();
                wait(10);
                a=L2.getCurrentPosition()-a;
            }
            y=t.milliseconds();
            setAllDrivePower(0);
        }
        */
        if(zheng(this.gamepad1.right_bumper,bF)) {
            setAllDrivePower(0);
        }
        telemetry.addData("RFRBLFLB", "%.2f %.2f %.2f %.2f",speedRF,speedRB,speedLF,speedLB);
        //telemetry.addData("x: ",x);
        //telemetry.addData("y: ",y);
        //telemetry.addData("wait: ",WaitingTime);
        //telemetry.addData("Imu: ","%.2f",getHeading());
        //telemetry.addData("Speed: ","%.2f" ,speed);
        //telemetry.addData("enc X", xOdometry.getCurrentPosition());
        telemetry.addData("enc Y", LF.getCurrentPosition()/1305);
        telemetry.addData("ss", -platform_grabber.getCurrentPosition());
        telemetry.update();
    }

    //move
    protected void moveInchesGOY(double yInch, double speed) {
        offsetY = getYOdometry();
        speed = Math.abs(speed);
        double multiply_factor = 1;
        int odometryYGoal = offsetY + (int) (yInch * odometryEncPerInch);
        double vx = 0;
        double vy = (yInch == 0) ? 0 : (yInch / Math.abs(yInch) * speed);
        long IError = 0;
        setAllDrivePowerG((vy), (vy), (-vy), (-vy));
        int previousPos = getYOdometry();
        int Dterm;
        //platform_grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (multiply_factor>0.1) {
            multiply_factor = -Math.min(1, Math.max(-1, (kP * (getYOdometry() - odometryYGoal) / odometryEncPerInch) + (kI * IError) + (kD * (getYOdometry() - previousPos))));
            Dterm = getYOdometry() - previousPos;
            previousPos = getYOdometry();
            IError += (getYOdometry() - odometryYGoal) / odometryEncPerInch;
            setAllDrivePowerG(multiply_factor * (-vx - vy), multiply_factor * (vx - vy), multiply_factor * (-vx + vy), multiply_factor * (vx + vy));
            /*
            telemetry.addData("kP", kP);
            telemetry.addData("P term", (getYOdometry() - odometryYGoal) / odometryEncYPerInch);
            telemetry.addData("kI", kI);
            telemetry.addData("I term", IError);
            telemetry.addData("kD", kD);
            telemetry.addData("D term", Dterm);
            telemetry.addData("current", getYOdometry());
            telemetry.addData("Y goal", odometryYGoal);
            telemetry.update();
            */
        }
        setAllDrivePower(0);
    }

    protected void moveInches(double xInch, double yInch, double speed){
        setMode_RESET_AND_RUN_TO_POSITION();
        double p_mult = 80;
        double xmult = 232.5088/12, ymult = 232.7551/12;
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
}
