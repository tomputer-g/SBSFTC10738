package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private double speeed, speed,x,y, GYRO_kp, side_distance, kp,kd,moveInches_kP = 0.5,odometryEncPerInch =1313.13;
    private int offsetX = 0, offsetY = 0;
    private boolean[] bF={true}, lF = {true}, e = {true}, f = {true}, ee = {true}, ff = {true}, eee = {true}, fff = {true}, m = {true},mm={true},mmm={true},jk={true};
    private ElapsedTime t=new ElapsedTime();
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
        //initVuforiaWebcam();
        setNewGyro0();
        rangeSensorSide = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "side");
        speed=0.25;
        speeed = 0.03;
        dir=1;
        y = 0;
        x = 0;

        // 三天之内刹了你();
    }

    @Override
    public void loop(){
        if(zheng(this.gamepad1.dpad_left,eee))speeed*=-1;
        if(zheng(this.gamepad1.dpad_right,fff))x+=10;
        if(zheng(this.gamepad1.dpad_up,ee))y+=4;
        if(zheng(this.gamepad1.dpad_down,ff))y-=4;
        if(zheng(this.gamepad1.y,m))speed+=.01;
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
            reset_ENCODER();
            setMode_RUN_WITHOUT_ENCODER();
            ElapsedTime t=new ElapsedTime();
            LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LF.setTargetPosition((int)y*1313);
            while(LF.isBusy()&&t.milliseconds()<3000){
                telemetry.update();
                setAllDrivePowerG(-speed,-speed,speed,speed);}
            setAllDrivePower(0.0);
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

        if(zheng(this.gamepad1.right_bumper,bF)){
            //moveInchesGO(x,y,speed);
            //setAllDrivePower(0);
            //setAllDrivePower(speeed,speeed,-speeed,-speeed);
            setAllDrivePower(-speed,-speed,speed,speed);
        }

        if(zheng(this.gamepad1.right_bumper,bF)){
            //moveInchesGO(x,y,speed);
            //setAllDrivePower(0);
            //setAllDrivePower(speeed,speeed,-speeed,-speeed);
            setAllDrivePower(-speed,-speed,speed,speed);
            wait(1000);
            int a=L2.getCurrentPosition();
            wait(2000);
            a=L2.getCurrentPosition()-a;
            setAllDrivePower(0);
            y=a/2;
        }

        telemetry.addData("x: ",x);
        telemetry.addData("y: ",y);
        telemetry.addData("Imu: ",imuHeading);
        telemetry.addData("Speed: ", speed);
        telemetry.addData("Speeed: ", speeed);
        //telemetry.addData("enc X", xOdometry.getCurrentPosition());
        telemetry.addData("enc Y", RF.getCurrentPosition());
        telemetry.update();
    }


    protected void moveInchesGO(double xInch, double yInch, double speed){
        offsetX = platform_grabber.getCurrentPosition();
        offsetY = L2.getCurrentPosition();
        speed=Math.abs(speed);
        double multiply_factor=1;
        ElapsedTime stable_timer = null;
        int stable_timer_time = 1500;
        int odometryXGoal = offsetX + (int)(xInch * odometryEncPerInch), odometryYGoal = offsetY + (int)(yInch * odometryEncPerInch);
        double theta=(yInch==0)?90:Math.abs(Math.atan(xInch/yInch));
        double vx=(xInch==0)?0:(xInch/Math.abs(xInch)*Math.sin(theta)*speed);
        double vy=(yInch==0)?0:(yInch/Math.abs(yInch)*Math.cos(theta)*speed);
        while( stable_timer == null || stable_timer.milliseconds() < stable_timer_time){//!near(odometryYGoal, L2.getCurrentPosition(), 0.5*odometryEncPerInch) && !near(odometryXGoal, platform_grabber.getCurrentPosition(), 0.5*odometryEncPerInch)
            multiply_factor = -1*Math.min(1, Math.max(-1, moveInches_kP * (L2.getCurrentPosition() - odometryYGoal)/odometryEncPerInch));
            setAllDrivePowerG(multiply_factor*(-vx-vy),multiply_factor*(vx-vy),multiply_factor*(-vx+vy),multiply_factor*(vx+vy),0.8);
            if(near(multiply_factor, 0, 0.1) && stable_timer == null){
                stable_timer = new ElapsedTime();
            }
            if(stable_timer != null)telemetry.addData("stable timer", stable_timer.milliseconds());
            telemetry.addData("current",L2.getCurrentPosition());
            telemetry.addData("Y goal",odometryYGoal);
            telemetry.update();
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
