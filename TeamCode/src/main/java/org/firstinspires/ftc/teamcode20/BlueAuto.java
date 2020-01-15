package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class BlueAuto extends TractionControl {
    protected final double odometryEncPerInch = 1316;//4096.0/Math.PI;
    protected int offsetY = 0;
    private double speed = 0.3, kP = 0.5, kI = 0, kD = 0.0025;

    protected void moveInchesGO(double yInch, double speed) {
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
        while (multiply_factor>0.1){
            multiply_factor = -Math.min(1, Math.max(-1, (kP * (getYOdometry() - odometryYGoal) / odometryEncPerInch) + (kI * IError) + (kD * (getYOdometry() - previousPos))));
            Dterm = getYOdometry() - previousPos;
            previousPos = getYOdometry();
            IError += (getYOdometry() - odometryYGoal) / odometryEncPerInch;
            setAllDrivePowerG(multiply_factor * (-vx - vy), multiply_factor * (vx - vy), multiply_factor * (-vx + vy), multiply_factor * (vx + vy));
/*
            telemetry.addData("kP", kP);
            telemetry.addData("P term", (getYOdometry() - odometryYGoal) / odometryEncPerInch);
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
    @Override
    public void init() {
        showTelemetry = false;
        initDrivetrain();
        initIMU();
        initGrabber();
        initLinSlide();
        initPlatformGrabber();
        initVuforia();
        initSensors();
        setNewGyro0();
        speed=0.3;
    }
    @Override
    public void loop() {
        //go forward 1 floor mat (24")w
        //vuforia - recognize block & move to pick up
        //after pickup: turn 90 deg. move to platform, drop off
        //move to platform, drag into position, release
        //repeat until run out of time; first on other skystones

        //initialization
        grabber_extend1.setPosition(1);
        grabber_extend2.setPosition(0);
        grabber.setPosition(grabber_open);
        platform_grabber.setPower(1);
        wait(300);
        platform_grabber.setPower(0.0);
        moveInchesG(0,15,0.3);
        if(showTelemetry)telemetry.clear();
        int pos = skystonePosition();
        shutdownVuforia();

        int shift;
        if(pos == 1){shift = 0;}
        else if (pos == 0){
            moveInchesG(-6.5,0,0.4);
            shift=-8;
        }
        else {
            moveInchesG(6.5, 0, 0.4);
            shift=8;
        }

        //move to the first blocc
        ElapsedTime p = new ElapsedTime();
        reset_ENCODER();
        setMode_RUN_WITHOUT_ENCODER();
        while ( (ymult*8>Math.abs(LB.getCurrentPosition())) && 1.3 < (left.getDistance(DistanceUnit.INCH)) && (1.3 < right.getDistance(DistanceUnit.INCH)) && p.milliseconds()<1000){
            setAllDrivePowerG(-0.25, -0.25, 0.25, 0.25);
        }

        //grab 1st block
        grabber.setPosition(grabber_closed);
        wait(300);
        setAllDrivePower(0.0);
        moveInchesG(0,-15,0.3);

        //move forward & approach foundation
        turn(90, 0.3, 1);
        setNewGyro(90);
        p.reset();
        moveInchesG(0,88+shift,0.3);
        setAllDrivePowerG(-.35,.35,-.35,.35);
        wait(1200);

        //move foundation
        platform_grabber.setPower(-.8);
        wait(200);
        turn(90, 0.7, 2);

        //drag foundation
        setNewGyro(180);
        /*
        double koe=0.75;
        p.reset();
        while(10<rangeSensorFront.getDistance(DistanceUnit.INCH) || p.milliseconds() < 3400){
            setAllDrivePowerG(koe*(0.25-0.55+0.37),koe*(0.25-0.55-0.37),koe*(0.25+0.55+0.37),koe*(0.22+0.5-0.37)); //turn+f0rwrd+side
        }
        */
        setAllDrivePowerG(-.5,.5,-.5,.5,2);
        wait(1200);
        setAllDrivePower(0);
        //align to the right wall
        /*
        while(30>rangeSensorFront.getDistance(DistanceUnit.INCH)){
            //telemetry.addData("Side",rangeSensorSide.getDistance(DistanceUnit.INCH));
            //telemetry.update();
            setAllDrivePowerG(0.5,-0.5,0.5,-0.5);
        }
        setAllDrivePower(0.0);
         */
/*
        //turn and drop the block
        platform_grabber.setPower(1);
        wait(300);
        platform_grabber.setPower(0.0);
        moveInchesG(-4,0,0.4);
        turn(-90,0.5,3);

        //L1.setPower(-0.35);
        //L2.setPower(0.35);
        //wait(500);
        grabber_extend1.setPosition(0.5);
        grabber_extend2.setPosition(0.5);
        wait(1000);
        //L1.setPower(0.0);
        //L2.setPower(0.0);
        grabber.setPosition(1);
        wait(300);
        //L1.setPower(0.5);
        //L2.setPower(-0.5);
        grabber_extend1.setPosition(1);
        grabber_extend2.setPosition(0);
        wait(2000);
        //L1.setPower(0.0);
        //L2.setPower(0.0);

        //park
        setNewGyro(90);
        moveInchesGO( -19, 0.4);

 */
        requestOpModeStop();
    }
}
