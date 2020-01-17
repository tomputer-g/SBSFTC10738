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
        moveInchesG(0,15,0.3);
        platform_grabber.setPower(0.0);
        if(showTelemetry)telemetry.clear();

        //find skystone
        int pos = skystonePosition();
        shutdownVuforia();

        //shift to align to skystone
        int shift;
        if(pos == 1){shift = 0;}
        else if (pos == 0){
            moveInchesG(-6.5,0,0.4);
            shift=-10;
        }
        else {
            moveInchesG(8, 0, 0.4);
            shift=8;
        }

        //move forward to the skystone
        ElapsedTime p = new ElapsedTime();
        reset_ENCODER();
        setMode_RUN_WITHOUT_ENCODER();

        while (p.milliseconds()<1000) setAllDrivePowerG(-0.25, -0.25, 0.25, 0.25);

        //grab 1st block
        grabber.setPosition(grabber_closed);
        wait(300);
        setAllDrivePower(0.0);
        moveInchesG(0,-10,0.3);

        //move forward & approach foundation
        turn(90, 0.3, 1);
        setNewGyro(90);
        p.reset();
        moveInchesG(0,88+shift,0.4);
        setAllDrivePowerG(-.35,.35,-.35,.35);
        wait(1500);

        //turn foundation
        platform_grabber.setPower(-.8);
        wait(200);
        turn(90, 0.9, 2);

        //drag foundation forward
        setNewGyro(180);
        /*
        double koe=0.75;
        p.reset();
        while(10<rangeSensorFront.getDistance(DistanceUnit.INCH) || p.milliseconds() < 3400){
            setAllDrivePowerG(koe*(0.25-0.55+0.37),koe*(0.25-0.55-0.37),koe*(0.25+0.55+0.37),koe*(0.22+0.5-0.37)); //turn+f0rwrd+side
        }
        */
        setAllDrivePower(0);
        double tempY = getYOdometry();
        double targetdist = getYOdometry()-12*1316;
        p = new ElapsedTime();
        while(getYOdometry()>targetdist&&p.milliseconds()<1500)
            setAllDrivePowerG(-0.5,-0.5,0.5,0.5,2);
        setAllDrivePower(0);

        //push it in
        setAllDrivePowerG(-.7,.7,-.7,.7,2);
        wait(1000);

        //release grabber
        platform_grabber.setPower(1);
        wait(300);
        setAllDrivePower(0);
        platform_grabber.setPower(0.0);

        //strafe left to put the block

        servoThread.setTarget(0.5);
        moveInchesG(-6.5,0,0.5);
        turn(-90,0.4,1);
        setNewGyro(90);
       // holdSlide((int)slideEncoderPerInch/10);
        //wait(1000);
        //moveInchesG(0,4,0.43);
        setAllDrivePowerG(-0.4,-0.4,0.4,0.4);
       // wait(200);
        servoThread.setTarget(.8);
        wait(1300);

        grabber.setPosition(grabber_open);
        platform_grabber.setPower(1);
        //park
        moveInchesG(-3,0,0.4);
        moveInchesG(0,-38,0.4);

        setAllDrivePower(0.0);
        servoThread.stopThread();
        setNewGyro0();
        platform_grabber.setPower(0);
        requestOpModeStop();
    }
}