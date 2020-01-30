package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class RedAuto extends TractionControl {
    private double speed = 0.4;
    private boolean[] ca={true},cb={true};
    @Override
    public void init() {
        super.init();
        initDrivetrain();
        initIMU();
        initGrabber();
        initLinSlide();
        initPlatformGrabber();
        initVuforia();
        initSensors();
        setNewGyro0();
        speed=0.3;
        //if(showTelemetry)telemetry.setAutoClear(false);
    }
    @Override
    public void loop() {
        //go forward 1 floor mat (24")w
        //vuforia - recognize block & move to pick up
        //after pickup: turn 90 deg. move to platform, drop off
        //move to platform, drag into position, release
        //repeat until run out of time; first on other skystones
        grabber.setPosition(0.03);
        grabber_extend1.setPosition(1);
        grabber_extend2.setPosition(0);
        grabber.setPosition(0.03);
        platform_grabber.setPower(1);
        moveInchesG(0,12,0.3);
        platform_grabber.setPower(0.0);
        if(showTelemetry)telemetry.clear();
        int[] resultcounter = {0,0,0};
        //find skystone
        for (int i = 0;i<10;++i){
            resultcounter[new_skystoneposition()]++;
        }
        int curmax = -1;
        int pos = 0;
        for (int i = 0;i<3;++i){
            if(resultcounter[i]>curmax){pos = i;curmax=resultcounter[i];}
        }

        grabber.setPosition(grabber_open);
        shutdownVuforia();

        //shift to align to skystone
        int shift;
        if(pos == 1){
            shift = 0;
        }
        else if (pos == 0){
            moveInchesG(-7.5,0,0.4);
            shift=8;
        }
        else {
            moveInchesG(8, 0, 0.4);
            shift=-8;
        }

        //move forward to the skystone
        ElapsedTime p = new ElapsedTime();
        reset_ENCODER();
        setMode_RUN_WITHOUT_ENCODER();
        grabber.setPosition(grabber_open);
        while (p.milliseconds()<1300) setAllDrivePowerG(-0.25, -0.25, 0.25, 0.25);

        //grab 1st block
        grabber.setPosition(grabber_closed);
        wait(300);
        setAllDrivePower(0.0);
        moveInchesG(0,-15,0.3);

        //move forward & approach foundation
        turn(90, 0.4, 0.8);
        servoThread.setTarget(0.95);
        setNewGyro(90);
        p.reset();
        moveInchesG(0,-(88+shift-8),0.4);
        setAllDrivePowerG(-.35,.35,-.35,.35);
        wait(1500);


        //turn foundation
        platform_grabber.setPower(-.8);
        wait(200);

        setAllDrivePowerG(0.5,-0.5,0.5,-0.5);
        wait(1000);
        setAllDrivePower(0);
        turn(-90, 0.85, 4);

        //drag foundation forward
        setNewGyro(0);
        /*
        double koe=0.75;
        p.reset();
        while(10<rangeSensorFront.getDistance(DistanceUnit.INCH) || p.milliseconds() < 3400){
            setAllDrivePowerG(koe*(0.25-0.55+0.37),koe*(0.25-0.55-0.37),koe*(0.25+0.55+0.37),koe*(0.22+0.5-0.37)); //turn+f0rwrd+side
        }
        */
        setAllDrivePower(0);

        double tempY = getY1Odometry();
        double targetdist = getY1Odometry()+12*1316;
        p = new ElapsedTime();
        while(getY1Odometry()<targetdist&&p.milliseconds()<1500)
            setAllDrivePowerG(0.5,0.5,-0.5,-0.5,2);
        setAllDrivePower(0);

        //push it in
        setAllDrivePowerG(-.7,.7,-.7,.7,2);
        wait(700);
        platform_grabber.setPower(0.0);

        //strafe left to put the block

        servoThread.setTarget(0.5);
        moveInchesG(-6.5,0,0.5);
        turn(-85,0.4,1);
        setNewGyro(-90);
        // holdSlide((int)slideEncoderPerInch/10);
        //wait(1000);
        //moveInchesG(0,4,0.43);
        setAllDrivePowerG(-0.4,-0.4,0.4,0.4);
        // wait(200);
        servoThread.setTarget(.75);
        wait(1300);
        setAllDrivePower(0);

        grabber.setPosition(grabber_open);
        platform_grabber.setPower(1);
        wait(500);
        //park
        //moveIncheszG(2.67,0,0.4);
        moveInchesG(0,-35,0.5);

        setAllDrivePower(0.0);
        servoThread.stopThread();
        setNewGyro0();
        platform_grabber.setPower(0);

        requestOpModeStop();
    }
}
