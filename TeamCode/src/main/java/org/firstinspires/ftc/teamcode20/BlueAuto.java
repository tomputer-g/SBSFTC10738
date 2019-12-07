package org.firstinspires.ftc.teamcode20;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode20.BaseAuto;
@Autonomous
public class BlueAuto extends BaseAuto {
    private double speed = 0.4;
    private boolean[] ca={true},cb={true};
    @Override
    public void init() {
        initDrivetrain();
        initIMU();
        initGrabber();
        initLinSlide();
        initVuforiaWebcam();
        left = hardwareMap.get(Rev2mDistanceSensor.class,"left");
        right = hardwareMap.get(Rev2mDistanceSensor.class,"right");
        setNewGyro0();
        grabber.setPosition(grabber_open);
        grabber_extender.setPower(1);
        wait(400);
        grabber_extender.setPower(0);
        speed=0.18;
        //telemetry.setAutoClear(false);
    }
    @Override
    public void loop() {
        //go forward 1 floor mat (24")w
        //vuforia - recognize block & move to pick up
        //after pickup: turn 90 deg. move to platform, drop off
        //move to platform, drag into position, release
        //repeat until run out of time; first on other skystones
        //moveInches(-8,0,speed);


        moveInchesG(0,12,0.3);
        telemetry.clear();
        int pos = skystonePosition();
        telemetry.addData("pos: ",pos);
        telemetry.update();
        if(pos == 1){moveInchesG(0,0,0.3);}
        else if (pos == 0) moveInchesG(-8,0,0.3);

        //buffer
        turn(0,0.3,1);

        //move to blocc
        while ((3.5 < left.getDistance(DistanceUnit.INCH)) && (3.5 < right.getDistance(DistanceUnit.INCH))) {
            setAllDrivePowerG(-speed, -speed, speed, speed);
            /*
            telemetry.addData("L: ", left.getDistance(DistanceUnit.INCH));
            telemetry.addData("R:", right.getDistance(DistanceUnit.INCH));
            telemetry.addData("power", LF.getPower());
            telemetry.addData("power", LB.getPower());
            telemetry.addData("power", RF.getPower());
            telemetry.addData("power", RB.getPower());
            telemetry.update();
             */
        }
        setAllDrivePower(0);
        grabber_extender.setPower(1);
        wait(500);
        grabber.setPosition(.55);
        wait(700);
        moveInchesG(0, -3.5, 0.3);
        setAllDrivePower(0);
        turn(90, 0.4, 8);
        grabber_extender.setPower(-1);
        wait(500);
        grabber_extender.setPower(0);
        setNewGyro(90);
        moveInchesG(0, 60, 0.4);

        //drop
        grabber_extender.setPower(1);
        wait(500);
        grabber_extender.setPower(0);
        grabber.setPosition(grabber_open);
        wait(700);

        //going bacc for the second blocc
        moveInchesG(0, -84, 0.3);
        turn(-90,0.37,5);
        setNewGyro0();
        moveInchesG(4,0,0.25);
        turn(0,0.3,3);
        //grab the second blocc
        while (2.5 < left.getDistance(DistanceUnit.INCH) && 2.5 < right.getDistance(DistanceUnit.INCH)){
            setAllDrivePowerG(-speed, -speed, speed, speed);
        }
        setAllDrivePower(0);
        grabber_extender.setPower(1);
        wait(500);
        grabber.setPosition(0.55);
        wait(700);

        moveInchesG(0, -5.5, 0.3);
        setAllDrivePower(0);
        grabber.setPosition(grabber_closed);
        turn(90, 0.4, 8);
        grabber_extender.setPower(-1);
        wait(500);
        grabber_extender.setPower(0);
        setNewGyro(90);
        moveInchesG(0, 84, 0.35);

        //drop
        grabber.setPosition(grabber_open);
        wait(200);

        //park
        moveInchesG(0, -19, 0.5);


        requestOpModeStop();
    }
}
