package org.firstinspires.ftc.teamcode20.Frank;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;
@TeleOp
public class SensorGrab extends BaseAuto {
    private double speed=.4,ratio=0.69,threshhold=9;
    private boolean[] qq = {true}, bF={true}, lF = {true}, e = {true}, f = {true}, ee = {true}, ff = {true}, eee = {true}, fff = {true}, m = {true},mm={true},mmm={true},jk={true};

    @Override
    public void runOpMode() throws InterruptedException {
        initIMU();
        initDrivetrain();
        initOdometry();
        initLinSlide();
        initGrabber();
        initSensors();
        waitForStart();
        while(opModeIsActive()){
            if(zheng(this.gamepad1.y,m))speed+=.1;
            if(zheng(this.gamepad1.a,mm))speed-=.1;
            if(zheng(this.gamepad1.b,f))setNewGyro0();

            if(zheng(this.gamepad1.left_bumper,e)){
                moveInchesG(0,(right.getDistance(DistanceUnit.INCH)-2.6)*ratio,speed);
            }

            if(zheng(this.gamepad1.right_bumper,bF)){
                double rightd=right.getDistance(DistanceUnit.INCH);
                while(threshhold<rightd) {
                    setAllDrivePowerG(-speed, speed, -speed, speed);
                    rightd=right.getDistance(DistanceUnit.INCH);
                    telemetry.update();
                }
                //while(left.getDistance(DistanceUnit.INCH)<threshhold)
                //    setAllDrivePowerG(-.3,.3,-.3,.3);
                setAllDrivePowerG(0);
                moveInchesGOX(3,.8);
                moveInchesGOY((rightd-2.6)*ratio,.4);
            }
            //telemetry.addData("Imu: ",getHeading());
            telemetry.addData("Speed: ",speed);
            telemetry.addData("Ultrasonic: ",rangeSensorSide.getDistance(DistanceUnit.INCH));
            telemetry.addData("Left: ",left.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right: ",right.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }



}
