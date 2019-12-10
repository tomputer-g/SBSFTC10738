package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Thread.sleep;

@TeleOp(group = "Final")
public class TeleOp_Drive extends BaseAuto {
    private boolean BPrimed = false, RBPrimed = false, YPrimed = false, XPrimed = false, LP, RP, LTPrimed = false;
    private boolean movingExtender = false;
    //slide
    private double kickstartSpeed = 0.18, lowSpeed = 0.03, PWMSpeed = 0.09, cycleTimeMS = 20;
    private boolean platformGrabbed = false;


    @Override public void init() {
        initDrivetrain();
        initGrabber();
        initLinSlide();
        initSensors();
        initPlatformGrabber();
        initIMU();
        L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabber.setPosition(grabber_open);
        platform_grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        platform_grabber.setPower(1);
        wait(150);
        platform_grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        platform_grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        platform_grabber.setPower(0);
    }

    @Override
    public void loop() {

        runSlide();
        if(holdSet){if(telemetryOn)telemetry.addData("Hold pos", hold);}

        if(this.gamepad1.y){YPrimed = true;}if(!this.gamepad1.y && YPrimed){YPrimed = false;
            slow = !slow;
        }


        if(this.gamepad1.left_trigger> 0.5) {
            LTPrimed = true;
        }
        if(this.gamepad1.left_trigger < 0.5 && LTPrimed){
            LTPrimed = false;
            if(platformGrabbed){//already held
                platformGrabbed = false;
                platform_grabber.setPower(1);
                wait(100);
                platform_grabber.setPower(0);
            }else{
                platformGrabbed = true;
                platform_grabber.setPower(-0.4);
            }
        }


        if(this.gamepad1.b){BPrimed = true;}if(!this.gamepad1.b && BPrimed){BPrimed = false;
            if(grabber.getPosition() > 0.45){
                grabber.setPosition(grabber_open);
            }else{
                grabber.setPosition(grabber_closed);
            }
        }

        /*
        if(this.gamepad1.x){XPrimed = true;}if(!this.gamepad1.x && XPrimed){XPrimed = false;
            setNewGyro(0);
            while ( !(near(left.getDistance(DistanceUnit.INCH),9.027, 0.6)&&near(right.getDistance(DistanceUnit.INCH),8.125, 0.6)) ){
                double l = left.getDistance(DistanceUnit.INCH), r = right.getDistance(DistanceUnit.INCH);
                l = Math.min(1,Math.abs(l-9.027));
                if     (r > 9.027)setAllDrivePowerG(-0.13*l,-0.13*l,0.13*l,0.13*l);
                else if(r < 9.027)setAllDrivePowerG(0.13*l,0.13*l,-0.13*l,-0.13*l);
            }
            setAllDrivePower(0);
        }

         */

        if(this.gamepad1.dpad_up
                ||this.gamepad1.dpad_down
                ||this.gamepad1.right_bumper
                ||(this.gamepad1.left_bumper && !near(this.gamepad1.right_stick_y, 0, 0.05))){

            RTState = -1; //driver interrupt auto movement
        }

        if(this.gamepad1.right_trigger > 0.3
                && L1.getCurrentPosition() < (2000 - 12 * encoderPerInch)
                && grabber_extender.getCurrentPosition() < -200
                && RTState == -1){
            //when can go 12in above & extender is extended & not started
            holdSet = false;
            RTState = 0;
        }
        handleRTState();

        if(this.gamepad1.right_bumper){RBPrimed = true;}if(!this.gamepad1.right_bumper && RBPrimed){RBPrimed = false;
            movingExtender = true;
            grabber_extender.setPower(1);
            if(grabber_extender.getCurrentPosition() > -110){
                grabber_extender.setTargetPosition(-330);
            }else{
                grabber_extender.setTargetPosition(0);
            }
            grabber_extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }//if RB toggle positions


        if(this.gamepad1.dpad_up){
            movingExtender = false;
            grabber_extender.setPower(-1);
        }else if(this.gamepad1.dpad_down){
            movingExtender = false;
            grabber_extender.setPower(1);
        }else{
            if(RTState == -1) {
                if (!movingExtender) {
                    grabber_extender.setPower(0);
                } else {
                    if (!grabber_extender.isBusy()) {
                        movingExtender = false;
                        grabber_extender.setPower(0);
                        grabber_extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }
            }
        }

        if(this.gamepad1.a){
            grabber_extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            grabber_extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        /*
        if(zheng(this.gamepad1.x,xprime)) {
            double v = 0;
            while (left.getDistance(DistanceUnit.INCH) < 8.5 && right.getDistance(DistanceUnit.INCH) < 8.5) {
                v = (right.getDistance(DistanceUnit.INCH) - left.getDistance(DistanceUnit.INCH)) / 2;
                v = Math.min(Math.max(v, -0.2), 0.2);
                setAllDrivePower(-0.2 + v, -0.2 + v, .2 + v, .2 + v);

            }

        }

        */

        if(L1.getCurrentPosition() > 200){
            if(telemetryOn)telemetry.addLine("Placing block: ultra slow");
            /*if(-this.gamepad1.left_stick_y > ctrl_deadzone){
                try {
                    movePWM(0.09);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }else if(this.gamepad1.left_stick_y > ctrl_deadzone){
                try {
                    movePWM(-0.09);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

             */
            scaledMove(-this.gamepad1.left_stick_x*0.3,-this.gamepad1.left_stick_y*0.15, (this.gamepad1.left_bumper?0:-this.gamepad1.right_stick_x*0.2));//replace with enc-based move
        }else if(slow){
            if(telemetryOn)telemetry.addLine("slide and drivetrain slowed");
            scaledMove(-this.gamepad1.left_stick_x*0.4,-this.gamepad1.left_stick_y*0.4, (this.gamepad1.left_bumper?0:-this.gamepad1.right_stick_x*0.3));
        } else{
            scaledMove(-this.gamepad1.left_stick_x,-this.gamepad1.left_stick_y, (this.gamepad1.left_bumper?0:-this.gamepad1.right_stick_x));
        }


        //if(telemetryOn)telemetry.addData("a",a);
        if(telemetryOn)telemetry.addLine("Dist: "+left.getDistance(DistanceUnit.INCH)+", "+right.getDistance(DistanceUnit.INCH));
        if(telemetryOn)telemetry.addData("ext", grabber_extender.getCurrentPosition());
        if(telemetryOn)telemetry.addData("slide",L1.getCurrentPosition());
        if(telemetryOn)telemetry.addData("RT state", RTState);
        if(telemetryOn)telemetry.update();
    }

    private void handleRTState(){//call in loop; non-blocking
        switch (RTState) {
            case -1: //none
                break;
            case 0: //just pressed button / moving upward 12 in
                holdSlide((int) (L1.getCurrentPosition() + 12 * encoderPerInch));
                grabber.setPosition(0);
                if (near(hold, L1.getCurrentPosition(), 40))//close enough
                    RTState = 1;
                break;
            case 1:
                grabber_extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                grabber_extender.setPower(1);
                if (near(grabber_extender.getCurrentPosition(), 0, 20)){
                    grabber_extender.setPower(0);
                    RTState = 2;
                }
                break;
            case 2://need -.5 power going down, test this
                holdSet = false;
                L1.setPower(-0.4);
                L2.setPower(0.4);
                if(L1.getCurrentPosition() < 40){
                    RTState = -1;
                    L1.setPower(0);
                    L2.setPower(0);
                }
                break;
        }
    }

    private double joystick_quad(double input){//up 1.2, down 0.5
        if(input < 0)
            return - (input * input);
        return input * input;
    }

    private void winstonSetPower(double LF, double LB, double RF, double RB){
        setAllDrivePower(-LF, -LB, RF, RB);//L motors reversed because it's winston power
        displayMotorPowers(LF, LB, RF, RB);
        if(telemetryOn)telemetry.addLine("Result|----X----|----Y----|----R----");
        if(telemetryOn)telemetry.addLine("           |"+to3dstr(RF-LF-RB+LB)+"|"+to3dstr(RF+RB+LF+LB)+"|"+to3dstr(RF+RB-LF-LB));
    }


    private double linear(double input, double minLimit, double maxLimit){
        if(input > -ctrl_deadzone && input < ctrl_deadzone){return 0;}

        double m = (maxLimit - minLimit)/(1-ctrl_deadzone);
        if(input < 0)
            return m * input - minLimit;
        return m * input + minLimit;
    }

    private void movePWM(double vy)throws InterruptedException {//3,9,18
        if(vy > kickstartSpeed){
            setAllDrivePower(-vy, -vy, vy, vy);
        }else if(vy >= lowSpeed){
            if(telemetryOn)telemetry.addLine("PWM move active");//set 20; change delay between 20 and 0
            double dutyPercent = (vy - lowSpeed) / (kickstartSpeed-lowSpeed);
            if(telemetryOn)telemetry.addLine("Duty "+(int)(100*dutyPercent)+"%; high "+to3d(dutyPercent * cycleTimeMS)+", low "+to3d((1-dutyPercent) * cycleTimeMS));

            setAllDrivePower(-kickstartSpeed, -kickstartSpeed, kickstartSpeed, kickstartSpeed);
            sleep(0,(int)(1000 * dutyPercent * cycleTimeMS));
            setAllDrivePower(-lowSpeed, -lowSpeed, lowSpeed, lowSpeed);
            sleep(0, (int)((1-dutyPercent) * cycleTimeMS));

        }else{
            setAllDrivePower(0);
            if(telemetryOn)telemetry.addLine("speed is below PWM minimum!");
        }
    }
}
