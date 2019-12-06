package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(group = "Final")
public class TeleOp_Drive extends BaseAuto {


    private boolean BPrimed = false, RBPrimed = false, YPrimed = false, LP, RP;
    private boolean movingExtender = false;
    //slide


    @Override public void init() {
        initDrivetrain();
        initGrabber();
        initLinSlide();
        initPlatformGrabber();
        L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabber.setPosition(0.35);
    }

    @Override
    public void loop() {

        runSlide();
        if(holdSet){telemetry.addData("Hold pos", hold);}

        if(this.gamepad1.y){YPrimed = true;}if(!this.gamepad1.y && YPrimed){YPrimed = false;
            slow = !slow;
        }

        /*
        if(this.gamepad1.dpad_left){LP = true;}if(!this.gamepad1.dpad_left && LP) { LP = false;
            a -= 0.01;
        }
        if(this.gamepad1.dpad_right){RP = true;}if(!this.gamepad1.dpad_right && RP){ RP = false;
            a += 0.01;
            if(a > 0.25)a = 0.25;
        }

         */

        if(this.gamepad1.dpad_left){//move -130
            platform_grabber.setPower(-0.2);
        }else if(this.gamepad1.dpad_right){
            platform_grabber.setPower(0.2);
        }else{
            platform_grabber.setPower(0);
        }

        if(this.gamepad1.b){BPrimed = true;}if(!this.gamepad1.b && BPrimed){BPrimed = false;
            if(grabber.getPosition() > 0.45){
                grabber.setPosition(0.35);
            }else{
                grabber.setPosition(0.55);
            }
        }

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
        if(整(this.gamepad1.x,xprime)) {
            double v = 0;
            while (left.getDistance(DistanceUnit.INCH) < 8.5 && right.getDistance(DistanceUnit.INCH) < 8.5) {
                v = (right.getDistance(DistanceUnit.INCH) - left.getDistance(DistanceUnit.INCH)) / 2;
                v = Math.min(Math.max(v, -0.2), 0.2);
                setAllDrivePower(-0.2 + v, -0.2 + v, .2 + v, .2 + v);

            }

        }

        */

        if(slow){
            scaledMove(-this.gamepad1.left_stick_x*0.4,-this.gamepad1.left_stick_y*0.4, (this.gamepad1.left_bumper?0:-this.gamepad1.right_stick_x*0.4));
        }else{
            scaledMove(-this.gamepad1.left_stick_x,-this.gamepad1.left_stick_y, (this.gamepad1.left_bumper?0:-this.gamepad1.right_stick_x));
        }


        if(slow)telemetry.addLine("slide and drivetrain slowed");
        //telemetry.addData("a",a);
        telemetry.addData("ext pos", grabber_extender.getCurrentPosition());
        telemetry.addData("slide pos",L1.getCurrentPosition());
        telemetry.addData("RT state", RTState);
        telemetry.addData("platform pos", platform_grabber.getCurrentPosition());

        telemetry.update();
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
                L1.setPower(-0.5);
                L2.setPower(0.5);
                if(near(L1.getCurrentPosition(), 0, 40)){
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
        telemetry.addLine("Result|----X----|----Y----|----R----");
        telemetry.addLine("           |"+to3dstr(RF-LF-RB+LB)+"|"+to3dstr(RF+RB+LF+LB)+"|"+to3dstr(RF+RB-LF-LB));
    }


    private double linear(double input, double minLimit, double maxLimit){
        if(input > -ctrl_deadzone && input < ctrl_deadzone){return 0;}

        double m = (maxLimit - minLimit)/(1-ctrl_deadzone);
        if(input < 0)
            return m * input - minLimit;
        return m * input + minLimit;
    }
}
