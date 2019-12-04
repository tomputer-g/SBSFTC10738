package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(group = "Final")
public class TeleOp_Drive extends BaseOpMode {

    private final double ctrl_deadzone = 0.2;
    private boolean slow = false;
    private boolean BPrimed = false, RBPrimed = false, YPrimed = false, LP, RP;
    private boolean movingExtender = false;
    //slide
    private int hold = 0;
    private boolean holdSet;
    private double a = 0.2;



    @Override public void init() {
        initDrivetrain();
        initGrabber();
        initLinSlide();
        L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabber.setPosition(0);

    }

    @Override
    public void loop() {

        runSlide();
        if(holdSet){telemetry.addData("Hold pos", hold);}

        if(this.gamepad1.y){YPrimed = true;}if(!this.gamepad1.y && YPrimed){YPrimed = false;
            slow = !slow;
        }

        if(this.gamepad1.dpad_left){LP = true;}if(!this.gamepad1.dpad_left && LP) { LP = false;
            a -= 0.01;
        }
        if(this.gamepad1.dpad_right){RP = true;}if(!this.gamepad1.dpad_right && RP){ RP = false;
            a += 0.01;
            if(a > 0.25)a = 0.25;
        }

        if(this.gamepad1.b){BPrimed = true;}if(!this.gamepad1.b && BPrimed){BPrimed = false;
            if(grabber.getPosition() < 0.1){
                grabber.setPosition(.25);
            }else{
                grabber.setPosition(0);
            }
        }

        if(this.gamepad1.right_bumper){RBPrimed = true;}if(!this.gamepad1.right_bumper && RBPrimed){RBPrimed = false;
            movingExtender = true;
            grabber_extender.setPower(1);
            if(grabber_extender.getCurrentPosition() > -110){
                grabber_extender.setTargetPosition(-230);
            }else{
                grabber_extender.setTargetPosition(0);
            }
            grabber_extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }//if RB toggle positions


        if(this.gamepad1.dpad_up){
            movingExtender = false;
            grabber_extender.setPower(1);
        }else if(this.gamepad1.dpad_down){
            movingExtender = false;
            grabber_extender.setPower(-1);
        }else{
            if(!movingExtender){
                grabber_extender.setPower(0);
            }else{
                if(!grabber_extender.isBusy()){
                    movingExtender = false;
                    grabber_extender.setPower(0);
                    grabber_extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }
        }

        if(this.gamepad1.a){
            grabber_extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            grabber_extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(slow){
            scaledMove(-this.gamepad1.left_stick_x*0.4,-this.gamepad1.left_stick_y*0.4, (this.gamepad1.left_bumper?0:-this.gamepad1.right_stick_x*0.4));
        }else{
            scaledMove(-this.gamepad1.left_stick_x,-this.gamepad1.left_stick_y, (this.gamepad1.left_bumper?0:-this.gamepad1.right_stick_x));
        }

        if(slow){telemetry.addLine("slide and drivetrain slowed");}
        telemetry.addData("a",a);
        telemetry.addData("actual",L1.getCurrentPosition());
        telemetry.update();

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

    private void scaledMove(double vx, double vy, double vr){
        telemetry.addLine("vX: "+to3d(vx)+", vY: "+to3d(vy)+", vR: "+to3d(vr));
        double[] speeds = {vx - vy + vr, -vy - vx + vr, vx + vy + vr, -vx + vy + vr};
        double absMax = 0;
        for(double d : speeds)
            absMax = Math.max(Math.abs(d),absMax);
        if(absMax <= 1){
            setAllDrivePower(speeds[0], speeds[1], speeds[2], speeds[3]);
            /*
            telemetry.addData("vLF",to3d(speeds[0]));
            telemetry.addData("vLB",to3d(speeds[1]));
            telemetry.addData("vRF",to3d(speeds[2]));
            telemetry.addData("vRB",to3d(speeds[3]));
            */
        }else{
            telemetry.addLine("SCALED power: max was "+absMax);
            /*
            telemetry.addLine("vLF: "+to3d(speeds[0])+" -> "+to3d(speeds[0]/absMax));
            telemetry.addLine("vLB: "+to3d(speeds[1])+" -> "+to3d(speeds[1]/absMax));
            telemetry.addLine("vRF: "+to3d(speeds[2])+" -> "+to3d(speeds[2]/absMax));
            telemetry.addLine("vRB: "+to3d(speeds[3])+" -> "+to3d(speeds[3]/absMax));
             */
            setAllDrivePower(speeds[0]/absMax, speeds[1]/absMax, speeds[2]/absMax,speeds[3]/absMax);
        }
    }

    private void runSlide(){
        if(this.gamepad1.left_bumper && !near(this.gamepad1.right_stick_y, 0, 0.05)) {//long-dist
            if (this.gamepad1.right_stick_y < 0 && L1.getCurrentPosition() < 2000) {//up
                holdSet = false;
                telemetry.addLine("CHANGING SLIDE");
                L1.setPower(-this.gamepad1.right_stick_y);
                L2.setPower(this.gamepad1.right_stick_y);
            } else if (this.gamepad1.right_stick_y > 0 && L1.getCurrentPosition() > 0) {
                holdSet = false;
                telemetry.addLine("CHANGING SLIDE");
                if(slow){
                    L1.setPower(-((a+(2000-L1.getCurrentPosition())/2000.0)*(0.2-a) ) * this.gamepad1.right_stick_y);
                    L2.setPower(((a+(2000-L1.getCurrentPosition())/2000.0)*(0.2-a) )* this.gamepad1.right_stick_y);
                    telemetry.addData("power",-((a+(2000-L1.getCurrentPosition())/2000.0)*(0.2-a) ) * this.gamepad1.right_stick_y);
                }else{
                    L1.setPower(-0.5 * this.gamepad1.right_stick_y);
                    L2.setPower(0.5 * this.gamepad1.right_stick_y);
                }

            } else {
                holdSlide();
            }
        }else{
            holdSlide();
        }
    }

    private void holdSlide(){
        if (!holdSet) {
            holdSet = true;
            hold = Math.max(0,Math.min(2000,L1.getCurrentPosition()));
        }
        int error = hold - L1.getCurrentPosition();
        double power = Math.min(1, Math.max(0, error/60.0));
        if(hold == 0){power = 0;}
        telemetry.addLine("error: "+hold+" - "+(hold-error) + " = "+error);
        telemetry.addData("PWR", power);
        L1.setPower(power);
        L2.setPower(-power);
    }

    private double linear(double input, double minLimit, double maxLimit){
        if(input > -ctrl_deadzone && input < ctrl_deadzone){return 0;}

        double m = (maxLimit - minLimit)/(1-ctrl_deadzone);
        if(input < 0)
            return m * input - minLimit;
        return m * input + minLimit;
    }
}
