package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(group = "Final")
public class TeleOp_Drive extends BaseAuto {

    private double v = 0, x = 0;
    private double limit = 0.5;
    private final double ctrl_deadzone = 0.2;
    private ElapsedTime t;
    private double value = 0, a_up = 1.2, a_down = 0.8;
    private int slideLimit = 2000;
    private boolean slow = false;
    private boolean BPrimed = false, RBPrimed = false, YPrimed = false;
    private boolean[] xprime={true};
    private boolean movingExtender = false;

    @Override
    public void init() {
        initDrivetrain();
        initGrabber();
        initLinSlide();
        initSensors();
        grabber.setPosition(1);
        L1.setTargetPosition(0);
        L2.setTargetPosition(0);
        L1.setPower(1);
        L2.setPower(1);
        L1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {

        if(this.gamepad1.y){YPrimed = true;}if(!this.gamepad1.y && YPrimed){YPrimed = false;
            slow = !slow;
        }

        if(this.gamepad1.b){BPrimed = true;}if(!this.gamepad1.b && BPrimed){BPrimed = false;
            if(grabber.getPosition() < 0.5){
                grabber.setPosition(.7);
            }else{
                grabber.setPosition(0.2);
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
        if(整(this.gamepad1.x,xprime)) {
            double v = 0;
            while (left.getDistance(DistanceUnit.INCH) < 8.5 && right.getDistance(DistanceUnit.INCH) < 8.5) {
                v = (right.getDistance(DistanceUnit.INCH) - left.getDistance(DistanceUnit.INCH)) / 2;
                v = Math.min(Math.max(v, -0.2), 0.2);
                setAllDrivePower(-0.2 + v, -0.2 + v, .2 + v, .2 + v);

            }

        }

        if(slow){
            scaledMove(-this.gamepad1.left_stick_x/2,-this.gamepad1.left_stick_y/2, (this.gamepad1.left_bumper?0:-this.gamepad1.right_stick_x/2));
        }else{
            scaledMove(-this.gamepad1.left_stick_x,-this.gamepad1.left_stick_y, (this.gamepad1.left_bumper?0:-this.gamepad1.right_stick_x));
        }

        if(t == null){t = new ElapsedTime();}

        if(this.gamepad1.left_bumper){
            telemetry.addLine("CHANGING SLIDE");
            if(this.gamepad1.right_stick_y > 0){
                value += (slow? 0.5 : 1) * a_down * (joystick_quad(-this.gamepad1.right_stick_y)) * t.milliseconds();
            }else{
                value += a_up * (joystick_quad(-this.gamepad1.right_stick_y)) * t.milliseconds();
            }

            if(value > slideLimit)
                value = slideLimit;
            if(value < 0)
                value = 0;
        }
        t.reset();

        L1.setTargetPosition((int)value);
        L2.setTargetPosition(-(int)value);
        if(slow){telemetry.addLine("slide and drivetrain slowed");}
        telemetry.addData("target",value);
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
            telemetry.addData("vLF",to3d(speeds[0]));
            telemetry.addData("vLB",to3d(speeds[1]));
            telemetry.addData("vRF",to3d(speeds[2]));
            telemetry.addData("vRB",to3d(speeds[3]));
        }else{
            telemetry.addLine("SCALED power: max was "+absMax);
            telemetry.addLine("vLF: "+to3d(speeds[0])+" -> "+to3d(speeds[0]/absMax));
            telemetry.addLine("vLB: "+to3d(speeds[1])+" -> "+to3d(speeds[1]/absMax));
            telemetry.addLine("vRF: "+to3d(speeds[2])+" -> "+to3d(speeds[2]/absMax));
            telemetry.addLine("vRB: "+to3d(speeds[3])+" -> "+to3d(speeds[3]/absMax));
            setAllDrivePower(speeds[0]/absMax, speeds[1]/absMax, speeds[2]/absMax,speeds[3]/absMax);
        }
    }

    private double linear(double input, double minLimit, double maxLimit){
        if(input > -ctrl_deadzone && input < ctrl_deadzone){return 0;}

        double m = (maxLimit - minLimit)/(1-ctrl_deadzone);
        if(input < 0)
            return m * input - minLimit;
        return m * input + minLimit;
    }
}
