package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class TeleOp_RocketLeague extends BaseOpMode {

    private double v = 0, x = 0;
    private double limit = 0.5;
    private boolean isStrafeCtrl = false;
    private final double ctrl_deadzone = 0.2;

    private boolean yPrimed = false, dpadLPrimed = false, dpadRPrimed = false, BPrimed = false;
    @Override
    public void init() {
        initDrivetrain();
        initGrabber();
        initGrabber();
        grabber.setPosition(1);
    }

    @Override
    public void loop() {
        //y = switch movement, b = servo toggle, U/D = grab motor,
        if(this.gamepad1.y){
            yPrimed = true;
        }
        if(!this.gamepad1.y && yPrimed){
            yPrimed = false;
            isStrafeCtrl = !isStrafeCtrl;//toggle control scheme
        }

        if(this.gamepad1.b){
            BPrimed = true;
        }
        if(!this.gamepad1.b && BPrimed){
            BPrimed = false;
            if(grabber.getPosition() < 0.5){
                grabber.setPosition(1);
            }else{
                grabber.setPosition(0.2);
            }
        }

        if(this.gamepad1.dpad_up){
            grabber_motor.setPower(-1);
        }else if(this.gamepad1.dpad_down){
            grabber_motor.setPower(1);
        }else{
            grabber_motor.setPower(0);
        }


        if(isStrafeCtrl){
            telemetry.addLine("STRAFE mode");
            scaledMove(-this.gamepad1.left_stick_x,-this.gamepad1.left_stick_y, -this.gamepad1.right_stick_x);

        }else {
            telemetry.addLine("TANK mode");
            if (this.gamepad1.dpad_left) {
                dpadLPrimed = true;
            }
            if (!this.gamepad1.dpad_left && dpadLPrimed) {
                dpadLPrimed = false;
                limit -= 0.01;
                if (limit < 0) {
                    limit = 0;
                }
            }

            if (this.gamepad1.dpad_right) {
                dpadRPrimed = true;
            }
            if (!this.gamepad1.dpad_right && dpadRPrimed) {
                dpadRPrimed = false;
                limit += 0.01;
                if (limit > 1) {
                    limit = 1;
                }
            }
            telemetry.addData("limit", limit);
            if (this.gamepad1.right_trigger > 0.1) {//if triggers don't show, are gamepads set to XBOX?
                v = 0.5;
            }else if (this.gamepad1.left_trigger > 0.1){
                v = -0.5;
            }else{
                v = 0;
            }

            if(this.gamepad1.x)
                v *= 2;

            x = this.gamepad1.left_stick_x;

            telemetry.addLine("speed: "+to3d(v)+", turn: "+to3d(x));
            if(v > 0){
                if(x > 0){//right
                    if(v+x > 1){
                        if(x > limit) {
                            winstonSetPower(1, 1 - limit, 1 - limit, 1 - 2 * limit);
                            telemetry.addLine("V Comp. limit");
                        }else {
                            winstonSetPower(1, 1 - x, 1 - x, 1 - 2 * x);
                            telemetry.addLine("Compromising V...");
                        }
                    }else{
                        winstonSetPower(v+x, v, v, v-x);
                    }
                }else if(x < 0) {
                    if (v - x > 1) {
                        if(x < -limit){
                            winstonSetPower(1-limit, 1-2*limit, 1, 1-limit);
                            telemetry.addLine("V Comp. limit");
                        }else{
                            winstonSetPower(1+x, 1+2*x, 1, 1+x);
                            telemetry.addLine("Compromising V...");
                        }
                    } else {
                        winstonSetPower(v, v + x, v - x, v);
                    }
                }else{//x = 0 (straight line)
                    winstonSetPower(v,v,v,v);
                }

            }else if(v < 0){
                x = -x;
                if(x > 0){//right
                    if(v-x < -1){
                        if(x > limit){
                            winstonSetPower(-1+limit, -1, -1+2*limit, -1+limit);
                            telemetry.addLine("V Comp. limit");
                        }else{
                            winstonSetPower(-1+x, -1, -1+2*x, -1+x);
                            telemetry.addLine("Compromising V...");
                        }
                    }else{
                        winstonSetPower(v, v-x, v+x, v);
                    }

                }else if(x < 0) {
                    if (v+x < -1) {
                        if(x < -limit){
                            winstonSetPower(-1+2*limit, -1+limit, -1+limit, -1);
                            telemetry.addLine("V Comp. limit");
                        }else{
                            winstonSetPower(-1-2*x, -1-x, -1-x, -1);
                            telemetry.addLine("Compromising V...");
                        }
                    } else {
                        winstonSetPower(v-x, v, v, v+x);
                    }
                }else{//x = 0 (straight line)
                    winstonSetPower(v,v,v,v);
                }
            }else{//v = 0 (turn only)
                winstonSetPower(x,x,-x,-x);
            }
        }
        telemetry.update();

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
