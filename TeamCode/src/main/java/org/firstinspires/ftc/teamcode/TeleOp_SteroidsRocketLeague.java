package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


public class TeleOp_SteroidsRocketLeague extends BaseOpMode {

    private double v = 0, x = 0;
    @Override
    public void init() {
        initDrivetrain();
    }

    @Override
    public void loop() {



        if(this.gamepad1.right_trigger > 0.1)
            v = 0.5;
        else if(this.gamepad1.left_trigger > 0.1)
            v = -0.5;
        else
            v = 0;

        if(this.gamepad1.x){
            //hold = fast
            v *= 2;
        }

        x = this.gamepad1.right_stick_x;
        //v = -this.gamepad1.left_stick_y;//y stick is inverted default

        telemetry.addLine("speed: "+to3d(v)+", turn: "+to3d(x));
        if(v > 0){
            if(x > 0){//right
                if(v+x > 1){
                    winstonSetPower(1, 1-x, 1-x, 1-2*x);
                    telemetry.addLine("Compromising V...");
                }else{
                    winstonSetPower(v+x, v, v, v-x);
                }
            }else if(x < 0) {
                if (v - x > 1) {
                    winstonSetPower(1+x, 1+2*x, 1, 1+x);
                    telemetry.addLine("Compromising V...");
                } else {
                    winstonSetPower(v, v + x, v - x, v);
                }
            }else{//x = 0 (straight line)
                winstonSetPower(v,v,v,v);
            }

        }else if(v < 0){
            if(x > 0){//right
                if(v-x < -1){
                    winstonSetPower(-1+x, -1, -1+2*x, -1+x);
                    telemetry.addLine("Compromising V...");
                }else{
                    winstonSetPower(v, v-x, v+x, v);
                }

            }else if(x < 0) {
                if (v+x < -1) {
                    winstonSetPower(-1-2*x, -1-x, -1-x, -1);
                    telemetry.addLine("Compromising V...");
                } else {
                    winstonSetPower(v-x, v, v, v+x);
                }
            }else{//x = 0 (straight line)
                winstonSetPower(v,v,v,v);
            }
        }else{//v = 0 (turn only)
            winstonSetPower(x,x,-x,-x);
        }
        telemetry.update();

    }

    private void winstonSetPower(double LF, double LB, double RF, double RB){
        setAllDrivePower(-LF, -LB, RF, RB);//L motors reversed because it's winston power
        displayMotorPowers(LF, LB, RF, RB);
        telemetry.addLine("Result|----X----|----Y----|----R----");
        telemetry.addLine("           |"+to3dstr(RF-LF-RB+LB)+"|"+to3dstr(RF+RB+LF+LB)+"|"+to3dstr(RF+RB-LF-LB));
    }
}
