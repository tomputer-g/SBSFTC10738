package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.text.DecimalFormat;

@TeleOp
public class TeleOp_RocketLeague extends BaseOpMode {

    private double v = 0, x = 0;
    @Override
    public void init() {
        super.init();
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

        x = this.gamepad1.left_stick_x;

        telemetry.addLine("V: "+to3d(v)+", X: "+to3d(x));
        if(v > 0){
            if(x > 0){//right
                if(v+x > 1){
                    winstonSetPower(1, 0.5, 0.5, 0);
                    telemetry.addLine("Plan B pwr scaling");
                }else{
                    winstonSetPower(v+x, v, v, v-x);
                }
            }else if(x < 0) {
                if (v - x > 1) {
                    winstonSetPower(0.5, 0, 1, 0.5);
                    telemetry.addLine("Plan B pwr scaling");
                } else {
                    winstonSetPower(v, v + x, v - x, v);
                }
            }else{//x = 0 (straight line)
                winstonSetPower(v,v,v,v);
            }

        }else if(v < 0){
            if(x > 0){//right
                if(v-x < -1){
                    winstonSetPower(-0.5, -1, 0, -0.5);
                    telemetry.addLine("Plan B pwr scaling");
                }else{
                    winstonSetPower(v, v-x, v+x, v);
                }

            }else if(x < 0) {
                if (v+x < -1) {
                    winstonSetPower(0, -0.5, -0.5, 1);
                    telemetry.addLine("Plan B pwr scaling");
                } else {
                    winstonSetPower(v-x, v, v, v+x);
                }
            }else{//x = 0 (straight line)
                winstonSetPower(v,v,v,v);
            }
        }else{//v = 0 (turn only)
            winstonSetPower(-x,-x,x,x);
        }
        telemetry.update();

    }

    private void winstonSetPower(double LF, double LB, double RF, double RB){
        setAllDrivePower(-LF, -LB, RF, RB);
        telemetry.addLine();
        telemetry.addLine(""+to3dstr(LF)+"  |  "+to3dstr(RF));
        telemetry.addLine("-----------------------");
        telemetry.addLine(""+to3dstr(LB)+"  |  "+to3dstr(RB));
    }

    protected String to3dstr(double d){
        DecimalFormat df = new DecimalFormat("##0.000");
        return df.format(d);
    }
}
