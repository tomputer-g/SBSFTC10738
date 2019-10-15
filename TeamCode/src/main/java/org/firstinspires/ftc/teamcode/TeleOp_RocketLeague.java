package org.firstinspires.ftc.teamcode;

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
        if(x > 0){//right
            if(v+x > 1){
                winstonSetPower(1, v-x, v-x, v-2*x);
                telemetry.addLine("Plan B pwr scaling");
            }else{
                winstonSetPower(v+x, v, v, v-x);

            }

        }else if(x < 0){//left
            if(v-x > 1){
                winstonSetPower(v+x, 1, v+2*x, v+x);
                telemetry.addLine("Plan B pwr scaling");
            }else{
                winstonSetPower(v, v-x, v+x, v);//apparently this is exactly the same resultant as above
            }

        }else{//straight
            winstonSetPower(v,v,v,v);
        }
        telemetry.update();

    }

    private void winstonSetPower(double LF, double LB, double RF, double RB){
        setAllDrivePower(-LF, -LB, RF, RB);
        telemetry.addLine("Powers: "+to3d(LF)+", "+to3d(LB)+", "+to3d(RF)+", "+to3d(RB));
    }
}
