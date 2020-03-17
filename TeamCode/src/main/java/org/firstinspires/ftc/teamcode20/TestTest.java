package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
public class TestTest extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        double time =0;
        initIMU();
        initDrivetrain();
        initOdometry();
        initHubs();
        waitForStart();
        ElapsedTime t = new ElapsedTime();
        moveInchesGOXT(15,0.9,1,1000);
        align(0);
        int x = getXOdometry();

        moveInchesGOY_XF_F(97,0.9,1,x);
        align(0);
        moveInchesGOY_XF_F(-24-97,0.6,1,x);
        align(0);
        moveInchesGOY_XF_F(24+97,0.9,1,x);
        align(0);
        moveInchesGOY_XF_F(-97,0.6,1,x);
        align(0);
        moveInchesGOY_XF_F(97-16,0.9,1,x);
        align(0);
        moveInchesGOY_XF_F(-97-8,0.6,1,x);
        align(0);
        moveInchesGOY_XF_F(97+8,0.9,1,x);
        align(0);
        moveInchesGOY_XF_F(-97-8-8,0.6,1,x);
        align(0);
        moveInchesGOY_XF_F(97+8+8,0.9,1,x);
        align(0);
        time=t.milliseconds();
        time-=10000;
        telemetry.addData("time: ",time);
        telemetry.update();
    }
}
