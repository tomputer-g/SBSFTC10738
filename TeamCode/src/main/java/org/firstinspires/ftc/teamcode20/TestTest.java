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
        int x=getXOdometry();
        moveInchesGOY_XF_F(88,.9,1,x);
        align(0);
        moveInchesGOY_XF_F_T(-112,.6,1,x,2500);
        align(0);
        moveInchesGOY_XF_F(112,.9,1,x);
        align(0);
        moveInchesGOY_XF_F(-80,.6,1,x);
        align(0);
        moveInchesGOY_XF_F(72,.9,1,x);
        align(0);
        moveInchesGOY_XF_F(-88,.6,1,x);
        align(0);
        moveInchesGOY_XF_F(88,.9,1,x);
        align(0);
        moveInchesGOY_XF_F(-94,.6,1,x);
        align(0);
        moveInchesGOY_XF_F(94,.9,1,x);
        align(0);
        time=t.milliseconds();
        time-=10000;
        telemetry.addData("time: ",time);
        telemetry.update();
    }
}
