package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
public class TestTest extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        double time = 0;
        boolean[] st={true},up={true},left={true},right={true},down={true},a={true},x={true};
        initIMU();
        initDrivetrain();
        initOdometry();
        initHubs();
        waitForStart();
        initSideGrabber();
        ElapsedTime t = new ElapsedTime();
        double elbowpos = 0.0;
        double clawpos = 0.0;
        while(!this.gamepad1.b){
            if(zheng(this.gamepad1.a,a)){
                RGrabClaw.setPosition(0.6);
                RGrabElbow.setPosition(0.53);
                Thread.sleep(2000);
                RGrabClaw.setPosition(0.0);
                Thread.sleep(500);
                RGrabElbow.setPosition(0);
            }

            telemetry.addData("RGrabElbow: ",elbowpos);
            telemetry.addData("RGrabClaw: ",clawpos);
            telemetry.update();
        }
        //setAllDrivePower(-.3,-.3,.3,.3);
    }
}
