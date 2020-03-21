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
        initPlatformGrabber();
        ElapsedTime t = new ElapsedTime();
        double value =0.0;
        double elbowpos = 0.0;
        double clawpos = 0.0;
        while(!this.gamepad1.b){
            if(zheng(this.gamepad1.x,x)){
                platformGrab();
                lefty();
            }
        }
        //setAllDrivePower(-.3,-.3,.3,.3);
    }
}
