package org.firstinspires.ftc.teamcode20;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp

public class NoSlipDrive extends BaseAuto {
    ElapsedTime t;
    final double ymult = 232.7551/12;
    final double odomult = 4096/Math.PI;
    final double maxodc = 1000;
    double mili = 0;
    int dist=72;
    int phase =0;
    //button press booleans
    boolean[] rb={true},lb={true},xx={true},yy={true};

    //previous motor counts
    int lfmc=0,lbmc=0,rfmc=0,rbmc=0;

    double speed;

    //delta odometry count, previoud odometry count
    double odc=0; int omc=0;

    @Override
    public void init() {
        initDrivetrain();
        initIMU();
        initOdometry();
        wait(200);
        speed=.5;
    }
    @Override
    public  void init_loop(){
        if(zheng(this.gamepad1.y,xx)){speed+=.1;}
        if(zheng(this.gamepad1.a,yy)){speed-=.1;}
        if(zheng(this.gamepad1.left_bumper,lb)){dist+=2;}
        if(zheng(this.gamepad1.right_bumper,rb)){dist-=2;}
        telemetry.addData("speed: ",speed);
        telemetry.addData("dist: ",dist);
        telemetry.update();
    }
    @Override
    public void loop() {
        if(zheng(this.gamepad1.left_bumper,lb)){//odobrake();
            phase=1;
        }
        //telemetry.addLine(LF.getCurrentPosition()+" "+RF.getCurrentPosition()+" "+LB.getCurrentPosition()+" "+RB.getCurrentPosition()+" "+L2.getCurrentPosition());
        if(phase==1){
            //setNewGyro0();
            //reset_ENCODER();
            double target1 = -1313*Math.max((1-speed/2)*dist,dist*0.78);
            while(platform_grabber.getCurrentPosition()>target1)
                setAllDrivePowerG(-speed,-speed,speed,speed);
            double target2 = -1313*(dist-1);
            while(platform_grabber.getCurrentPosition()>target2)
                setAllDrivePowerG(0,0,0,0);
            odobrake();
            phase=0;
        }
        updateOC();
        telemetry.addData("odc",odc);
        telemetry.addData("omc",omc);
    }

    private void updateOC(){
            odc = (double) (-platform_grabber.getCurrentPosition()-omc) / odomult;
            omc = -platform_grabber.getCurrentPosition();
    }

    protected void odobrake() {
        setAllDrivePowerG(0);
        setAllDrivePowerG(1, 1, -1, -1);
        while(odc>0){
            updateOC();
        }
        for (int i = 0; i < 10; i++) {
            double koe = (1 + (10 - i) / 15)*1.2;
            setAllDrivePowerG(speed / koe / 2.3, speed / koe / 2.3, -speed / 2.3 / koe - 0.1, -speed / koe / 2 - 0.1);
            wait(8);
            setAllDrivePowerG(0.0);
        }
        //turn(-getHeading()+angle,.5,2);
    }


}
