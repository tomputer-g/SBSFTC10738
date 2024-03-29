package org.firstinspires.ftc.teamcode20.Frank;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;

@Disabled

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

    //delta motor counts
    double lfdc,lbdc,rfdc,rbdc,speed;

    //delta odometry count, previoud odometry count
    double odc=0; int omc;
    double od2c=0; int om2c;

    @Override
    public void runOpMode() throws InterruptedException {
        initDrivetrain();
        initPlatformGrabber();
        initIMU();
        initOdometry();
        initSensors();
        platform_grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        platform_grabber.setPower(1);
        wait(150);
        platform_grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        platform_grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        platform_grabber.setPower(0);
        wait(500);
        t = new ElapsedTime();
        platform_grabber.setPower(1);
        wait(300);
        platform_grabber.setPower(0.0);
        speed=.5;
        while (!isStarted()){
            if(zheng(this.gamepad1.y,xx)){speed+=.1;}
            if(zheng(this.gamepad1.a,yy)){speed-=.1;}
            if(zheng(this.gamepad1.left_bumper,lb)){dist+=4;}
            if(zheng(this.gamepad1.right_bumper,rb)){dist-=1;}
            telemetry.addData("speed: ",speed);
            telemetry.addData("dist: ",dist);
            telemetry.update();
        }

        while (opModeIsActive()){
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
            telemetry.addLine("haha");
            //t.reset();
            //scaledMove(-this.gamepad1.left_stick_x * 0.3, -this.gamepad1.left_stick_y * 0.15, (this.gamepad1.left_bumper ? 0 : -this.gamepad1.right_stick_x * 0.2));
        /*3
        if(zheng(this.gamepad1.right_bumper,rb)){
            phase = 1;
        }
        telemetry.addLine("Odo: "+L2.getCurrentPosition());
        telemetry.addLine("LF: "+LF.getCurrentPosition());
        telemetry.addLine("LB: "+LB.getCurrentPosition());
        telemetry.addLine("RF: "+RF.getCurrentPosition());
        telemetry.addLine("RB: "+RB.getCurrentPosition());
        if(zheng(this.gamepad1.left_bumper,lb)){
            t = new ElapsedTime();
            for(int i=0;i<1000000;i++) {
                if(phase==1)noslippower(-0.5, -0.5, 0.5, 0.5);
                if(true){
                    updateMC();
                    updateOC();
                    telemetry.addLine("dLF: "+lfdc+" dRF: "+rfdc);
                    telemetry.addLine("odc: "+odc);
                    telemetry.update();
                }
            }
        }

        if(zheng(this.gamepad1.right_bumper,rb)) {
            platform_grabber.setPower(-.8);
            wait(200);
            turn(90, 0.60, 5);
            /*
            //winston attempt
            while (!near(getHeading(),90,3)) setAllDrivePower(-0.6,0.2,0.8,-0.4);


            //drag foundation
            setNewGyro(180);
            double koe = 1;
            while (13 < rangeSensorFront.getDistance(DistanceUnit.INCH)) {
                setAllDrivePowerG(koe * (0.25 - 0.55 + 0.37), koe * (0.25 - 0.55 - 0.37), koe * (0.25 + 0.55 + 0.37), koe * (0.22 + 0.5 - 0.37)); //turn+f0rwrd+side
                koe=Math.abs(koe-1)<0.2?0.7:1;
            }
            setAllDrivePower(0.0);

            //align to the right wall
            while (30 > rangeSensorFront.getDistance(DistanceUnit.INCH)) {
                //telemetry.addData("Side",rangeSensorSide.getDistance(DistanceUnit.INCH));
                //telemetry.update();
                setAllDrivePowerG(0.5, -0.5, 0.5, -0.5);
            }
            setAllDrivePower(0.0);

            //turn and drop the block
            platform_grabber.setPower(1);
            wait(300);
            platform_grabber.setPower(0.0);
        }
        */
        }
    }


    protected void noslippower(double lf,double lb, double rf, double rb){
        setAllDrivePower(lfdc>=odc?0.1:lf,lbdc>=odc?0.1:lb,rfdc>=odc?0.1:rf,rbdc>=odc?0.1:rb);
    }

    private void updateMC(){
        //update the delta MC
        lfdc=(double)(-lfmc+LF.getCurrentPosition())/ymult;lbdc=(double)(-lbmc+LB.getCurrentPosition())/ymult;rfdc=(double)(-RF.getCurrentPosition()+rfmc)/ymult;rbdc=(double)(-RB.getCurrentPosition()+rbmc)/ymult;
        //update the previous MC
        lfmc=LF.getCurrentPosition();lbmc=LB.getCurrentPosition();rfmc=RF.getCurrentPosition();rbmc=RB.getCurrentPosition();
    }

    private void updateOC(){
            odc = (double) (-platform_grabber.getCurrentPosition()-omc) / odomult;
            omc = -platform_grabber.getCurrentPosition();
            od2c = (double)(-platform_grabber.getCurrentPosition()-omc) / odomult;
            omc = -platform_grabber.getCurrentPosition();
    }

    protected void odobrake() throws InterruptedException {
        setAllDrivePowerG(0);
        setAllDrivePowerG(1, 1, -1, -1);
        /*
        while(odc>0){
            updateOC();
        }
        while(odc>0){
            updateOC();
        }

         */
        for (int i = 0; i < 10; i++) {
            double koe = (1 + (10 - i) / 15)*1.2;
            setAllDrivePowerG(speed / koe / 2, speed / koe / 2, -speed / 2 / koe - 0.1, -speed / koe / 2 - 0.1);
            Thread.sleep(8);
            setAllDrivePowerG(0.0);
        }
        //turn(-getHeading()+angle,.5,2);
    }

    protected void odobrakeduo() throws InterruptedException {
        updateOC();
        Thread.sleep(10);
        updateOC();
        double power = Math.abs(odc)/odc;
        if(power>0)
            while(odc>power){
                setAllDrivePowerG(power,power,-power,-power);
            }
        else
            while(odc<power){
                setAllDrivePowerG(power,power,-power,-power);
            }
        setAllDrivePower(0.0);
    }


}
