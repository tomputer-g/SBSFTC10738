package org.firstinspires.ftc.teamcode20;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.openftc.revextensions2.ExpansionHubEx;

import java.nio.ByteBuffer;

@Autonomous
public class BlueAuto extends BaseAuto {
    int pos = 0; int result;
    @Override
    public void init() {
        initAutonomous();
    }

    @Override
    public void init_loop(){
        VuforiaLocalizer.CloseableFrame frame = null;
        Image image = null;
        result = 0;
        int red_L = 0, red_R = 0, blue_L=0,blue_R=0,green_L=0,green_R=0, red_M = 0,blue_M=0,green_M=0;
        int curpixel_L, curpixel_R, curpixel_M;
        boolean l = true, r=true, m = true;

        try {frame = this.vuforia.getFrameQueue().take(); }
        catch (Exception e)
        {throw new RuntimeException(e);}

        for(int i = 0;i<frame.getNumImages();++i){
            if(frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565){
                image = frame.getImage(i);
                break;
            }
        }

        if(image!=null){
            ByteBuffer pixels = image.getPixels();
            byte[] pixelArray = new byte[pixels.remaining()];
            pixels.get(pixelArray, 0, pixelArray.length);
            //1280x720
            int imageWidth = image.getWidth();
            int imageHeight = image.getHeight();
            pixels.rewind();
            Bitmap bm = Bitmap.createBitmap(imageWidth,imageHeight,Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(pixels);
            for(int i = -10;i<10;++i){
                for(int j = -10;j<10;++j){
                    curpixel_L = bm.getPixel(1147+i,180+j);//"left"
                    curpixel_M = bm.getPixel(927+i,180+j);//"mid"
                    curpixel_R = bm.getPixel(662+i,207+j);//"right"
                    red_L+= Color.red(curpixel_L);
                    blue_L+=Color.blue(curpixel_L);
                    green_L+=Color.green(curpixel_L);
                    red_M+=Color.red(curpixel_M);
                    blue_M+=Color.blue(curpixel_M);
                    green_M+=Color.green(curpixel_M);
                    red_R+=Color.red(curpixel_R);
                    blue_R+=Color.blue(curpixel_R);
                    green_R+=Color.green(curpixel_R);
                }
            }
            red_L/=400;red_R/=400;red_M/=400;blue_M/=400;blue_L/=400;blue_R/=400;green_M/=400;green_L/=400;green_R/=400;

            telemetry.addData("rgb@L" ,"red: %d blue: %d green: %d",red_L, blue_L, green_L);
            telemetry.addData("rgb@M" ,"red: %d blue: %d green: %d",red_M, blue_M, green_M);
            telemetry.addData("rgb@R" ,"red: %d blue: %d green: %d",red_R, blue_R, green_R);
            /*
            int a = bm.getPixel(960,180);
            int b = bm.getPixel(640,180);
            int reda = Color.red(a), bluea = Color.blue(a), greena = Color.green(a);
            int redb = Color.red(b), blueb = Color.blue(b), greenb = Color.green(b);

            telemetry.clear();
            telemetry.addData("rgb@960x180" ,"red: %d blue: %d green: %d",reda, bluea, greena);
            telemetry.addData("rgb@400x180" ,"red: %d blue: %d green: %d",redb, blueb, greenb);

            telemetry.log().add("rgb@1270x710 red: %d blue: %d green: %d",reda, bluea, greena);
            telemetry.log().add("rgb@10x10 red: %d blue: %d green: %d",redb, blueb, greenb);
            telemetry.log().add("rgb@10x710 red: %d blue: %d green: %d",redc, bluec, greenc);
            telemetry.log().add("rgb@1270x10 red: %d blue: %d green: %d",redd, blued, greend);
            telemetry.update();
            */
        }

        if(red_L>100&&green_L>90) l = false;
        if(red_R>100&&green_R>90) r = false;
        if(red_M>100&&green_M>90) m = false;

        if(l)result=0;
        if(m)result=1;
        if(r)result=2;
        telemetry.addData("position: ", result);

        telemetry.update();
        frame.close();
        wait(200);
    }

    @Override
    public void loop() {
        pos = result;
        //go forward 1 floor mat (24")w
        //vuforia - recognize block & move to pick up
        //after pickup: turn 90 deg. move to platform, drop off
        //move to platform, drag into position, release
        //repeat until run out of time; first on other skystones

        //initialization
        //shutdownVuforia();
        servoThread.setTarget(0.95);
        platform_grabber.setPower(1);
        platform_grabber.setPower(0.0);
        if(showTelemetry)telemetry.clear();
        grabber.setPosition(grabber_open);
        //wait(500);
        //shift to align to skystone
        int shift;
        if(pos == 1){
            shift = 0;
        }
        else if (pos == 0){
            moveInchesGOXT(-8,0.8,1,1200);
            shift=-8;
        }
        else {
            moveInchesGOXT(8,0.8,1,1200);
            shift=8;
        }

        //move forward to the skystone
        ElapsedTime p = new ElapsedTime();
        moveInchesGOY(30.5,0.6,(1+(13.65-hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS))/13.65));
        //grab 1st block
        grabber.setPosition(grabber_closed);
        wait(300);
        servoThread.setTarget(0.85);
        //setAllDrivePower(0.0);
        moveInchesG(0,-6,0.4);

        //move forward & approach foundation
        //turn(90, 0.5,1);
        PIDturnfast(90,false);
        setNewGyro(90);
        p.reset();
        resetXOdometry();
        moveInchesGOY((85.25+shift),0.9);
        p.reset();
        //while (p.milliseconds()<900)setAllDrivePowerG(-.5,.5,-.5,.5);


        moveInchesGOXT(13.5-getXOdometry()/odometryEncXPerInch,.5,1,1300); //drag +errordistance

        platform_grabber.setPower(-1);
        wait(300);
        moveInchesGOX_platform(-19,0.8,1+(13.65-hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS))/13.65);
        int steps = 20;
        double basespeed = 0.3;
        for(int i = 10;i<=steps;++i){
            RF.setPower(  i*basespeed/steps);
            LB.setPower(2*i*basespeed/steps);
            LF.setPower(3*i*basespeed/steps);
            wait(20);
            //LB.setPower(0);
        }
        double curAng = getHeading();
        while (curAng<70){
            curAng = getHeading();
        }
        while (curAng<88){
            curAng = getHeading();
            RF.setPower(RF.getPower()*getError(90,curAng)/20);
            LB.setPower(LB.getPower()*getError(90,curAng)/20);
            LF.setPower(LF.getPower()*getError(90,curAng)/20);

        }
        setNewGyro(180);
        setAllDrivePower(0);
        after_dragged_foundation_B();

        setNewGyro(90);
        moveInchesGOXT(-6,0.5,1,1000);
        /*
        p.reset();
        while (p.milliseconds()<1100)setAllDrivePowerG(.7,.7,-.7,-.7);
        p.reset();
        while (p.milliseconds()<600)setAllDrivePowerG(.2,.2,-.2,-.2);

         */
        int shiftt = 0;
        if(pos == 1 || pos == 2) shiftt = -8;
        moveInchesGOY(-96.5+shiftt,0.6);
        servoThread.setTarget(0.95);
        PIDturnfast(-90,false);
        setNewGyro(0);
        /*
        p.reset();
        while (p.milliseconds()<1500)setAllDrivePowerG(-.5,.5,-.5,.5);

        int shiftt = -4;
        if(pos==0)shiftt=-12;
        moveInchesGOXT(shiftt,0.8,1,1200);


         */
        //setAllDrivePowerG(-.5,-.5,.5,.5);
        //wait(300);
        align(0);
        if(pos==1||pos==2)moveInchesGOXT(4,0.8,1,1000);
        moveInchesGOY(10,0.3);
        //moveInchesGOY((right.getDistance(DistanceUnit.INCH)-2.6)*.69,.4);
        grabber.setPosition(grabber_closed);
        wait(300);
        setAllDrivePower(0);
        servoThread.setTarget(0.85);
        //setAllDrivePower(0.0);
        moveInchesG(0,-9.5,0.4);
        PIDturnfast(90,false);
        setNewGyro(90);
        int sfi = 0;
        if(pos==0)sfi = -9;
        moveInchesGOY(77+sfi,0.9);
        grabber.setPosition(grabber_open);
        moveInchesG(0,-13,0.5);

        requestOpModeStop();
        //telemetry.addData()
        /*


        //turn foundation
        platform_grabber.setPower(-.8);
        wait(200);
        setAllDrivePowerG(0.55,-0.55,0.55,-0.55);
        wait(1000);
        setAllDrivePower(0);
        turn(90, 0.8, 4);
        //drag foundation forward
        setNewGyro(180);

        //double koe=0.75;
        //p.reset();
        //while(10<rangeSensorFront.getDistance(DistanceUnit.INCH) || p.milliseconds() < 3400){
        //    setAllDrivePowerG(koe*(0.25-0.55+0.37),koe*(0.25-0.55-0.37),koe*(0.25+0.55+0.37),koe*(0.22+0.5-0.37)); //turn+f0rwrd+side
        //}

        setAllDrivePower(0);
        double tempY = getY1Odometry();
        double targetdist = getY1Odometry()-15*1316;
        p = new ElapsedTime();
        while(getY1Odometry()>targetdist&&p.milliseconds()<2000)
            setAllDrivePowerG(-0.5,-0.5,0.5,0.5,2);
        setAllDrivePower(0);

        //push it in
        setAllDrivePowerG(-.7,.7,-.7,.7,2);
        wait(1000);

        //release grabber
        platform_grabber.setPower(1);
        wait(300);
        setAllDrivePower(0);
        platform_grabber.setPower(0.0);

        //strafe left to put the block

        servoThread.setTarget(0.5);
        moveInchesG(-6.5,0,0.5);
        turn(-90,0.4,1);
        setNewGyro(90);
        //holdSlide((int)slideEncoderPerInch/10);
        //wait(1000);
        //moveInchesG(0,4,0.43);
        setAllDrivePowerG(-0.4,-0.4,0.4,0.4);
        //wait(200);
        servoThread.setTarget(.8);
        wait(1300);
        setAllDrivePower(0.0);

        grabber.setPosition(grabber_open);
        platform_grabber.setPower(1);
        wait(300);
        //park
        moveInchesG(0,-38,0.3);

        setAllDrivePower(0.0);
        servoThread.stopThread();
        setNewGyro0();
        platform_grabber.setPower(0);

        */
    }
}