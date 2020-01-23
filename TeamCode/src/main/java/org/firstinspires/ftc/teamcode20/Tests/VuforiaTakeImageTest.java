package org.firstinspires.ftc.teamcode20.Tests;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode20.BaseAuto;

import java.lang.reflect.Array;
import java.nio.ByteBuffer;
import java.util.Arrays;

/*
   created by Lucien Liu on Jan 22 2020
 */

@TeleOp
public class VuforiaTakeImageTest extends BaseAuto {
    protected ElapsedTime p = new ElapsedTime();
    public void init(){
        initVuforia();
    }

    @Override
    public void loop(){
        p.reset();
        VuforiaLocalizer.CloseableFrame frame = null;
        Image image = null;
        int result = 0;
        int red_L = 0, red_R = 0, blue_L=0,blue_R=0,green_L=0,green_R=0, red_M = 0,blue_M=0,green_M=0;
        int curpixel_L, curpixel_R, curpixel_M;
        boolean l = true, r=true, m = true;

        try {frame = this.vuforia.getFrameQueue().take(); }
        catch (InterruptedException e)
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
                    curpixel_L = bm.getPixel(1030+i,320+j);//"left"
                    curpixel_M = bm.getPixel(780+i,320+j);//"mid"
                    curpixel_R = bm.getPixel(530+i,320+j);//"right"
                    red_L+=Color.red(curpixel_L);
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

        if(red_L>100&&blue_L<30&&green_L>90) l = false;
        if(red_R>100&&blue_R<30&&green_R>90) r = false;
        if(red_M>100&&blue_M<30&&green_M>90) m = false;

        if(l)result=0;
        if(m)result=1;
        if(r)result=2;
        telemetry.addData("position: ", result);
        telemetry.addData("time in ms: ", p.milliseconds());

        telemetry.update();
        frame.close();
        wait(500);
        //requestOpModeStop();
    }

}
