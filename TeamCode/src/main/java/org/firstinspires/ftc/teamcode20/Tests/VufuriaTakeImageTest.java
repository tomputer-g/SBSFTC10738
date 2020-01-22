package org.firstinspires.ftc.teamcode20.Tests;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode20.BaseAuto;
@TeleOp
public class VufuriaTakeImageTest extends BaseAuto {
    protected VuforiaLocalizer.CloseableFrame frame;
    protected Image image;
    protected Bitmap bm;
    public void init(){
        initVuforia();
        vuforia.setFrameQueueCapacity(6);
        telemetry.log().add("Frame Capacity = %d",vuforia.getFrameQueueCapacity());
        telemetry.update();
        frame = null;
    }

    @Override
    public void loop(){
       Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565,true);
        try
        {
            frame = this.vuforia.getFrameQueue().take();
        }
        catch (InterruptedException e)
        {
            throw new RuntimeException(e);
        }
        telemetry.log().add("Number of images",frame.getNumImages());
        image = frame.getImage(0);
        telemetry.log().add("Picture type = %d %d x %d",image.getFormat(),image.getWidth(),image.getHeight());
        telemetry.log().add("Num Bytes = %d",image.getPixels().remaining());
        telemetry.update();

        bm = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(image.getPixels());
        
        //wait(5000);
    }

    @Override
    public void stop(){
        frame.close();
    }

}
