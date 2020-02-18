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
   created by Lucien Liu on Feb 18 2020
 */

@TeleOp
public class VuforiaTakeImageTestR extends BaseAuto {
    protected ElapsedTime p = new ElapsedTime();
    double i;

    @Override
    public void runOpMode() throws InterruptedException {
        initVuforia();
        waitForStart();
        while(opModeIsActive()){
            p.reset();
            new_skystonepositionR();
            i = p.milliseconds();
            wait(1500);
            telemetry.addData("time", i);
            telemetry.update();
            wait(500);
        }
    }

}
