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
    int pos = 0;
    @Override
    public void init() {
        initAutonomous();
        initViewMarks();
	}
    @Override
    public void init_loop(){
        pos = new_skystoneposition();
        wait(200);
    }

    @Override
    public void loop() {
        
    }
}
