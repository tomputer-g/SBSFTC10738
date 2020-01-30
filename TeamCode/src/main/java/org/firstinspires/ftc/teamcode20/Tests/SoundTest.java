package org.firstinspires.ftc.teamcode20.Tests;

import android.media.MediaPlayer;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode20.BaseAuto;

@TeleOp
@Disabled

public class SoundTest extends BaseAuto {
    private int soundID = hardwareMap.appContext.getResources().getIdentifier("sound1", "raw", hardwareMap.appContext.getPackageName());
    private boolean[] bf = {true};
    @Override
    public void init(){
    }
    @Override
    public void loop(){
        if(zheng(this.gamepad1.right_bumper,bf))
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);
    }
}
