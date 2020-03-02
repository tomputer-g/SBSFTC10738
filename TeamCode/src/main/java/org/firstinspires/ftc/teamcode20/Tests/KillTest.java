package org.firstinspires.ftc.teamcode20.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode20.BaseAuto;

import static java.lang.Thread.dumpStack;

@TeleOp
public class KillTest extends BaseAuto {
    @Override
    public void runOpMode() {
        waitForStart();
        new TimeoutThread().start();
        /*main:
        {
            initAutonomous();
            //we cannot passively check for stop and break out of loop by another Thread's call. That would be really bad in real life.
        }

         */

        //Log.w("KillTest","After task");
    }

    class TimeoutThread extends Thread{
        @Override
        public void run() {
            while(!check5s());
        }

        boolean check5s(){ //true = killed
            if(time > 5.0){
                kill("5s timeout");
                return true;
            }
            return false;
        }
    }


}
