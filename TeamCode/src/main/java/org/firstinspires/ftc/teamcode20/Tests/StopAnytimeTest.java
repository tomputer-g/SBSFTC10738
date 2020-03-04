package org.firstinspires.ftc.teamcode20.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.firstinspires.ftc.teamcode20.BlueAuto;

@Autonomous
public class StopAnytimeTest extends BaseAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        new OpModeStopThread(Thread.currentThread()).start();
        main:{
            Log.i("StopAnytime","Start sleeping");
            try {
                while(true){Thread.sleep(20);}
            }catch (Exception e){
                Log.i("StopAnytime","Sleep Exception caught: ");
                e.printStackTrace();
            }
        }
        Log.i("StopAnytime", "End of method");
    }

    class OpModeStopThread extends Thread{
        private Thread parentRef;

        public OpModeStopThread(Thread parentRef) {
            super();
            this.parentRef = parentRef;
        }

        @Override
        public void run() {
            Log.i("StopAnytimeThread","Waiting");
            while(!isInterrupted() && time < 10 && !isStopRequested());
            if(isStopRequested())Log.i("StopAnytimeThread","Stopping because stop requested");
            if(time > 10)Log.i("StopAnytimeThread","Stopping because overtime");
            if(isInterrupted())Log.i("StopAnytimeThread","Stopping because interrupt");
            parentRef.interrupt();
            StopAnytimeTest.this.stop();
            Log.i("StopAnytimeThread","after stop call");
        }
    }
}
