package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public class OpenFTCExtensionTest extends OpMode {
    ExpansionHubEx hub2, hub4;
    ExpansionHubMotor LF, LB, RF, RB, L1, L2, xOdo, platform;
    LEDThread ledThread;
    RevBulkData bulkData2, bulkData4;

    double bulk100=0, control100=0;
    boolean YP;
    @Override
    public void init() {
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub4 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 4");
        LF = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "LF");
        LB = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "LB");
        RF = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "RF");
        RB = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "RB");
        L1 = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "L1");
        L2 = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "L2");
        xOdo = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "xOdo");
        platform = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, "platform");
        hub2.setPhoneChargeEnabled(true);
        ledThread = new LEDThread();
        ledThread.start();
    }

    @Override
    public void loop() {

        //Servo current draw is broken right now, apparently
        if(this.gamepad1.y){YP = true;}if(!this.gamepad1.y && YP){
            YP = false;
            ElapsedTime t = new ElapsedTime();
            for(int i = 0;i < 100; i++){
                RevBulkData dataTmp = hub2.getBulkInputData();
            }
            bulk100 = t.milliseconds();
            t.reset();
            for(int i = 0;i < 100; i++){
                LF.getCurrentPosition();
                LB.getCurrentPosition();
                RF.getCurrentPosition();
                RB.getCurrentPosition();
            }
            control100 = t.milliseconds();
        }
        /*telemetry.addData("Battery voltage",hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS));
        telemetry.addData("*Hub 2 total*",hub2.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) );
        /*telemetry.addData("LF",LF.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("LB",LB.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("RF",RF.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("RB",RB.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));


        telemetry.addLine("Phone charging is ON");
        telemetry.addData("*Hub 4 total*",hub4.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        /*telemetry.addData("L1",L1.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("L2",L2.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("xOdo",xOdo.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("platform",platform.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));

         */
        telemetry.addData("Bulk * 100",bulk100);
        telemetry.addData("Control * 100", control100);
        telemetry.update();
    }

    @Override
    public void stop() {
        ledThread.stopThread();
        super.stop();
    }

    class LEDThread extends Thread{
        int r = 255, g = 0, b = 0;
        int phase = 1;

        boolean stop = false;
        @Override
        public void run() {
            while(!stop && !isInterrupted()){
                switch(phase){
                    case 0://red++
                        r++;
                        b--;
                        if(r > 255|| b<0){
                            r = 255;
                            b = 0;
                            phase++;
                        }
                        break;
                    case 1://red--, green++
                        r--;
                        g++;
                        if(r < 0 || g > 255) {
                            r = 0;
                            g = 255;
                            phase++;
                        }
                        break;
                    case 2://green--, blue++
                        g--;
                        b++;
                        if(g < 0 || b > 255){
                            g = 0;
                            b = 255;
                            phase=0;
                        }
                        break;
                }
                hub2.setLedColor(r,g,b);
                hub4.setLedColor(r,g,b);
            }
        }

        public void stopThread(){
            stop = true;
        }
    }
}
