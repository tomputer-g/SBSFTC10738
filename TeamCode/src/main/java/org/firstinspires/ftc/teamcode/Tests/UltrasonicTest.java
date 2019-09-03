package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode18.Tests.Wire;
//Wire and ArrayQueue libraries: credit to FTC5866! (http://olliesworkshops.blogspot.com/2015/10/i2cxl-maxsonar-with-wire-library.html)
//anything below 20cm is 20cm

/**
 * Created by Ziming Gao on 12/19/2017.
 */
@Disabled
@Deprecated
@TeleOp(name = "MB1242 ultra test")
public class UltrasonicTest extends OpMode {
    private Wire ultra1;
    private int distance1;
    private long pingTime;
    private int offset1 = 3;
    @Override
    public void init() {
        ultra1 = new Wire(hardwareMap,"MB1242",0x66);
    }

    @Override
    public void start() {
        super.start();
        ultra1.beginWrite(0x51);
        ultra1.write(0);
        ultra1.endWrite();

        pingTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        if(this.gamepad1.b){
            while(this.gamepad1.b);
            ultra1.requestFrom(0,3);
            ultra1.beginWrite(0xAA);
            ultra1.write(0xA5);
            ultra1.write(0x60);//new address here (even numbers only, except 0x00, 0x50, 0xA4, 0xAA)
            ultra1.endWrite();
            telemetry.addLine("Address changed");
            telemetry.update();
        }//change address
        if(this.gamepad1.a){
            while(this.gamepad1.a);
            ultra1.requestFrom(0,3);
            ultra1.beginWrite(0xAA);
            ultra1.write(0xA5);
            ultra1.write(0x66);//new address here (even numbers only, except 0x00, 0x50, 0xA4, 0xAA)
            ultra1.endWrite();
            telemetry.addLine("Address changed");
            telemetry.update();
        }
        if(this.gamepad1.y){
            while(this.gamepad1.y);
            ultra1.requestFrom(0,3);
            ultra1.beginWrite(0xAA);
            ultra1.write(0xA5);
            ultra1.write(0x68);//new address here (even numbers only, except 0x00, 0x50, 0xA4, 0xAA)
            ultra1.endWrite();
            telemetry.addLine("Address changed");
            telemetry.update();
        }

        if(System.currentTimeMillis() - pingTime > 250) {
            pingTime = System.currentTimeMillis();
            while (!(ultra1.responseCount() > 0)){telemetry.addLine("Waiting for resp...");
                telemetry.update();}
            ultra1.getResponse();
            if (ultra1.isRead()) {
                distance1 = ultra1.readHL();
                if (distance1 < 760) {
                    telemetry.addData("cm 1", distance1);
                    telemetry.addData("calibrated 1 cm", distance1 + offset1);
                    telemetry.addData("time", System.currentTimeMillis() - pingTime);
                    telemetry.update();
                }
            }
        }


            /***}else if(on2){
                pingTime = System.currentTimeMillis();
                pingUS(ultra2);
                while(!(ultra2.responseCount() > 0));
                ultra2.getResponse();
                if(ultra2.isRead()){
                    distance2 = ultra2.readHL();
                    if(distance2 < 760){
                        readCount2++;
                        telemetry.addData("Count 2", readCount2);
                        telemetry.addData("cm 2", distance2);
                        telemetry.addData("calibrated 2 cm", distance2 + offset2);
                        telemetry.addData("time", System.currentTimeMillis() - pingTime);
                        telemetry.update();
                    }
                }
            }
        }**/
    }

    @Override
    public void stop() {
        ultra1.close();
        super.stop();
    }
    private void pingUS(Wire ultra){
        ultra.requestFrom(0,2);
        ultra.beginWrite(0x51);
        ultra.write(0);
        ultra.endWrite();
    }
}
