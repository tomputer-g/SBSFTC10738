package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Ziming Gao on 1/16/2018.
 */
@Disabled
@TeleOp(name = "Servo test", group = "test")
public class ServoTest extends OpMode {
    private Servo servo;
    private Servo servo2;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class,"servo");
        //servo2 = hardwareMap.get(Servo.class,"servo2");
        servo.setPosition(0.5);
        //servo2.setPosition(0.5);
    }

    @Override
    public void loop() {

        if(this.gamepad1.x){
            servo.setPosition(0.5);
            //servo2.setPosition(0.5);
        }

        else if(this.gamepad1.dpad_left){
            while(this.gamepad1.dpad_left) {
                if (servo.getPosition() >= 0.2)
                    servo.setPosition(servo.getPosition() - 0.2);
                if (servo.getPosition() > 0 && servo.getPosition() < 0.2)
                    servo.setPosition(0);
                /*if (servo2.getPosition() <= 0.95)
                    servo2.setPosition(servo2.getPosition() + 0.2);
                if (servo2.getPosition() > 0.95 && servo2.getPosition() < 1)
                    servo2.setPosition(1);*/
            }
        }
        else if(this.gamepad1.dpad_right){
            while(this.gamepad1.dpad_right) {
                if (servo.getPosition() <= 0.95)
                    servo.setPosition(servo.getPosition() + 0.2);
                if (servo.getPosition() > 0.95 && servo.getPosition() < 1)
                    servo.setPosition(1);
                /*if (servo2.getPosition() >= 0.2)
                    servo2.setPosition(servo2.getPosition() - 0.2);
                if (servo2.getPosition() > 0 && servo2.getPosition() < 0.2)
                    servo2.setPosition(0);*/
            }
        }
        telemetry.addData("servo: ",servo.getPosition());
        telemetry.update();

//        if(this.gamepad1.left_bumper){
//            while(this.gamepad1.left_bumper){
//                servo.setPosition(0);
//               // servo2.setPosition(1);
//            }
//
//        }else if(this.gamepad1.right_bumper){
//            while(this.gamepad1.right_bumper) {
//                servo.setPosition(1);
//                //servo2.setPosition(0);
//            }
//        }
//        if(this.gamepad1.x){
//            servo.setPosition(0.5);
//            //servo2.setPosition(0.5);
//        }
    }
}
