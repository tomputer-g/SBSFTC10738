package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Ziming Gao on 1/16/2018.
 */
@TeleOp
public class ServoTest extends OpMode{
    private boolean lP, rP, lb, rb;
    private Servo currentServo;
    private Servo LPlatform, RPlatform, Ext1, Ext2, XOdoEnable, LGClaw, LGElbow, Grabber, Capstone, RGClaw, RGElbow, Tape;
    private Servo[] servos = new Servo[12];
    private int selectIndex = 0;
    private double[] positions = new double[12];


    @Override public void init() {
        LPlatform = hardwareMap.get(Servo.class, "LPlatformGrabber");
        RPlatform = hardwareMap.get(Servo.class, "RPlatformGrabber");
        Ext1 = hardwareMap.get(Servo.class, "servo1");
        Ext2 = hardwareMap.get(Servo.class, "servo2");
        XOdoEnable = hardwareMap.get(Servo.class, "xOdoEnable");
        LGClaw = hardwareMap.get(Servo.class, "LGrabberClaw");
        LGElbow = hardwareMap.get(Servo.class, "LGrabberElbow");
        RGClaw = hardwareMap.get(Servo.class, "RGrabberClaw");
        RGElbow = hardwareMap.get(Servo.class, "RGrabberElbow");
        Grabber = hardwareMap.get(Servo.class, "grabber");
        Capstone = hardwareMap.get(Servo.class, "capstone");
        Tape = hardwareMap.get(Servo.class, "tapeMeasure");
        servos[0] = LPlatform;
        servos[1] = RPlatform;
        servos[2] = Ext1;
        servos[3] = Ext2;
        servos[4] = XOdoEnable;
        servos[5] = LGClaw;
        servos[6] = LGElbow;
        servos[7] = RGClaw;
        servos[8] = RGElbow;
        servos[9] = Grabber;
        servos[10] = Capstone;
        servos[11] = Tape;
        for(int i = 0;i < 12; i++){
            positions[i] = 0.5;
        }
        for(Servo servo : servos){
            servo.setPosition(0.5);
        }
        currentServo = servos[selectIndex];
    }

    @Override public void loop() {
        if(this.gamepad1.dpad_left){lP = true;}if(lP && !this.gamepad1.dpad_left){
            lP = false;
            positions[selectIndex] -= 0.05;
            if(positions[selectIndex] < 0){positions[selectIndex] = 0;}
            currentServo.setPosition(positions[selectIndex]);
            //servo.setPosition(Math.max(servo.getPosition() - 0.05, 0));
        }
        if(this.gamepad1.dpad_right){rP = true;}if(rP && !this.gamepad1.dpad_right) {
            rP = false;
            positions[selectIndex]+= 0.05;
            if(positions[selectIndex] > 1){positions[selectIndex] = 1;}
            currentServo.setPosition(positions[selectIndex]);
            //servo.setPosition(Math.min(servo.getPosition() + 0.05, 1));
        }
        if(this.gamepad1.left_bumper){lb = true;}if(!this.gamepad1.left_bumper && lb){
            lb = false;
            selectIndex--;
            if(selectIndex < 0){
                selectIndex = servos.length - 1;
            }
        }
        if(this.gamepad1.right_bumper){rb = true;}if(!this.gamepad1.right_bumper && rb){
            rb = false;
            selectIndex++;
            if(selectIndex >= servos.length){
                selectIndex = 0;
            }
        }
        //telemetry.addData("servo: ",servo.getPosition());
        currentServo = servos[selectIndex];
        telemetry.addData("Current servo pos",positions[selectIndex]);
        telemetry.addData("Servo #",selectIndex);
        telemetry.update();
    }
}
