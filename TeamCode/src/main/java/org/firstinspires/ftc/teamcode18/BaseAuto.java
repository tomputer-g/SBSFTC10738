package org.firstinspires.ftc.teamcode18;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Ziming Gao on 1/21/2018.
 */

public class BaseAuto extends org.firstinspires.ftc.teamcode18.BaseOpMode {
    protected final double ARM_HOME = 0.26, ARM_GOAL = 0.87;
    protected RelicRecoveryVuMark finalVuMark;
    protected VuforiaTrackable relicTemplate;//Vuforia vars


    protected ColorSensor cs;
    protected Servo CSArm;
    protected final double autonomousRotSpeed = 0.1;
    protected boolean autonomousDone = false;

    @Override
    public void init() {
        super.init();

        msStuckDetectLoop = 20000;
        VuforiaLocalizer vuforia;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);//leave Parameters brackets blank to disable camera view
        parameters.vuforiaLicenseKey = "ATRaSk7/////AAAAGYLgjlimQklSl+oDhmYIEjhzUV34Rljx8+M72Lzbu408S2XaUuMmL8Z0SRdMoKdoQ0dZ4/MeKas+GaC6AGw9GOFc4XyrUVlne2Cue3tTjC75ZTPbhh4odsJQVBlXkb88Ww38LX0oWeUnRS9b2GhGhCqPwhKA+HlZk6SCPSBqMVQg/T3TLKPSpouwA74gpdbw0wtdgp+X6K/1zUkSkp43hx7DATnoDEy467aFKlC/V/vgpVfxMbEVZbiHp8rSgmiVlEfPQuIGSq/pMWdmSNEor5LNY1SpV8BBwSp65OxB6ct9WdmOHJHxlhHdPhqpNRtJSdleNSCO4xAjmXuZ+8dkaU+kmTV/+x/4Po4yxuJVBGKo";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //noinspection deprecation
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
        //INIT VUFORIA

        CSArm = hardwareMap.get(Servo.class,"csarm");
        cs = hardwareMap.get(ColorSensor.class, "cs");
        cs.enableLed(true);

        grabber.setTargetPosition(90);
        CSArm.setPosition(ARM_HOME);

    }

    protected void moveForwardDistance(int d, double pwr){
        setMode_RESET_ENCODER();
        setMode_RUN_TO_POSITION();
        LF.setTargetPosition(-d);//grabber is front
        LB.setTargetPosition(-d);
        RF.setTargetPosition(d);
        RB.setTargetPosition(d);
        setAllDrivePower(pwr);
        while(LF.isBusy() && LB.isBusy() && RF.isBusy() && RB.isBusy());
        setAllDrivePower(0);
    }
    protected void moveLeftDistance(int d, double pwr){
        setMode_RESET_ENCODER();
        setMode_RUN_TO_POSITION();
        LF.setTargetPosition(-d);
        LB.setTargetPosition(d);
        RF.setTargetPosition(-d);
        RB.setTargetPosition(d);
        setAllDrivePower(pwr);
        while(LF.isBusy() && LB.isBusy() && RF.isBusy() && RB.isBusy());
        setAllDrivePower(0);
    }
    protected void moveRightDistance(int d, double pwr){
        setMode_RESET_ENCODER();
        setMode_RUN_TO_POSITION();

        setAllDrivePower(pwr);
        LF.setTargetPosition(d);
        LB.setTargetPosition(-d);
        RF.setTargetPosition(d);
        RB.setTargetPosition(-d);
        while(LF.isBusy() && LB.isBusy() && RF.isBusy() && RB.isBusy());
        setAllDrivePower(0);
    }

    protected String CSDetect() {
        return (cs.red() > cs.blue() ? "Red" : "Blue");
    }

    protected void doJewel(String team){//packaged code for entire jewel section
        CSArm.setPosition(ARM_GOAL-0.1);
        wait(800);
        CSArm.setPosition(ARM_GOAL);
        wait(500);
        if(CSDetect().equals(team)){
            turn(15);
            CSArm.setPosition(ARM_HOME);
        }
        else{
            turn(-15);
            CSArm.setPosition(ARM_HOME);
            wait(1000);
            turn(30);
        }
        wait(700);
    }


}
