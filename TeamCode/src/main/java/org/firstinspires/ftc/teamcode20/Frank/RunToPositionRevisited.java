package org.firstinspires.ftc.teamcode20.Frank;

import android.util.Log;

import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.hardware.motors.Matrix12vMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode20.BaseOpMode;

public class RunToPositionRevisited extends BaseOpMode{

    protected DcMotor LF, LB, RF, RB;//all four are backwards?

    @Override
    public void runOpMode() throws InterruptedException {
        Log.d("RunToPositionRevisited","Log starts here");
        LF = hardwareMap.get(DcMotor.class, "LF");
        LF.setMotorType(MotorConfigurationType.getMotorType(GoBILDA5202Series.class));
        /*LB = hardwareMap.get(DcMotor.class, "LB");
        LB.setMotorType(MotorConfigurationType.getMotorType(GoBILDA5202Series.class));
        RF = hardwareMap.get(DcMotor.class, "RF");
        RF.setMotorType(MotorConfigurationType.getMotorType(GoBILDA5202Series.class));
        RB = hardwareMap.get(DcMotor.class, "RB");
        RB.setMotorType(MotorConfigurationType.getMotorType(GoBILDA5202Series.class));

         */

        //if(LB == null) {Log.d("RunToPositionRevisited", "LB is null on type");}
        //Log.d("RunToPositionRevisited","Name: "+LB.getMotorType().getName());//"GoBILDA 5202 series"
        //Log.d("RunToPositionRevisited","Desc: "+LB.getMotorType().getDescription());//""
        //Log.d("RunToPositionRevisited", "XML Tag: "+LB.getMotorType().getXmlTag());//"goBILDA5202SeriesMotor"

        waitForStart();

        LF.setTargetPosition(1000);
        LF.setPower(.1);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*LB.setTargetPosition(1000);
        LB.setPower(.1);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setTargetPosition(1000);
        RF.setPower(.1);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setTargetPosition(1000);
        RB.setPower(.1);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         */

        while(opModeIsActive()){
            telemetry.addData("LF enc", LF.getCurrentPosition());
            telemetry.addData("LF pwr", LF.getPower());
            telemetry.update();
            if(this.gamepad1.a){

            }
        }
    }
}
