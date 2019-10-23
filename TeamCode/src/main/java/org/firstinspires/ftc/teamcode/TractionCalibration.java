package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
public class TractionCalibration extends BaseAuto{
    double pcLF=0,pcLB=0,pcRF=0,pcRB=0,cLF,cLB,cRF,cRB,min,minp;
    protected void setAllpowerTC(double pLF,double pLB,double pRF,double pRB)
    {
        cLF = getMC(LF)-pcLF; cLB = getMC(LB)-pcLB; cRF = getMC(RF)-pcRF; cRB = getMC(RB)-pcRB;
        min=Math.min(cLF,Math.min(cLB,Math.min(cRF,cRB)));
        minp=Math.min(pLF,Math.min(pLB,Math.min(pRF,pRB)));
        setAllDrivePower(cLF/min>pLF/minp?0:pLF, cLB/min>pLB/minp?0:pLB, cRF/min>pRF/minp?0:pRF, cRB/min>pRB/minp?0:pRB);
        pcLF=cLF; pcLB=cLB; pcRF=cRF; pcRB=cRB;
    }
    protected double getMC(DcMotor i){
        //stub should return motor count given a motor
        return 0;

    }
}
