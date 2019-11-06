package org.firstinspires.ftc.teamcode20;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TractionDrive extends BaseAuto{
    private static double pcLF=0,pcLB=0,pcRF=0,pcRB=0,cLF,cLB,cRF,cRB,min,minp,pLF,pLB,pRF,pRB;
    public void setTDpower(double ipLF,double ipLB,double ipRF,double ipRB)
    {
        pLF=Math.abs(ipLF); pLB=Math.abs(ipLB); pRF=Math.abs(ipRF); pRB=Math.abs(ipRB);
        cLF = Math.abs(getMC(LF)-pcLF); cLB = Math.abs(getMC(LB)-pcLB); cRF = Math.abs(getMC(RF)-pcRF); cRB = Math.abs(getMC(RB)-pcRB);
        min=Math.min(cLF,Math.min(cLB,Math.min(cRF,cRB)));
        minp=Math.min(pLF,Math.min(pLB,Math.min(pRF,pRB)));
        setAllPDrivePower1(cLF/min>pLF/minp?0:pLF, cLB/min>pLB/minp?0:pLB, cRF/min>pRF/minp?0:pRF, cRB/min>pRB/minp?0:pRB);
        pcLF=cLF; pcLB=cLB; pcRF=cRF; pcRB=cRB;
    }
    public static double getMC(DcMotor i){
        return i.getCurrentPosition();
    }
    public void brakeTD(double brakespeed,double tolerance){
        pcLF=0;pcLB=0;pcRF=0;pcRB=0;
        while(!near(cLF,0,10)||!near(cLB,0,10)||!near(cRF,0,10)||!near(cRB,0,10)){
                cLF = Math.abs(getMC(LF)-pcLF); cLB = Math.abs(getMC(LB)-pcLB); cRF = Math.abs(getMC(RF)-pcRF); cRB = Math.abs(getMC(RB)-pcRB);
                setAllDrivePower(posneg(cLF)*brakespeed,posneg(cLB)*brakespeed,posneg(cRF)*brakespeed,posneg(cRB)*brakespeed);
        }
        setAllDrivePower(0);
    }
    private double posneg(double a){
        if(a>0) return -1;
        else if(a==0) return 0;
        else return 1;
    }
}
