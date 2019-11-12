package org.firstinspires.ftc.teamcode20;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TractionControl extends BaseAuto{
    private static double pcLF=0,pcLB=0,pcRF=0,pcRB=0,cLF,cLB,cRF,cRB,min,minp,pLF,pLB,pRF,pRB;
    protected void setTDpower(double ipLF,double ipLB,double ipRF,double ipRB)
    {
        pLF=Math.abs(ipLF); pLB=Math.abs(ipLB); pRF=Math.abs(ipRF); pRB=Math.abs(ipRB);
        cLF = Math.abs(getMC(LF)-pcLF); cLB = Math.abs(getMC(LB)-pcLB); cRF = Math.abs(getMC(RF)-pcRF); cRB = Math.abs(getMC(RB)-pcRB);
        min=Math.min(cLF,Math.min(cLB,Math.min(cRF,cRB)));
        minp=Math.min(pLF,Math.min(pLB,Math.min(pRF,pRB)));
        setAllDrivePower(cLF/min>pLF/minp?0:ipLF, cLB/min>pLB/minp?0:ipLB, cRF/min>pRF/minp?0:ipRF, cRB/min>pRB/minp?0:ipRB);
        pcLF=getMC(LF); pcLB=getMC(LB); pcRF=getMC(RF); pcRB=getMC(RB);
    }
    protected static double getMC(DcMotor i){
        return i.getCurrentPosition();
    }
    protected void brakeTD(double brakespeed,double tolerance){
        pcLF=0;pcLB=0;pcRF=0;pcRB=0;
        while(!near(cLF,0,tolerance)||!near(cLB,0,tolerance)||!near(cRF,0,tolerance)||!near(cRB,0,tolerance)){
                cLF = Math.abs(getMC(LF)-pcLF); cLB = Math.abs(getMC(LB)-pcLB); cRF = Math.abs(getMC(RF)-pcRF); cRB = Math.abs(getMC(RB)-pcRB);
                setAllDrivePower(posneg(cLF)*brakespeed,posneg(cLB)*brakespeed,posneg(cRF)*brakespeed,posneg(cRB)*brakespeed);
                pcLF=getMC(LF);pcLB=getMC(LB);pcRB=getMC(RB);pcRF=getMC(RF);
        }
        setAllDrivePower(0);
    }
    private double posneg(double a){
        if(a>0) return -1;
        else if(a==0) return 0;
        else return 1;
    }
}
