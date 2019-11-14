package org.firstinspires.ftc.teamcode20;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TractionControl extends BaseAuto{
    private static double pcLF=0,pcLB=0,pcRF=0,pcRB=0,cLF,cLB,cRF,cRB,min,minp,pLF,pLB,pRF,pRB,iVLF,iVLB,iVRF,iVRB;
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
        pcLF=getMC(LF);pcLB=getMC(LB);pcRB=getMC(RB);pcRF=getMC(RF);
        iVLF=pcLF;iVLB=pcLB;iVRF=pcRF;iVRB=pcRB;
        wait(100);
        cLF = Math.abs(getMC(LF)-pcLF); cLB = Math.abs(getMC(LB)-pcLB); cRF = Math.abs(getMC(RF)-pcRF); cRB = Math.abs(getMC(RB)-pcRB);
        while(!checkMC(cLF,-1)||!checkMC(cLB,-1)||!checkMC(cRF,1)||!checkMC(cRB,1)){
                cLF = Math.abs(getMC(LF)-pcLF); cLB = Math.abs(getMC(LB)-pcLB); cRF = Math.abs(getMC(RF)-pcRF); cRB = Math.abs(getMC(RB)-pcRB);
                pcLF=cLF;pcLB=cLB;pcRB=cRB;pcRF=cRF;
                setAllDrivePower(posneg(cLF,tolerance)*brakespeed,posneg(cLB,tolerance)*brakespeed,posneg(cRF,tolerance)*brakespeed,posneg(cRB,tolerance)*brakespeed);
                pcLF=getMC(LF);pcLB=getMC(LB);pcRB=getMC(RB);pcRF=getMC(RF);
        }
        setAllDrivePower(0);
    }
    protected void brakeTDS(double a, double tolerance){
        reset();

    }

    protected void brakeTD2(double brakespeed, double tolerance){
        pcLF=getMC(LF);pcLB=getMC(LB);pcRB=getMC(RB);pcRF=getMC(RF); wait(200);
        while(!near(cLF,pcLF,tolerance)||!near(cLB,pcLB,tolerance)||!near(cRF,pcRF,tolerance)||!near(cRB,pcRB,tolerance)){
            setAllDrivePower(posneg(cLF,pcLF)*brakespeed,posneg(cLB,pcLB)*brakespeed,posneg(cRF,cRB)*brakespeed,posneg(cLB,pcLB)*brakespeed);
        }
        setAllDrivePower(0);
    }

    private double posneg(double a,double tolerance){
        if(a>tolerance) return -1;
        else if(a<-tolerance) return 1;
        else return 0;
    }
    private boolean checkMC(double deltaMC,double initialV){
        if(initialV>0)
            return deltaMC<-1;
        else if (deltaMC<0)
            return deltaMC>1;
        return true;
    }
//initialize the traction control variables by resetting the previous motor count and new motor count
    private void reset(){
        pcLF=0;pcLB=0;pcRF=0;pcRB=0;
        cLF=getMC(LF);cLB=getMC(LB);cRB=getMC(RB);cRF=getMC(RF);
    }
    //initialize the traction control variables by resetting the previous motor count and new motor count
    private void deltaMCUpdate(){
        pcLF=0;pcLB=0;pcRF=0;pcRB=0;
        cLF=getMC(LF);cLB=getMC(LB);cRB=getMC(RB);cRF=getMC(RF);
    }

}
