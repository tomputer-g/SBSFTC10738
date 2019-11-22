package org.firstinspires.ftc.teamcode20;
import android.graphics.Path;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode19.Tests.WallVUMarkTesting;

public class TractionControl extends BaseAuto{
    //previous motor counts
    private static double pcLF,pcLB,pcRF,pcRB;

    //change in motor counts
    private static double deltaLF, deltaLB, deltaRF, deltaRB;

    //the min motor count(min) and min motor power(minp)
    private static double min,minp;

    //the abs value of powers(one time variable)
    private static double pLF,pLB,pRF,pRB;

    //the initial motor counts(one time variable) not used
    private static double icLF,icLB,icRF,icRB;

    private static double mult = 0.001;

    /*
    //the old TD power
    protected void setTDpower(double ipLF,double ipLB,double ipRF,double ipRB)
    {
        pLF=Math.abs(ipLF); pLB=Math.abs(ipLB); pRF=Math.abs(ipRF); pRB=Math.abs(ipRB);
        deltaLF = Math.abs(getMC(LF)-pcLF); deltaLB = Math.abs(getMC(LB)-pcLB); deltaRF = Math.abs(getMC(RF)-pcRF); deltaRB = Math.abs(getMC(RB)-pcRB);
        min=Math.min(deltaLF,Math.min(deltaLB,Math.min(deltaRF, deltaRB)));
        minp=Math.min(pLF,Math.min(pLB,Math.min(pRF,pRB)));
        setAllDrivePower(deltaLF /min>pLF/minp?0:ipLF, deltaLB /min>pLB/minp?0:ipLB, deltaRF /min>pRF/minp?0:ipRF, deltaRB /min>pRB/minp?0:ipRB);
        pcLF=getMC(LF); pcLB=getMC(LB); pcRF=getMC(RF); pcRB=getMC(RB);
    }
    */

    /*setPower method that detects slipping wheels by finding proportions of (abs)motorCount/(abs)minMotorCount > (abs)motorPower/(abs)minMotorPower
     * set slipping wheels to power 0 and other wheels to their original power
     * */
    protected void setTDpower(double ipLF,double ipLB,double ipRF,double ipRB){
        initTDdrive(ipLF, ipLB, ipRF, ipRB);
        absDeltaMCUpdate();
        setAllDrivePower(deltaLF /min>pLF/minp?0:ipLF, deltaLB /min>pLB/minp?0:ipLB, deltaRF /min>pRF/minp?0:ipRF, deltaRB /min>pRB/minp?0:ipRB);
    }

    protected void moveTD(double motorCount,double speed){
        reset();
        moveMCUpdate();
        while(!moveDone(motorCount)){
            setAllDrivePower1(speed,speed+getPID(pcLB),speed+getPID(pcRF),speed+getPID(pcRB));
            moveMCUpdate();
        }
        setAllDrivePower(0);
        brakeTD(1,1);
    }

    //get the Motor Count(Left Motors positive forward, Right Motors negative forward)
    protected static double getMC(DcMotor i){
        return i.getCurrentPosition();

    }
    /*
    protected void brakeTD(double brakespeed,double tolerance){
        pcLF=getMC(LF);pcLB=getMC(LB);pcRB=getMC(RB);pcRF=getMC(RF);
        iVLF=pcLF;iVLB=pcLB;iVRF=pcRF;iVRB=pcRB;
        wait(100);
        deltaLF = Math.abs(getMC(LF)-pcLF); deltaLB = Math.abs(getMC(LB)-pcLB); deltaRF = Math.abs(getMC(RF)-pcRF); deltaRB = Math.abs(getMC(RB)-pcRB);
        while(!checkMC(deltaLF,-1)||!checkMC(deltaLB,-1)||!checkMC(deltaRF,1)||!checkMC(deltaRB,1)){
                deltaLF = Math.abs(getMC(LF)-pcLF); deltaLB = Math.abs(getMC(LB)-pcLB); deltaRF = Math.abs(getMC(RF)-pcRF); deltaRB = Math.abs(getMC(RB)-pcRB);
                pcLF= deltaLF;pcLB= deltaLB;pcRB= deltaRB;pcRF= deltaRF;
                setAllDrivePower(posneg(deltaLF,tolerance)*brakespeed,posneg(deltaLB,tolerance)*brakespeed,posneg(deltaRF,tolerance)*brakespeed,posneg(deltaRB,tolerance)*brakespeed);
                pcLF=getMC(LF);pcLB=getMC(LB);pcRB=getMC(RB);pcRF=getMC(RF);
        }
        setAllDrivePower(0);
    }
    */
    protected void brakeTD(double brakespeed,double tolerance){
        reset();
        pLF=LF.getPower(); pLB=LF.getPower(); pRF=RF.getPower(); pRB=RB.getPower();
        while(!brakeDone(tolerance)){
            deltaMCUpdate();
            setAllDrivePower(check(pLF)*brakespeed,check(pLB)*brakespeed,check(pRF)*brakespeed,check(pRB)*brakespeed);
        }
        setAllDrivePower(0);

    }

    //initialize the traction control variables by resetting the previous motor count and new motor count
    private void reset(){
        pcLF=0;pcLB=0;pcRF=0;pcRB=0;
        deltaLF =getMC(LF); deltaLB =getMC(LB); deltaRB =getMC(RB); deltaRF =getMC(RF);
        icLF=0;icLB=0;icRF=0;icRB=0;
        pLF=0;pLB=0;pRF=0;pRB=0;
    }

    //update the motor counts(Negate the right motor encoder count changes to positive forward) for the brake
    private void deltaMCUpdate(){
        deltaLF =getMC(LF)-pcLF; deltaLB =getMC(LB)-pcLB; deltaRB =pcRB-getMC(RB); deltaRF =pcLF-getMC(RF);
        pcLF=getMC(LF);pcLB=getMC(LB);pcRB=getMC(RB);pcRF=getMC(RF);
    }

    //for the TD drive
    private void absDeltaMCUpdate(){
        deltaLF = Math.abs(getMC(LF)-pcLF); deltaLB = Math.abs(getMC(LB)-pcLB); deltaRF = Math.abs(pcLF-getMC(RF)); deltaRB = Math.abs(pcRB-getMC(RB));
        pcLF=getMC(LF);pcLB=getMC(LB);pcRB=getMC(RB);pcRF=getMC(RF);
        min=Math.min(deltaLF,Math.min(deltaLB,Math.min(deltaRF, deltaRB)));
    }

    //update for the moveTD
    private void moveMCUpdate(){
        pcLF = icLF-getMC(LF); pcLB =icLB-getMC(LB); pcRF = getMC(RF)-icRF; pcRB = getMC(RB)-icRB;
    }

    //check if the delta motor counts are at expected values
    private boolean brakeDone(double tolerance){
        return (near(deltaLB,0,tolerance) && near(deltaLF,0,tolerance) && near(deltaLB,0,tolerance) && near(deltaLF,0,tolerance));
    }

    private boolean moveDone(double target,double tolerance){
        return near(pcLF,target,tolerance)&&near(pcLB,target,tolerance)&&near(pcRF,target,tolerance)&&near(pcRB,target,tolerance);
    }

    private boolean moveDone(double target){
        return pcLF>=target&&pcLB>=target&&pcRB>=target&&pcRF>=target;
    }

    //called beginning each setTDpower to initialize the power ad
    private void initTDdrive(double ipLF,double ipLB,double ipRF,double ipRB){
        pLF=Math.abs(ipLF); pLB=Math.abs(ipLB); pRF=Math.abs(ipRF); pRB=Math.abs(ipRB);
        minp=Math.min(pLF,Math.min(pLB,Math.min(pRF,pRB)));
        reset();
    }

    private double check(double power){
        if(power>0)
            return -1;
        else if(power<0)
            return 1;
        return 0;
    }

    //
    private double getSpeed(double i){
        if((i-pcLF)>0) return -0.1;
        else if (i-pcLF==0) return 0;
        return 0.1;
    }

    //get the pid adjustment value
    private double getPID(double i){
        return (pcLF-i)*mult;
    }

    //set the multiplier of pid move
    protected void setTDMult(double i){mult=i;}
}
