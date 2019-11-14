package org.firstinspires.ftc.teamcode20;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    private static double iVLF,iVLB,iVRF,iVRB;


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
    protected void brakeTD(double brakespeed, double target){
        reset();
        while(!brakeDone(target)){
            setAllDrivePower1(-brakespeed,-brakespeed,-brakespeed,-brakespeed);
            deltaMCUpdate();
        }
        setAllDrivePower(0);

    }

    //initialize the traction control variables by resetting the previous motor count and new motor count
    private void reset(){
        pcLF=0;pcLB=0;pcRF=0;pcRB=0;
        deltaLF =getMC(LF); deltaLB =getMC(LB); deltaRB =getMC(RB); deltaRF =getMC(RF);
        iVLF=0;iVLB=0;iVRF=0;iVRB=0;
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

    //check if the delta motor counts are at expected values
    private boolean brakeDone(double target){
        return (deltaLB<target && deltaLF<target && deltaLB<target && deltaLF<target);
    }

    //called beginning each setTDpower to initialize the power ad
    private void initTDdrive(double ipLF,double ipLB,double ipRF,double ipRB){
        pLF=Math.abs(ipLF); pLB=Math.abs(ipLB); pRF=Math.abs(ipRF); pRB=Math.abs(ipRB);
        minp=Math.min(pLF,Math.min(pLB,Math.min(pRF,pRB)));
        reset();
    }

}
