package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.firstinspires.ftc.teamcode20.BaseOpMode;
@TeleOp
public class AAA_AdvRobotics_SplineTest extends BaseAuto {
    private Vector2 end = new Vector2(-44,48);
    private final Vector2 start = new Vector2(-8.55,8.55);
    private boolean[] a = {true};
    private Thread u=new UD();
    @Override
    public void init() {
        coo.setX(8.55);coo.setY(8.55);
        initDrivetrain();
        initOdometry();
    }
    @Override
    public void start(){
        u.start();
    }
    @Override
    public void stop(){
        u.stopThread();
    }
    @Override
    public void loop() {

       // moveCubicSpline(new int[] {0,0}, testPoints);
        //movement: protected void setAllDrivePower(double pX, double pY), where pX and pY are speeds in cartesian coordinates relative to robot
    if(zheng(this.gamepad1.right_bumper,a))
        movetoCoo(end);
    }

    private void movetoCoo(Vector2 target){
        while(target.x<coo.x){
            setAllDrivePowerG(-0.35,0.35,-0.35,0.35);
        }
        while(target.y>coo.y){
            setAllDrivePowerG(-.2,-.2,.2,.2);
        }
        setAllDrivePower(0);
    }
    private class UD extends Thread{
        volatile public boolean stop = false;
        public void run(){
            while(!isInterrupted() && !stop){
                updateCoo();
            }
        }
        public void stopThread(){
            stop=true;
        }
    }
    /*
    private void moveCubicSpline(int[] currentPos, int[][] wayPoints){
        if(wayPoints[0].length != 2)
            throw new IllegalArgumentException("Points must be 2-dimensional");
        if(wayPoints.length < 2)
            return; //no waypoints to be run to

        boolean areWeThereYet = false;
        int[][] currentSplinePoints = {currentPos.clone(), wayPoints[0].clone(), wayPoints[1].clone()};
        int currentEndSplineIdx = 1; // idx of point IN WAYPOINTS
        while(currentEndSplineIdx < (wayPoints.length - 1)){//cycle through waypoints
            currentEndSplineIdx++;
            telemetry.addData("LF", LF.getCurrentPosition());
            telemetry.addData("LB", LB.getCurrentPosition());
            telemetry.addData("RF", RF.getCurrentPosition());
            telemetry.addData("RB", RB.getCurrentPosition());
            telemetry.update();
            //TODO: loop that runs with current spline curve

            currentSplinePoints = new int[][] {wayPoints[currentEndSplineIdx-2].clone(), wayPoints[currentEndSplineIdx-1].clone(), wayPoints[currentEndSplineIdx].clone()};

        }
        //TODO: loop that runs with current spline curve

     */

    }
    //TODO: actual spline method here
    /*
    Spline method requirements:
    PARAMS: array of 2D waypoints (we don't expect there to be more than 3 or 4, realistically)
    RETURN: void (we control the power of the drivetrain inside the method)

    * can be run from a loop (we will change msStuckDetectLoop accordingly if need be)
     */
