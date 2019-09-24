package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode19.BaseOpMode;

//@Autonomous(name = "SplineTest")
public class AAA_AdvRobotics_SplineTest extends BaseOpMode {
    private int[][] testPoints = {{2,0},{4,2}};
    @Override
    public void init() {
        initDrivetrain();

    }

    @Override
    public void loop() {

        moveCubicSpline(new int[] {0,0}, testPoints);
        //movement: protected void setAllDrivePower(double pX, double pY), where pX and pY are speeds in cartesian coordinates relative to robot

        requestOpModeStop();//finish
    }

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


    }
    //TODO: actual spline method here
    /*
    Spline method requirements:
    PARAMS: array of 2D waypoints (we don't expect there to be more than 3 or 4, realistically)
    RETURN: void (we control the power of the drivetrain inside the method)

    * can be run from a loop (we will change msStuckDetectLoop accordingly if need be)
     */
}
