package org.firstinspires.ftc.teamcode;
//@Autonomous(name = "SplineTest")
public class Odometry extends BaseOpMode{
    private Vector p,v1;
    private double theta;
    @Override
    public void init(){
        p=new Vector(0,0);
    }
    @Override
    public void loop(){
        
        p.add(new Vector(a*Math.cos(theta),a*Math.sin(theta)));
        p.add(new Vector(b*Math.cos(theta+Math.Pi/2),b*Math.sin(theta+Math.Pi/2)));
    }
}