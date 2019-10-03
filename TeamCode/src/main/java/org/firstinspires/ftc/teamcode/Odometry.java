package org.firstinspires.ftc.teamcode;
//@Autonomous(name = "SplineTest")
public class Odometry extends BaseOpMode{
    private Vector2 p,v1;
    private double theta,a,b;
    @Override
    public void init(){
        p=new Vector2(0,0);
    }
    @Override
    public void loop(){
        
        p.add(new Vector2(a*Math.cos(theta),a*Math.sin(theta)));
        p.add(new Vector2(b*Math.cos(theta+Math.PI/2),b*Math.sin(theta+Math.PI/2)));
    }
}