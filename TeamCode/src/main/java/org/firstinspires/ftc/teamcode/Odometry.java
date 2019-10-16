package org.firstinspires.ftc.teamcode;
//@Autonomous(name = "SplineTest")
public class AAA_AdvRobotics_SplineTest extends BaseOpMode{
    private Vector2 p,v1;
    private double theta;
    @Override
    public void init(){p=new Vector2()};
    protected void pup_date(a,b){
        p.add(new Vector2(a*Math.cos(theta),a*Math.sin(theta)));
        p.add(new Vector2(b*Math.cos(theta+Math.Pi/2),b*Math.sin(theta+Math.Pi/2)));
    }
    @Override
    public void loop(){
        pup_date();
    }
}