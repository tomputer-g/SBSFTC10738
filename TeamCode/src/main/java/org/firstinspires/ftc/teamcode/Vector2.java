package org.firstinspires.ftc.teamcode;

public class Vector2 {
    double x,y;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public Vector2 add(Vector2 b){return new Vector2(this.x + b.x, this.y + b.y);}
    public Vector2 scalarMult(double a){return new Vector2(this.x * a, this.y*a);}
    public double dist(){
        return Math.sqrt(Math.sqrt(x*x+y*y)*Math.sqrt(x*x+y*y));
    }
    public double theta(){
        return x>=0?Math.atan(y/x):Math.PI+Math.atan(y/x);
    }
    public Vector2 toPolar(){
        return new Vector2(theta(),dist());
    }
}
