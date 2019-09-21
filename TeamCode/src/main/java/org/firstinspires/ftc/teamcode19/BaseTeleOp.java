package org.firstinspires.ftc.teamcode19;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BaseTeleOp extends BaseOpMode {
    protected static final double[] movement_power_params = {.1, .3, 1};
    protected static final double ctrl_deadzone = 0.2;
    protected static int teleopMinDumpEncoder = 3600;
    protected ElapsedTime t;
    protected static final int enc_slide_offset = -750; //extending is minus dir
    protected static boolean limitSWCalibrated = false;
    protected static int isMovingTo = -1; //-1 for no movement, 0/1/2 for goal locations


    @Override
    public void start() {
        super.start();
        t = new ElapsedTime();
    }

    protected double custom_quad(boolean fast, double input){
        if(input > -ctrl_deadzone && input < ctrl_deadzone){return 0;}
        double min = movement_power_params[0], max = movement_power_params[1];
        if(fast){
            min = movement_power_params[1];
            max = movement_power_params[2];
        }
        double b = min, m = max - min;
        if(input < 0)
            return -m * input * input - b;
        return m * input * input + b;
    }

    protected double custom_linear(boolean fast, double input){
        if(input > -ctrl_deadzone && input < ctrl_deadzone){return 0;}
        double min = movement_power_params[0], max = movement_power_params[1];
        if(fast){
            min = movement_power_params[1];
            max = movement_power_params[2];
        }
        double b = min, m = (max - min)/(1-ctrl_deadzone);
        if(input < 0)
            return m * input - b;
        return m * input + b;
    }
}
