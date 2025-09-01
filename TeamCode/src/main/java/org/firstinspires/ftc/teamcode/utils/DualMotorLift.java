package org.firstinspires.ftc.teamcode.utils;


import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Global_Configs;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

@Config
public class DualMotorLift extends SubsystemBase {
    Robot_Hardware robot;


    public enum liftState{DOWN, TRANSFER,OUT, RESET, RESET_COMPLETE}
    public static liftState lift_state=liftState.DOWN;




    private PIDFController controller;

    private int kp=0,ki=1,kd=2,kf=3;
    public double kp_new=0, ki_new=0, kd_new=0,kf_new=0;
    double[] pidfCoeffs;//kp,ki,kd,kf

    private int currentPosition;
    private double targetPosition;
    private boolean inRange;

    private int tol;

    Telemetry telem;

    public DualMotorLift(Robot_Hardware robotHardware, double[] pidfCoeffs1 , Telemetry telemetry, int tolerance){
        robot=robotHardware;



        controller=new PIDFController(kp,ki,kd,kf);


        currentPosition = 0;
        targetPosition=0;



        pidfCoeffs=pidfCoeffs1;
        tol=tolerance;

        telem=telemetry;
    }


    public void set_target(double new_target){
        targetPosition=new_target;
    }

    public void setNewPIDFCoeffs(double[] pidfCoeffs1){
        pidfCoeffs=pidfCoeffs1;
        controller.setPIDF(pidfCoeffs[kp],pidfCoeffs[ki],pidfCoeffs[kd],pidfCoeffs[kf]);
    }

    public void set_tol(int tolerance){
        tol=tolerance;
    }

    public void updateCurrentPosition(){
        currentPosition=robot.liftM1.getCurrentPosition();
    }
    public void resetEncoder(){
        robot.liftM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getCurrentPosition(){
        return currentPosition;
    }

    public double getTarget(){
        return targetPosition;
    }

    public double[]  getPIDFCoeffs(){
        return pidfCoeffs;
    }
    public int get_tol(){
        return tol;
    }



    public boolean isInRange(){
        inRange=(currentPosition<=targetPosition+tol&&currentPosition>targetPosition-tol);
        return inRange;
    }





    public void update(liftState newState){
        lift_state=newState;
        state_updated();
    }

    private void state_updated(){
        //TODO: update
        inRange=false;
        switch(lift_state){
            case DOWN: set_target(Globals.liftDownPos);
                break;
            case TRANSFER:set_target(Globals.liftTransferPos);
                break;
            case OUT:set_target(Globals.liftOutPos);
                break;



            case RESET:set_target(0);
                break;
            case RESET_COMPLETE:
                resetEncoder();
                lift_state=liftState.DOWN;
                state_updated();
                break;
        }
    }





    public void loop(){
        updateCurrentPosition();
        double pow=controller.calculate(getCurrentPosition(),targetPosition);



        if(lift_state==liftState.RESET){
            pow=-0.1;
        }



        if(Global_Configs.liftM1Status== Global_Configs.DOFStatus.ACTIVE) {
            robot.liftM1.setPower(pow);
        }else{
            robot.liftM1.setPower(0);
        }
        if(Global_Configs.liftM2Status== Global_Configs.DOFStatus.ACTIVE)
            robot.liftM2.setPower(pow);
        else{
            robot.liftM2.setPower(0);
        }

        isInRange();
        telem();

    }

    public void telem(){
        isInRange();
        telem.addLine("lift target: "+ targetPosition);
        telem.addLine("lift position: "+ currentPosition);
        telem.addLine("lift in range: "+ currentPosition);
    }

    public void tuning_loop(){
        setNewPIDFCoeffs(new double[]{kp_new,ki_new,kd_new,kf_new});
        updateCurrentPosition();
        double pow=controller.calculate(getCurrentPosition(),targetPosition);

        if(Global_Configs.liftM1Status== Global_Configs.DOFStatus.ACTIVE) {
            robot.liftM1.setPower(pow);
        }else{
            robot.liftM1.setPower(0);
        }
        if(Global_Configs.liftM2Status== Global_Configs.DOFStatus.ACTIVE)
            robot.liftM2.setPower(pow);
        else{
            robot.liftM2.setPower(0);
        }

        isInRange();
        telem.addLine("lift target: "+ targetPosition);
        telem.addLine("lift position: "+ currentPosition);
        telem.addLine("lift in range: "+ currentPosition);

    }

}
