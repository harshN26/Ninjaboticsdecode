package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Global_Configs;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

public class ChamberSort {

    Robot_Hardware robot;
    Telemetry telem;

    public enum CHAMBER_STATE{IN,STOP,OUT, UP, LAST, IDLE}

    public CHAMBER_STATE state=CHAMBER_STATE.STOP;
    public ChamberSort(Robot_Hardware hardware, Telemetry telemetry){
        robot=hardware;
        telem=telemetry;
    }

    public void update(CHAMBER_STATE newState){
        state=newState;
    }

    public void setPowerAll(double pow){
        robot.sort1.setPower(pow);
    }

    public void loop(){
        if(Global_Configs.intakeStatus== Global_Configs.DOFStatus.ACTIVE){
            switch(state){
                case IN:setPowerAll(1.0);
                    robot.ramp.setPosition(Constants.rampDown);
                    robot.flickUp.setPosition(0.0);
                    break;
                case OUT:setPowerAll(-1.0);
                    robot.ramp.setPosition(Constants.rampUp);
                    robot.flickUp.setPosition(0.0);
                    break;
                case STOP:setPowerAll(0);
                    robot.ramp.setPosition(Constants.rampDown);
                    robot.flickUp.setPosition(0.0);
                    break;
                case IDLE:setPowerAll(0);
                    robot.ramp.setPosition(Constants.rampUp);
                    robot.flickUp.setPosition(0.0);
                    break;
                case UP:
                    setPowerAll(1.0);
                    robot.ramp.setPosition(Constants.rampUp);
                    robot.flickUp.setPosition(0.0);
                    break;
                case LAST:
                    setPowerAll(1.0);
                    robot.ramp.setPosition(Constants.rampUp);
                    robot.flickUp.setPosition(1.0);
                    break;
            }
        }else{
            setPowerAll(1.0);
            robot.ramp.setPosition(Constants.rampUp);
            robot.flickUp.setPosition(0.0);
        }


        telem();
    }

    public void telem(){
//        telem.addLine("chamber state: "+ state);
    }
}
