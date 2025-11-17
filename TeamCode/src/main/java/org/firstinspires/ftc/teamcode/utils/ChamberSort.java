package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Global_Configs;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

public class ChamberSort {

    Robot_Hardware robot;
    Telemetry telem;

    public enum CHAMBER_STATE{IN,STOP,OUT, UP}

    public CHAMBER_STATE state=CHAMBER_STATE.STOP;
    public ChamberSort(Robot_Hardware hardware, Telemetry telemetry){
        robot=hardware;
        telem=telemetry;
    }

    public void update(CHAMBER_STATE newState){
        state=newState;
    }

    public void loop(){
        //TODO: make this use both CServo (the melonobotics servos)
        if(Global_Configs.intakeStatus== Global_Configs.DOFStatus.ACTIVE){
            switch(state){
                case IN:robot.intake.setPower(1.0);
                    robot.ramp.setPosition(Constants.rampDown);
                    break;
                case OUT:robot.intake.setPower(-0.5);
                    robot.ramp.setPosition(Constants.rampUp);
                    break;
                case STOP:robot.intake.setPower(0);
                    robot.ramp.setPosition(Constants.rampDown);
                    break;
                case UP:
                    robot.ramp.setPosition(Constants.rampUp);
                    break;
            }
        }else{
            robot.intake.setPower(0);
        }
    }
}
