package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Global_Configs;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

public class ServoClaw extends SubsystemBase {
    Robot_Hardware robot;

    double pos;

    public enum clawState{OPEN,CLOSED,RESET}
    public static clawState currClawState= clawState.CLOSED;

    clawState last_state= clawState.CLOSED;
    Telemetry telem;

    public ServoClaw(Robot_Hardware hardware, Telemetry telemetry){
        robot=hardware;
        pos=0.0;
        telem=telemetry;
    }

    public void update(clawState newState){
        currClawState=newState;
        state_updated();
    }
    private void state_updated(){
        if(Global_Configs.clawServoStatus== Global_Configs.DOFStatus.ACTIVE&&currClawState!=last_state){
            switch(currClawState){
                case OPEN:
                    robot.clawServo.setPosition(Globals.clawOpenPos);

                    break;

                case CLOSED:
                    robot.clawServo.setPosition(Globals.clawClosePos);
                    telem.addData("Claw state: ","Closed");
                    break;
                case RESET:
                    robot.clawServo.setPosition(Globals.clawOpenPos);

                    currClawState= clawState.OPEN;
                    break;
                default:
            }
            last_state=currClawState;
        }
    }


    public void loop(){
        if(Global_Configs.clawServoStatus== Global_Configs.DOFStatus.ACTIVE&&currClawState!=last_state){
            switch(currClawState){
                case OPEN:
                    telem.addData("Claw state: ","Open");
                    break;

                case CLOSED:
                    telem.addData("Claw state: ","Closed");
                    break;
                case RESET:
                    telem.addData("Claw state: ","RESET");
                    break;
                default:
            }
        }

    }
    public void tuning_loop(double change){
        if(Global_Configs.clawServoStatus== Global_Configs.DOFStatus.ACTIVE){
            telem.addData("Claw pos: ",pos);
            pos+=change;
            robot.clawServo.setPosition(pos);

        }
    }
}
