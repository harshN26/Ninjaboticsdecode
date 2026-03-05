package org.firstinspires.ftc.teamcode.Subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Global_Configs;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

public class ChamberSort extends SubsystemBase {

    Robot_Hardware robot;
    Telemetry telem;

    public enum CHAMBER_STATE{IN,STOP,OUT, UP, LAST, IDLE}

    public enum SORT_STATE{THROUGHPUT,SHIFTIN,HOLD}

    public CHAMBER_STATE state=CHAMBER_STATE.STOP;
    public SORT_STATE sort_state=SORT_STATE.THROUGHPUT;
    public ChamberSort(Robot_Hardware hardware, Telemetry telemetry){
        robot=hardware;
        telem=telemetry;
    }

    public void update(CHAMBER_STATE newState){
        state=newState;
    }

    public void updateSort(SORT_STATE newState){
        sort_state=newState;
    }

    public void setPowerAll(double pow){

        robot.sort1.setPower(pow);
    }

    public void loop(){
        if(Global_Configs.intakeStatus== Global_Configs.DOFStatus.ACTIVE){
            switch(state){
                case IN:setPowerAll(1.0);
                    robot.ramp.setPosition(Constants.rampDown);
                    robot.flickUp.setPosition(0.5);
                    break;
                case OUT:setPowerAll(-1.0);
                    robot.ramp.setPosition(Constants.rampUp);
                    robot.flickUp.setPosition(0.5);
                    break;
                case STOP:setPowerAll(0);
                    robot.ramp.setPosition(Constants.rampDown);
                    robot.flickUp.setPosition(0.5);
                    break;
                case IDLE:setPowerAll(0);
                    robot.ramp.setPosition(Constants.rampUp);
                    robot.flickUp.setPosition(0.5);
                    break;
                case UP:
                    setPowerAll(1.0);
                    robot.ramp.setPosition(Constants.rampUp);
                    robot.flickUp.setPosition(0.5);
                    break;
                case LAST:
                    setPowerAll(1.0);
                    robot.ramp.setPosition(Constants.rampUp);
                    robot.flickUp.setPosition(0.5);
                    break;
            }
        }else{
            setPowerAll(1.0);
            robot.ramp.setPosition(Constants.rampUp);
            robot.flickUp.setPosition(0.0);
        }



        switch(sort_state){
            case THROUGHPUT:
                robot.flickSide.setPosition(Constants.FlickSideClose);
                robot.push.setPosition(Constants.PushClose);
                break;
            case HOLD:
                robot.flickSide.setPosition(Constants.FlickSideOpen);
                robot.push.setPosition(Constants.PushClose);
                break;
            case SHIFTIN:
                robot.flickSide.setPosition(Constants.FlickSideOpen);
                robot.push.setPosition(Constants.PushOpen);
                break;
            default:
                sort_state=SORT_STATE.THROUGHPUT;
        }


        telem();
    }

    public void telem(){
//        telem.addLine("chamber state: "+ state);
    }
}
