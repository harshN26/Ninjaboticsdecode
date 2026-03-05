package org.firstinspires.ftc.teamcode.Subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Global_Configs;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

public class Kickstand extends SubsystemBase {

    Robot_Hardware robot;
    Telemetry telem;

    public enum KICKSTAND_STATE{RETRACTED,DEPLOYED};


    public KICKSTAND_STATE state=KICKSTAND_STATE.RETRACTED;

    public Kickstand(Robot_Hardware hardware, Telemetry telemetry){
        robot=hardware;
        telem=telemetry;
    }

    public void update(KICKSTAND_STATE newState){
        state=newState;
    }




    public void loop(){

        switch(state){
            case RETRACTED: robot.kickstand.setPosition(Constants.kickstandRetract);
                break;
            case DEPLOYED: robot.kickstand.setPosition(Constants.kickstandDeploy);
                break;
        }

        telem();
    }

    public void telem(){
//        telem.addLine("chamber state: "+ state);
    }
}
