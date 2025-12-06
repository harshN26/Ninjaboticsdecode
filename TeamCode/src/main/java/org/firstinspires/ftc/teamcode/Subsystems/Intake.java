package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Global_Configs;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

public class Intake {
    Robot_Hardware robot;
    Telemetry telem;

    public enum INTAKE_STATE{IN,STOP,OUT}

    public INTAKE_STATE state=INTAKE_STATE.STOP;
    public Intake(Robot_Hardware hardware, Telemetry telemetry){
        robot=hardware;
        telem=telemetry;
    }

    public void update(INTAKE_STATE newState){
        state=newState;
    }

    public void loop(){
        if(Global_Configs.intakeStatus== Global_Configs.DOFStatus.ACTIVE){
            switch(state){
                case IN:robot.intake.setPower(1.0);
                    break;
                case OUT:robot.intake.setPower(-0.5);
                    break;
                case STOP:robot.intake.setPower(0);
                    break;
            }
        }else{
            robot.intake.setPower(0);
        }
        telem();
    }
    public void telem() {

//        telem.addLine("intake state: " + state);
    }
}
