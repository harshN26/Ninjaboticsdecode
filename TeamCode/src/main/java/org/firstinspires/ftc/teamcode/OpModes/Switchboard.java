package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot_Hardware;

@TeleOp(name="Setup", group="1")
public class Switchboard extends LinearOpMode {

    Robot_Hardware robot=Robot_Hardware.getInstance();
    @Override
    public void runOpMode() {
        robot=Robot_Hardware.getInstance();

        if(gamepad1.a){
            robot.alliance = Robot_Hardware.AllianceColor.RED;

        }
        if(gamepad1.b){
            robot.alliance = Robot_Hardware.AllianceColor.BLUE;
        }
        telemetry.addLine("Alliance Color: "+ robot.alliance);
        telemetry.update();

    }
}
