package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

@TeleOp(name="Switchboard", group="1")
public class Switchboard extends LinearOpMode {

    Robot_Hardware robot=Robot_Hardware.getInstance();
    @Override
    public void runOpMode() {
        robot=Robot_Hardware.getInstance();
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                robot.alliance = Robot_Hardware.AllianceColor.RED;
            }
            if (gamepad1.b) {
                robot.alliance = Robot_Hardware.AllianceColor.BLUE;
            }
            if(gamepad1.dpad_left){
                robot.pose= Constants.blueBackStartingPose;
                telemetry.addLine("blue back");
            }
            if(gamepad1.dpad_up){
                robot.pose= Constants.blueGoalStartingPose;
                telemetry.addLine("blue goal");
            }
            if(gamepad1.dpad_down){
                robot.pose= Constants.redGoalStartingPose;
                telemetry.addLine("red goal");
            }
            if(gamepad1.dpad_right){
                robot.pose= Constants.redBackStartingPose;
                telemetry.addLine("red back");
            }
            telemetry.addLine("Alliance Color: " + robot.alliance);
            telemetry.update();
        }

    }
}
