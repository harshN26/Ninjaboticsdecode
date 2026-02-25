package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

@TeleOp(name="Switchboard", group="1")
public class Switchboard extends OpMode {

    Robot_Hardware robot=Robot_Hardware.getInstance();
    public void init(){

    }
    @Override
    public void loop() {


            if (gamepad1.a) {
                robot.alliance = Robot_Hardware.AllianceColor.RED;
                robot.resetPoseClose=Constants.redResetPoseClose;
                robot.resetPoseFar=Constants.redResetPoseFar;
            }
            if (gamepad1.b) {
                robot.alliance = Robot_Hardware.AllianceColor.BLUE;
                robot.resetPoseClose=Constants.blueResetPoseClose;
                robot.resetPoseFar=Constants.blueResetPoseFar;
            }
            if(gamepad1.dpad_left){
                robot.pose= Constants.blueBackStartingPose;
                robot.startPose=Constants.blueBackStartingPose;
                telemetry.addLine("blue back");
            }
            if(gamepad1.dpad_up){
                robot.pose= Constants.blueGoalStartingPose;
                robot.startPose=Constants.blueGoalStartingPose;
                telemetry.addLine("blue goal");
            }
            if(gamepad1.dpad_down){
                robot.pose= Constants.redGoalStartingPose;
                robot.startPose=Constants.redGoalStartingPose;
                telemetry.addLine("red goal");
            }
            if(gamepad1.dpad_right){
                robot.pose= Constants.redBackStartingPose;
                robot.startPose=Constants.redBackStartingPose;
                telemetry.addLine("red back");
            }
            telemetry.addLine("Alliance Color: " + robot.alliance);
            telemetry.update();


    }

}
