package org.firstinspires.ftc.teamcode.OpModes.Teles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Global_Configs;

@TeleOp
public class hoodTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo hood1;
        waitForStart();
        hood1=hardwareMap.get(Servo.class, Global_Configs.hood1Name);
        while (opModeIsActive()){
            if(gamepad1.a){
                hood1.setPosition(Constants.hoodMinPos);
            }
            if(gamepad1.y){
                hood1.setPosition(Constants.hoodMaxPos);
            }
        }
    }
}
