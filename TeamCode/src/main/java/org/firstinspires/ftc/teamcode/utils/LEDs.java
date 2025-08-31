package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot_Hardware;

public class LEDs extends SubsystemBase {


    Robot_Hardware robot;
    RevBlinkinLedDriver.BlinkinPattern currPattern;

    public LEDs(Robot_Hardware hardware){
        robot=hardware;

    }

    public void loop(){
        setPattern();
    }

    public void update(RevBlinkinLedDriver.BlinkinPattern pattern){
        currPattern=pattern;
    }

    private void setPattern(){
        robot.leds.setPattern(currPattern);
    }

}
