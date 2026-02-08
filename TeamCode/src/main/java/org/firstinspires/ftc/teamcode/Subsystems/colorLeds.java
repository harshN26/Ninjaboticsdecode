package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Global_Configs;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

public class colorLeds extends SubsystemBase {

    Robot_Hardware robot;
    Telemetry telem;

    public enum LED {ON, OFF}

    LED state = LED.OFF;

    NormalizedRGBA in,out;
    double inDist=0,outDist=0;



    public colorLeds(Robot_Hardware hardware, Telemetry telemetry) {
        robot = hardware;
        telem = telemetry;
    }

    public void update(LED newState) {
        state = newState;
    }


    public void loop() {
        dataCollection();
        colorDetection();

        switch (state) {
            case ON:
                robot.led.setPosition(0.288);
                break;
            case OFF:
                robot.led.setPosition(0.0);
                break;

        }
        telem();
    }

    public void dataCollection(){
        in=robot.colorin.getNormalizedColors();
        out=robot.colorout.getNormalizedColors();
        inDist= robot.colorin.getDistance(DistanceUnit.MM);
        outDist= robot.colorin.getDistance(DistanceUnit.MM);
    }

    public void colorDetection(){




        if((in.green*10000>5&&in.blue*10000>5)&&(out.green*10000>9&&out.blue*10000>8)){
            update(LED.ON);
        }else {
            update(LED.OFF);
        }
    }


    public void telem(){
        telem.addLine("color on: "+ (state==LED.ON));
        telem.addLine("color in rgb: "+ in.red*10000 +", "+in.green*10000+", "+in.blue*10000);
        telem.addLine("color out rgb: "+ out.red*10000 +", "+out.green*10000+", "+out.blue*10000);
    }
}
