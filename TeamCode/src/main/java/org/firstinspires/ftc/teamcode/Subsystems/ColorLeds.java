package org.firstinspires.ftc.teamcode.Subsystems;

import android.sax.StartElementListener;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

public class ColorLeds extends SubsystemBase {

    Robot_Hardware robot;
    Telemetry telem;

    public enum LED {BALLS3,INTAKING,DISPLAYCOLOR, OFF}
    public enum ARTIFACT_COLOR{GREEN,PURPLE,NONE}

    LED state = LED.OFF;

    NormalizedRGBA in,out;
    double inDist=0,outDist=0;

    ARTIFACT_COLOR cIN=ARTIFACT_COLOR.NONE,cOUT=ARTIFACT_COLOR.NONE;





    public ColorLeds(Robot_Hardware hardware, Telemetry telemetry) {
        robot = hardware;
        telem = telemetry;
    }

    public void update(LED newState) {
        state = newState;
    }

    public void lightColor(double colorValue){
        if(robot.led.getPosition()!=colorValue){
            robot.led.setPosition(colorValue);
        }
    }

    public void loop() {
        dataCollection();
        cIN=processColors(in,1);
        cOUT=processColors(out,2);
        telem.addLine("raw in: "+in.red+","+in.green+","+in.blue+","+in.alpha);
        telem.addLine("raw out: "+out.red+","+out.green+","+out.blue+","+out.alpha);
        LEDProcessing();
        // black=0.0
        // red=0.277
        // orange=0.333
        // yellow=0.388
        // sage=0.444
        // green=0.5
        // azure=0.555
        // blue=0.611
        // indigo=0.666
        // violet=0.722
        // white=1.0
        switch (state) {
            case BALLS3:
                lightColor(0.28); //red
                break;
            case INTAKING:
                lightColor(0.388); //yellow
                break;
            case DISPLAYCOLOR:
                if(cIN==ARTIFACT_COLOR.GREEN){
                    lightColor(0.5); //green
                }else if(cIN==ARTIFACT_COLOR.PURPLE){
                    lightColor(0.722); //violet
                }
                else if(cOUT==ARTIFACT_COLOR.GREEN){
                    lightColor(0.5); //green
                }
                else if(cOUT==ARTIFACT_COLOR.PURPLE){
                    lightColor(0.722);//violet
                }
                else{
                    update(LED.INTAKING);
                }
                break;
            case OFF:
                lightColor(0.0); //off
                break;
            default:
                lightColor(1.0); //white

        }
        telem();
    }


    public void dataCollection(){
        in=robot.colorin.getNormalizedColors();
        out=robot.colorout.getNormalizedColors();
    }
    public ARTIFACT_COLOR processColors(NormalizedRGBA rgba, int number){
        double red= rgba.red/rgba.alpha;
        double green= rgba.green/rgba.alpha;
        double blue= rgba.blue/rgba.alpha;
        telem.addLine("C"+number+": "+red+", "+green+", "+blue);
        telem.addData("C"+number+" ratios: ",+green/red+", "+blue/green);
        if((green/red)> Constants.CGreen&&green>blue){
            return ARTIFACT_COLOR.GREEN;
        } else if ((blue/green)>Constants.CPurple&&blue>red) {
            return ARTIFACT_COLOR.PURPLE;
        }
        return ARTIFACT_COLOR.NONE;
    }

    public void LEDProcessing(){
        if(cIN!=ARTIFACT_COLOR.NONE&&cOUT!=ARTIFACT_COLOR.NONE){
            update(LED.BALLS3);
        }else if(cIN!=ARTIFACT_COLOR.NONE||cOUT!=ARTIFACT_COLOR.NONE){
            update(LED.DISPLAYCOLOR);
        }else if(Intake.state== Intake.INTAKE_STATE.IN){
            update(LED.INTAKING);
        }else{
            update(LED.OFF);
        }
    }


    public void telem(){
        telem.addLine("all 3: "+ (state==LED.BALLS3));

    }
}
