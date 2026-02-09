package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
        cIN=processColors(in);
        cOUT=processColors(out);
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
                lightColor(0.277);
                break;
            case INTAKING:
                lightColor(0.388);
                break;
            case DISPLAYCOLOR:
                if(cIN==ARTIFACT_COLOR.GREEN){
                    lightColor(0.5);
                }else if(cIN==ARTIFACT_COLOR.PURPLE){
                    lightColor(0.722);
                }
                else if(cOUT==ARTIFACT_COLOR.GREEN){
                    lightColor(0.5);
                }
                else if(cOUT==ARTIFACT_COLOR.PURPLE){
                    lightColor(0.722);
                }
                else{
                    update(LED.INTAKING);
                    lightColor(0.388);
                }




                break;
            case OFF:
                lightColor(0.0);
                break;
            default:
                lightColor(1.0);

        }
        telem();
    }


    public void dataCollection(){
        in=robot.colorin.getNormalizedColors();
        out=robot.colorout.getNormalizedColors();
        inDist= robot.colorin.getDistance(DistanceUnit.MM);
        outDist= robot.colorin.getDistance(DistanceUnit.MM);
    }
    public ARTIFACT_COLOR processColors(NormalizedRGBA rgba){
        double red= (double)rgba.red/Math.round(rgba.alpha);
        double green= (double)rgba.green/Math.round(rgba.alpha);
        double blue= (double)rgba.blue/Math.round(rgba.alpha);

        if((green/red)>2.0&&green>blue){
            return ARTIFACT_COLOR.GREEN;
        } else if ((blue/green)>1.3&&blue>red) {
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
        telem.addLine("color in rgb: "+ in.red*10000 +", "+in.green*10000+", "+in.blue*10000);
        telem.addLine("color out rgb: "+ out.red*10000 +", "+out.green*10000+", "+out.blue*10000);
    }
}
