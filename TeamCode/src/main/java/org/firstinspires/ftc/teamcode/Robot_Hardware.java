package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ResetCommandFinished;
import org.firstinspires.ftc.teamcode.Commands.multipartCommands.ResetCommandTrigger;
import org.firstinspires.ftc.teamcode.utils.DualMotorLift;
import org.firstinspires.ftc.teamcode.utils.LEDs;
import org.firstinspires.ftc.teamcode.utils.ServoClaw;

public class Robot_Hardware{
    HardwareMap hwMap;
    public DcMotor liftM1;
    public DcMotor liftM2;

    public Servo clawServo;
    public VoltageSensor voltageSensor;
    public double voltage;
    ElapsedTime voltageTimer;


    public RevBlinkinLedDriver leds;


    private static Robot_Hardware instance = null;

    public boolean enabled;

    public enum GameState{AUTO,TELE}
    public static GameState currGameState=GameState.AUTO;

    public static boolean inReset=false;


    public static Robot_Hardware getInstance() {
        if (instance == null) {
            instance = new Robot_Hardware();
        }
        instance.enabled = true;
        return instance;
    }
    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        hwMap=hardwareMap;
        voltageTimer=new ElapsedTime();

        //m1 MUST be ACTIVE
        liftM1=hardwareMap.get(DcMotor.class, Global_Configs.liftM1Name);
        liftM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(Global_Configs.liftM2Status== Global_Configs.DOFStatus.ACTIVE) {
            liftM2 = hardwareMap.get(DcMotor.class, Global_Configs.liftM2Name);
            liftM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftM2.setDirection(DcMotorSimple.Direction.REVERSE);
        }



        clawServo=hwMap.get(Servo.class,Global_Configs.clawServoName);



        voltageSensor=hardwareMap.voltageSensor.iterator().next();
        voltage = voltageSensor.getVoltage();

        leds=hardwareMap.get(RevBlinkinLedDriver.class, Global_Configs.ledsName);

    }

    public void loop(ServoClaw claw, DualMotorLift lift, LEDs leds){

        try {
            lift.loop();
        }catch(Exception ignored){

        }
        try {
            claw.loop();
        }catch(Exception ignored){

        }
        try{
            leds.loop();
        }catch (Exception ignored){

        }

        if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = voltageSensor.getVoltage();
        }

    }
    public void tune_loop(ServoClaw claw, DualMotorLift lift, double clawChange){

        try {
            lift.tuning_loop();
        }catch(Exception ignored){

        }
        try {
            claw.tuning_loop(clawChange);
        }catch(Exception ignored){

        }

        if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = voltageSensor.getVoltage();
        }

    }


    public double getVoltage(){
        return voltage;
    }

    public void floatMotors(){
        liftM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
