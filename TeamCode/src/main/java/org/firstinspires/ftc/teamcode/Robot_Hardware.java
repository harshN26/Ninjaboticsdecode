package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.utils.ChamberSort;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.TurretShooter;
import org.firstinspires.ftc.teamcode.*;


public class Robot_Hardware{
    HardwareMap hwMap;

    public DcMotorEx shooterM1;
    public DcMotorEx shooterM2;
    public DcMotorEx turret;
    public DcMotorEx intake;

    public CRServo sort1, sort2;
    public Servo push, ramp, flickSide, flickUp, hood1, hood2;

    public VoltageSensor voltageSensor;
    public double voltage;
    ElapsedTime voltageTimer;





    private static Robot_Hardware instance = null;

    public boolean enabled;

    public enum GameState{AUTO,TELE}
    public enum AllianceColor{RED,BLUE}
    public static GameState currGameState=GameState.AUTO;

    public static AllianceColor alliance=AllianceColor.RED;

    public Pose goal;
    public static boolean inReset=false;


    public Pose pose;
    Vector driveVector;


    public double x, xVelo;
    public double y,yVelo;
    public double heading, headingVelo;

    public double turrHeading;

    Telemetry telem;





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
        shooterM1=hwMap.get(DcMotorEx.class, Global_Configs.shooterM1Name);
        shooterM1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        if(Global_Configs.shooterM2Status== Global_Configs.DOFStatus.ACTIVE) {
            shooterM2=hwMap.get(DcMotorEx.class, Global_Configs.shooterM2Name);
            shooterM2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            shooterM2.setDirection(DcMotorEx.Direction.REVERSE);
        }
        if(Global_Configs.turretStatus== Global_Configs.DOFStatus.ACTIVE) {
            turret=hwMap.get(DcMotorEx.class, Global_Configs.turretName);
            turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if(Global_Configs.intakeStatus== Global_Configs.DOFStatus.ACTIVE) {
            intake=hwMap.get(DcMotorEx.class, Global_Configs.intakeName);
            intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }


        sort1=hwMap.get(CRServo.class, Global_Configs.sort1Name);
        sort2=hwMap.get(CRServo.class, Global_Configs.sort2Name);

        push=hwMap.get(Servo.class, Global_Configs.pushName);
        ramp=hwMap.get(Servo.class, Global_Configs.rampName);
        flickSide=hwMap.get(Servo.class, Global_Configs.flickSideName);
        flickSide=hwMap.get(Servo.class, Global_Configs.flickUpName);
        hood1=hwMap.get(Servo.class, Global_Configs.hood1Name);
        hood2=hwMap.get(Servo.class, Global_Configs.hood2Name);


        telem=telemetry;



        voltageSensor=hardwareMap.voltageSensor.iterator().next();


        if(alliance==AllianceColor.RED){
            goal= Constants.redGoal;
        }else{
            goal= Constants.blueGoal;
        }

//        leds=hardwareMap.get(RevBlinkinLedDriver.class, Global_Configs.ledsName);

    }

    public void loop(ChamberSort chamber, TurretShooter shooter, Intake intakeSubsytem, Follower follower){

        try {
            chamber.loop();
        }catch(Exception ignored){

        }
        try {
            shooter.loop();
        }catch(Exception ignored){

        }
        try{
            intakeSubsytem.loop();
        }catch (Exception ignored){

        }
        try{
            follower.update();
            pose=follower.getPose();
            driveVector=follower.getVelocity();
            x=pose.getX();
            y= pose.getY();
            heading=pose.getHeading();
            xVelo=driveVector.getXComponent();
            yVelo=driveVector.getYComponent();
            headingVelo=follower.getAngularVelocity();

        }catch (Exception ignored){

        }

        if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = voltageSensor.getVoltage();
        }

    }



    public double getVoltage(){
        return voltage;
    }

}
