package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Subsystems.LLPort;
import org.firstinspires.ftc.teamcode.Subsystems.ChamberSort;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.TurretShooter;
import org.firstinspires.ftc.teamcode.Subsystems.ColorLeds;


public class Robot_Hardware{
    HardwareMap hwMap;

    public DcMotorEx shooterM1;
    public DcMotorEx shooterM2;
    public DcMotorEx turret;
    public DcMotorEx intake;

    public CRServo sort1;
    public Servo push, ramp, flickSide, flickUp, hood1, hood2;

    public VoltageSensor voltageSensor;

    public RevTouchSensor turretZero;

    public Limelight3A limelight;

    public double voltage;
    ElapsedTime voltageTimer, loopTimer;

    public Pose startPose=new Pose(0,0,0);






    public boolean enabled;

    public enum GameState{AUTO,TELE}
    public enum AllianceColor{RED,BLUE}
    public static GameState currGameState=GameState.AUTO;

    public static AllianceColor alliance=AllianceColor.RED;

    public static Pose goal,pose=Constants.redGoalStartingPose,resetPose=Constants.redResetPose;

    public static Pose endPoseAutoShoot=new Pose(0,0,0);
    public static boolean autoPoseResetApproval=false;



    Vector driveVector;


    public double x, xVelo;
    public double y,yVelo;

    public double turretX,turretY;
    public double heading, headingVelo;

    public static boolean firing=false;

    public static int aprilTagID=24;

    Telemetry telem;

    public static int turretPos=0;

    public static NormalizedColorSensor colorin,colorout;

    public static Servo led;




    private static Robot_Hardware instance = null;
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
        loopTimer=new ElapsedTime();

        //m1 MUST be ACTIVE
        shooterM1=hwMap.get(DcMotorEx.class, Global_Configs.shooterM1Name);
        shooterM1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterM1.setDirection(DcMotorEx.Direction.REVERSE);

        if(Global_Configs.shooterM2Status== Global_Configs.DOFStatus.ACTIVE) {
            shooterM2=hwMap.get(DcMotorEx.class, Global_Configs.shooterM2Name);
            shooterM2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            shooterM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if(Global_Configs.turretStatus== Global_Configs.DOFStatus.ACTIVE) {
            turret=hwMap.get(DcMotorEx.class, Global_Configs.turretName);
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            turret.setDirection(DcMotorSimple.Direction.REVERSE);
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if(Global_Configs.intakeStatus== Global_Configs.DOFStatus.ACTIVE) {
            intake=hwMap.get(DcMotorEx.class, Global_Configs.intakeName);
            intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setDirection(DcMotorEx.Direction.REVERSE);
        }


        sort1=hwMap.get(CRServo.class, Global_Configs.sort1Name);


        push=hwMap.get(Servo.class, Global_Configs.pushName);
        push.setDirection(Servo.Direction.REVERSE);
        push.setPosition(0.05);
        ramp=hwMap.get(Servo.class, Global_Configs.rampName);
        flickSide=hwMap.get(Servo.class, Global_Configs.flickSideName);
        flickSide.setPosition(0);
        flickUp=hwMap.get(Servo.class, Global_Configs.flickUpName);
        hood1=hwMap.get(Servo.class, Global_Configs.hood1Name);

        hood2=hwMap.get(Servo.class, Global_Configs.hood2Name);
        hood2.setDirection(Servo.Direction.REVERSE);

        turretZero=hardwareMap.get(RevTouchSensor.class,Global_Configs.turretZeroName);


        limelight=hardwareMap.get(Limelight3A.class,Global_Configs.limelightName);
        limelight.setPollRateHz(250);
        limelight.pipelineSwitch(0);
        limelight.start();

        colorin=hardwareMap.get(NormalizedColorSensor.class, Global_Configs.colorIn);
        colorout=hardwareMap.get(NormalizedColorSensor.class, Global_Configs.colorOut);

        led=hardwareMap.get(Servo.class,Global_Configs.ledsName);
        telem=telemetry;


        voltageSensor=hardwareMap.voltageSensor.iterator().next();

        if(alliance==AllianceColor.RED){
            goal= Constants.redGoal;
            aprilTagID=24;
        }else{
            goal= Constants.blueGoal;
            aprilTagID=20;
        }

    }



    public void loop(ChamberSort chamber, TurretShooter shooter, Intake intakeSubsystem, Follower follower, LLPort ll, ColorLeds led){


        try {
            chamber.loop();
        }catch(Exception ignored){
            telem.addLine("New Chamber error: "+ignored);
        }
        try {
            ll.loop();
            ll.telem();
        }catch(Exception ignored){
            telem.addLine("New Limelight error: "+ignored);
        }
        try {
            shooter.loop();
            turretPos=shooter.getTurretPos();
        }catch(Exception ignored){
            telem.addLine("New Shooter error: "+ignored);
        }
        try{
            intakeSubsystem.loop();
        }catch (Exception ignored){
            telem.addLine("New Intake error: "+ignored);
        }
        try{
            led.loop();
        }catch (Exception ignored){
            telem.addLine("New Intake error: "+ignored);
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
            turretX = x + (Constants.turretOffsetX * Math.cos(heading) - Constants.turretOffsetY * Math.sin(heading));
            turretY = y + (Constants.turretOffsetX * Math.sin(heading) + Constants.turretOffsetY * Math.cos(heading));
//            telem.addLine("goal"+ y);
//            telem.addLine("pose"+ x);
            if(follower.isBusy()&&currGameState==GameState.AUTO) {
                endPoseAutoShoot = follower.getCurrentPathChain().lastPath().endPose();
            }

        }catch (Exception ignored){
            telem.addLine("New follower error: "+ignored);
        }



        if (voltageTimer.milliseconds() > 100) {
            voltageTimer.reset();
            voltage = voltageSensor.getVoltage();
        }

        telem.addLine("Loop time (milliseconds): "+loopTimer.milliseconds());
        loopTimer.reset();

        telem.addLine("pose"+ new Pose(x, y,heading));
        telem.update();

    }

    public void end(){
        limelight.stop();
        colorin.close();
        colorout.close();
    }

}
