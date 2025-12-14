package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;


@Configurable
public class Constants {







    //TODO: add all positions and update
    public static final int ShooterResetRPM =-150;
    public static final int ShooterIdleRPM =500;
    public static final int ShooterStopRPM =0;

    public static final double hoodMaxPos =0.9;
    public static double hoodMinPos =0.1;
    public static final double hoodResetPos =0.5;
    public static final double hoodTolerance=0.02;

    public static double shooterMinAngle=15;//deg
    public static double shooterMaxAngle=45;//deg





    public static final Pose redGoal=new Pose(0,0);
    public static final Pose blueGoal=new Pose(0,0);
    public static double zGoal=60; //in

    public static double shooterHeight=15;//in
    public static final double shooterWheelRadius=1.5;//in

    public static final double g = 386.09; // in/s^2
    public static double dZ = zGoal - shooterHeight;



    public static final double MIN_WHEEL_RPM = 1000.0;
    public static final double MAX_WHEEL_RPM = 5500.0;

    // Close-shot RPM bias tuning
    public static final double CLOSE_RPM_BIAS = 600.0;   // extra RPM at zero distance
    public static final double CLOSE_BIAS_DECAY = 2.5;   // meters (larger = fades slower)

    // Optional global tuning
    public static final double RPM_TUNING_FACTOR = 1.0;
    public static final double EFFECTIVE_RPM_FACTOR = 0.97;




    public static double[] pidCoeffs_turret ={0,0,0};
    public static final int tolerance_turret=5;

    public static final double tolerance_shooter=1000;

    public static final double TICKS_PER_REV_Shooter = 28;

    public static final double TICKS_PER_REV_Turret = 384.5;

    public static double shooter_kp=0.0003;
    public static double shooter_kv=0.00024;
    public static double shooter_ks=0.0;



    public static final double rampUp=0.6;
    public static final double rampDown=0.5;









    public static final Pose redGoalStartingPose=new Pose(0,0,0);
    public static final Pose redBackStartingPose=new Pose(0,0,0);
    public static final Pose blueGoalStartingPose=new Pose(0,0,0);
    public static final Pose blueBackStartingPose=new Pose(0,0,0);


    public static final Pose blueResetPose=new Pose(0,0,0);
    public static final Pose redResetPose=new Pose(0,0,0);




}
