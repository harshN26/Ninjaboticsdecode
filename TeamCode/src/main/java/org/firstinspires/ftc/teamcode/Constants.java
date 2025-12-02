package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;


@Config
public class Constants {







    //TODO: add all positions and update
    public static final int ShooterResetRPM =-100;
    public static final int ShooterIdleRPM =3000;
    public static final int ShooterStopRPM =0;

    public static final double hoodMaxPos =0.9;
    public static final double hoodMinPos =0.1;
    public static final double hoodResetPos =0.5;
    public static final double hoodTolerance=0.02;

    public static final double shooterMinAngle=15;//deg
    public static final double shooterMaxAngle=45;//deg





    public static final Pose redGoal=new Pose(0,0);
    public static final Pose blueGoal=new Pose(0,0);
    public static final double zGoal=60; //in

    public static final double shooterHeight=15;//in
    public static final double shooterWheelRadius=1.5;//in

    public static final double g = 386.09; // in/s^2
    public static final double dZ = zGoal - shooterHeight;
    public static final double MAX_WHEEL_RPM = 6000;
    public static final double EFFECTIVE_RPM_FACTOR = 0.97;




    public static final double[] pidCoeffs_turret ={0,0,0};
    public static final int tolerance_turret=0;

    public static final double tolerance_shooter=0;

    public static final double TICKS_PER_REV_Shooter = 560; //TODO: change to match motor

    public static final double TICKS_PER_REV_Turret = 560; //TODO: change to match motor

    public static final double turr_kp=0.0;
    public static final double turr_kv=0.0;
    public static final double turr_ks=0.0;



    public static final double rampUp=0.0;
    public static final double rampDown=0.0;









    public static final Pose redGoalStartingPose=new Pose(0,0,0);
    public static final Pose redBackStartingPose=new Pose(0,0,0);
    public static final Pose blueGoalStartingPose=new Pose(0,0,0);
    public static final Pose blueBackStartingPose=new Pose(0,0,0);


    public static final Pose blueResetPose=new Pose(0,0,0);
    public static final Pose redResetPose=new Pose(0,0,0);




}
