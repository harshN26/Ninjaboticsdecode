package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;


@Configurable
public class Constants {







    //TODO: add all positions and update
    public static final int ShooterResetRPM =0;
    public static final int ShooterIdleRPM =0;
    public static final int ShooterStopRPM =0;

    public static final double hoodMaxPos =0.6;
    public static double hoodMinPos =0.1;
    public static final double hoodResetPos =0.5;
    public static final double hoodTolerance=0.02;

    public static double shooterMinAngle=15;//deg
    public static double shooterMaxAngle=45;//deg





    public static final Pose redGoal=new Pose(138,142);
    public static final Pose blueGoal=new Pose(0,138);
    public static double zGoal=40; //in

    public static double shooterHeight=16;//in
    public static final double shooterWheelRadius=1.5;//in

    public static final double g = 386.09; // in/s^2
    public static double dZ = zGoal - shooterHeight;



    public static final double MIN_WHEEL_RPM = 2900.0;
    public static final double MAX_WHEEL_RPM = 5000.0;


    public static double EFFECTIVE_RPM_FACTOR = 0.5;

    public static double ShooterDistanceSlope=13;




    public static double[] pidCoeffs_turret ={0.02,0.0368,0.0007};
    public static final int tolerance_turret=5;

    public static double tolerance_shooter=95;

    public static final double TICKS_PER_REV_Shooter = 28;

    public static final double TICKS_PER_REV_Turret = 537.7;

    public static double shooter_kp=0.0009;
    public static double shooter_kv=0.0002;
    public static double shooter_ks=0.0;



    public static final double rampUp=0.6;
    public static final double rampDown=0.5;



    public static final int turretMaxTicks=800;





    public static final Pose redGoalStartingPose=new Pose(122,124,Math.toRadians(37));
    public static final Pose redBackStartingPose=new Pose(89,9,Math.toRadians(90));
    public static final Pose blueGoalStartingPose=new Pose(22,124,Math.toRadians(143));
    public static final Pose blueBackStartingPose=new Pose(55,9,Math.toRadians(90));


    public static final Pose blueResetPose=new Pose(136,9,Math.toRadians(90));
    public static final Pose redResetPose=new Pose(9,9,Math.toRadians(90));



    public static final Pose autoCloseBLUEShoot=new Pose(55.000, 83.000);
    public static final Pose autoFarBLUEShoot=new Pose(54.000, 13.000);
    public static final Pose autoCloseREDShoot=new Pose(87.000, 83.000);
    public static final Pose autoFarREDShoot=new Pose(90.000, 13.000);










    public static final double filterQ=0.3;
    public static final double filterR=5.0;
    public static final int filterN=3;




}
