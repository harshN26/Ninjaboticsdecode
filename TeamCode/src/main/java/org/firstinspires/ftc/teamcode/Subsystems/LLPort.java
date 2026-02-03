package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.teamcode.Subsystems.TurretShooter.shooterState.AUTOCLOSE;
import static org.firstinspires.ftc.teamcode.Subsystems.TurretShooter.shooterState.AUTOFAR;
import static org.firstinspires.ftc.teamcode.Subsystems.TurretShooter.shooterState.FIRE;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

public class LLPort {

    static Robot_Hardware robot;


    public static Pose robotUpdatedPose;

    public static boolean resultValid=false;

    Telemetry telemetry;
    ElapsedTime timer;
    KalmanFilter filterX,filterY;
    public static double tx=0.0;

    public static boolean takeover=false;

    public LLPort(Telemetry telem, Robot_Hardware hardware){
        robot=hardware;
        telemetry=telem;
        timer=new ElapsedTime();
        filterX=new KalmanFilter(Constants.filterQ,Constants.filterR,Constants.filterN);
        filterY=new KalmanFilter(Constants.filterQ,Constants.filterR,Constants.filterN);

    }
    public void start(){
        timer.startTime();
        robot.limelight.start();
    }
    public void loop(){


        robot.limelight.updateRobotOrientation(Math.toDegrees(robot.heading));

        LLResult result=robot.limelight.getLatestResult();


        if((resultValid = result!=null &&
                result.isValid() &&
                result.getFiducialResults().size()>0 &&
                result.getFiducialResults().get(0).getFiducialId()==robot.aprilTagID)
        ){
            tx=result.getFiducialResults().get(0).getTargetXDegrees();
            if((TurretShooter.state==FIRE||TurretShooter.state==AUTOCLOSE||TurretShooter.state==AUTOFAR)&&robot.x==0){
                takeover=true;
            }
        }
        else tx=0.0;
    }
    public static double getRotation() {

        double motorRevs = -(tx / (360));
        int targetTicks = (int)(motorRevs * Constants.TICKS_PER_REV_Turret)+robot.turretPos;
        targetTicks = Math.max(-Constants.turretMaxTicks, Math.min(Constants.turretMaxTicks, targetTicks));
        return targetTicks;
    }






    public void telem(){

        telemetry.addLine("tx:" + tx);
        telemetry.addLine("rotation:" + getRotation());
//        telemetry.addLine("Limelight Y:" + robot.limelight.getLatestResult().getBotpose().getPosition().y*39.37);
    }



}
