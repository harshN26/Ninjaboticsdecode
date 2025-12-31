package org.firstinspires.ftc.teamcode.Subsystems;


import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

public class LLPort {

    Robot_Hardware robot;

    public static Pose robotPose;
    public static Pose robotUpdatedPose;

    public boolean resultValid=false;
    private boolean resultValidLast=false;

    Telemetry telemetry;
    ElapsedTime timer;
    KalmanFilter filterX,filterY;
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
        resultValidLast = resultValid;

        robot.limelight.updateRobotOrientation(Math.toDegrees(robot.heading));

        LLResult result=robot.limelight.getLatestResult();

        Pose3D llPose = result.getBotpose();

        robotPose = new Pose(
                72 + llPose.getPosition().y * 39.37007874,
                -llPose.getPosition().x * 39.37007874 + 72,
                robot.heading
        );


        if ((resultValid=result.isValid())&&resultValid!=resultValidLast) {
            resetFilter();
        }

        if(resultValid) {
            double currentValueX = robotPose.getX();
            double estimateX = filterX.estimate(currentValueX);


            double currentValueY = robotPose.getY();
            double estimateY = filterY.estimate(currentValueY);

            robotUpdatedPose = new Pose(
                    estimateX,
                    estimateY,
                    robot.heading
            );
        }




    }
    public Pose returnPose(){

        return robotUpdatedPose; //robotUpdatedPose
    }

    public void telem(){
        telemetry.addLine("Pose: "+ robotUpdatedPose); //robotUpdatedPose
        telemetry.addLine("Limelight X:" + robot.limelight.getLatestResult().getBotpose().getPosition().x*39.37);
        telemetry.addLine("Limelight Y:" + robot.limelight.getLatestResult().getBotpose().getPosition().y*39.37);
    }

   private void resetFilter(){
        filterX.setX(robot.x);
        filterY.setX(robot.y);
   }


}
