package org.firstinspires.ftc.teamcode.OpModes.Limelight;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot_Hardware;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class LLPort {

    Robot_Hardware robot;
    public static Pose robotUpdatedPose;
    public static boolean currentPose;
    private Pose3D robotPoseEstimate;

    Telemetry telemetry;
    public LLPort(Telemetry telem, Robot_Hardware hardware){
        robot=hardware;
        telemetry=telem;
    }

    public void loop(){
        robotPoseEstimate=robot.limelight.getLatestResult().getBotpose();
        currentPose=false;
        if((robotUpdatedPose=new Pose(robotPoseEstimate.getPosition().x, robotPoseEstimate.getPosition().y,robotPoseEstimate.getOrientation().getPitch())).equals(robot.pose)
        && robotUpdatedPose!=null){
            currentPose=true;
            robot.pose=robotUpdatedPose;
        }
    }



}
