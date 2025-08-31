package org.firstinspires.ftc.teamcode.OpModes.OpenCvPipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.*;

public class ColorObjectFinder extends OpenCvPipeline {
    private final Scalar
            BLUE    = new Scalar(0, 255, 255),
            RED = new Scalar(255, 0, 255);


    public Mat processFrame(Mat input){
        Mat gray_scale=new Mat();
        Mat filterGround=new Mat();
        Imgproc.cvtColor(input,gray_scale,Imgproc.COLOR_RGB2GRAY);
        Imgproc.threshold(gray_scale,filterGround,0,255,3);




        gray_scale.release();
        return input;
    }


}
