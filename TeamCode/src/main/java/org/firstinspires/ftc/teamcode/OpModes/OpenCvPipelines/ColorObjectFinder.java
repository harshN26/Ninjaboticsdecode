package org.firstinspires.ftc.teamcode.OpModes.OpenCvPipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class colorIdentifier extends OpenCvPipeline {
    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            BLUE    = new Scalar(0, 255, 255),
            RED = new Scalar(255, 0, 255);

    private  Point sample_pointA= new Point(0,0), sample_PointB=new Point(100,100);

    public Mat processFrame(Mat input){

        Mat areaMat=input.submat(new Rect(sample_pointA, sample_PointB));
        areaMat.release();
        Scalar sumColors = Core.sumElems(areaMat);

        double minColor = Math.min(sumColors.val[0], Math.min(sumColors.val[1], sumColors.val[2]));

        if (sumColors.val[0] == minColor) {

            Imgproc.rectangle(
                    input,
                    sample_pointA,
                    sample_PointB,
                    BLUE,
                    2
            );

        } else if (sumColors.val[1] == minColor) {

            Imgproc.rectangle(
                    input,
                    sample_pointA,
                    sample_PointB,
                    RED,
                    2
            );
        } else {

            Imgproc.rectangle(
                    input,
                    sample_pointA,
                    sample_PointB,
                    YELLOW,
                    2
            );
        }
        return input;
    }


}
