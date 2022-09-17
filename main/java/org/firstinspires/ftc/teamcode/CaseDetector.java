package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class CaseDetector extends OpenCvPipeline {

    Telemetry telemetry;
    Mat mat = new Mat();

    int location = -1;
    ///static final Rect LEFT_ROI = new Rect( new Point(130, 20),new Point(220, 70) );
    static final Rect MIDDLE_ROI = new Rect( new Point(150, 10),new Point(200, 50) );
    ///static final Rect RIGHT_ROI = new Rect( new Point(130, 90),new Point(220, 140) );
    static final Rect DOWN_ROI = new Rect( new Point(150, 70),new Point(200, 110) );
    static double THRESHOLD = 0.3;
    public CaseDetector (Telemetry t)
    {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(40,52,72);
        Scalar highHSV = new Scalar(102,255,255);


        Core.inRange(mat,lowHSV,highHSV,mat);

        Mat left = mat.submat(MIDDLE_ROI);
        Mat right = mat.submat(DOWN_ROI);

        double leftValue = Core.sumElems(left).val[0] / MIDDLE_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / DOWN_ROI.area() / 255;

        left.release();
        right.release();

        boolean middleCase = leftValue > THRESHOLD;
        boolean downCase = rightValue >THRESHOLD;

        if(!downCase && !middleCase)
        {
            location = 1;
            telemetry.addData("TSE location", "up");
        }
        else if(middleCase)
        {
            location = 2;
            telemetry.addData("TSE location", "middle");
        }
        else
        {
            location = 3;
            telemetry.addData("TSE location", "down");
        }

        telemetry.addData("Webcam", "READY");
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar bad = new Scalar(255, 0, 0);
        Scalar good = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, MIDDLE_ROI, location == 2 ? good:bad);
        Imgproc.rectangle(mat, DOWN_ROI, location == 3 ? good:bad);

        return mat;
    }

    public int getLocation()
    {
        return location;
    }

}

