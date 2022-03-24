package org.firstinspires.ftc.teamcode.Amin.incercareDetectie3Patrate;

import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.b_h;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.b_l;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.g_h;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.g_l;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.r_h;
import static org.firstinspires.ftc.teamcode.Amin.NuSeMaiUmbla.r_l;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Detectie extends OpenCvPipeline {
    Mat mat = new Mat();

    public int a;
    public UNDE_E unde_e;

    static final Rect LEFT_ROI = new Rect(
            new Point(0, 35),
            new Point(90, 75)
    );

    static final Rect CENTER_ROI = new Rect(
            new Point(120, 35),
            new Point(190, 75)
    );

    static final Rect RIGHT_ROI = new Rect(
            new Point(250, 35),
            new Point(320, 75)
    );


    static double PERCENT_COLOR_THRESHOLD = 0.1;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(r_l, g_l, b_l);
        Scalar highHSV = new Scalar(r_h, g_h, b_h);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat center = mat.submat(CENTER_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        center.release();
        right.release();

        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneCenter = centerValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (stoneCenter) {
            a = 1;
            unde_e = UNDE_E.CENTER;
        } else if (stoneLeft) {
            a = -1;
            unde_e = UNDE_E.LEFT;
        } else if (stoneRight) {
            a = 2;
            unde_e = UNDE_E.RIGHT;
        }

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar nup = new Scalar(45, 100, 10);
        Scalar verde = new Scalar(75, 255, 255);

        Imgproc.rectangle(mat, LEFT_ROI, unde_e == UNDE_E.LEFT ? verde : nup);
        Imgproc.rectangle(mat, CENTER_ROI, unde_e == UNDE_E.CENTER ? verde : nup);
        Imgproc.rectangle(mat, RIGHT_ROI, unde_e == UNDE_E.RIGHT ? verde : nup);

        return mat;
    }

    public int getA() {
        return a;
    }

    public UNDE_E getUnde_e() {
        return unde_e;
    }

}
