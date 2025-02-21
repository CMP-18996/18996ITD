package org.firstinspires.ftc.teamcode.common.vision;

import android.graphics.Canvas;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ColorPipeline implements VisionProcessor {
    // Constants and Scalars TODO: Tune min and max size
    Scalar lowerBound;
    Scalar upperBound;
    double minSize = 100000.0;
    double maxSize = 200000.0;

    // State values
    boolean currentlyDetected = false;
    double detectedAngle = 0;



    // Auxiliary Variables
    // Do NOT initialize these in processFrame(), will cause memory leak unless you know what you're doing
    Mat blurred;
    Mat colorCorrected;
    Mat postBlueThresh;

    ArrayList<MatOfPoint> contours = new ArrayList<>();
    MatOfPoint contour;
    MatOfPoint largestContourSeen;
    MatOfPoint2f contour2f;
    RotatedRect rect;
    double preProcessingAngle;
    @Override
    public Mat processFrame(Mat frame, long captureTimeNanos) {
        imageProcessing(frame);
        frameDrawing(frame);

        // Can be changed to any auxiliary frame for testing
        return frame;
    }

    private void imageProcessing(Mat frame) {
        // remove if too slow
        Imgproc.GaussianBlur(frame, blurred, new Size(5, 5), 0);

        // TODO: Needs testing to see if should be RGB2HSV or BGR2HSV
        Imgproc.cvtColor(blurred, colorCorrected, Imgproc.COLOR_RGB2HSV);
        Core.inRange(colorCorrected, lowerBound, upperBound, postBlueThresh);

        Mat hierarchy = new Mat(); // unused but needed
        Imgproc.findContours(postBlueThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();
        if (contours.isEmpty()) {
            currentlyDetected = false;
            return;
        };

        contour = findLargestContour(contours);
        if (contour == null) {
            currentlyDetected = false;
            return;
        }

        contour2f = new MatOfPoint2f(contour.toArray());
        rect = Imgproc.minAreaRect(contour2f);
        preProcessingAngle = rect.angle;
        if (rect.size.width > rect.size.height) {
            detectedAngle = preProcessingAngle - 90;
        }
        else {
            detectedAngle = preProcessingAngle;
        }
    }

    private void frameDrawing(Mat frame) {

    }

    private MatOfPoint findLargestContour(ArrayList<MatOfPoint> contours) {
        double largestSeenSize = 0;
        int maxIndex = -1;
        for (int i = 0; i < contours.size(); i++) {
            double area = Imgproc.contourArea(contours.get(i));
            if (area > maxSize || area < minSize) return null;
            if (largestSeenSize < area) {
                largestContourSeen = contours.get(i);
                maxIndex = i;
            }
        }
        return contours.get(maxIndex);
    }

    public boolean specimenDetected() {
        return currentlyDetected;
    }

    public double getAngle() {
        return detectedAngle;
    }


    public ColorPipeline(Color color) {
        switch (color) {
            case RED:
                lowerBound = new Scalar(116, 100, 0);
                upperBound = new Scalar(120, 255, 255);
                break;
            case BLUE:
                lowerBound = new Scalar(0, 100, 0);
                upperBound = new Scalar(22, 255, 255);
                break;
            case YELLOW:
                lowerBound = new Scalar(100, 100, 0);
                upperBound = new Scalar(110, 255, 255);
                break;
        }
    }

    public void setMinSize(double newSize) {
        this.minSize = newSize;
    }

    public void setMaxSize(double newSize) {
        this.maxSize = newSize;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }
}
