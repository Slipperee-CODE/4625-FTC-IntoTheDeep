package org.firstinspires.ftc.teamcode.customclasses.processors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.customclasses.helpers.ProcessorInterface;
import org.opencv.core.Mat;

public class ExampleProcessor implements ProcessorInterface {

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    @Override
    public int getResult() {
        return 0;
    }
}
