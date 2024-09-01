package org.firstinspires.ftc.teamcode.customclasses.helpers;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public interface ProcessorInterface extends VisionProcessor {
    @Override
    public abstract Object processFrame(Mat frame, long captureTimeNanos);

    public abstract int getResult();
}