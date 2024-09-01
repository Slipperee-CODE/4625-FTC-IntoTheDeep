package org.firstinspires.ftc.teamcode.customclasses.helpers;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.customclasses.processors.ExampleProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class MultiProcessorWebcam {
    private final ProcessorInterface customProcessor;
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private static class LensIntrinsics {
        static double fx = 835.64;
        static double fy = 835.64;
        static double cx = 459.22;
        static double cy = 261.933;

    }
    public enum Processor {
        APRIL_TAG,
        CUSTOM,
        NONE
    }
    private Processor activeProcessor;

    public MultiProcessorWebcam(HardwareMap hardwareMap) {
        customProcessor = new ExampleProcessor();
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(LensIntrinsics.fx, LensIntrinsics.fy, LensIntrinsics.cx, LensIntrinsics.cy)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
                .build();

        visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(960, 544))
                .addProcessors(customProcessor,aprilTagProcessor)
                .build();

        setActiveProcessor(Processor.NONE);
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            CustomOpMode.sleep(1);
        }
        visionPortal.getCameraControl(ExposureControl.class).setMode(ExposureControl.Mode.Manual);
    }

    public int getResultFromCustomProcessor(){
        return customProcessor.getResult();
    }

    public void setExposure(int millis) {
        visionPortal.getCameraControl(ExposureControl.class).setExposure(millis, TimeUnit.MILLISECONDS);
    }

    public void setGain(int grain) {
        visionPortal.getCameraControl(GainControl.class).setGain(grain);
    }

    public void setActiveProcessor(Processor processor) {
        activeProcessor = processor;
        switch (processor) {
            case APRIL_TAG:
                visionPortal.setProcessorEnabled(customProcessor,false);
                visionPortal.setProcessorEnabled(aprilTagProcessor,true);
                break;
            case CUSTOM:
                visionPortal.setProcessorEnabled(customProcessor,true);
                visionPortal.setProcessorEnabled(aprilTagProcessor,false);
                break;
            case NONE:
                visionPortal.setProcessorEnabled(customProcessor,false);
                visionPortal.setProcessorEnabled(aprilTagProcessor,false);
                break;
        }
    }

    public void update() {
        switch (activeProcessor) {
            case APRIL_TAG:
                VisibleTagsStorage.stored = aprilTagProcessor.getDetections();
                break;
            case CUSTOM:
                break;
            case NONE:
                break;
        }
    }
}
