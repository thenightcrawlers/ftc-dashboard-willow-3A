package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@Autonomous()
public class AprilTagCM extends OpMode {
private AprilTagProcessor aprilTag;
private VisionPortal visionPortal;
private static final boolean USE_WEBCAM = true;
    @Override
public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, aprilTag);
        }
    @Override
public void init_loop() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        StringBuilder idsFound = new StringBuilder();


        for (AprilTagDetection detection : currentDetections) {
            idsFound.append(detection.id);
            idsFound.append(' ');
        }
        telemetry.addData("April Tags", idsFound);
    }

    @Override
public void start() {
        visionPortal.stopStreaming();
        }

        @Override
public void loop() {
        }
}