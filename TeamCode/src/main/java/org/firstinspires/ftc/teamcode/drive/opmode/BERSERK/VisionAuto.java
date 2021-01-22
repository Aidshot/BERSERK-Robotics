package org.firstinspires.ftc.teamcode.drive.opmode.BERSERK;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.UGContourPipelineBERSERK;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(group = "BERSERK")
public class VisionAuto extends LinearOpMode {
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final int HORIZON = 150; // horizon value to tune
    private static final boolean DEBUG = false; // if debug is wanted, change to true
    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "webcam1"; // insert webcam name from configuration if using webcam

    private UGContourPipelineBERSERK pipeline;
    private OpenCvCamera camera;

    public enum State {
        FOUR,
        ONE,
        ZERO
    }

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new UGContourPipelineBERSERK(telemetry, DEBUG));
        UGContourPipelineBERSERK.Config.setCAMERA_WIDTH(CAMERA_WIDTH);
        UGContourPipelineBERSERK.Config.setHORIZON(HORIZON);
        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        State state = State.ZERO;

        while (!isStarted()) {
            switch (pipeline.getHeight()) {
                case ZERO:
                    state = State.ZERO;
                    break;
                case ONE:
                    state = State.ONE;
                    break;
                case FOUR:
                    state = State.FOUR;
                    break;
            }
            waitForStart();

            while (opModeIsActive()) {
                String height = "[HEIGHT]" + " " + pipeline.getHeight();
                telemetry.addData("[Ring Stack] >>", height);
                telemetry.update();
            }

            switch (state) {
                case ZERO:
                    //telemetry.addData("Stack","Zero");
                    //telemetry.update();

                case ONE:
                    //telemetry.addData("Stack","One");
                    //telemetry.update();

                case FOUR:
                    //telemetry.addData("Stack","Four");
                    //telemetry.update();
            }
        }
    }
}
