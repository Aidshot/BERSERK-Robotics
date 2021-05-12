package org.firstinspires.ftc.teamcode.drive.opmode.BERSERK.Powershot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.BERSERK.HardwareBERSERK;
import org.firstinspires.ftc.teamcode.drive.opmode.BERSERK.PoseStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import static com.arcrobotics.ftclib.vision.UGContourRingPipeline.Config;

@Autonomous(group = "BERSERK")
public class RED_POWERSHOT_BB_AUTO extends LinearOpMode {

    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final int HORIZON = 50; // horizon value to tune
    private static final boolean DEBUG = false; // if debug is wanted, change to true
    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "webcam1"; // insert webcam name from configuration if using webcam

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    enum State {
        FOUR,
        ONE,
        ZERO
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareBERSERK robot    = new HardwareBERSERK();
        robot.init(hardwareMap);

        RevBlinkinLedDriver.BlinkinPattern pattern;
        pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        robot.blinkinLedDriver.setPattern(pattern);

        double foldout = 0; //SET TO -1 TO FOLDOUT INTAKE, 0 TO DISABLE

        double shooter_target_velo = 1830;
        double launch_angle = 0.125; //0.173
        double kicker_out = 0.7;
        double kicker_in = 0.25; //02
        double wobble_close = 0.18;
        double wobble_open = 0.6;
        double wobble_up = 0.3;
        double wobble_down = 0.8;
        long shootWait = 330;
        double webcam_right = 0.1;
        double webcam_left = 0.3;

        double pos1 = 0.0;
        double pos2 = -9.0;
        double pos3 = -15.0;

        robot.webcam_servo.setPosition(webcam_right);

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
                    .createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));
        Config.setCAMERA_WIDTH(CAMERA_WIDTH);
        Config.setHORIZON(HORIZON);
        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        State state = State.ZERO;

        robot.kicker.setPosition(kicker_out);
        Pose2d startPose = new Pose2d(-63.0,-26, Math.toRadians(0.0));
        drive.setPoseEstimate(startPose);


        //   A AUTO TRAJECTORIES   //
        //SHOOT POSITION
        Trajectory A1 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-50.0, 2.0, Math.toRadians(0.0)), Math.toRadians(0.0))
                .addTemporalMarker(0.1, () -> {
                    robot.foldout_lift.setPower(foldout);
                })
                .addTemporalMarker(1.8, () -> {
                    robot.foldout_lift.setPower(0);
                })
                .splineToSplineHeading(new Pose2d(-5.0, 2.0, Math.toRadians(pos1)), Math.toRadians(0.0)) //0
                .build();

        Trajectory A1a = drive.trajectoryBuilder(A1.end())
                .splineToLinearHeading(new Pose2d(-4.0, 4.0, Math.toRadians(pos2)), Math.toRadians(0.0)) //8
                .build();

        Trajectory A1b = drive.trajectoryBuilder(A1a.end())
                .splineToLinearHeading(new Pose2d(-5.0, 6.0, Math.toRadians(pos3)), Math.toRadians(0.0)) //15
                .build();

        //INTAKE FLOOR RINGS
        Trajectory A2 = drive.trajectoryBuilder(A1b.end().plus(new Pose2d(0, 0, Math.toRadians(0))))
                .splineToSplineHeading(new Pose2d(56.0, -5.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
                .splineToSplineHeading(new Pose2d(59.0, -35.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
                .splineToSplineHeading(new Pose2d(59.0, -16.0, Math.toRadians(-90.0)), Math.toRadians(90.0))
                .splineToSplineHeading(new Pose2d(-5.0, -6.0, Math.toRadians(-15.0)), Math.toRadians(180.0))
                .build();

        //Wobble POSITION
        Trajectory A3 = drive.trajectoryBuilder(A2.end())
                .splineToLinearHeading(new Pose2d(15.0, -42.0, Math.toRadians(180.0)), Math.toRadians(90.0))
                .build();

        //PARK
        Trajectory A4 = drive.trajectoryBuilder(A3.end())
                .strafeRight(33)
                .build();

        //   B AUTO TRAJECTORIES   //
        //SHOOT POSITION
        Trajectory B1 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-50.0, 12.0, Math.toRadians(0.0)), Math.toRadians(0.0))
                .addTemporalMarker(0.1, () -> {
                    robot.foldout_lift.setPower(foldout);
                })
                .addTemporalMarker(1.8, () -> {
                    robot.foldout_lift.setPower(0);
                })
                .splineToSplineHeading(new Pose2d(-5.0, 11.0, Math.toRadians(pos1)), Math.toRadians(0.0)) //0
                .build();

        Trajectory B1a = drive.trajectoryBuilder(B1.end())
                .splineToLinearHeading(new Pose2d(-5.0, 13.0, Math.toRadians(pos2)), Math.toRadians(0.0)) //8
                .build();

        Trajectory B1b = drive.trajectoryBuilder(B1a.end())
                .splineToLinearHeading(new Pose2d(-5.0, 14.0, Math.toRadians(pos3)), Math.toRadians(0.0)) //15
                .build();

        //INTAKE FLOOR RINGS
        Trajectory B2 = drive.trajectoryBuilder(B1b.end().plus(new Pose2d(0, 0, Math.toRadians(0))))
                .splineToSplineHeading(new Pose2d(56.0, 5.0, Math.toRadians(90.0)), Math.toRadians(90.0))
                .splineToSplineHeading(new Pose2d(59.0, 35.0, Math.toRadians(90.0)), Math.toRadians(90.0))
                .splineToSplineHeading(new Pose2d(59.0, 16.0, Math.toRadians(90.0)), Math.toRadians(-90.0))
                .splineToSplineHeading(new Pose2d(-5.0, 6.0, Math.toRadians(27.0)), Math.toRadians(180.0))
                .build();

        //Wobble POSITION
        Trajectory B3 = drive.trajectoryBuilder(B2.end())
                .splineToLinearHeading(new Pose2d(35.0, 24.0, Math.toRadians(5.0)), Math.toRadians(90.0))
                .build();

        //PARK
        Trajectory B4 = drive.trajectoryBuilder(B3.end())
                .splineToSplineHeading(new Pose2d(10.0, 10.0, Math.toRadians(5.0)), Math.toRadians(180.0))
                .build();

        //   C AUTO TRAJECTORIES   //
        //SHOOT POSITIONadb connect 192.168.43.1
        Trajectory C1 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-50.0, 12.0, Math.toRadians(0.0)), Math.toRadians(0.0))
                .addTemporalMarker(0.1, () -> {
                    robot.foldout_lift.setPower(foldout);
                })
                .addTemporalMarker(1.8, () -> {
                    robot.foldout_lift.setPower(0);
                })
                .splineToSplineHeading(new Pose2d(-5.0, 11.0, Math.toRadians(pos1)), Math.toRadians(0.0)) //0
                .build();

        Trajectory C1a = drive.trajectoryBuilder(C1.end())
                .splineToLinearHeading(new Pose2d(-5.0, 13.0, Math.toRadians(pos2)), Math.toRadians(0.0)) //8
                .build();

        Trajectory C1b = drive.trajectoryBuilder(C1a.end())
                .splineToLinearHeading(new Pose2d(-5.0, 14.0, Math.toRadians(pos3)), Math.toRadians(0.0)) //15
                .build();

        //INTAKE FLOOR RINGS
        Trajectory C2 = drive.trajectoryBuilder(C1b.end().plus(new Pose2d(0, 0, Math.toRadians(0))))
                .splineToSplineHeading(new Pose2d(56.0, 5.0, Math.toRadians(90.0)), Math.toRadians(90.0))
                .splineToSplineHeading(new Pose2d(59.0, 35.0, Math.toRadians(90.0)), Math.toRadians(90.0))
                .splineToSplineHeading(new Pose2d(59.0, 16.0, Math.toRadians(90.0)), Math.toRadians(-90.0))
                .splineToSplineHeading(new Pose2d(-5.0, 6.0, Math.toRadians(27.0)), Math.toRadians(180.0))
                .build();

        //Wobble POSITION
        Trajectory C3 = drive.trajectoryBuilder(C2.end())
                .splineToLinearHeading(new Pose2d(55.0, 45.0, Math.toRadians(5.0)), Math.toRadians(90.0))
                .build();

        //PARK
        Trajectory C4 = drive.trajectoryBuilder(C3.end(),true)
                .splineToSplineHeading(new Pose2d(10.0, 10.0, Math.toRadians(5.0)), Math.toRadians(180.0))
                .build();

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
        }

        waitForStart();
        if (isStopRequested()) return;

        // AUTO CASE STATEMENT
        switch (state) {
            case ZERO:
                telemetry.addData("Stack:", "ZERO");
                telemetry.update();

                //SET SERVOS
                robot.wobble_lift.setPosition(wobble_up);
                robot.wobble_claw.setPosition(wobble_close);
                robot.flap.setPosition(launch_angle);
                robot.kicker.setPosition(kicker_out);
                robot.flap.setPosition(0.174);
                ((DcMotorEx) robot.shooter1).setVelocity(1750);

                //SHOOT POSITION
                drive.followTrajectory(A1);

                //SHOOT POWERSHOTS
                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                drive.followTrajectory(A1a);
                //drive.turn(Math.toRadians(6.0));

                sleep(800);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                drive.followTrajectory(A1b);
                //drive.turn(Math.toRadians(6.0));

                sleep(800);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                //drive.turn(Math.toRadians(6.0));

                //TURN OFF SHOOTER
                ((DcMotorEx) robot.shooter1).setVelocity(0);
                ((DcMotorEx) robot.shooter2).setVelocity(0);

                //PICK UP RINGS
                robot.intake.setPower(0.9);
                robot.feeder_turn.setPower(1);

                drive.followTrajectory(A2);

                robot.intake.setPower(0);
                robot.feeder_turn.setPower(0);

                //SHOOT X 3
                ((DcMotorEx) robot.shooter1).setVelocity(1750);
                robot.flap.setPosition(0.121);

                sleep(1000);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                ((DcMotorEx) robot.shooter2).setVelocity(0);

                drive.followTrajectory(A3);

                //DROP WOBBLE 2
                robot.wobble_lift.setPosition(wobble_down);
                sleep(700);
                robot.wobble_claw.setPosition(wobble_open);
                sleep(200);
                robot.wobble_lift.setPosition(wobble_up);

                drive.followTrajectory(A4);

                PoseStorage.currentPose = drive.getPoseEstimate();

                break;
            case ONE:
                // B AUTO //
                telemetry.addData("Stack:", "ONE");
                telemetry.update();

                //SET SERVOS
                robot.wobble_lift.setPosition(wobble_up);
                robot.wobble_claw.setPosition(wobble_close);
                robot.flap.setPosition(launch_angle);
                robot.kicker.setPosition(kicker_out);
                robot.flap.setPosition(0.174);
                ((DcMotorEx) robot.shooter1).setVelocity(shooter_target_velo);

                //SHOOT POSITION
                drive.followTrajectory(B1);

                //SHOOT POWERSHOTS
                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                drive.followTrajectory(B1a);
                //drive.turn(Math.toRadians(6.0));

                sleep(800);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                drive.followTrajectory(B1b);
                //drive.turn(Math.toRadians(6.0));

                sleep(800);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                //drive.turn(Math.toRadians(6.0));

                //TURN OFF SHOOTER
                ((DcMotorEx) robot.shooter1).setVelocity(0);
                ((DcMotorEx) robot.shooter2).setVelocity(0);

                //PICK UP RINGS
                robot.intake.setPower(0.9);
                robot.feeder_turn.setPower(1);

                drive.followTrajectory(B2);

                robot.intake.setPower(0);
                robot.feeder_turn.setPower(0);

                //SHOOT X 3
                ((DcMotorEx) robot.shooter1).setVelocity(shooter_target_velo);
                robot.flap.setPosition(0.121);

                sleep(1000);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                ((DcMotorEx) robot.shooter2).setVelocity(0);

                drive.followTrajectory(B3);

                //DROP WOBBLE 2
                robot.wobble_lift.setPosition(wobble_down);
                sleep(700);
                robot.wobble_claw.setPosition(wobble_open);
                sleep(200);
                robot.wobble_lift.setPosition(wobble_up);

                drive.followTrajectory(B4);

                PoseStorage.currentPose = drive.getPoseEstimate();
                break;

            case FOUR:

                // C AUTO //
                telemetry.addData("Stack:", "FOUR");
                telemetry.update();

                //SET SERVOS
                robot.wobble_lift.setPosition(wobble_up);
                robot.wobble_claw.setPosition(wobble_close);
                robot.flap.setPosition(launch_angle);
                robot.kicker.setPosition(kicker_out);
                robot.flap.setPosition(0.174);
                ((DcMotorEx) robot.shooter1).setVelocity(shooter_target_velo);

                //SHOOT POSITION
                drive.followTrajectory(C1);

                //SHOOT POWERSHOTS
                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                drive.followTrajectory(C1a);
                //drive.turn(Math.toRadians(6.0));

                sleep(800);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                drive.followTrajectory(C1b);
                //drive.turn(Math.toRadians(6.0));

                sleep(800);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                //drive.turn(Math.toRadians(6.0));

                //TURN OFF SHOOTER
                ((DcMotorEx) robot.shooter1).setVelocity(0);
                ((DcMotorEx) robot.shooter2).setVelocity(0);

                //PICK UP RINGS
                robot.intake.setPower(0.9);
                robot.feeder_turn.setPower(1);

                drive.followTrajectory(C2);

                robot.intake.setPower(0);
                robot.feeder_turn.setPower(0);

                //SHOOT X 3
                ((DcMotorEx) robot.shooter1).setVelocity(shooter_target_velo);
                robot.flap.setPosition(0.121);

                sleep(1000);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                ((DcMotorEx) robot.shooter2).setVelocity(0);

                drive.followTrajectory(C3);

                //DROP WOBBLE 2
                robot.wobble_lift.setPosition(wobble_down);
                sleep(700);
                robot.wobble_claw.setPosition(wobble_open);
                sleep(200);
                robot.wobble_lift.setPosition(wobble_up);

                drive.followTrajectory(C4);

                PoseStorage.currentPose = drive.getPoseEstimate();
                break;
        }
    }
}