package org.firstinspires.ftc.teamcode.drive.opmode.BERSERK.Competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.BERSERK.HardwareBERSERK;
import org.firstinspires.ftc.teamcode.drive.opmode.BERSERK.PoseStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.Arrays;

import static com.arcrobotics.ftclib.vision.UGContourRingPipeline.Config;

@Autonomous(group = "BERSERK")
public class BLUE_FULL_AUTO_V3 extends LinearOpMode {

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
        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        robot.blinkinLedDriver.setPattern(pattern);

        double foldout = -1; //SET TO -1 TO FOLDOUT INTAKE, 0 TO DISABLE

        double shooter_target_velo = 1630;
        double launch_angle = 0.644; //0.173
        double kicker_out = 0.7;
        double kicker_in = 0.25; //02
        double wobble_close = 0.18;
        double wobble_open = 0.6;
        double wobble_up = 0.3;
        double wobble_down = 0.8;
        long shootWait = 330;
        double webcam_right = 0.1;
        double webcam_left = 0.3;

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

        Pose2d startPose = new Pose2d(-63.0,50, Math.toRadians(0.0));
        drive.setPoseEstimate(startPose);

        //   A AUTO TRAJECTORIES   //
        //SHOOT POSITION
        Trajectory A1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-25.0, 55.0), Math.toRadians(0.0))
                .addTemporalMarker(0.1, () -> {
                    robot.foldout_lift.setPower(foldout);
                    robot.intake.setPower(0.8);
                })
                .addTemporalMarker(1.8, () -> {
                    robot.foldout_lift.setPower(0);
                    robot.intake.setPower(0.0);
                })
                .splineTo(new Vector2d(-5.0, 41.0), Math.toRadians(2.0))
                .build();

        //WOBBLE A POSITION
        Trajectory A2 = drive.trajectoryBuilder(A1.end())
                .splineToLinearHeading(new Pose2d(5.0, 53.0, Math.toRadians(0.0)), Math.toRadians(0.0))
                .build();

        //MOVE TO WOBBLE 2
        Trajectory A3 = drive.trajectoryBuilder(A2.end())
                .strafeRight(10)
                //   .splineTo(new Vector2d(-32.0, 24.0),Math.toRadians(80))
                //   .splineTo(new Vector2d(-32.0, 24.0),Math.toRadians(-10))
                .splineTo(new Vector2d(-20.0, 18.0),Math.toRadians(350))
                .addTemporalMarker(2.0, () -> {
                    robot.wobble_lift.setPosition(wobble_down);
                })
                //  .splineToConstantHeading(new Vector2d(-40, 25.0),Math.toRadians(80),
                .splineToConstantHeading(new Vector2d(-41.5, 21.5),Math.toRadians(80),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                //  new MecanumVelocityConstraint(7, DriveConstants.TRACK_WIDTH)
                                new MecanumVelocityConstraint(12, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //WOBBLE A POSITION
        Trajectory A4 = drive.trajectoryBuilder(A3.end())
                .splineToLinearHeading(new Pose2d(10.0, 44.0, Math.toRadians(0.0)), Math.toRadians(0.0))
                .build();

        //PARK
        Trajectory A5 = drive.trajectoryBuilder(A4.end())
                .strafeRight(5)
                .build();

        //   B AUTO TRAJECTORIES   //
        //SHOOT POSITION
        Trajectory B1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-25.0, 55.0), Math.toRadians(0.0))
                .addTemporalMarker(0.1, () -> {
                    robot.foldout_lift.setPower(foldout);
                    robot.intake.setPower(0.8);
                })
                .addTemporalMarker(1.8, () -> {
                    robot.foldout_lift.setPower(0);
                    robot.intake.setPower(0.0);
                })
                .splineTo(new Vector2d(-5.0, 43.0), Math.toRadians(0.0))
                .build();

        //WOBBLE B POSITION
        Trajectory B2 = drive.trajectoryBuilder(B1.end())
                .splineToLinearHeading(new Pose2d(29.0, 34.0, Math.toRadians(-90.0)), Math.toRadians(0.0))
                .build();

        //MOVE TOWARDS STACK
        //  Trajectory B3 = drive.trajectoryBuilder(B2.end(),true)
        //         .splineToLinearHeading(new Pose2d(-19.0, 42.0, Math.toRadians(180.0)), Math.toRadians(180.0))
        //          .build();

        //PICKUP WOBBLE
        Trajectory B4 = drive.trajectoryBuilder(B2.end(),true)
                .strafeRight(10)
                .splineToSplineHeading(new Pose2d(-19.0, 39.0, Math.toRadians(180.0)), Math.toRadians(180.0))
                .splineToSplineHeading( new Pose2d(-35.0, 25.0, Math.toRadians(115.0)), Math.toRadians(180)) //135
                .splineToConstantHeading( new Vector2d(-39.0, 26.0), Math.toRadians(100),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(8, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(8))
                .build();

        //SHOOT
        Trajectory B5 = drive.trajectoryBuilder(B4.end())
                .splineToLinearHeading( new Pose2d(-5.0,40.0, Math.toRadians(0.0)), Math.toRadians(0.0))
                .build();

        //DROP WOBBLE
        Trajectory B6 = drive.trajectoryBuilder(B5.end())
                .splineToLinearHeading( new Pose2d(20.0, 42.0, Math.toRadians(-90.0)), Math.toRadians(0.0))
                .build();

        //PARK
        Trajectory B7 = drive.trajectoryBuilder(B6.end())
                .splineToLinearHeading( new Pose2d(6.0,34.0, Math.toRadians(0.0)), Math.toRadians(-90.0))
                .build();

        //   C AUTO TRAJECTORIES   //
        //SHOOT POSITIONadb connect 192.168.43.1
        Trajectory C1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-42,35, Math.toRadians(3.0)))
                .addTemporalMarker(0.1, () -> {
                    robot.foldout_lift.setPower(foldout);
                    robot.intake.setPower(0.8);
                })
                .addTemporalMarker(1.8, () -> {
                    robot.foldout_lift.setPower(0);
                    robot.intake.setPower(0.0);
                })
                .build();

        //INTAKE FIRST 2
        Trajectory C2 = drive.trajectoryBuilder(C1.end())
                .forward(12,
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(7, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(7))
                .build();

        //INTAKE SECOND 2
        Trajectory C3 = drive.trajectoryBuilder(C2.end())
                .forward(18,
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(7, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(7))
                .build();

        //WOBBLE C POSITION
        //   Trajectory C4 = drive.trajectoryBuilder(C3.end())
        //           .build();

        //PICK UP WOBBLE 2
        Trajectory C5 = drive.trajectoryBuilder(C3.end())

                //Approach Zone C
                .splineToSplineHeading( new Pose2d(38.0, 48.0, Math.toRadians(0)), Math.toRadians(0),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(70, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(60))

                //Lower Wobble Arm
                .addSpatialMarker(new Vector2d(30, 50), () -> {
                    robot.wobble_lift.setPosition(wobble_down);
                })

                //Pass Zone C
                //  .splineToSplineHeading( new Pose2d(52.0, 50.0, Math.toRadians(0)), Math.toRadians(0),
                //          new MinVelocityConstraint(Arrays.asList(
                //                  new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                //                 new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                //         )
                //         ), new ProfileAccelerationConstraint(10))

                //Open Wobble Claw
                .addSpatialMarker(new Vector2d(55, 48), () -> {
                    robot.wobble_claw.setPosition(wobble_open);
                })

                //Approach Wobble 2
                .splineToSplineHeading( new Pose2d(20.0, 12, Math.toRadians(90)), Math.toRadians(180),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(70, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(60))

                //Continue to Approach Wobble 2
                .splineToSplineHeading( new Pose2d(-36.0, 12, Math.toRadians(90)), Math.toRadians(180),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(70, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(60))

                //Slow and Grab Wobble 2
                //     .splineToSplineHeading( new Pose2d(-34.0, 15.0, Math.toRadians(90)), Math.toRadians(180),
                //             new MinVelocityConstraint(Arrays.asList(
                //                     new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                //                     new MecanumVelocityConstraint(8, DriveConstants.TRACK_WIDTH)
                //             )
                //             ), new ProfileAccelerationConstraint(8))
                .build();

        //WOBBLE C POSITION
        Trajectory C6 = drive.trajectoryBuilder(C5.end())
                .splineToSplineHeading( new Pose2d(42.0, 56.0, Math.toRadians(-45.0)), Math.toRadians(0),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(70, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(60))
                .build();

        //PARK
        Trajectory C7 = drive.trajectoryBuilder(C6.end(),true)
                .splineToLinearHeading( new Pose2d(15.0,34.0, Math.toRadians(5.0)), Math.toRadians(180.0),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(70, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(60))
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
                robot.kicker.setPosition(kicker_out);
                robot.wobble_lift.setPosition(wobble_up);
                robot.wobble_claw.setPosition(wobble_close);
                robot.flap.setPosition(launch_angle);
                ((DcMotorEx) robot.shooter1).setVelocity(shooter_target_velo);

                //SHOOT POSITION
                drive.followTrajectory(A1);

                //SHOOT x 3
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

                //TURN OFF SHOOTER
                ((DcMotorEx) robot.shooter1).setVelocity(0);
                ((DcMotorEx) robot.shooter2).setVelocity(0);

                //WOBBLE A POSITION
                drive.followTrajectory(A2);

                //DROP WOBBLE 1
                robot.wobble_lift.setPosition(wobble_down);
                sleep(700);
                robot.wobble_claw.setPosition(wobble_open);
                sleep(400);
                robot.wobble_lift.setPosition(wobble_up);

                drive.followTrajectory(A3);

                //GRAB WOBBLE 1
                robot.wobble_claw.setPosition(wobble_close);
                sleep(700);
                robot.wobble_lift.setPosition(wobble_up);
                sleep(800);

                drive.followTrajectory(A4);

                //DROP WOBBLE 2
                robot.wobble_lift.setPosition(wobble_down);
                sleep(700);
                robot.wobble_claw.setPosition(wobble_open);
                sleep(200);
                robot.wobble_lift.setPosition(wobble_up);

                drive.followTrajectory(A5);
                PoseStorage.currentPose = drive.getPoseEstimate();

                break;
            case ONE:
                // B AUTO //
                telemetry.addData("Stack:", "ONE");
                telemetry.update();

                //SET SERVOS
                robot.kicker.setPosition(kicker_out);
                robot.wobble_lift.setPosition(wobble_up);
                robot.wobble_claw.setPosition(wobble_close);
                robot.flap.setPosition(launch_angle);

                ((DcMotorEx) robot.shooter1).setVelocity(shooter_target_velo);

                //SHOOT POSITION
                drive.followTrajectory(B1);

                //SHOOT x 3
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

                //TURN OFF SHOOTER
                ((DcMotorEx) robot.shooter1).setVelocity(0);
                ((DcMotorEx) robot.shooter2).setVelocity(0);

                //WOBBLE B POSITION
                drive.followTrajectory(B2);

                //DROP WOBBLE 1
                robot.wobble_lift.setPosition(wobble_down);
                sleep(700);
                robot.wobble_claw.setPosition(wobble_open);
                sleep(200);

                //MOVE TOWARDS STACK
                robot.intake.setPower(0.8);
                robot.feeder_turn.setPower(1);
                //   drive.followTrajectory(B3);

                //PICKUP WOBBLE
                drive.followTrajectory(B4);
                robot.wobble_claw.setPosition(wobble_close);
                sleep(700);
                robot.wobble_lift.setPosition(wobble_up);
                sleep(190);

                //SHOOT POSITION
                ((DcMotorEx) robot.shooter1).setVelocity(shooter_target_velo);
                drive.followTrajectory(B5);

                //SHOOT X 1
                sleep(1500);

                robot.kicker.setPosition(kicker_out);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                robot.kicker.setPosition(kicker_out);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                //TURN OFF INTAKE AND SHOOTER
                robot.intake.setPower(0);
                robot.feeder_turn.setPower(0);
                ((DcMotorEx) robot.shooter1).setVelocity(0);
                ((DcMotorEx) robot.shooter2).setVelocity(0);

                //DROP WOBBLE 2
                drive.followTrajectory(B6);
                robot.wobble_lift.setPosition(wobble_down);
                sleep(700);
                robot.wobble_claw.setPosition(wobble_open);
                sleep(400);
                robot.wobble_lift.setPosition(wobble_up);

                //PARK
                drive.followTrajectory(B7);

                PoseStorage.currentPose = drive.getPoseEstimate();
                break;

            case FOUR:

                // C AUTO //
                telemetry.addData("Stack:", "FOUR");
                telemetry.update();

                //SET SERVOS
                robot.kicker.setPosition(kicker_out);
                robot.wobble_lift.setPosition(wobble_up);
                robot.wobble_claw.setPosition(wobble_close);
                robot.flap.setPosition(launch_angle);
                ((DcMotorEx) robot.shooter1).setVelocity(1690); //1820

                //SHOOT POSITION
                drive.followTrajectory(C1);

                //SHOOT x 3
                sleep(300);
                robot.kicker.setPosition(kicker_out);
                sleep(180);
                robot.kicker.setPosition(kicker_in);
                sleep(180);

                robot.kicker.setPosition(kicker_out);
                sleep(180);
                robot.kicker.setPosition(kicker_in);
                sleep(180);

                robot.kicker.setPosition(kicker_out);
                sleep(180);
                robot.kicker.setPosition(kicker_in);
                sleep(180);
                robot.kicker.setPosition(kicker_out);

                robot.intake.setPower(0.75);
                robot.feeder_turn.setPower(1);

                //INTAKE 2
                drive.followTrajectory(C2);

                sleep(800);

                robot.kicker.setPosition(kicker_out);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);

                robot.kicker.setPosition(kicker_out);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);

                //INTAKE 2 MORE
                drive.followTrajectory(C3);

                ((DcMotorEx) robot.shooter1).setVelocity(1650); //1820

                sleep(600);
                robot.kicker.setPosition(kicker_out);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);

                robot.kicker.setPosition(kicker_out);
                sleep(450);
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

                robot.intake.setPower(0);
                robot.feeder_turn.setPower(0);
                ((DcMotorEx) robot.shooter1).setVelocity(0);

                //PICKUP WOBBLE 2
                drive.followTrajectory(C5);

                //GRAB WOBBLE 2
                robot.wobble_claw.setPosition(wobble_close);
                sleep(700);
                robot.wobble_lift.setPosition(wobble_up);
                sleep(800);

                //WOBBLE C POSITION
                drive.followTrajectory(C6);

                //DROP WOBBLE 2
                robot.wobble_lift.setPosition(wobble_down);
                sleep(400);
                robot.wobble_claw.setPosition(wobble_open);
                sleep(200);
                robot.wobble_lift.setPosition(wobble_up);

                //PARK
                drive.followTrajectory(C7);
                PoseStorage.currentPose = drive.getPoseEstimate();
                break;
        }
    }
}