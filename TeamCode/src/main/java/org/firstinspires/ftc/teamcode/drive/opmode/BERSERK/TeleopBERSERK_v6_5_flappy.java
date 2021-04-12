package org.firstinspires.ftc.teamcode.drive.opmode.BERSERK;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

//Teleop utilizes Align to point and Shooter Velocity PID Control

//@Config
@TeleOp(group = "BERSERK")
@Disabled
public class TeleopBERSERK_v6_5_flappy extends LinearOpMode {

    public static double DRAWING_TARGET_RADIUS = 1; //2

    enum Mode {
        NORMAL_CONTROL,
        AUTO_POWERSHOT,
        ALIGN_TO_POINT
    }

    enum ShooterState {
        RESTING,
        FLICKING,
        DRAWING_BACK
        }

    ShooterState shooterState = ShooterState.RESTING;

    private Mode currentMode = Mode.NORMAL_CONTROL;
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    RevBlinkinLedDriver.BlinkinPattern pattern;

    //Target Position of TowerGoal
    private Vector2d targetPosition = new Vector2d(72, 37);

    //Shooter Velocity PID
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.003, 0, 0);
    public static double kV = 0.00039;
    public static double kA = 0.00015;
    public static double kStatic = 0;
    private double lastTargetVelo = 0.0;
    private final PIDFController veloController = new PIDFController(MOTOR_VELO_PID, kV, kA, kStatic);

    //Timers
    private final ElapsedTime veloTimer = new ElapsedTime();
    private final ElapsedTime shooterTimer = new ElapsedTime();
    private final ElapsedTime alignTimer = new ElapsedTime();
    private final ElapsedTime ringTimer = new ElapsedTime();
    private final ElapsedTime ringTimer2 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        //Initializations
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareBERSERK robot       = new HardwareBERSERK();
        robot.init(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        // Motor Setup //
        DcMotorEx myMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx myMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        myMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //Start Pose from PoseStorage      Left tape: (new Pose2d(-63,50));
        drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);
        headingController.setInputBounds(-Math.PI, Math.PI);

        pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        robot.blinkinLedDriver.setPattern(pattern);

        waitForStart();

        ////VARIABLES\\\\\

        //Flap
        double launch_angle = 0.121;
        double launch_angle_offset = 0;
        double max_launch_angle = 0.174;
        double min_launch_angle = 0.02;

        //Kicker
        double kicker_out = 0.68;
        double kicker_in = 0.4; //0.2 //0.3

        //Wobble Claw
        double wobble_close = 0.18;
        double wobble_open = 0.6;

        //Wobble Lift
        double wobble_up = 0.6;
        double wobble_down = 0.2;

        //Initial Shooter Velocity
        double targetVelo = 0;
        double targetVelo_offset= 0;

        //Ring Counter
        boolean ringState = false;

        //Set Servos
        robot.wobble_lift.setPosition(wobble_up);
        robot.wobble_claw.setPosition(wobble_open);
        robot.kicker.setPosition(kicker_out);

        if (isStopRequested()) return;

        // Start the veloTimer
        veloTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {

            //Gamepad Input
            double ly = -gamepad1.left_stick_y;
            double lx = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            //Initialize Localizer
            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();
            Pose2d driveDirection = new Pose2d();

            //Initialize FTC Dashboard
            Canvas fieldOverlay = packet.fieldOverlay();
            //TelemetryPacket packet = new TelemetryPacket();

            // Call necessary controller methods
            veloController.setTargetPosition(targetVelo + targetVelo_offset);
            veloController.setTargetVelocity(targetVelo + targetVelo_offset);
            veloController.setTargetAcceleration((targetVelo + targetVelo_offset - lastTargetVelo) / veloTimer.seconds());
            veloTimer.reset();

            //(Math.min(Math.max(launch_angle + launch_angle_offset, min_launch_angle), max_launch_angle))

            lastTargetVelo = targetVelo + targetVelo_offset;

            // Get the velocity from the motor with the encoder
            double motorVelo = myMotor1.getVelocity();

            // Update the controller and set the power for each motor
            double power = veloController.update(motorVelo);
            myMotor1.setPower(power);
            myMotor2.setPower(power);

            //// Gamepad Controls \\\\

            //Intake + Indexer
            if (gamepad1.right_bumper) {
                robot.intake.setPower(.80);
                robot.feeder_turn.setPower(1);
            } else if (gamepad1.left_bumper) {
                robot.intake.setPower(0);
                robot.feeder_turn.setPower(0);
            }

            //Gamepad 2 Right Bumper reverses intake
            if (gamepad2.right_bumper) {
                robot.intake.setPower(-0.8);
                robot.feeder_turn.setPower(-1);
            }

            //Shooter
            if (gamepad1.a || gamepad2.a) {
                targetVelo = 1760; //1700
            } else if (gamepad1.b || gamepad2.b) {
                targetVelo = 0;
            }

            //Distance to Tower
            double getDistance = Math.sqrt(Math.pow(targetPosition.getX() - poseEstimate.getX(), 2) + Math.pow(targetPosition.getY() - poseEstimate.getY(), 2));

            if (getDistance > 74 && getDistance < 85) {
                launch_angle = 0.14;
            }
            if (getDistance > 85 && getDistance < 95) {
                launch_angle = 0.13;
            }
            if (getDistance > 95 && getDistance < 105) {
                launch_angle = 0.12;
            }
            if (getDistance > 105 && getDistance < 115) {
                launch_angle = 0.1;
            }


            //Backwall= 133 in, Shooting Spot= 74 in

            //Flap Set Position
            robot.flap.setPosition(Math.min(Math.max(launch_angle + launch_angle_offset, min_launch_angle), max_launch_angle));

            //Flap Manual Offset
            if (gamepad1.dpad_up) {
                launch_angle_offset += -0.00013;
            } else if (gamepad1.dpad_down) {
                launch_angle_offset += 0.00013;
            } else if (gamepad1.dpad_right) {
                launch_angle_offset = 0.0;
            }

            if (gamepad1.dpad_right) {
                launch_angle = 0.121;
            } else if (gamepad1.dpad_left) {
                launch_angle = 0.147;
            }

            // Y prepares for endgame by dropping flap and powering up shooter
            if (gamepad1.y) {
                targetVelo = 1700;
                targetVelo_offset = 0;
                launch_angle_offset = 0.174;
            }

            //Wobble Arm
            if (gamepad2.dpad_up) {
                robot.wobble_lift.setPosition(wobble_up);
            } else if (gamepad2.dpad_down) {
                robot.wobble_lift.setPosition(wobble_down);
            } else if (gamepad2.dpad_left) {
                robot.wobble_claw.setPosition(wobble_open);
            } else if (gamepad2.dpad_right) {
                robot.wobble_claw.setPosition(wobble_close);
            }

            //Fold-Out Lift
            if (gamepad2.right_stick_y < 0) {
                robot.foldout_lift.setPower(1);
            }
            else if (gamepad2.right_stick_y > 0) {
                robot.foldout_lift.setPower(-1);
            }
            else robot.foldout_lift.setPower(0);

            //Turn LEDS blue when no ring is present & LEDs haven't changed in 1 sec & if gamepad x is not being pressed
            if(robot.color_sensor.alpha() < 80 && ringTimer.seconds() > 1){
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                robot.blinkinLedDriver.setPattern(pattern);
                ringTimer2.reset();
            }

            //Turn LEDS red when  ring is present & LEDs haven't changed in 1 sec
            if(robot.color_sensor.alpha() > 80 && ringTimer2.seconds() > 1 ) { //150
                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                robot.blinkinLedDriver.setPattern(pattern);
                ringTimer.reset();
            }

            switch(shooterState){
                case RESTING:
                    if(gamepad1.x){
                        shooterTimer.reset();
                        shooterState = ShooterState.FLICKING;

                        //pattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;
                        robot.blinkinLedDriver.setPattern(pattern);
                    }
                    break;
                case FLICKING:
                    if(shooterTimer.milliseconds() > 120){ //150
                        shooterTimer.reset();
                        shooterState = ShooterState.DRAWING_BACK;
                    }
                    break;
                case DRAWING_BACK:
                    if(shooterTimer.milliseconds() > 120){ //150
                        shooterTimer.reset();
                        shooterState = ShooterState.RESTING;
                    }
                    break;
            }

            robot.kicker.setPosition(shooterState == ShooterState.FLICKING ? kicker_in : kicker_out);

            // MODE SWITCH \\
            switch (currentMode) {
                case NORMAL_CONTROL:

                    //Normal Robot Control
                    driveDirection = new Pose2d(
                            Math.signum(ly) * ly * ly * 0.85,
                            Math.signum(lx) * lx * lx * 0.85,
                            Math.signum(rx) * rx * rx * 0.8
                    );

                    // Switch to align to point if gamepad1 left trigger is activated
                    if (Math.abs(gamepad1.left_trigger) >= 0.2) {
                        currentMode = Mode.ALIGN_TO_POINT;
                    }
                    break;

                case AUTO_POWERSHOT:

                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.dpad_right) {
                        drive.cancelFollowing();
                        currentMode = Mode.NORMAL_CONTROL;
                    }
                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.NORMAL_CONTROL;
                    }
                    break;

                case ALIGN_TO_POINT: // Switch to normal control if gamepad1 right trigger is activated

                    alignTimer.reset();
                    if (Math.abs(gamepad1.right_trigger) >= 0.2) {
                    //if (alignTimer.milliseconds() > 1000) {
                        currentMode = Mode.NORMAL_CONTROL;
                    }

                    Vector2d fieldFrameInput = new Vector2d(
                            Math.signum(ly)  * ly * ly * 0.8,
                            Math.signum(lx)  * lx * lx * 0.8
                    );

                    //Align to Point Movement
                    Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());
                    Vector2d difference = targetPosition.minus(poseEstimate.vec());
                    double theta = difference.angle();
                    double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());
                    headingController.setTargetPosition(theta);

                    double headingInput = (headingController.update(poseEstimate.getHeading())
                            * DriveConstants.kV + thetaFF)
                            * DriveConstants.TRACK_WIDTH;

                    driveDirection = new Pose2d(
                            robotFrameInput,
                            headingInput
                    );

                    //Dashboard Setup
                    fieldOverlay.setStroke("#dd2c00");
                    fieldOverlay.strokeCircle(targetPosition.getX(), targetPosition.getY(), DRAWING_TARGET_RADIUS);
                    fieldOverlay.setStroke("#b89eff");
                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
                    fieldOverlay.setStroke("#ffce7a");
                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), targetPosition.getX(), poseEstimate.getY());
                    fieldOverlay.strokeLine(targetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());
                    break;
            }

            //Dashboard View
            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate);
            drive.setWeightedDrivePower(driveDirection);
            headingController.update(poseEstimate.getHeading());
            drive.getLocalizer().update();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            //Driver Station Telemetry
            
            //telemetry.addData("Alpha", robot.color_sensor.alpha());
            telemetry.addData("mode", currentMode);
            telemetry.addData("launch angle", launch_angle+launch_angle_offset);
            telemetry.addData("Distance", getDistance);
            telemetry.addData("Flywheel", motorVelo);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            //Dashboard Telemetry
          //  packet.put("FlyWheel Velocity", motorVelo);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
