
/*
package org.firstinspires.ftc.teamcode.OldCode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Disabled

//@Autonomous(name = "AutoRed1", group = "Automonous")

public class StandardAutonBase extends LinearOpMode{
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private TfodProcessor tfod;
    private VisionPortal visionPortal;


    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // Neverest 40:1
    static final double DRIVE_GEAR_REDUCTION = .5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.2;
    static final double STRAFE_SPEED = 0.2;
    static final double TURN_SPEED = 0.2;

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.02;     // Larger is more responsive, but also less stable
    //int size = 0;



    // Calls up IMU (Inertial Measruement Unit within REV Hub)
    BNO055IMU imu;

    Orientation angles;

    HardwareRobot robot = new HardwareRobot();



    @Override
    public void runOpMode() {


        robot.init(hardwareMap);

        initTfod();



        //IMU Initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //sensorDistance = hardwareMap.get(DistanceSensor.class, "left_distance_sensor");
        //sensorDistanceBack = hardwareMap.get(DistanceSensor.class, "back_distance");

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }


        telemetry.addData("Wheel Encoders", "Starting at %7d :%7d :%7d :%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition(),
                robot.leftRearDrive.getCurrentPosition(),
                robot.rightRearDrive.getCurrentPosition()
        );


        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        //telemetry.addData("Distance (cm)",String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));


        telemetry.update();


        waitForStart();


//release pressure on the wheel
//servoWobble.setPosition(.7);

        int x = 0;
        if (opModeIsActive()) {
            //  while (opModeIsActive()) {
            if (tfod != null) {

                List<Recognition> updatedRecognitions = tfod.getRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    x = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());


                        if (recognition.getLabel() == "1 Bolt") {
                            x = 1;
                        }
                        if (recognition.getLabel() == "2 Bulb") {
                            x = 2;
                        }
                        if (recognition.getLabel() == "3 Panel") {
                            x = 3;
                        }

                    }


                }
                telemetry.update();
            }

        }


        if (x == 1) {
            telemetry.addData(">", "Left");
            telemetry.update();
            //duck is on left and place block on bottom

            //robot.grabber.setPosition(0);

            sleep(750);


            gyroDrive(.3, 13, 0);
            gyroTurn(.3, 199.5);
            sleep(1000);

        }


        if (x == 2) {
            telemetry.addData(">", "Center");
            telemetry.update();
            //duck is on middle and place block on middle
            //robot.grabber.setPosition(0);

        }


        if (x == 3) {
            telemetry.addData(">", "Right");
            telemetry.update();
            //duck is on right and place block on top

            //robot.grabber.setPosition(0);
        }


    }




    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.

        robot.rightFrontDrive.setPower(rightSpeed);
        robot.leftFrontDrive.setPower(leftSpeed);
        robot.rightRearDrive.setPower(rightSpeed);
        robot.leftRearDrive.setPower(leftSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }


    public void gyroDrive ( double speed,  double distance,  double angle) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newFrontRightTarget = robot.rightFrontDrive.getCurrentPosition() + moveCounts;
            newFrontLeftTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
            newRearRightTarget = robot.rightRearDrive.getCurrentPosition() + moveCounts;
            newRearLeftTarget = robot.leftRearDrive.getCurrentPosition() + moveCounts;

            robot.rightFrontDrive.setTargetPosition(newFrontRightTarget);
            robot.leftFrontDrive.setTargetPosition(newFrontLeftTarget);
            robot.rightRearDrive.setTargetPosition(newRearRightTarget);
            robot.leftRearDrive.setTargetPosition(newRearLeftTarget);

            // Turn On RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.rightFrontDrive.setPower(speed);
            robot.leftFrontDrive.setPower(speed);
            robot.rightRearDrive.setPower(speed);
            robot.leftRearDrive.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.rightFrontDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.rightRearDrive.isBusy()
                            && robot.leftRearDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;



                //Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }





                robot.rightFrontDrive.setPower(rightSpeed);
                robot.leftFrontDrive.setPower(leftSpeed);
                robot.rightRearDrive.setPower(rightSpeed);
                robot.leftRearDrive.setPower(leftSpeed);


                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontRightTarget, newFrontLeftTarget,
                        newRearRightTarget, newRearLeftTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.rightFrontDrive.getCurrentPosition(),
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightRearDrive.getCurrentPosition(),
                        robot.leftRearDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Heading", angles.firstAngle);
                //telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motion;
            robot.rightFrontDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightRearDrive.setPower(0);
            robot.leftRearDrive.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


    }

    public void gyroDriveNoBrake ( double speed,  double distance,  double angle) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newFrontRightTarget = robot.rightFrontDrive.getCurrentPosition() + moveCounts;
            newFrontLeftTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
            newRearRightTarget = robot.rightRearDrive.getCurrentPosition() + moveCounts;
            newRearLeftTarget = robot.leftRearDrive.getCurrentPosition() + moveCounts;

            robot.rightFrontDrive.setTargetPosition(newFrontRightTarget);
            robot.leftFrontDrive.setTargetPosition(newFrontLeftTarget);
            robot.rightRearDrive.setTargetPosition(newRearRightTarget);
            robot.leftRearDrive.setTargetPosition(newRearLeftTarget);

            // Turn On RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.rightFrontDrive.setPower(speed);
            robot.leftFrontDrive.setPower(speed);
            robot.rightRearDrive.setPower(speed);
            robot.leftRearDrive.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.rightFrontDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.rightRearDrive.isBusy()
                            && robot.leftRearDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;



                //Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }





                robot.rightFrontDrive.setPower(rightSpeed);
                robot.leftFrontDrive.setPower(leftSpeed);
                robot.rightRearDrive.setPower(rightSpeed);
                robot.leftRearDrive.setPower(leftSpeed);


                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontRightTarget, newFrontLeftTarget,
                        newRearRightTarget, newRearLeftTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.rightFrontDrive.getCurrentPosition(),
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightRearDrive.getCurrentPosition(),
                        robot.leftRearDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Heading", angles.firstAngle);
                //telemetry.addData("Correction", correction);
                telemetry.update();
            }




            // Turn off RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


    }

    public void gyroStrafe ( double speed,  double distance,  double angle) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newFrontRightTarget = robot.rightFrontDrive.getCurrentPosition() - moveCounts;
            newFrontLeftTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
            newRearRightTarget = robot.rightRearDrive.getCurrentPosition() + moveCounts;
            newRearLeftTarget = robot.leftRearDrive.getCurrentPosition() - moveCounts;


            robot.rightFrontDrive.setTargetPosition(newFrontRightTarget);
            robot.leftFrontDrive.setTargetPosition(newFrontLeftTarget);
            robot.rightRearDrive.setTargetPosition(newRearRightTarget);
            robot.leftRearDrive.setTargetPosition(newRearLeftTarget);

            // Turn On RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.rightFrontDrive.setPower(speed);
            robot.leftFrontDrive.setPower(speed);
            robot.rightRearDrive.setPower(speed);
            robot.leftRearDrive.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.rightFrontDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.rightRearDrive.isBusy()
                            && robot.leftRearDrive.isBusy()))  {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;



                //Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.rightFrontDrive.setPower(leftSpeed);
                robot.leftFrontDrive.setPower(speed);
                robot.rightRearDrive.setPower(speed);
                robot.leftRearDrive.setPower(rightSpeed);



                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontRightTarget, newFrontLeftTarget,
                        newRearRightTarget, newRearLeftTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.rightFrontDrive.getCurrentPosition(),
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightRearDrive.getCurrentPosition(),
                        robot.leftRearDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Heading", angles.firstAngle);
                //telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motion;
            robot.rightFrontDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightRearDrive.setPower(0);
            robot.leftRearDrive.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        }

    }




    /**
     * Initialize the Vuforia localization engine.
     */

/*
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */

/*
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()






    boolean onHeadingShoot(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);
        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }
        // Send desired speeds to motors.

        robot.rightFrontDrive.setPower(rightSpeed);
        robot.leftFrontDrive.setPower(leftSpeed);
        robot.rightRearDrive.setPower(rightSpeed);
        robot.leftRearDrive.setPower(leftSpeed);



//motorfeeder.setPower(1);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }



    public void lift(double power, double inches)
    {
        int newLiftTarget;

        if (opModeIsActive()) {

            //   newLiftTarget = robot.arm.getCurrentPosition() + (int) (inches * (1140/(3.5 * 3.1415)));

            //  robot.arm.setTargetPosition(newLiftTarget);

            //   robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            //   robot.arm.setPower(Math.abs(power));

            while (opModeIsActive() &&
                    //               robot.arm.isBusy()) {
                    //   telemetry.addData("Lift", "Running at %7d",
                    //                   robot.arm.getCurrentPosition());
                    telemetry.update());

        }
        //  robot.arm.setPower(0);
        //  robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}






*/