package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class HardwareRobot {

    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightRearDrive = null;
    public DcMotor liftup = null;
    public DcMotor liftout = null;
    public DcMotor intake = null;

    public DcMotor launcher = null;
    public Servo pickupwrist = null;

    public Servo servoDropper = null;
    public Servo droneGuard = null;
    public CRServo PickupSpecimen = null;
    public CRServo leftPickup = null;
    public RevBlinkinLedDriver blinkinLedDriver = null;




    //public Servo grabber = null;

    HardwareMap hwMap = null;
   
    private ElapsedTime period = new ElapsedTime();

    public HardwareRobot(){
    }
   
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap; //saves a reference Hardware Map

        leftFrontDrive = hwMap.get(DcMotor.class, "motor_front_left");
        rightFrontDrive = hwMap.get(DcMotor.class, "motor_front_right");
        leftRearDrive = hwMap.get(DcMotor.class, "motor_rear_left");
        rightRearDrive = hwMap.get(DcMotor.class, "motor_rear_right");
        liftup = hwMap.get(DcMotor.class, "lift_up");
        liftout = hwMap.get(DcMotor.class, "lift_out");
        intake = hwMap.get(DcMotor.class, "motor_intake");
        launcher = hwMap.get(DcMotor.class, "launcher");


        pickupwrist = hwMap.get(Servo.class, "servo_pickupwrist");
        servoDropper = hwMap.get(Servo.class, "servo_dropper");
        droneGuard = hwMap.get(Servo.class, "servo_drone_guard");

        //leftPickup = hwMap.get(CRServo.class, "servo_left_pickup");
        PickupSpecimen = hwMap.get(CRServo.class, "pickup_specimen");


        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        liftup.setDirection(DcMotor.Direction.REVERSE);
        liftup.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftout.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftup.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftout.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);

        pickupwrist.setPosition(0);
        servoDropper.setPosition(.6);
        droneGuard.setPosition(.5);
    }

}

