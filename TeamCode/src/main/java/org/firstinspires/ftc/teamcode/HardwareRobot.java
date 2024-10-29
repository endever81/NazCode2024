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
    public DcMotor liftleft = null;
    public DcMotor liftright = null;
    public DcMotor intake = null;

    public DcMotor Robot_Lift = null;
    public Servo Spinner = null;
    public Servo Flipper = null;
    public Servo Dumper = null;
    //public Servo droneGuard = null;
    public CRServo Extender = null;
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
        liftleft = hwMap.get(DcMotor.class, "lift_left");
        liftright = hwMap.get(DcMotor.class, "lift_right");
        intake = hwMap.get(DcMotor.class, "motor_intake");


        // 2024 Code
        Dumper = hwMap.get(Servo.class, "Dumper");
        Extender = hwMap.get(CRServo.class, "Extender");
        Spinner = hwMap.get(Servo.class, "Spinner");
        Flipper = hwMap.get(Servo.class, "Flipper");
        Robot_Lift = hwMap.get(DcMotor.class, "Robot_Lift");
        // robot lift is the acuator kit Anna built.

        // droneGuard = hwMap.get(Servo.class, "servo_drone_guard");

        //leftPickup = hwMap.get(CRServo.class, "servo_left_pickup");



        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        liftleft.setDirection(DcMotor.Direction.REVERSE);
        liftleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
        Extender.setPower(0);
        //Spinner.setPosition(0);
        Flipper.setPosition(.625);
        Dumper.setPosition(.55);
    }

}

