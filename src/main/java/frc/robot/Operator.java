package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;

public final class Operator {
  //public static final XboxController controller = new XboxController(OperatorConstants.kOperatorControllerPort);
  public static final CommandXboxController Controller = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
  public static double getLeftX() { return Controller.getLeftX();}
  public static double getLeftY() { return Controller.getLeftY();}
  public static double getLeftTrigger() { return Controller.getLeftTriggerAxis();}
  
  public static double getRightX() { return Controller.getRightX();}
  public static double getRightY() { return Controller.getRightY();}
  public static double getRightTrigger() { return Controller.getRightTriggerAxis();}

  //Reefscape custom controller mapping
  public static final CommandGenericHID CustomController = new CommandGenericHID(OperatorConstants.kCustomControllerPort);
  public static final int CUST_CONTROLLER_BUTTON_1  = 1;
  public static final int CUST_CONTROLLER_BUTTON_2  = 2;
  public static final int CUST_CONTROLLER_BUTTON_3  = 3;
  public static final int CUST_CONTROLLER_BUTTON_4  = 4;
  public static final int CUST_CONTROLLER_BUTTON_5  = 5;
  public static final int CUST_CONTROLLER_BUTTON_6  = 6;
  public static final int CUST_CONTROLLER_BUTTON_7  = 7;
  public static final int CUST_CONTROLLER_BUTTON_8  = 8;
  public static final int CUST_CONTROLLER_BUTTON_9  = 9;
  public static final int CUST_CONTROLLER_BUTTON_10 = 10;
  public static final int CUST_CONTROLLER_BUTTON_11 = 11;
  public static final int CUST_CONTROLLER_BUTTON_12 = 12;

  public static Trigger getButton1() {
    return CustomController.button(CUST_CONTROLLER_BUTTON_1);
  }
  public static Trigger getButton2() {
    return CustomController.button(CUST_CONTROLLER_BUTTON_2);
  }
  public static Trigger getButton3() {
    return CustomController.button(CUST_CONTROLLER_BUTTON_3);
  }
  public static Trigger getButton4() {
    return CustomController.button(CUST_CONTROLLER_BUTTON_4);
  }
  public static Trigger getButton5() {
    return CustomController.button(CUST_CONTROLLER_BUTTON_5);
  }
  public static Trigger getButton6() {
    return CustomController.button(CUST_CONTROLLER_BUTTON_6);
  }
  public static Trigger getButton7() {
    return CustomController.button(CUST_CONTROLLER_BUTTON_7);
  }
  public static Trigger getButton8() {
    return CustomController.button(CUST_CONTROLLER_BUTTON_8);
  }
  public static Trigger getButton9() {
    return CustomController.button(CUST_CONTROLLER_BUTTON_9);
  }
  public static Trigger getButton10() {
    return CustomController.button(CUST_CONTROLLER_BUTTON_10);
  }
  public static Trigger getButton11() {
    return CustomController.button(CUST_CONTROLLER_BUTTON_11);
  }
  public static Trigger getButton12() {
    return CustomController.button(CUST_CONTROLLER_BUTTON_12);
  }
  
  //TODO: Create methods for the other buttons by following this convention

  /*public static final class XboxButtons {
    
    // Left side controls
    public static final JoystickButton LeftStick = new JoystickButton(Controller,XboxController.Button.kLeftStick.value);
    public static final JoystickButton LeftBumper = new JoystickButton(Controller, XboxController.Button.kLeftBumper.value);
    public static final  


    // Right side controls
    public static final JoystickButton RightStick = new JoystickButton(Controller, XboxController.Button.kRightStick.value);
    public static final JoystickButton RightBumper = new JoystickButton(Controller, XboxController.Button.kRightBumper.value);*/ 
    
    /*  Logitech 310 Buttons
    __                      _______
      \____________________/        \
      [Back]        [Start]    Y    |
             [Logi]          X + B  |
      [Mode]                   A    |

    */
    
    // Action Buttons
    /*public static final JoystickButton A = new JoystickButton(Controller, XboxController.Button.kA.value); 
    public static final JoystickButton B = new JoystickButton(Controller, XboxController.Button.kB.value); 
    public static final JoystickButton X = new JoystickButton(Controller, XboxController.Button.kX.value); 
    public static final JoystickButton Y = new JoystickButton(Controller, XboxController.Button.kY.value); 
    
    // Other buttons
    public static final JoystickButton Back = new JoystickButton(Controller, XboxController.Button.kBack.value);
    public static final JoystickButton Start = new JoystickButton(Controller, XboxController.Button.kStart.value);
    // "Mode" button swaps the D-pad and left stick
    // "Logitech" button is like Guide or Home

    // Directional Pad
    public static final POVButton North = new POVButton(Controller, 0);
    public static final POVButton NorthEast = new POVButton(Controller, 45);
    public static final POVButton East = new POVButton(Controller, 90);
    public static final POVButton SouthEast = new POVButton(Controller, 135);
    public static final POVButton South = new POVButton(Controller, 180);
    public static final POVButton SouthWest = new POVButton(Controller, 225);
    public static final POVButton West = new POVButton(Controller, 270);
    public static final POVButton NorthWest = new POVButton(Controller, 315);
  }*/
}
