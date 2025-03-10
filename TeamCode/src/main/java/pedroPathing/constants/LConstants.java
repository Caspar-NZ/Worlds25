package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = .001989436789*0.98856;
        ThreeWheelConstants.strafeTicksToInches = .001989436789*0.98856;
        ThreeWheelConstants.turnTicksToInches = 0.00196667763*1.0036666;
        ThreeWheelConstants.leftY = 6.27362205;
        ThreeWheelConstants.rightY = -6.27362205;
        ThreeWheelConstants.strafeX = -4.5472441;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "rightFront";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "leftRear";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightRear";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}

/*
right front = left side odo
left back = right side odo
right back = back odo
 */




