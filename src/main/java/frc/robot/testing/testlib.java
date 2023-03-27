package frc.robot.testing;

public class testlib {

    public static void main( String [] Args) {    
        double currentAngle = Double.parseDouble(Args[0]);
        System.out.println("Current Position:" +currentAngle);
        for (double degrees = -360; degrees <= 360; degrees += 30) {
            double result = placeInAppropriate0To360Scope(currentAngle, degrees);
            System.out.println("Target Position: " +degrees + " --->  Optimized position:" +result);
        }
        
    }


        /**
     * @param currentAngle Current Angle
     * @param newAngle       Target Angle
     * @return Closest angle within scope
     */
    public static double placeInAppropriate0To360Scope(double currentAngle, double newAngle) {
        double lowerBound;
        double upperBound;
        
        // this makes loweroffset in the range -360 <= lowerOffset <= 360
        double lowerOffset = currentAngle % 360;

        if (lowerOffset >= 0) {
            lowerBound = currentAngle - lowerOffset;   // lowerbound should be 0 in this case if currentangle is 0 <= currentAngle <= 360
            upperBound = currentAngle + (360 - lowerOffset); //upperbound should be 360 inb this case if currentangle is 0 <= currentAngle <= 360
        } else {
            upperBound = currentAngle - lowerOffset;  //upperbound should be 0 in this case if currentangle is -360 <= currentAngle <= 0
            lowerBound = currentAngle - (360 + lowerOffset); //lowerbound should be -360 in this case if currentangle is -360 <= currentAngle <= 0
        }


        while (newAngle < lowerBound) {
            newAngle += 360;
        }   

        while (newAngle > upperBound) {
            newAngle -= 360;
        }

        if (newAngle - currentAngle > 180) {
            newAngle -= 360;
        } else if (newAngle - currentAngle < -180) {
            newAngle += 360;
        }

        return newAngle;
    }
}
