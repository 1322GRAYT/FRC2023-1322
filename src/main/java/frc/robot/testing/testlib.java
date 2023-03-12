package frc.robot.testing;

//import frc.robot.utils.RFSLIB;

public class testlib {

    private static double optimizeGyroNope(double degrees) {
        return (360+(degrees%360)) % 360;
      }
      private static double optimizeGyro(double degrees) {
        // 0 < degrees < 360
        if ((degrees > 0.0) && (degrees < 360.0)) {
          return degrees;
        } else {
          int m = (int) Math.floor(degrees / 360.0);
          double optimizedDegrees = degrees - (m * 360.0);
          return Math.abs(optimizedDegrees);
        }
      }
    public static void main( String [] Args) {

    
        double []degrees = {20.0,30.0,90.0,-30.0,-770.0,-315.0};

        for (int i = 0 ; i< degrees.length; i++) {
            System.out.printf("Degrees:  %f  -- Old(%f) -- New(%f)]\n", degrees[i], optimizeGyro(degrees[i]), optimizeGyroNope(degrees[i]));
        }
    }
         
        /*
        double tmp;
        long start, end;

        System.out.print("Testing Math.signum ... ");
        start = System.nanoTime();
        for (int i=0; i<10000; i++ )
            tmp = Math.signum(i*1.0);
        end = System.nanoTime();
        System.out.println("Time: "+(end-start));

        System.out.print("Testing getsignbitwise ... ");
        start = System.nanoTime();
        for (int i=0; i<10000; i++ )
            tmp = getSignBitwise(i*1.0);
        end = System.nanoTime();
        System.out.println("Time: "+(end-start));

        System.out.print("Testing getsignbycomp ... ");
        start = System.nanoTime();
        for (int i=0; i<10000; i++ )
            tmp = getSignByComp(i*1.0);
        end = System.nanoTime();
        System.out.println("Time: "+(end-start));


        System.out.print("Testing inline ... ");
        start = System.nanoTime();

        for (int i=0; i<10000; i++ ) {
            tmp = i*1.0;
            tmp = (tmp>0?1:(tmp<0?-1:0));
        }
        end = System.nanoTime();
        System.out.println("Time: "+(end-start));



    }

*/


/*        start = System.nanoTime();
        
        for (int i=0; i< 1000; i++) {
            double PowerRequested = i/1000.0;
            //System.out.println("RFSLib.OldApplyDeadBand_Scaled("+PowerRequested+","+DeadBandThreshold+","+powerLimit+") = "+Math.round(RFSLIB.ApplyDeadBand_Scaled(PowerRequested,DeadBandThreshold,powerLimit)/0.001)*0.001); 
            RFSLIB.OldApplyDeadBand_Scaled(PowerRequested,DeadBandThreshold,powerLimit);
        }
        end = System.nanoTime();
        System.out.println("OldApplyDeadBand_Scaled " + (end-start));
        start = System.nanoTime();
        for (int i=0; i< 1000; i++) {
            double PowerRequested = i/1000.0;
            //System.out.println("RFSLib.ApplyDeadBand_Scaled("+PowerRequested+","+DeadBandThreshold+","+powerLimit+") = "+Math.round(RFSLIB.NewApplyDeadBand_Scaled(PowerRequested,DeadBandThreshold,powerLimit)/0.001)*0.001); 
            RFSLIB.ApplyDeadBand_Scaled(PowerRequested,DeadBandThreshold,powerLimit);
        }    
        end = System.nanoTime();
        System.out.println("ApplyDeadBand_Scaled " + (end-start));
    }
    public static void printArray( double [] array) {
        for (int i =0; i<array.length; i++) {
            System.out.println("["+i+"] = "+ array[i]);
        }
*/
    public static double getSignBitwise(double value) {
        return ((Double.doubleToLongBits(value) & (1<<63))>0?1.0:-1.0);
    }

    public static double getSignByComp(double value) {
        return (value>0?1:(value<0?-1:0));
    }
}
