package poplib.error_handling;

import com.revrobotics.REVLibError;

public class ErrorHandling {
    public static void handlRevLibError(REVLibError error, String useCase) {
        if (error != REVLibError.kOk) {
            System.out.println("REVLibError: " + error.toString() + " when " + useCase);
        }
    } 
}
