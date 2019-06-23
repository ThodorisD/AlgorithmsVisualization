package exception;

public class FileFormatException extends Exception {

	private static final long serialVersionUID = -2061264479862093369L;

	public FileFormatException(String line, int lineNumber) {
        System.err.println("ERROR: File could not be formatted properly. Check the syntax.");
        System.err.println("HINT: Check line " + "(" + lineNumber + ") | " + line);
        System.exit(-1);
    }
}
