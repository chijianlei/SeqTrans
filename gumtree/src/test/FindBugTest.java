package test;

public class FindBugTest {

	private static String string;

	public static void main(String[] args) {
		string = null;
		if(string.equals("0")) {
			System.out.println(string);
		}
	}
}
