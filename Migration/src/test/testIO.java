package test;

import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class testIO {
	
	public static void main(String[] args) throws Exception{
		ArrayList<String> tests = new ArrayList();
		tests.add("abc");
		if(tests.get(0).contains("ab"))
			System.out.println("True");
		else
			System.out.println("False");

	}

	private static void test(){
		String test = "M B JUA:  Assertion of type org.springframework.messaging.converter.CompositeMessageConverter in org.springframework.web.socket.config.MessageBrokerBeanDefinitionParserTests.annotationMethodMessageHandler() at MessageBrokerBeanDefinitionParserTests.java:[line 337]"
				+ " may hide useful information about why a cast may have failed.  At MessageBrokerBeanDefinitionParserTests.java:[line 337]";
		String pattern = "[aA]t\\b.*.java:\\[line\\s\\d+\\]";
		Pattern p = Pattern.compile(pattern);
		Matcher m = p.matcher(test);
		m.find();
		System.out.println(m.group());
		String location1 = m.group().split("]")[0]+"]";
		System.out.println(location1);
		Pattern p1 = Pattern.compile("At\\b.*.java:\\[line\\s\\d+\\]");
		Matcher m1 = p1.matcher(m.group());
		m1.find();
		String location2 = m1.group();
		System.out.println(location2);
		if (m.find()) {
			String name = m.group().split("\\.java")[0];
			System.out.println(name);
			name = name.substring(3, name.length());
			System.out.println("Found value: " + name);
		} else {
			System.out.println("NO MATCH");
		}

		String test1 = "I:\\20210714-Srqtrans_testcase\\Vulnerability_testcase\\cp12\\8748ba4c4b96a8a82c20c88373b2fcc77c30011f\\spring-messaging\\src\\main\\java\\org\\springframework\\messaging\\simp\\broker\\SimpleBrokerMessageHandler.java";
		String fileName = test1.split("\\\\")[test1.split("\\\\").length-1].split("\\.")[0];
		System.out.println(fileName);
	}

	
	
	

}
