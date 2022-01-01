package utils;

import java.io.*;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashSet;

import structure.API;

public class ReadAPI {
	public static void main(String[] args) throws Exception{
		String path = "apis_test";
		LinkedHashSet<API> apis = readAPI(path);
		System.out.println(apis.size());
//		for(API api : apis) {
//			System.out.println("-------------");
//			System.out.println(api.getLongName());
//			System.out.println(api.getClassName());
//			System.out.println(api.getMethodName());
//			for(String tmp : api.getParams()) {
//				System.out.println(tmp);
//			}
//		}
	}
	
	public static LinkedHashSet<API> readAPI(String path) throws Exception {
		LinkedHashSet<API> apis = new LinkedHashSet<API>();
		File dir = new File(path);
		File[] files = dir.listFiles();
		for(File file : files) {
			BufferedReader br = new BufferedReader(new FileReader(file));
			String tmpline = "";
			while((tmpline=br.readLine())!=null) {
				String className = "";
				String methodName = "";
				ArrayList<String> params = new ArrayList<String>();
				String longName = tmpline.split(";")[0];
				if(tmpline.split(";").length>1) {
					String[] tmps = tmpline.split(";")[1].split(",");
					for(String tmp : tmps) {
						params.add(tmp);
					}
				}
				if(longName.contains("Anon_")) {//匿名内部类情况，去除.(Anon_xx)
					continue;
//					className = longName.substring(0, longName.indexOf("(")-1);
//					methodName = longName.substring(longName.indexOf(")")+1, longName.length());
				}else {
					className = longName.substring(0, longName.lastIndexOf("."));
					methodName = longName.substring(longName.lastIndexOf(".")+1, longName.length());
				}
				longName = className+"."+methodName;
				API api = new API(longName, className, methodName, params);
				apis.add(api);
			}
			br.close();
		}
		return apis;
	}
	
	public static HashSet<String> readClass(String path) throws Exception {
		HashSet<String> classes = new HashSet<String>();
		File dir = new File(path);
		File[] files = dir.listFiles();
		for(File file : files) {
			BufferedReader br = new BufferedReader(new FileReader(file));
			String tmpline = "";
			while((tmpline=br.readLine())!=null) {
				String api = tmpline;
				String className = "";
				if(api.contains("Anon_")) {//匿名内部类情况，去除.(Anon_xx)
					className = api.substring(0, api.indexOf("(")-1);
				}else {
					className = api.substring(0, api.indexOf("(")-1);
					className = className.substring(0, className.lastIndexOf("."));
				}			
				classes.add(className);
			}
			br.close();
		}
		return classes;
	}

}
