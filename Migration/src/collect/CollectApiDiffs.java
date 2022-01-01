package collect;

import structure.API;
import utils.ReadAPI;

import java.io.*;
import java.util.ArrayList;
import java.util.LinkedHashSet;

public class CollectApiDiffs {

	private static LinkedHashSet<API> apis = new LinkedHashSet<API>();
	
	public static void main(String args[]) throws Exception{
		String path = "apis";
		apis = ReadAPI.readAPI(path);
		System.out.println(apis.size());
		ArrayList<String> realDiffs = new ArrayList<String>();
		String diffPath = "data\\";
		File diffDir = new File(diffPath);
		File[] diffs = diffDir.listFiles();
		for(int i=0;i<diffs.length;i++) {
			File diffFile = diffs[i];
			System.out.println("processing:"+diffFile.getName());
			if(diffFile.getName().contains("def")) {
				BufferedReader br = new BufferedReader(new FileReader(diffFile));
				String tmpline = "";
				while((tmpline=br.readLine())!=null) {
//					System.out.println(tmpline);
					if(containsAPI(tmpline, tmpline)) {
						realDiffs.add(tmpline);
					}
				}
				br.close();
			}
		}
		File outFile = new File("realDiff.txt");
		BufferedWriter wr = new BufferedWriter(new FileWriter(outFile));
		for(String line : realDiffs) {
			wr.append(line);
			wr.newLine();
			wr.flush();
		}
		wr.close();
	}
		
	public static Boolean containsAPI(String src, String dst) {
		for(API api : apis) {
			String cName = api.getClassName();
			String mName = api.getMethodName();
			if(src.contains(mName)) {
//				if(src.contains(cName)) {
//					System.out.println("find public static api");
//				}
//				else	System.out.println("find api");	
				return true;
			}
			if(dst.contains(mName)) {
//				if(dst.contains(cName)) {
//					System.out.println("find public static api");
//				}
//				else	System.out.println("find api");	
				return true;
			}
		}
		return false;
	}	
}
