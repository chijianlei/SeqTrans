package utils;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.io.FileUtils;

public class CountCPs {

	public static void main(String[] args) throws Exception{
		String path = "J:\\Vulnerability_commit";
		List<String> cpList = countCPs(path);
		System.out.println(cpList.size());
	}
	
	public static List<String> countCPs(String path) throws IOException{
		List<String> cpList = new ArrayList<String>();
		File root = new File(path);
		File[] cpRoots = root.listFiles();
		for(File cpRoot : cpRoots) {
			File diffFile = new File(cpRoot.getAbsoluteFile()+"\\diffs.txt");
			List<String> lines = FileUtils.readLines(diffFile, "UTF-8");
			lines.remove(0);
			lines.remove(0);
			cpList.addAll(lines);
		}
		return cpList;
	}
}
