package collect.testcase;

import org.apache.commons.io.FileUtils;
import org.apache.commons.io.filefilter.RegexFileFilter;
import org.apache.commons.lang3.StringUtils;
import org.apache.commons.lang3.tuple.Pair;

import java.io.File;
import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class SearchLocation {
	private static int total_match = 0;
	
	public static void main(String[] args) throws Exception {
		String cpRoot = "I:\\20210714-Srqtrans_testcase\\Vulnerability_testset_full\\";
		String diffPath = "D:\\workspace\\eclipse2018\\Migration\\data_num";			
		
//		searchLocation(new File(cpRoot+"cp12"));
//		multiSearch(cpRoot);
		compareLocation(cpRoot, diffPath);
	}
	
	public static void compareLocation(String cpRoot, String diff_location) throws Exception {
		File[] cpList = (new File(cpRoot)).listFiles();
		System.out.println(cpList.length);
		ArrayList<String> skipList = new ArrayList<String>();
		skipList.add("cp19");
		skipList.add("cp233");
		skipList.add("cp241");
		skipList.add("cp433");
		skipList.add("cp870");
		int totalNum = 0;
		for(int i=0;i<cpList.length;i++) {	
			File cpDir = cpList[i];		
			if(skipList.contains((cpDir.getName())))
				continue;
			System.out.println("Analyse: "+cpDir.getName());
			File matchFile = new File(cpDir.getAbsoluteFile()+"\\matchLines1.txt");
			List<String> matchLines = FileUtils.readLines(matchFile, "utf-8");
			System.out.println(matchLines.size());
			Collection<File> collections = FileUtils.listFiles(new File(diff_location), 
					new RegexFileFilter(".*"+cpDir.getName()+".txt"), null);
			ArrayList<File> filters = new ArrayList<>(collections);
//			System.out.println(filters.get(0).getName());
			
			List<String> diffLines = FileUtils.readLines(filters.get(0), "utf-8");
			HashMap<String, ArrayList<Integer>> matchMap = analyseSpotsResults(matchLines);
			HashMap<String, ArrayList<ArrayList<Integer>>> diffMap = analyseGumtreeResults(diffLines);

			totalNum += compareMap(matchMap, diffMap);
	    }
		System.out.println(totalNum);
		System.out.println(total_match);
	}
	
	private static int compareMap(HashMap<String, ArrayList<Integer>> matchMap,
								   HashMap<String, ArrayList<ArrayList<Integer>>> diffMap) {
		int DiffNum = 0;
		int matchNum = 0;
		for(Map.Entry<String, ArrayList<ArrayList<Integer>>> entry : diffMap.entrySet()) {
			String fileName = entry.getKey();
			ArrayList<ArrayList<Integer>> lineList = entry.getValue();
			if(matchMap.containsKey(fileName)) {
				for(ArrayList<Integer> lineNums : lineList) {
					ArrayList<Integer> matchList = matchMap.get(fileName);
					for(int n : matchList){
						if(lineNums.contains(n)){
							matchNum++;
							total_match++;
						}

					}
				}
			}
			DiffNum += lineList.size();
		}
		System.out.println("Total:"+DiffNum);
		System.out.println("Match:"+matchNum);
		return DiffNum;
	}
	
	public static HashMap<String, ArrayList<ArrayList<Integer>>> analyseGumtreeResults(List<String> diffLines) throws Exception {
		HashMap<String, ArrayList<ArrayList<Integer>>> diffMap = new HashMap<String, ArrayList<ArrayList<Integer>>>();
		for(String line : diffLines) {
			String[] splits = line.split(";");
			String fileName = splits[0].split("\\\\")[splits[0].split("\\\\").length-1]
					.split("\\.")[0];
			String lineNum = splits[2];
			int begin = Integer.valueOf(lineNum.split("->")[0].split(",")[0]);
			int end = Integer.valueOf(lineNum.split("->")[1].split(",")[0]);
			ArrayList tmpList = new ArrayList();

			for(int i=begin;i<=end;i++) {
				tmpList.add(i);
			}
			if(fileName!=null&&lineNum!=null) {
				if(diffMap.containsKey(fileName)) {
					diffMap.get(fileName).add(tmpList);
				}else {
					ArrayList<ArrayList<Integer>> lineList = new ArrayList();
					lineList.add(tmpList);
					diffMap.put(fileName, lineList);
				}
			}else
				throw new Exception("Check location!");
		}	
		return diffMap;
	}
	
	public static HashMap<String, ArrayList<Integer>> analyseSpotsResults(List<String> matchLines) throws Exception {
		String regex = "[aA]t\\b.*.java:\\[line\\s\\d+\\]";		
		Pattern p = Pattern.compile(regex);
		HashMap<String, ArrayList<Integer>> matchMap = new HashMap<String, ArrayList<Integer>>();
		for(String line : matchLines) {
//			System.out.println(line);
	        Matcher m = p.matcher(line);
	        if (m.find()) {
	        	String location = m.group();
//	        	System.out.println("Found value: " + location);
	        	if(location.contains("at")&&location.contains("At")) {
	        		if(StringUtils.countMatches(location, "[")==2&&StringUtils.countMatches(location, "]")==2) {
	        			String location1 = location.split("]")[0]+"]";
	        			addMap(matchMap, location1);
	        		}	        		
	        		Pattern p1 = Pattern.compile("At\\b.*.java:\\[line\\s\\d+\\]");
	        		Matcher m1 = p1.matcher(location);
	        		m1.find();
	        		String location2 = m1.group();
	        		
	        		addMap(matchMap, location2);
	        	}else {
	        		addMap(matchMap, location);
	        	}	            	            
	        }else
				continue;	        	        	       
		}
		return matchMap;
	}
	
	private static HashMap<String, ArrayList<Integer>> addMap(
			HashMap<String, ArrayList<Integer>> map, String location) throws Exception {
		String fileName = location.split("\\.java")[0];
        fileName = fileName.substring(3, fileName.length());
        String lineNum = location.split("\\.java")[1];
        lineNum = lineNum.substring(7, lineNum.length()-1);
		
		if(fileName!=null&&lineNum!=null) {
//			System.out.println("Name:"+fileName);
//			System.out.println(lineNum);
        	if(map.containsKey(fileName)) {
        		map.get(fileName).add(Integer.valueOf(lineNum));
        	}else {
        		ArrayList<Integer> lineList = new ArrayList<Integer>();
        		lineList.add(Integer.valueOf(lineNum));
        		map.put(fileName, lineList);
        	}
        }else {
        	System.out.println(location);
        	System.out.println(fileName);
        	System.out.println(lineNum);
        	throw new Exception("Check location!"); 
        }
		return map;
	}
	
	public static void multiSearch(String cpRoot) throws Exception {
		File[] cpList = (new File(cpRoot)).listFiles();
		System.out.println(cpList.length);
		for(File cpDir : cpList) {	
			System.out.println(cpDir.getName());
			searchLocation(cpDir);
		}
	}
	
	public static void searchLocation(File cpDir) throws Exception {
		ArrayList<Pair<String, String>> diffFiles = new ArrayList<Pair<String,String>>();
		ArrayList<String> matchList = new ArrayList<String>();
		File diffFile = new File(cpDir.getAbsoluteFile()+"\\diffs.txt");
		List<String> lines = FileUtils.readLines(diffFile, "utf-8");
		lines.remove(0);
		lines.remove(0);					
		for(String line : lines) {
			String[] srcLines = line.split(";")[0].split("/");
			String[] dstLines = line.split(";")[1].split("/");
			String srcName = srcLines[srcLines.length-1];
			srcName = srcName.split("\\.")[0];			
			String dstName = dstLines[dstLines.length-1];
			dstName = dstName.split("\\.")[0];
			Pair<String, String> pair = Pair.of(srcName, dstName);
			diffFiles.add(pair);		
		}
		File diffFile1 = new File(cpDir.getAbsoluteFile()+"\\sec_output.txt");
		List<String> lines1 = FileUtils.readLines(diffFile1, "utf-8");		
		for(Pair<String, String> pair : diffFiles) {
			String srcName = pair.getLeft();
			System.out.println(srcName);
			for(String line : lines1) {
//				System.out.println(line);
				if(line.contains(srcName)) {
					System.out.println("Find: "+line);
					matchList.add(line);
				}
			}
		}
		File writeFile = new File(cpDir.getAbsoluteFile()+"\\matchLines1.txt");
		FileUtils.writeLines(writeFile, matchList);
	}

	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
}
