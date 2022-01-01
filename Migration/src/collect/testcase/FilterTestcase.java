package collect.testcase;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import org.apache.commons.io.FileUtils;

public class FilterTestcase {
	
	public static void main(String[] args) throws Exception{	
		String cpPath = "J:\\Vulnerability_commit";
		String rootPath = "I:\\20210714-Srqtrans_testcase\\Vulnerability_testcase";
		String copyPath = "I:\\20210714-Srqtrans_testcase\\Vulnerability_trainset";
//		filterTestcases(rootPath, copyPath);
//		copyTop5(rootPath, copyPath);
		filterTrainset(cpPath, rootPath, copyPath);
	}
	
	private static void copyTop5(String rootPath, String copyPath) throws Exception {
		CountBugNum cb = new CountBugNum();
		File copyRoot = new File(copyPath);
		List<Map.Entry<String, Integer>> repoList = cb.countRepos(rootPath);
		int count = 0;
		ArrayList<String> topList = new ArrayList<String>();
		for(Map.Entry<String, Integer> mapping: repoList){      
            String repoName = mapping.getKey();
            int frequency = mapping.getValue();            
            System.out.println(repoName+":"+frequency);
            if(count<5) {
            	topList.add(repoName);
            	count++;
            }
        }
		
		File rootFile = new File(rootPath);
		File[] fileList = rootFile.listFiles();	
		for(String repo : topList) {
			for(File cpFile : fileList) {
				String diffPath = cpFile.getAbsolutePath()+"\\diffs.txt";
				File diffFile = new File(diffPath);
				if(!diffFile.exists())
					throw new Exception("file is not existed!");
				List<String> lines = FileUtils.readLines(diffFile, "UTF-8");	
				String repoName = lines.get(0);	
				if(repoName.equals(repo)) {
					FileUtils.copyDirectoryToDirectory(cpFile, copyRoot);
				}
			}
		}		
	}
	
	private static void filterTrainset(String cpPath, String testPath, String copyPath) throws IOException {
		File testRoot = new File(testPath);
		File[] testList = testRoot.listFiles();
		ArrayList<String> testNames = new ArrayList<String>();
		for(File testFile : testList) {
			testNames.add(testFile.getName());
		}
		
		ArrayList<File> copyList = new ArrayList<File>();
		File rootFile = new File(cpPath);
		File[] cpList = rootFile.listFiles();
		for(File cpFile : cpList) {
			String cpName = cpFile.getName();
			if(testNames.contains(cpName)) {
				continue;
			}else {
				copyList.add(cpFile);
			}
		}
		if(copyPath!=null) {
			for(File cpFile : copyList) {
				File copyDir = new File(copyPath+"\\"+cpFile.getName());
				FileUtils.copyDirectory(cpFile, copyDir);
			}
		}
	}
	
	private static ArrayList<File> filterTestcases(String rootPath, String copyPath) throws Exception{
		ArrayList<File> cpList = new ArrayList<File>();
		File rootFile = new File(rootPath);
		File[] fileList = rootFile.listFiles();	
		for(int i=0;i<fileList.length;i++) {
			File cpFile = fileList[i];
			System.out.println(i+":"+cpFile.getName());
			String cpPath = cpFile.getAbsolutePath();
			ArrayList<String> matchedList = filterTestcase(cpPath);
			int matchSize = matchedList.size();
			if(matchSize%4!=0)
				throw new Exception("check the matchedList!");
			if (matchedList.size()!=0) {
				cpList.add(cpFile);
			}
		}
		
		System.out.println("cpSize:"+cpList.size());
		
		if(copyPath!=null) {
			for(File cpFile : cpList) {
				File copyDir = new File(copyPath+"\\"+cpFile.getName());
				FileUtils.copyDirectory(cpFile, copyDir);
			}
		}		
		return cpList;
	}//Filter and copy cpFiles that contain test cases.
	
	private static ArrayList<String> filterTestcase(String path) throws Exception{	
		ArrayList<String> matchedList = new ArrayList<String>();
		File cpFile = new File(path);
		System.out.println("Analyse:"+ cpFile.getName());
		String diffPath = cpFile.getAbsolutePath()+"\\diffs.txt";
		File diffFile = new File(diffPath);
		if(!diffFile.exists())
			throw new Exception("file is not existed!");
		List<String> lines = FileUtils.readLines(diffFile, "UTF-8");	
		String repoName = lines.get(0);	
		System.out.println(repoName);
//		String srcHash = lines.get(1).split(";")[0];
//		String dstHash = lines.get(1).split(";")[1];
		lines.remove(0);
		lines.remove(0);//remove two lines of the diff file	
		
		for(int i=0;i<lines.size();i++) {
			String tmpString = lines.get(i);
//			System.out.println(tmpString);
			String path1 = tmpString.split(";")[0];
			String path2 = tmpString.split(";")[1];
			String[] names1 = path1.split("/");
			String src_filename = names1[names1.length-1];
			src_filename = src_filename.substring(0, src_filename.length()-5);//remove ".java"
			String[] names2 = path2.split("/");
			String dst_filename = names2[names2.length-1];		
			dst_filename = dst_filename.substring(0, dst_filename.length()-5);//remove ".java"
			
			if(containTest(src_filename)&&containTest(dst_filename)) {
				continue;
			}else{	
				for(int j=0;j<lines.size();j++) {
					String tmpString1 = lines.get(j);
					String path3 = tmpString1.split(";")[0];
					String path4 = tmpString1.split(";")[1];
					String[] names3 = path3.split("/");
					String src_testname = names3[names3.length-1];
					String[] names4 = path4.split("/");
					String dst_testname = names4[names4.length-1];
					if(src_testname.contains(src_filename)&&containTest(src_testname)
							&&dst_testname.contains(dst_filename)&&containTest(dst_testname)) {
						matchedList.add(src_filename);
						matchedList.add(src_testname);
						matchedList.add(dst_filename);
						matchedList.add(dst_testname);
						break;
					}
				}
			}
		}		
		System.out.println("matchSize:"+matchedList.size());
		return matchedList;
	}//filter files that with the same named testcases
	
	private static Boolean containTest(String name) {
		if((name.contains("TestCase")||name.contains("Test"))){
			return true;
		}else
			return false;
	}

}
