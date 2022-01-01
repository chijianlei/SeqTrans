package collect.testcase;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import org.apache.commons.io.FileUtils;
import org.apache.commons.io.filefilter.DirectoryFileFilter;
import org.apache.commons.io.filefilter.FileFilterUtils;
import org.apache.commons.lang3.tuple.Pair;

/**
 * @author chi
 * 搜索跑完自动化测试用例的项目，是否有未编译成功的
 */

public class SearchUncompiledProject {
	
	public static void main(String[] args) throws Exception {
		String cpRoot = "I:\\20210714-Srqtrans_testcase\\Vulnerability_testset_full\\";
		ArrayList<String> uncompileList = startMultiSearch(cpRoot);
		for(String name : uncompileList) {
			File cpDir = new File(cpRoot+name);
			ArrayList<Pair<String, String>> missFiles = searchSingleProject(cpDir);
			String repoName = getRepoName(cpDir);
			System.out.println(name+","+repoName);
			for(Pair<String, String> pair : missFiles) {
				String srcName = pair.getLeft();
				String dstName = pair.getRight();
				System.out.println(srcName+","+dstName);
			}
		}
	}	
	
	public static ArrayList<String> startMultiSearch(String cpRoot) throws IOException {
		File[] cpList = (new File(cpRoot)).listFiles();
		ArrayList<String> uncompileList = new ArrayList<String>();
		for(File cpDir : cpList) {
			ArrayList<Pair<String, String>> missFiles = searchSingleProject(cpDir);
			if(missFiles.size()>0)
				uncompileList.add(cpDir.getName());				
		}
		return uncompileList;
	}
	
	public static ArrayList<Pair<String, String>> searchSingleProject(File cpDir) throws IOException {		
		ArrayList<Pair<String, String>> diffFiles = new ArrayList<Pair<String,String>>();
		ArrayList<Pair<String, String>> missFiles = new ArrayList<Pair<String,String>>();
		File diffFile = new File(cpDir.getAbsoluteFile()+"\\diffs.txt");
		List<String> lines = FileUtils.readLines(diffFile, "utf-8");
		lines.remove(0);
		lines.remove(0);					
		for(String line : lines) {
			String[] srcLines = line.split(";")[0].split("/");
			String[] dstLines = line.split(";")[1].split("/");
			String srcName = srcLines[srcLines.length-1];
			srcName = srcName.split("\\.")[0]+".class";			
			String dstName = dstLines[dstLines.length-1];
			dstName = dstName.split("\\.")[0]+".class";
//			System.out.println(srcName);
			Pair<String, String> pair = Pair.of(srcName, dstName);
			diffFiles.add(pair);				
		}
		
		for(Pair<String, String> pair : diffFiles) {
			String srcName = pair.getLeft();
			String dstName = pair.getRight();
			if(srcName.equals(dstName)) {
				Collection<File> collections = FileUtils.listFiles(cpDir, 
						FileFilterUtils.suffixFileFilter(srcName), DirectoryFileFilter.INSTANCE);
				ArrayList<File> filters = new ArrayList<>(collections);
				if(filters.size()<2) {
					missFiles.add(pair);
				}
			}else {
				Collection<File> collections1 = FileUtils.listFiles(cpDir, 
						FileFilterUtils.suffixFileFilter(srcName), DirectoryFileFilter.INSTANCE);
				ArrayList<File> filters1 = new ArrayList<>(collections1);
				Collection<File> collections2 = FileUtils.listFiles(cpDir, 
						FileFilterUtils.suffixFileFilter(dstName), DirectoryFileFilter.INSTANCE);
				ArrayList<File> filters2 = new ArrayList<>(collections2);
				if(filters1.size()<1||filters2.size()<1) {
					missFiles.add(pair);
				}
			}	
		}
		return missFiles;
	}
	
	private static String getRepoName(File cpDir) throws IOException {	
		File diffFile = new File(cpDir.getAbsoluteFile()+"\\diffs.txt");
		List<String> lines = FileUtils.readLines(diffFile, "utf-8");
		String repoName = lines.get(0);
		return repoName;
	}
}
