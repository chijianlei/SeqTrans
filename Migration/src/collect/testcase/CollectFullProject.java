package collect.testcase;

import java.io.File;
import java.io.IOException;
import java.util.List;

import org.apache.commons.exec.CommandLine;
import org.apache.commons.exec.DefaultExecutor;
import org.apache.commons.exec.ExecuteException;
import org.apache.commons.exec.ExecuteWatchdog;
import org.apache.commons.io.FileUtils;

public class CollectFullProject {
	
	/**
	 * Collecting the full project for test sets
	 * @throws Exception 
	 */

	public static void main(String[] args) throws Exception {
		String repoPath = "J:\\git_repo";
		String testPath = "I:\\20210714-Srqtrans_testcase\\Vulnerability_testcase";
		String output = "I:\\20210714-Srqtrans_testcase\\Vulnerability_testset_full";
//		extractProject(repoPath, testPath, output);
		addDiffFile(testPath, output);
	}
	
	private static void addDiffFile(String testPath, String output) throws IOException {
		File testRoot = new File(testPath);
		File[] testDirs = testRoot.listFiles();
		for(File testDir : testDirs) {
			String cpName = testDir.getName();
			String diffPath = testDir.getAbsolutePath()+"\\diffs.txt";
			String copyRoot = output+"\\"+cpName;
			FileUtils.copyFileToDirectory(new File(diffPath), new File(copyRoot));
		}						
	}
	
	public static void extractProject(String repoPath, String testPath, String output) throws Exception {
		File testRoot = new File(testPath);
		File[] testDirs = testRoot.listFiles();
		for(File testDir : testDirs) {
			String cpName = testDir.getName();
			String diffPath = testDir.getAbsolutePath()+"\\diffs.txt";
			File diffFile = new File(diffPath);
			if(!diffFile.exists())
				throw new Exception("file is not existed!");
			List<String> lines = FileUtils.readLines(diffFile, "UTF-8");
			String projectName = lines.get(0);
			String classPath = repoPath +"\\"+projectName;
			String oldcommit = lines.get(1).split(";")[0];
			String copyRoot1 = output+"\\"+cpName+"\\"+oldcommit;
			String newcommit = lines.get(1).split(";")[1];
			String copyRoot2 = output+"\\"+cpName+"\\"+newcommit;
			switchProject(classPath, oldcommit);
			FileUtils.copyDirectoryToDirectory(new File(classPath), new File(copyRoot1));
			switchProject(classPath, newcommit);
			FileUtils.copyDirectoryToDirectory(new File(classPath), new File(copyRoot2));
		}
	}
	
	public static void switchProject(String classPath, String commit) throws Exception{
//		String movePath = diskpath+"cp"+String.valueOf(n)+"\\"+newCommitName+"\\";
		String line = "cmd.exe /C git checkout -f "+commit;	
		int[] exitvalues = {0, 1};
		System.out.println(line);
		CommandLine cmdLine = CommandLine.parse(line);
		DefaultExecutor executor = new DefaultExecutor();	
		ExecuteWatchdog watchdog = new ExecuteWatchdog(300000);//timeout 5min
		executor.setExitValues(exitvalues);		
		executor.setWorkingDirectory(new File(classPath));
		executor.setWatchdog(watchdog);
		executor.execute(cmdLine);
		Thread.sleep(6000);
	}
}
