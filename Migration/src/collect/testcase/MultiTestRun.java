package collect.testcase;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;

import org.apache.commons.exec.CommandLine;
import org.apache.commons.exec.DefaultExecuteResultHandler;
import org.apache.commons.exec.DefaultExecutor;
import org.apache.commons.exec.ExecuteException;
import org.apache.commons.exec.ExecuteWatchdog;
import org.apache.commons.io.FileUtils;
import org.apache.commons.io.filefilter.DirectoryFileFilter;
import org.apache.commons.io.filefilter.FileFilterUtils;

public class MultiTestRun {
	
	public static void main(String[] args) throws Exception {
		String cpRoot = "I:\\20210714-Srqtrans_testcase\\Vulnerability_testset_full";
//		countRepos(cpRoot);
		autoTest(cpRoot);
		
//		String testPath = "I:\\20210714-Srqtrans_testcase\\Vulnerability_testset_full\\cp994\\4f942064d85454a4bcc4da04cd482d114816c14a\\uaa";
//		Collection<File> collections = FileUtils.listFiles(new File(testPath), 
//				FileFilterUtils.suffixFileFilter("build.gradle"), null);
//		ArrayList<File> gradleFilters = new ArrayList<>(collections);
//		System.out.println(gradleFilters.size());
//        runGradleTest(testPath);
	}
	
	public static void autoTest(String cpRoot) throws Exception {
		File[] cpList = (new File(cpRoot)).listFiles();
		System.out.println(cpList.length);
		int count = 0;
		String continue_point = "cp1118";
		Boolean afterPoint = false;
		for(File cpFile : cpList) {
			if(cpFile.getName().equals(continue_point)) {
				count++;
				afterPoint = true;
				continue;
			}				
			if(!afterPoint) {
				count++;
				continue;
			}						
			File[] cpDirs = cpFile.listFiles();
			System.out.println("test");
			for(File cpDir : cpDirs) {
				if(cpDir.isDirectory()) {
					String classPath = cpDir.listFiles()[0].getAbsolutePath();
					File directory = cpDir.listFiles()[0];
					System.err.println(count+" "+cpFile.getName()+": "+cpDir.getName());
					Collection<File> collections = FileUtils.listFiles(directory, 
							FileFilterUtils.suffixFileFilter("build.gradle"), null);
					ArrayList<File> gradleFilters = new ArrayList<>(collections);
					collections.clear();
					collections = FileUtils.listFiles(directory, 
							FileFilterUtils.suffixFileFilter("pom.xml"), null);
					ArrayList<File> mavenFilters = new ArrayList<>(collections);
					collections.clear();
					collections = FileUtils.listFiles(directory, 
							FileFilterUtils.suffixFileFilter("build.xml"), null);
					ArrayList<File> antFilters = new ArrayList<>(collections);
					try {
						if(gradleFilters.size()!=0&&mavenFilters.size()==0
								&&antFilters.size()==0) {
							runGradleTest(classPath);
						}else if(mavenFilters.size()!=0&&gradleFilters.size()==0
								&&antFilters.size()==0) {
							runMavenTest(classPath);
						}else if(antFilters.size()!=0&&gradleFilters.size()==0
								&&mavenFilters.size()==0) {
							runAntTest(classPath);
						}else {
							throw new Exception("check it! "+classPath);
						}
						Thread.sleep(3000);
					} catch (Exception e) {
						System.err.println(count+" "+cpFile.getName()+": "+cpDir.getName());
						// TODO: handle exception
					}									
				}
			}
			count++;
		}
	}
	
	public static void countRepos(String cpRoot) throws IOException {
		File[] cpList = (new File(cpRoot)).listFiles();
		HashSet<String> repoNames = new HashSet<String>();
		for(File cpFile : cpList) {
			String diffPath = cpFile.getAbsolutePath()+"\\diffs.txt";
			List<String> lines = FileUtils.readLines(new File(diffPath), "UTF-8");
			String repoName = lines.get(0);
			repoNames.add(repoName);
		}
		for(String name : repoNames) {
			System.out.println(name);
		}
	}
	
	private static void runGradleTest(String classPath) throws Exception {
		String line = "cmd.exe /C gradlew.bat check --continue";
		CommandLine cmdLine = CommandLine.parse(line);
		DefaultExecutor executor = new DefaultExecutor();
		ExecuteWatchdog watchdog = new ExecuteWatchdog(1800000);//timeout 30min
		executor.setWatchdog(watchdog);
		executor.setWorkingDirectory(new File(classPath));
		executor.setExitValue(1);
		executor.execute(cmdLine);
	}
	
	private static void runMavenTest(String classPath) throws Exception {
		String line = "cmd.exe /C mvn test -DtestFailureIgnore=true";
		CommandLine cmdLine = CommandLine.parse(line);
		DefaultExecutor executor = new DefaultExecutor();
		ExecuteWatchdog watchdog = new ExecuteWatchdog(1800000);//timeout 30min
		executor.setWatchdog(watchdog);
		executor.setWorkingDirectory(new File(classPath));
		executor.setExitValue(1);
		executor.execute(cmdLine);
	}

	private static void runAntTest(String classPath) throws Exception {
		String line = "cmd.exe /C ant test -keep-going";
		CommandLine cmdLine = CommandLine.parse(line);
		DefaultExecutor executor = new DefaultExecutor();
		ExecuteWatchdog watchdog = new ExecuteWatchdog(1800000);//timeout 30min
		executor.setWatchdog(watchdog);
		executor.setWorkingDirectory(new File(classPath));
		executor.setExitValue(1);
		executor.execute(cmdLine);
	}
	
	
	
	
	
	
}
