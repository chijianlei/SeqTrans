package collect.testcase;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileDescriptor;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URL;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Date;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Scanner;

import org.apache.commons.exec.CommandLine;
import org.apache.commons.exec.DefaultExecuteResultHandler;
import org.apache.commons.exec.DefaultExecutor;
import org.apache.commons.exec.ExecuteWatchdog;
import org.apache.commons.io.FileUtils;
import org.eclipse.jgit.api.Git;
import org.eclipse.jgit.api.errors.GitAPIException;
import org.eclipse.jgit.diff.DiffEntry;
import org.eclipse.jgit.diff.DiffFormatter;
import org.eclipse.jgit.diff.RawTextComparator;
import org.eclipse.jgit.lib.CheckoutEntry;
import org.eclipse.jgit.lib.ObjectId;
import org.eclipse.jgit.lib.ObjectReader;
import org.eclipse.jgit.lib.Ref;
import org.eclipse.jgit.lib.ReflogEntry;
import org.eclipse.jgit.lib.Repository;
import org.eclipse.jgit.revwalk.RevCommit;
import org.eclipse.jgit.revwalk.RevObject;
import org.eclipse.jgit.revwalk.RevWalk;
import org.eclipse.jgit.storage.file.FileRepositoryBuilder;
import org.eclipse.jgit.treewalk.CanonicalTreeParser;

import structure.ChangePair;
import utils.FileOperation;

public class CollectTestcase {

	/**
	 * Collecting change pairs from git commit diff logs with traditional test cases
	 * @throws Exception 
	 */
	public static void main(String[] args) throws Exception {
		// TODO Auto-generated method stub
		String dataPath="D:\\workspace\\Pycharm\\20191222-Vulnerability-dataset\\dataset.csv";//��Ҫ������Commit Hash		
		String rootPath="J:\\git_repo\\";
		String diskpath = "I:\\20210714-Srqtrans_testcase\\Vulnerability_trainset\\";
		String testPath = "I:\\20210714-Srqtrans_testcase\\Vulnerability_testcase\\";
		autoExtraction(dataPath, rootPath, diskpath, testPath);
//		getChangeList(versionCommit, path);
	}
	
	public static void autoExtraction(String dataPath, String rootPath, String diskpath, 
			String testPath) throws Exception {
		File csvFile = new File(dataPath);
		BufferedReader br = new BufferedReader(new FileReader(csvFile));
		String tmpline = "";
		ArrayList<String> lines = new ArrayList<String>();
		while((tmpline=br.readLine())!=null) {
			lines.add(tmpline);
		}
		br.close();
		
		ArrayList<String> testHashs = new ArrayList<String>();
		if(testPath!=null) {
			testHashs = checkExistTests(testPath);
		}//skip
//		int n = continueProcess(diskpath);//continue from the next cpNum
		int n = 344;
		int errorNum = 0;
		BufferedWriter wr = new BufferedWriter(new FileWriter(new File("err.txt"), true));
		for(int i=n;i<lines.size();i++,n++) {
			System.out.println("round:"+i);			
			String line = lines.get(i);
			String[] tokens = line.split(",");
			String CVE = tokens[0];
			String URL = tokens[1];
			String commit = tokens[2];
			String repoName = URL.split("/")[URL.split("/").length-1];
			System.out.println(repoName+","+commit);
			
			String classPath = rootPath+repoName+"\\.git";
			System.out.println(classPath);
			File repoDir = new File(classPath);
			if (!repoDir.exists()) {
				System.err.println(repoName+" not exists!");
				continue;
			}
			FileRepositoryBuilder builder = new FileRepositoryBuilder();
			builder.setMustExist(true);
//			builder.addCeilingDirectory(new File(classPath));
			Repository repo = builder.setGitDir(new File(classPath))
			  .readEnvironment()
			  .findGitDir()
			  .build();
			RevWalk walk = new RevWalk(repo);
			ObjectId versionId=repo.resolve(commit);						
			ChangePair cp = new ChangePair();
			try {
				String fullCommitId = versionId.getName();
				if(testHashs.contains(fullCommitId)) {
					continue;
				}//skip test set
				RevCommit currentCommit=walk.parseCommit(versionId);	
				System.out.println("Commit:"+currentCommit.getName());
				cp = getChangPair(currentCommit, repo);	
				cp.setRootPath(rootPath);
				cp.setRepoName(repoName);
            } catch (Exception e) {               
                errorNum++;
                wr.append(repoName+","+commit);
    			wr.newLine();
    			e.printStackTrace();
                continue;
            }
			RevCommit newCommit = cp.getNewCommit();
			RevCommit oldCommit = cp.getOldCommit();
			String newCommitName = newCommit.getName();
			String oldCommitName = oldCommit.getName();
			System.out.println("cp"+n+":"+oldCommitName+";"+newCommitName);
			n = runExec(CVE, cp, repo, n, diskpath);		
			walk.close();						
		}	
		wr.close();
		System.out.println("Error number:"+errorNum);
		System.out.println("CPsize:"+n);
	}
	
	private static int runExec(String CVE, ChangePair cp, Repository repo, int n, String diskpath) throws Exception {
		RevCommit newCommit = cp.getNewCommit();
		RevCommit oldCommit = cp.getOldCommit();
		String newCommitName = newCommit.getName();
		String oldCommitName = oldCommit.getName();
		String rootPath = cp.getRootPath();
		String repoName = cp.getRepoName();
		String classPath = rootPath+repoName+"\\";
		String movePath = diskpath+"cp"+String.valueOf(n)+"\\"+newCommitName+"\\";
		String line = "cmd.exe /C git checkout -f "+newCommitName;
		
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
		
		List<DiffEntry> diffs = cp.getDiffs();
		ArrayList<DiffEntry> filterDiffs = getUsefulDiffs(diffs);
		System.out.println("Diffsize:"+filterDiffs.size());
		if(filterDiffs.size()==0) {
			return n;// continue the next iter
		}
		String diffDir = diskpath+"cp"+String.valueOf(n)+"\\diff_logs\\";
		File diffDirFile = new File(diffDir);
		if (!diffDirFile.exists()) {
			diffDirFile.mkdirs();
		}
		int count = 0;
		for (DiffEntry entry : filterDiffs) {
			ByteArrayOutputStream out = new ByteArrayOutputStream();
			DiffFormatter df = new DiffFormatter(out);
			df.setDiffComparator(RawTextComparator.WS_IGNORE_ALL);
	        df.setRepository(repo);
			String path = diffDir+"diff"+String.valueOf(count)+".txt";
			BufferedWriter wr = new BufferedWriter(new FileWriter(new File(path)));
			df.format(entry);
			String diffText = out.toString("UTF-8");
//			System.out.println(diffText);
			wr.append(diffText);
			wr.close();
			df.close();
			count++;
        }
		String diffPath = diskpath+"cp"+String.valueOf(n)+"\\diffs.txt";
		File diffFile = new File(diffPath);
		if (!diffFile.getParentFile().exists()) {
			diffFile.getParentFile().mkdirs();
		}
		String tagPath = diskpath+"cp"+String.valueOf(n)+"\\tags.txt";
		BufferedWriter wr = new BufferedWriter(new FileWriter(diffFile));
		BufferedWriter wr1 = new BufferedWriter(new FileWriter(tagPath));
		wr.append(cp.getRepoName());
		wr.newLine();
		wr.append(oldCommitName+";"+newCommitName);
		wr.newLine();
		wr.flush();
		wr1.append("newCommit:\n"+newCommit.getFullMessage());
		wr1.newLine();
		wr1.append("oldCommit:\n"+oldCommit.getFullMessage());
		wr1.close();
		
		for (DiffEntry entry : filterDiffs) {
			wr.append(entry.getOldPath()+";"+entry.getNewPath());
			wr.newLine();
			wr.flush();
			String newFilePath = classPath+entry.getNewPath();
			String copyPath = movePath+entry.getNewPath();
			FileOperation.copyFile(new File(newFilePath), new File(copyPath));//copy changeFile	
        }
		wr.close();
//		Thread.sleep(5000);
		
		String movePath1 = diskpath+"cp"+String.valueOf(n)+"\\"+oldCommitName+"\\";
		String line1 = "cmd.exe /C git checkout -f "+oldCommitName;
		CommandLine cmdLine1 = CommandLine.parse(line1);
		DefaultExecuteResultHandler resultHandler1 = new DefaultExecuteResultHandler();
		DefaultExecutor executor1 = new DefaultExecutor();		
		ExecuteWatchdog watchdog1 = new ExecuteWatchdog(300000);//timeout 10s
		executor1.setExitValues(exitvalues);
		executor1.setWorkingDirectory(new File(classPath));
		executor1.setWatchdog(watchdog1);
		executor1.execute(cmdLine1, resultHandler1);
		Thread.sleep(6000);
		
		for (DiffEntry entry : filterDiffs) {
			String oldFilePath = classPath+entry.getOldPath();
			String copyPath = movePath1+entry.getOldPath();
			FileOperation.copyFile(new File(oldFilePath), new File(copyPath));//copy changeFile	
        }
		resultHandler1.waitFor();
//		Thread.sleep(5000);				
		return n;
	}//Execute checkout and copy diffs
	
	
	public static ArrayList<DiffEntry> getUsefulDiffs(List<DiffEntry> diffs){
		Boolean containTestcase = true;
//		for (DiffEntry entry : diffs) {
//			String oldFilePath = entry.getOldPath();
//			String newFilePath = entry.getNewPath();
//			if(oldFilePath.contains("Test")&&newFilePath.contains("Test")){
//				containTestcase = true;
//				break;
//			}
//		}
			
		ArrayList<DiffEntry> filterDiffs = new ArrayList<DiffEntry>();
		if(containTestcase) {
			for (DiffEntry entry : diffs) {
				String oldFilePath = entry.getOldPath();
				System.out.println("old:"+oldFilePath);
				String newFilePath = entry.getNewPath();
				System.out.println("new:"+newFilePath);
				System.out.println("---------");
				if(oldFilePath.contains("/dev/null")||newFilePath.contains("/dev/null")) {
					continue;//filter changepair
				}else if(oldFilePath.contains(".java")&&newFilePath.contains(".java")){
					filterDiffs.add(entry);
				}
//				else if((oldFilePath.contains(".cpp")||oldFilePath.contains(".CPP")||
//						oldFilePath.contains(".cc")||oldFilePath.contains(".h")||oldFilePath.contains(".c"))
//						&&(newFilePath.contains(".cpp")||newFilePath.contains(".CPP")||
//								newFilePath.contains(".cc")||newFilePath.contains(".h")||newFilePath.contains(".c"))){
//					filterDiffs.add(entry);
//				}			
	        }
		}
		return filterDiffs;
	}
	
	public static Integer continueProcess(String rootPath) {
		File rootFile = new File(rootPath);
		File[] dirs = rootFile.listFiles();
		int n = 0;
		if (dirs.length==0)
			return n;
		for(File dir : dirs) {
			String cpName = dir.getName();
			int cpNum = Integer.valueOf(cpName.substring(2, cpName.length()));
			if(cpNum>=n) {
				n = cpNum+1;
			}
		}
		return n;
	}//get cpNumber and continue from the next number
	
	public static ChangePair getChangPair(RevCommit revCommit, Repository repo) throws Exception {
		List<DiffEntry> returnDiffs = null;			
		RevCommit previousCommit=getPrevHash(revCommit, repo);
		System.out.println("PrevCommit:"+previousCommit.getName());
		
		try {			
			ObjectId head=revCommit.getTree().getId();			
			ObjectId oldHead=previousCommit.getTree().getId();			
			System.out.println("Printing diff between the Revisions: " + revCommit.getName() + " and " + previousCommit.getName());

            // prepare the two iterators to compute the diff between
    		try (ObjectReader reader = repo.newObjectReader()) {
        		CanonicalTreeParser oldTreeIter = new CanonicalTreeParser();
        		oldTreeIter.reset(reader, oldHead);
        		CanonicalTreeParser newTreeIter = new CanonicalTreeParser();
        		newTreeIter.reset(reader, head);

        		// finally get the list of changed files
        		try (Git git = new Git(repo)) {
                    List<DiffEntry> diffs= git.diff()
            		                    .setNewTree(newTreeIter)
            		                    .setOldTree(oldTreeIter)
            		                    .call();
                    returnDiffs=diffs;
        		} catch (GitAPIException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
    		}			
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		ChangePair cp = new ChangePair(revCommit, previousCommit, returnDiffs);
		return cp;
	}
	
	private static ArrayList<String> checkExistTests(String testPath) throws IOException{
		File[] testCPs = (new File(testPath)).listFiles();
		ArrayList<String> testHashs = new ArrayList<String>();
		for(File testCP : testCPs) {
			String diffPath = testCP.getAbsolutePath()+"\\diffs.txt";
			List<String> lines = FileUtils.readLines(new File(diffPath), "UTF-8");
//			String srcHash = lines.get(1).split(";")[0];
			String dstHash = lines.get(1).split(";")[1];
			testHashs.add(dstHash);
		}
		return testHashs;
	}
	
	public static RevCommit getPrevHash(RevCommit commit, Repository repo)  throws  IOException {
		RevWalk revWalk = new RevWalk(repo);
	    RevCommit previous = revWalk.parseCommit(commit.getParent(0).getId());
	    //Reached end and no previous commits.
	    revWalk.close();
	    return previous;
	}
	
	static void printTime(int commitTime) {
		SimpleDateFormat formatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
		String timestampString=String.valueOf(commitTime);
        Long timestamp = Long.parseLong(timestampString) * 1000;
        String date = formatter.format(new Date(timestamp));
        System.out.println(date);
	}
}
