package collect;

import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.apache.commons.exec.CommandLine;
import org.apache.commons.exec.DefaultExecuteResultHandler;
import org.apache.commons.exec.DefaultExecutor;
import org.eclipse.jgit.diff.DiffEntry;
import org.eclipse.jgit.diff.DiffFormatter;
import org.eclipse.jgit.diff.RawTextComparator;
import org.eclipse.jgit.lib.ObjectId;
import org.eclipse.jgit.lib.Repository;
import org.eclipse.jgit.revwalk.RevCommit;
import org.eclipse.jgit.revwalk.RevWalk;
import org.eclipse.jgit.storage.file.FileRepositoryBuilder;

import structure.API;
import structure.ChangePair;
import structure.Diff;
import utils.FileOperation;
import utils.ReadAPI;

public class DiffAnalysis {
	/**
	 * Analysing diffs between each commit pair
	 * @throws Exception 
	 */
	private static LinkedHashSet<API> apis = new LinkedHashSet<API>();
	private static LinkedHashSet<Diff> diffs = new LinkedHashSet<Diff>();
	private static HashMap<String, ArrayList<Diff>> diffMap = new HashMap<String, ArrayList<Diff>>();
	
	public static void main(String[] args) throws Exception{
		String path = "apis";
		apis = ReadAPI.readAPI(path);
		String versionCommit="28eb8dfd0ef959fd5ad7d5d22f1d32879707c0a0";//需要分析的Commit Hash		
		String endCommit = "0f4b89015308ca85c5304dd6e16c0c4b4c3cad3f";//到该Commit停止
		String path1="J:\\Telegram\\";//对应项目在本地Repo的路径
		autoExtraction(versionCommit, path1, endCommit);
	}		
	
	public static void autoExtraction(String versionCommit, String classPath, String endCommit) throws Exception {
		FileRepositoryBuilder builder = new FileRepositoryBuilder();
		builder.setMustExist(true);
		builder.addCeilingDirectory(new File(classPath));
		builder.findGitDir(new File(classPath));
		
		Repository repo;
		repo = builder.build();
		RevWalk walk = new RevWalk(repo);
		ObjectId versionId=repo.resolve(versionCommit);
		RevCommit currentCommit=walk.parseCommit(versionId);
		walk.close();			
		System.out.println(currentCommit.getName());
		LinkedList<RevCommit> commits = CommitFileList.getCommitList(currentCommit, repo);
		ArrayList<ChangePair> changePairs = new ArrayList<ChangePair>();
		for(int i=0;i<commits.size()-1;i++) {
			RevCommit newCommit = commits.get(i);
			RevCommit oldCommit = commits.get(i+1);
			ChangePair cp = CommitFileList.getChangPair(newCommit, oldCommit, repo);
			changePairs.add(cp);//all the changePairs in this list
			if(oldCommit.getId().toString().contains(endCommit))
				break;
		}
		for(ChangePair cp : changePairs) {			
			List<DiffEntry> diffs = cp.getDiffs();
			RevCommit newCommit = cp.getNewCommit();			
			RevCommit oldCommit = cp.getOldCommit();
			List<DiffEntry> filterDiffs = CommitFileList.getUsefulDiffs(diffs);
			for(int i=0;i<filterDiffs.size();i++) {
				DiffEntry diffEntry = filterDiffs.get(i);
				analyseDiffText(diffEntry, newCommit, oldCommit, repo);
			}
		}
		System.out.println("size:"+diffs.size());
		System.out.println("Mapsize:"+diffMap.size());
		int count=0;
		for(Map.Entry<String, ArrayList<Diff>> entry : diffMap.entrySet()) {
			String newCommitName = entry.getKey();
			ArrayList<Diff> diffList = entry.getValue();
			String oldCommitName = diffList.get(0).getOldCommitId();
			count = runExec(classPath, newCommitName, oldCommitName, count, diffList, repo);			
		}
	}
	
	private static int runExec(String classPath, String newCommitName, String oldCommitName, int i, ArrayList<Diff> diffList, Repository repo) throws Exception {
		String diskpath = "J:\\Telegram_test\\";
		String rootPath = diskpath+"cp"+String.valueOf(i)+"\\"+newCommitName+"\\";
		String line = "cmd.exe /C git checkout "+newCommitName;
		CommandLine cmdLine = CommandLine.parse(line);
		System.out.println("commit at:"+newCommitName);
		DefaultExecuteResultHandler resultHandler = new DefaultExecuteResultHandler();
		DefaultExecutor executor = new DefaultExecutor();			 			
		executor.setExitValue(1);	//设置命令执行退出值为1，如果命令成功执行并且没有错误，则返回1		
		executor.setWorkingDirectory(new File(classPath));//设置工作目录
		executor.execute(cmdLine, resultHandler);
		Thread.sleep(5000);
				
		System.out.println("Diffsize:"+diffList.size());
		if(diffList.size()==0) {
			return i;// continue the next iter
		}
		String diffPath = diskpath+"cp"+String.valueOf(i)+"\\diffs.txt";
		File diffFile = new File(diffPath);
		if (!diffFile.getParentFile().exists()) {
			diffFile.getParentFile().mkdirs();
		}
		BufferedWriter wr = new BufferedWriter(new FileWriter(diffFile));
		wr.append(oldCommitName+";"+newCommitName);
		wr.newLine();
		wr.flush();//第一行添加commit hash
		
		HashSet<String> fileList = new HashSet<String>();
		for (Diff diff : diffList) {			
			String filePath = diff.getNewPath();
			if(!fileList.contains(filePath)) {
				wr.append(diff.getOldPath()+";"+diff.getNewPath());
				wr.newLine();
				wr.flush();
				String newFilePath = classPath+filePath;
				String copyPath = rootPath+filePath;
				FileOperation.copyFile(new File(newFilePath), new File(copyPath));//copy changeFile	
			    fileList.add(filePath);
			}			
        }
		System.out.println("copy complete");
		resultHandler.waitFor();
		Thread.sleep(5000);
		
		String rootPath1 = diskpath+"cp"+String.valueOf(i)+"\\"+oldCommitName+"\\";
		String line1 = "cmd.exe /C git checkout "+oldCommitName;
		CommandLine cmdLine1 = CommandLine.parse(line1);
		System.out.println("commit at:"+oldCommitName);
		DefaultExecuteResultHandler resultHandler1 = new DefaultExecuteResultHandler();
		DefaultExecutor executor1 = new DefaultExecutor();			 			
		executor1.setExitValue(1);	//设置命令执行退出值为1，如果命令成功执行并且没有错误，则返回1		
		executor1.setWorkingDirectory(new File(classPath));//设置工作目录
		executor1.execute(cmdLine1, resultHandler1);
		Thread.sleep(5000);
		
		HashSet<String> fileList2 = new HashSet<String>();
		for (Diff diff : diffList) {
			String filePath = diff.getOldPath();
			if(!fileList2.contains(filePath)) {
				String oldFilePath = classPath+diff.getOldPath();
				String copyPath = rootPath1+diff.getOldPath();
				FileOperation.copyFile(new File(oldFilePath), new File(copyPath));//copy changeFile	
			    fileList2.add(filePath);
			}			
        }
		System.out.println("copy complete");
		resultHandler1.waitFor();
		Thread.sleep(5000);
		i++;
		wr.close();
		return i;
	}//Execute checkout and copy diffs
	
	public static void analyseDiffText(DiffEntry diffEntry, RevCommit newCommit, RevCommit oldCommit, Repository repo) throws Exception {
		ByteArrayOutputStream out = new ByteArrayOutputStream();
		DiffFormatter df = new DiffFormatter(out);
		df.setDiffComparator(RawTextComparator.WS_IGNORE_ALL);
        df.setRepository(repo);
		df.format(diffEntry);
		String diffText = out.toString("UTF-8");
		String[] texts = diffText.split("\n");
		String newCommitName = newCommit.getName();
		String oldCommitName = oldCommit.getName();
		for(int i=0;i<texts.length;i++) {
			String text = texts[i];
			int oldBeginLine = 0;
			int oldEndLine = 0;
			int newBeginLine = 0;
			int newEndLine = 0;
			if(i==0&&!text.contains(".java"))
				throw new Exception("error, not contains java file!");
			if(text.contains("@@")) {
				Pattern p = Pattern.compile("\\d{1,}");//这个1是指连续数字的最少个数  
				Matcher m = p.matcher(text);   
				int n = 0;
				while(m.find()) {
					if(n==0) {
						oldBeginLine = Integer.valueOf(m.group());
					}else if(n==1){
						oldEndLine = oldBeginLine+Integer.valueOf(m.group());
					}else if(n==2) {
						newBeginLine = Integer.valueOf(m.group());
					}else if(n==3) {
						newEndLine = newBeginLine+Integer.valueOf(m.group());
					}else
						throw new Exception("error n!");
					n++;						
				}
				i++;
				text = texts[i];
				String addLine = "";
				String deleteLine = "";
				while(!text.contains("@@")&&i<texts.length-1) {										
					String first = text.substring(0, 1);
					if(first.contains("+")) {
						addLine += text;
					}else if(first.contains("-")) {
						deleteLine += text;
					}
					i++;
					text = texts[i];
				}
				Boolean containsAPI = containsAPI(deleteLine, addLine);
				if (containsAPI) {
					String newid = newCommitName;
					String oldid = oldCommitName;
					String oldPath = diffEntry.getOldPath();
					String newPath = diffEntry.getNewPath();
					Diff diff = new Diff(oldid, newid, oldPath, newPath, deleteLine, addLine);
					diff.setOldBeginLine(oldBeginLine);
					diff.setOldEndLine(oldEndLine);
					diff.setNewBeginLine(newBeginLine);
					diff.setNewEndLine(newEndLine);
					diffs.add(diff);
					if(diffMap.get(newid)==null) {
						ArrayList<Diff> diffList = new ArrayList<Diff>();
						diffList.add(diff);
						diffMap.put(newid, diffList);
					}else {
						diffMap.get(newid).add(diff);
					}
				}
			}
		}
		
//		String path1 = "test//diff"+String.valueOf(count)+".txt";
//		BufferedWriter wr = new BufferedWriter(new FileWriter(new File(path1)));
//		System.out.println(diffText);
//		wr.append("test"+diffText);
//		wr.close();
		df.close();
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
