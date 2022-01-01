package collect;

import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedList;
import java.util.List;

import org.apache.commons.exec.CommandLine;
import org.apache.commons.exec.DefaultExecuteResultHandler;
import org.apache.commons.exec.DefaultExecutor;
import org.eclipse.jgit.api.Git;
import org.eclipse.jgit.api.errors.GitAPIException;
import org.eclipse.jgit.diff.DiffEntry;
import org.eclipse.jgit.diff.DiffFormatter;
import org.eclipse.jgit.diff.RawTextComparator;
import org.eclipse.jgit.lib.ObjectId;
import org.eclipse.jgit.lib.ObjectReader;
import org.eclipse.jgit.lib.Repository;
import org.eclipse.jgit.revwalk.RevCommit;
import org.eclipse.jgit.revwalk.RevWalk;
import org.eclipse.jgit.storage.file.FileRepositoryBuilder;
import org.eclipse.jgit.treewalk.CanonicalTreeParser;

import structure.ChangePair;
import utils.FileOperation;

public class CommitFileList {
	/**
	 * Collecting API change pairs from git commit diff logs
	 * @throws Exception 
	 */
	public static void main(String[] args) throws Exception {
		// TODO Auto-generated method stub
		String versionCommit="66471836f584d5c73be18367e1db4c4783b0cb48";//��Ҫ������Commit Hash		
		String path="D:\\workspace\\poi\\";//��Ӧ��Ŀ�ڱ���Repo��·��
//		autoExtraction(versionCommit, path);
		getChangeList(versionCommit, path);
	}
	public static void getChangeList(String versionCommit, String classPath) throws Exception {
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
		LinkedList<RevCommit> commits = getCommitList(currentCommit, repo);
		ArrayList<ChangePair> changePairs = new ArrayList<ChangePair>();
		for(int i=0;i<commits.size()-1;i++) {
			RevCommit newCommit = commits.get(i);
			RevCommit oldCommit = commits.get(i+1);
			ChangePair cp = getChangPair(newCommit, oldCommit, repo);
			changePairs.add(cp);//all the changePairs in this list			
		}
		
		File list = new File("changeList.txt");
		BufferedWriter wr = new BufferedWriter(new FileWriter(list));
		for(int i=0;i<changePairs.size();i++) {
			ChangePair cp = changePairs.get(i);
			RevCommit newCommit = cp.getNewCommit();
			String newCommitName = newCommit.getName();
			RevCommit oldCommit = cp.getOldCommit();
			String oldCommitName = oldCommit.getName();
			wr.append(newCommitName+","+oldCommitName);
			wr.newLine();
			wr.flush();
		}
		wr.close();						
	}
	
	public static void autoExtraction(String versionCommit, String classPath) throws Exception {
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
		LinkedList<RevCommit> commits = getCommitList(currentCommit, repo);
		ArrayList<ChangePair> changePairs = new ArrayList<ChangePair>();
		for(int i=0;i<commits.size()-1;i++) {
			RevCommit newCommit = commits.get(i);
			RevCommit oldCommit = commits.get(i+1);
			ChangePair cp = getChangPair(newCommit, oldCommit, repo);
			changePairs.add(cp);//all the changePairs in this list			
		}
		int count = 0;
		for(ChangePair cp : changePairs) {
			RevCommit newCommit = cp.getNewCommit();			
			RevCommit oldCommit = cp.getOldCommit();			
			List<DiffEntry> diffs = cp.getDiffs();
			count = runExec(classPath, newCommit, oldCommit, count, diffs, repo);
			System.out.println("endExec");
		}			
	}
	
	private static int runExec(String classPath, RevCommit newCommit, RevCommit oldCommit, int i, List<DiffEntry> diffs, Repository repo) throws Exception {
		String diskpath = "J:\\Telegram_commit\\";
		String newCommitName = newCommit.getName();
		String oldCommitName = oldCommit.getName();
		String rootPath = diskpath+"cp"+String.valueOf(i)+"\\"+newCommitName+"\\";
		String line = "cmd.exe /C git checkout "+newCommitName;
		CommandLine cmdLine = CommandLine.parse(line);
		DefaultExecuteResultHandler resultHandler = new DefaultExecuteResultHandler();
		DefaultExecutor executor = new DefaultExecutor();			 			
		executor.setExitValue(1);	//��������ִ���˳�ֵΪ1���������ɹ�ִ�в���û�д����򷵻�1		
		executor.setWorkingDirectory(new File(classPath));//���ù���Ŀ¼
		executor.execute(cmdLine, resultHandler);
		Thread.sleep(1000);
				
		ArrayList<DiffEntry> filterDiffs = getUsefulDiffs(diffs);
		System.out.println("Diffsize:"+filterDiffs.size());
		if(filterDiffs.size()==0) {
			return i;// continue the next iter
		}
		String diffDir = diskpath+"cp"+String.valueOf(i)+"\\diff_logs\\";
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
		String diffPath = diskpath+"cp"+String.valueOf(i)+"\\diffs.txt";
		File diffFile = new File(diffPath);
		if (!diffFile.getParentFile().exists()) {
			diffFile.getParentFile().mkdirs();
		}
		String tagPath = diskpath+"cp"+String.valueOf(i)+"\\tags.txt";
		BufferedWriter wr = new BufferedWriter(new FileWriter(diffFile));
		BufferedWriter wr1 = new BufferedWriter(new FileWriter(tagPath));
		wr.append(oldCommitName+";"+newCommitName);
		wr.newLine();
		wr.flush();//��һ�����commit hash
		wr1.append("newCommit:\n"+newCommit.getFullMessage());
		wr1.newLine();
		wr1.append("oldCommit:\n"+oldCommit.getFullMessage());
		wr1.close();
		
		for (DiffEntry entry : filterDiffs) {
			wr.append(entry.getOldPath()+";"+entry.getNewPath());
			wr.newLine();
			wr.flush();
			String newFilePath = classPath+entry.getNewPath();
			String copyPath = rootPath+entry.getNewPath();
			FileOperation.copyFile(new File(newFilePath), new File(copyPath));//copy changeFile	
        }
		resultHandler.waitFor();
		Thread.sleep(5000);
		
		String rootPath1 = diskpath+"cp"+String.valueOf(i)+"\\"+oldCommitName+"\\";
		String line1 = "cmd.exe /C git checkout "+oldCommitName;
		CommandLine cmdLine1 = CommandLine.parse(line1);
		DefaultExecuteResultHandler resultHandler1 = new DefaultExecuteResultHandler();
		DefaultExecutor executor1 = new DefaultExecutor();			 			
		executor1.setExitValue(1);	//��������ִ���˳�ֵΪ1���������ɹ�ִ�в���û�д����򷵻�1		
		executor1.setWorkingDirectory(new File(classPath));//���ù���Ŀ¼
		executor1.execute(cmdLine1, resultHandler1);
		Thread.sleep(1000);
		
		for (DiffEntry entry : filterDiffs) {
			String oldFilePath = classPath+entry.getOldPath();
			String copyPath = rootPath1+entry.getOldPath();
			FileOperation.copyFile(new File(oldFilePath), new File(copyPath));//copy changeFile	
        }
		resultHandler1.waitFor();
		Thread.sleep(5000);
		i++;
		wr.close();
		return i;
	}//Execute checkout and copy diffs
	
	
	public static ArrayList<DiffEntry> getUsefulDiffs(List<DiffEntry> diffs){
		ArrayList<DiffEntry> filterDiffs = new ArrayList<DiffEntry>();
		for (DiffEntry entry : diffs) {
			String oldFilePath = entry.getOldPath();
			String newFilePath = entry.getNewPath();
			if(oldFilePath.contains("/dev/null")||newFilePath.contains("/dev/null")) {
				continue;//������ɾ���ļ���������ļ����������ʾ·��Ϊ���������Ҫ����changepair
			}else if(oldFilePath.contains(".java")&&newFilePath.contains(".java")){
				filterDiffs.add(entry);
			}//ɾ�����ļ�commit���Ҳ���
        }
		return filterDiffs;
	}
	
	public static LinkedList<RevCommit> getCommitList(RevCommit startCommit, Repository repo) throws Exception{
		LinkedList<RevCommit> commits = new LinkedList<RevCommit>();
		RevWalk walk = new RevWalk(repo);
		walk.markStart(startCommit);
		for(RevCommit rev : walk) {
			commits.add(rev);
		}
		walk.close();
		return commits;		
	}
	
	public static ChangePair getChangPair(RevCommit commit1, RevCommit commit2, Repository repo) throws Exception {
		List<DiffEntry> returnDiffs = null;
		ObjectId head = commit1.getTree().getId();			
		ObjectId oldHead = commit2.getTree().getId();
		
//		System.out.println("Printing diff between the Revisions: " + commit1.getName() + " and " + commit2.getName());

        // prepare two iterators to compute the diffs
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
//                for (DiffEntry entry : returnDiffs) {
//    				System.out.println(entry.getNewPath());
//                    System.out.println("------------------");
//                }
    		} catch (GitAPIException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		ChangePair cp = new ChangePair(commit1, commit2, returnDiffs);
		return cp;
	}
	
	public static RevCommit getPrevHash(RevCommit commit, Repository repo)  throws  IOException {
	    RevCommit previous = null;
		try (RevWalk walk = new RevWalk(repo)) {
	        // Starting point
	        walk.markStart(commit);
	        int count = 0;	        
	        for (RevCommit rev : walk) {
	            // got the previous commit.
	            if (count == 1) {
	            	previous = rev;
	            }
	            count++;
	        }
	    }
	    //Reached end and no previous commits.
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
