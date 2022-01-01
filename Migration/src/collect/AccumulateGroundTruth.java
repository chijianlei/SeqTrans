package collect;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.apache.commons.exec.CommandLine;
import org.apache.commons.exec.DefaultExecuteResultHandler;
import org.apache.commons.exec.DefaultExecutor;
import org.apache.commons.exec.ExecuteException;
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

public class AccumulateGroundTruth {
	/**
	 * Collecting Ground Truth by Accumulating diffs in the same file.
	 * @throws Exception 
	 */
	private static HashMap<String, ArrayList<String>> accuDiffFileMaps = new HashMap<String, ArrayList<String>>();
    private static HashMap<String, Integer> accuDiffLineMaps = new HashMap<String, Integer>();
	
	public static void main(String[] args) throws Exception {
		String versionCommit="8fd0197cd3710786212a5bba1545bc9513fe74cc";//需要分析的Commit Hash		
		String path="J:\\junit4\\";//对应项目在本地Repo的路径
		autoExtraction(versionCommit, path);
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
//		System.out.println(currentCommit.getName());
//		List<DiffEntry> diffFix=getChangedFileList(currentCommit, repo);
//		System.out.println("size:"+diffFix.size());
		LinkedList<RevCommit> commits = getCommitList(currentCommit, repo);
		walk.close();
		for(int i=0;i<commits.size()-1;i++) {
			RevCommit newCommit = commits.get(i);
			RevCommit oldCommit = commits.get(i+1);
			filterDiffs(newCommit, oldCommit, repo);
		}	
		System.out.println("accuSize:"+accuDiffFileMaps.size());
		for(Map.Entry<String, ArrayList<String>> entry : accuDiffFileMaps.entrySet()) {
			String name = entry.getKey();
			ArrayList<String> arrys = entry.getValue();
			System.out.println(name+","+accuDiffLineMaps.get(name));
			if(arrys.size()>=2) {
				String oldCommit = arrys.get(0);
				System.out.println("oldCommit:"+oldCommit);
				String newCommit = arrys.get(arrys.size()-1);
				System.out.println("newCommit:"+newCommit);
				int oldLOC = readLOC(classPath, name, oldCommit, repo);
				int newLOC = readLOC(classPath, name, newCommit, repo);
				System.out.println("LOC:"+oldLOC+","+newLOC);
			}
		}
//		for(Map.Entry<String, Integer> entry : accuDiffLineMaps.entrySet()) {
//			String name = entry.getKey();
//			int DiffNum = entry.getValue();
//			System.out.println(name+","+DiffNum);
//		}
	}
	
	public static Integer readLOC(String classPath, String fileName, String commit, Repository repo) throws Exception {
		String commitName = commit;
		String command = "cmd.exe /C git checkout "+commitName;
		CommandLine cmdLine = CommandLine.parse(command);
		DefaultExecuteResultHandler resultHandler = new DefaultExecuteResultHandler();
		DefaultExecutor executor = new DefaultExecutor();			 			
		executor.setExitValue(1);	//设置命令执行退出值为1，如果命令成功执行并且没有错误，则返回1		
		executor.setWorkingDirectory(new File(classPath));//设置工作目录
		executor.execute(cmdLine, resultHandler);
		Thread.sleep(5000);
		
		RevWalk walk = new RevWalk(repo);
		ObjectId versionId=repo.resolve(commit);
		RevCommit currentCommit=walk.parseCommit(versionId);
		System.out.println(currentCommit.getName());
		List<DiffEntry> changedList = getChangedFileList(currentCommit, repo);
		walk.close();
		if(changedList==null)
			return 0;
		for(DiffEntry diff : changedList) {
			String newAddPath = diff.getNewPath();
			String[] arrys = newAddPath.split("/");
			String newAddFile = arrys[arrys.length-1];
			if(newAddFile.equals(fileName)) {
				String fileroot = classPath+"\\"+newAddPath;
				File file = new File(fileroot);
				BufferedReader br = new BufferedReader(new FileReader(file));
				String tmpLine = "";
				int count = 0;
				while((tmpLine=br.readLine())!=null) {
					if(tmpLine!=null)
						count++;
				}
				br.close();
				return count;
			}
		}
		return 0;
	}
	
	public static ChangePair filterDiffs(RevCommit commit1, RevCommit commit2, Repository repo) throws Exception {
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
                for (DiffEntry entry : returnDiffs) {
    				String newAddPath = entry.getNewPath();
//    				System.out.println(newAddPath);
    				String[] arrys = newAddPath.split("/");
    				String newAddFile = arrys[arrys.length-1];
    				if(!newAddFile.contains(".java"))
						continue;
                	ByteArrayOutputStream out = new ByteArrayOutputStream();
            		DiffFormatter df = new DiffFormatter(out);
            		df.setDiffComparator(RawTextComparator.WS_IGNORE_ALL);
                    df.setRepository(repo);
                	df.format(entry);
        			String diffText = out.toString("UTF-8");
//        			System.out.println(diffText);
        			String[] lines = diffText.split("\n");
        			int diffLineNum = 0;
        			for(String line : lines) {
        				if(line.length()<=1)
        					continue;
        				String firstChar = line.substring(0, 1);        				
        				String secondChar = line.substring(1, 2);
        				if(firstChar.equals("-")||firstChar.equals("+")) {
        					if(secondChar.equals("-")||secondChar.equals("+"))
        						continue;//这种句子不是diff语句是标记修改文件名的
        					String diffLine = line.substring(1);
//        					System.out.println("line:"+diffLine);
        					if (diffLine!=null) {
								diffLineNum++;
							}
        				}
        			}
        			if(accuDiffLineMaps.get(newAddFile)!=null) {
        				diffLineNum = diffLineNum+accuDiffLineMaps.get(newAddFile);
        			}
        			accuDiffLineMaps.put(newAddFile, diffLineNum);
        			
//        			System.out.println("lines:"+lines.length);
    				if(accuDiffFileMaps.get(newAddFile)==null) {   					
						ArrayList<String> commits = new ArrayList<String>();
						commits.add(commit1.getName());
						accuDiffFileMaps.put(newAddFile, commits);
					}else {
						accuDiffFileMaps.get(newAddFile).add(commit1.getName());
					}
    				df.close();
                }
    		} catch (GitAPIException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		ChangePair cp = new ChangePair(commit1, commit2, returnDiffs);
		return cp;
	}
	
	static List<DiffEntry> getChangedFileList(RevCommit revCommit, Repository repo) {
		List<DiffEntry> returnDiffs = null;		
		try {
			RevCommit previsouCommit=getPrevHash(revCommit,repo);
			if(previsouCommit==null)
				return null;
			ObjectId head=revCommit.getTree().getId();
			
			ObjectId oldHead=previsouCommit.getTree().getId();
			
			System.out.println("Printing diff between the Revisions: " + revCommit.getName() + " and " + previsouCommit.getName());

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
		return returnDiffs;
	}
	
	public static RevCommit getPrevHash(RevCommit commit, Repository repo)  throws  IOException {
	    try (RevWalk walk = new RevWalk(repo)) {
	        // Starting point
	        walk.markStart(commit);
	        int count = 0;
	        for (RevCommit rev : walk) {
	            // got the previous commit.
	            if (count == 1) {
	                return rev;
	            }
	            count++;
	        }
	        walk.dispose();
	    }
	    //Reached end and no previous commits.
	    return null;
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
}
