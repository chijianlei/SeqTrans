package test;

import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

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

public class Debug {
	public static void main(String[] args) throws IOException {
		// TODO Auto-generated method stub
		String versionCommit="d72bd78c19dfb7b57395a66ae8d9269d59a87bd2";//需要分析的Commit Hash		
		String path="J:\\git_repo\\cxf\\";//对应项目在本地Repo的路径
		FileRepositoryBuilder builder = new FileRepositoryBuilder();
		builder.setMustExist(true);
		builder.addCeilingDirectory(new File(path));
		builder.findGitDir(new File(path));
		
		Repository repo;
		repo = builder.build();
		RevWalk walk = new RevWalk(repo);
		ObjectId versionId=repo.resolve(versionCommit);
		RevCommit currentCommit=walk.parseCommit(versionId);
		System.out.println(currentCommit.getName());
		List<DiffEntry> diffFix=getChangedFileList(currentCommit, repo);
        int count = 0;  
        System.out.println("size:"+diffFix.size());
		for (DiffEntry entry : diffFix) {
			ByteArrayOutputStream out = new ByteArrayOutputStream();
			DiffFormatter df = new DiffFormatter(out);
			df.setDiffComparator(RawTextComparator.WS_IGNORE_ALL);
	        df.setRepository(repo);
			String path1 = "test//diff"+String.valueOf(count)+".txt";
			BufferedWriter wr = new BufferedWriter(new FileWriter(new File(path1)));
			df.format(entry);
			String diffText = out.toString("UTF-8");
//			System.out.println(diffText);
			wr.append("test"+diffText);
			wr.close();
			df.close();
			count++;
//			System.out.println(entry.getOldPath());
        }
		
		walk.close();
//		RevWalk walk2 = new RevWalk(repo);
//		ObjectId versionId2=repo.resolve(versionCommit);
//		RevCommit verCommit2=walk2.parseCommit(versionId2);
//		List<DiffEntry> diffFix2=RunJGit.getChangedFileList(verCommit2,repo);
//		for (DiffEntry entry : diffFix2) {
//			System.out.println(entry.getNewPath());
//      }	
	}
	
	public static List<DiffEntry> getChangedFileList(RevCommit revCommit, Repository repo) {
		List<DiffEntry> returnDiffs = null;
		try {
			RevCommit previousCommit=getPrevHash(revCommit,repo);
			if(previousCommit==null)
				return null;
			ObjectId head=revCommit.getTree().getId();
			
			ObjectId oldHead=previousCommit.getTree().getId();
			
			System.out.println("Printing diff between the Revisions: " + revCommit.getName() + " and " + previousCommit.getName());

            // prepare the two iterators to compute the diff between
    		ObjectReader reader = repo.newObjectReader();
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
                for (DiffEntry entry : diffs) {
//                    System.out.println("Entry: " + entry);
                }
                returnDiffs=diffs;
    		} catch (GitAPIException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return returnDiffs;
	}
	
	public static RevCommit getPrevHash(RevCommit commit, Repository repo)  throws  IOException {
	    try(RevWalk walk = new RevWalk(repo)){
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
	
	static void printTime(int commitTime) {
		SimpleDateFormat formatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
		String timestampString=String.valueOf(commitTime);
        Long timestamp = Long.parseLong(timestampString) * 1000;
        String date = formatter.format(new Date(timestamp));
        System.out.println(date);
	}
	
}

