package test;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import org.eclipse.jgit.errors.IncorrectObjectTypeException;
import org.eclipse.jgit.lib.ObjectId;
import org.eclipse.jgit.lib.Ref;
import org.eclipse.jgit.lib.Repository;
import org.eclipse.jgit.revwalk.RevCommit;
import org.eclipse.jgit.revwalk.RevSort;
import org.eclipse.jgit.revwalk.RevWalk;
import org.eclipse.jgit.storage.file.FileRepositoryBuilder;

public class testJGit {
	
	public static void main(String[] args) throws Exception {
		String classPath = "F:\\tmp\\activemq\\";
		String commit = "943158555356e8caa1068f6bde02596569b1d391";
		FileRepositoryBuilder builder = new FileRepositoryBuilder();
		builder.setMustExist(true);
		builder.addCeilingDirectory(new File(classPath));
		builder.findGitDir(new File(classPath));
		Repository repo;
		repo = builder.build();
		testJGit test = new testJGit();
		
		List<ObjectId> branchesIds = test.getAllBranchObjectId(null, repo);
		System.out.println(branchesIds.size());
		RevWalk revWalk = test.getAllRevWalk(branchesIds, repo);
        Iterator<RevCommit> iter = revWalk.iterator();
        ArrayList<RevCommit> commits1 = new ArrayList<>();
        while (iter.hasNext()) {
            RevCommit tmp = iter.next();
            commits1.add(tmp);
        }
        System.out.println(commits1.size());
		
		RevWalk walk = new RevWalk(repo);
		ObjectId versionId=repo.resolve(commit);
		RevCommit currentCommit = walk.parseCommit(versionId);
		ArrayList<RevCommit> commits2 = test.getCommits(currentCommit, repo);
		System.out.println(commits2.size());
	}
	
    /**
     * Get jsonops ids of all the branches named like %branch% in a repository
     * If branch == null, retrieve all branches
     */
    public List<ObjectId> getAllBranchObjectId(String branch, Repository repo) throws Exception{
        List<ObjectId> currentRemoteRefs = new ArrayList<ObjectId>();
        for (Ref ref: repo.getRefDatabase().getRefs()){
            String refName = ref.getName();
            if (branch == null || refName.endsWith("/" + branch)) {
                currentRemoteRefs.add(ref.getObjectId());
            }
        }
        return currentRemoteRefs;
    }
    
    public RevWalk getAllRevWalk(List<ObjectId> remoteRefs, Repository repo) throws Exception{
        RevWalk walk = createReverseRevWalk(repo);
        for (ObjectId refId: remoteRefs){
            RevCommit start;
            try {
                start = walk.parseCommit(refId);
            } catch (IncorrectObjectTypeException e){
                continue;
            }
            walk.markStart(start);
        }
        // Filter all merge changes
        //walk.setRevFilter(RevFilter.NO_MERGES);
        return walk;
    }
    
    private RevWalk createReverseRevWalk(Repository repo){
        RevWalk walk = new RevWalk(repo);
        walk.sort(RevSort.COMMIT_TIME_DESC, true);
        walk.sort(RevSort.REVERSE, true);
        return walk;
    }
    
	public ArrayList<RevCommit> getCommits(RevCommit commit, Repository repo)  throws  IOException {
	    ArrayList<RevCommit> commits = new ArrayList<>();
		try (RevWalk walk = new RevWalk(repo)) {
	        // Starting point
	        walk.markStart(commit);
	        int count = 0;
	        for (RevCommit rev : walk) {
	            // got the previous commit.
	        	commits.add(rev);
	            count++;
	        }
	        walk.dispose();
	    }
	    //Reached end and no previous commits.
	    return commits;
	}

}
