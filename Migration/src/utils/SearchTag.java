package utils;

import java.io.File;
import java.io.IOException;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;

import org.eclipse.jgit.api.Git;
import org.eclipse.jgit.api.LogCommand;
import org.eclipse.jgit.api.errors.GitAPIException;
import org.eclipse.jgit.lib.ObjectId;
import org.eclipse.jgit.lib.Ref;
import org.eclipse.jgit.lib.Repository;
import org.eclipse.jgit.revwalk.RevCommit;
import org.eclipse.jgit.revwalk.RevTag;
import org.eclipse.jgit.storage.file.FileRepositoryBuilder;

public class SearchTag {
	public static void main(String[] args) throws Exception{
		String path = "I:\\uaa";
		String commit = "5dc5ca9176ed5baa870680d99f37e7e559dddc5d";
		searchTag(path, commit);
	}
	
	public static void searchTag(String classPath, String commit) throws Exception {
		FileRepositoryBuilder builder = new FileRepositoryBuilder();
		builder.setMustExist(true);
		builder.addCeilingDirectory(new File(classPath));
		builder.findGitDir(new File(classPath));
		Repository repo;
		repo = builder.build();
		Git git = new Git(repo);
		List<Ref> list = git.tagList().call();
		ObjectId commitId = ObjectId.fromString(commit);
//		Collection<String> commits = new LinkedList<String>();
//		for (Ref tag : list) {
//			System.out.println(tag.getName());
//			ObjectId object = tag.getObjectId();
//			
//			System.out.println(object.toString());
//		    if (object.equals(commitId)) {
//		        System.out.println(tag.getName());
//		        break;
//		    }
//		}		        
        
        for(Ref testTag : list) {
        	@SuppressWarnings("resource")
			LogCommand log = new Git(repo).log();
//        	System.out.println(testTag.getName());
            Ref peeledRef = repo.getRefDatabase().peel(testTag);
            if(peeledRef.getPeeledObjectId() != null) {
                log.add(peeledRef.getPeeledObjectId());
            } else {
                log.add(testTag.getObjectId());
            }

            Iterable<RevCommit> logs = log.call();
            for (RevCommit rev : logs) {
    		    if (rev.getId().equals(commitId)) {
    	        System.out.println(testTag.getName());
    	        break;
    	    }
//                System.out.println("Commit: " + rev /* + ", name: " + rev.getName() + ", id: " + rev.getId().getName() */);
            }
        }             
        git.close();
	}

}
