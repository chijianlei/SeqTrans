package collect.testcase;

import java.io.File;

import org.eclipse.jgit.lib.ObjectId;
import org.eclipse.jgit.lib.Repository;
import org.eclipse.jgit.revwalk.RevCommit;
import org.eclipse.jgit.revwalk.RevWalk;
import org.eclipse.jgit.storage.file.FileRepositoryBuilder;

public class TestMissingUnkonw {
	
	public static void main(String[] args) throws Exception {
		// TODO Auto-generated method stub
		String classPath="J:\\git_repo\\tomcat70\\";
		String commit = "a4bfa01d4e6fd677f6831ab7b3e513c8b94c6185";
		FileRepositoryBuilder builder = new FileRepositoryBuilder();
		builder.setMustExist(true);
		builder.addCeilingDirectory(new File(classPath));
		builder.findGitDir(new File(classPath));
		Repository repo;
		repo = builder.build();
		RevWalk walk = new RevWalk(repo);
		ObjectId versionId=repo.resolve(commit);
		RevCommit currentCommit=walk.parseCommit(versionId);
	}

}
