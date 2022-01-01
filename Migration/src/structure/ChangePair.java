package structure;

import java.util.List;

import org.eclipse.jgit.diff.DiffEntry;
import org.eclipse.jgit.revwalk.RevCommit;

public class ChangePair {
	private RevCommit newCommit;
	private RevCommit oldCommit;
	private String rootPath;
	private String repoName;
	private List<DiffEntry> diffs;
	
	public ChangePair(RevCommit newCommit, RevCommit oldCommit, List<DiffEntry> diffs) {
		this.newCommit = newCommit;
		this.oldCommit = oldCommit;
		this.diffs = diffs;
	}
	
	public ChangePair() {}
	
	public RevCommit getNewCommit() {
		return newCommit;
	}
	public RevCommit getOldCommit() {
		return oldCommit;
	}
	
	public String getRootPath() {
		return rootPath;
	}

	public String getRepoName() {
		return repoName;
	}

	public void setRootPath(String rootPath) {
		this.rootPath = rootPath;
	}

	public void setRepoName(String repoName) {
		this.repoName = repoName;
	}

	public List<DiffEntry> getDiffs() {
		return diffs;
	}
	public void setNewCommit(RevCommit newCommit) {
		this.newCommit = newCommit;
	}
	public void setOldCommit(RevCommit oldCommit) {
		this.oldCommit = oldCommit;
	}
	public void setDiffs(List<DiffEntry> diffs) {
		this.diffs = diffs;
	}
	
	

}
