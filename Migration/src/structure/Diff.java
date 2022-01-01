package structure;

public class Diff {
	
	private String oldCommitId;
	private String newCommitId;
	private String oldPath;
	private String newPath;
	private String addLine;
	private String deleteLine;
	private int oldBeginLine;
	private int oldEndLine = 0;
	private int newBeginLine = 0;
	private int newEndLine = 0;

	public Diff(String oldCommitId, String newCommitId, String oldPath, 
			String newPath, String deleteLine, String addLine) {
		this.oldCommitId = oldCommitId;
		this.newCommitId = newCommitId;
		this.oldPath = oldPath;
		this.newPath = newPath;
		this.deleteLine = deleteLine;
		this.addLine = addLine;
	}

	public String getOldCommitId() {
		return oldCommitId;
	}

	public String getNewCommitId() {
		return newCommitId;
	}

	public String getOldPath() {
		return oldPath;
	}

	public String getNewPath() {
		return newPath;
	}

	public int getOldBeginLine() {
		return oldBeginLine;
	}

	public int getOldEndLine() {
		return oldEndLine;
	}

	public int getNewBeginLine() {
		return newBeginLine;
	}

	public int getNewEndLine() {
		return newEndLine;
	}

	public void setOldBeginLine(int oldBeginLine) {
		this.oldBeginLine = oldBeginLine;
	}

	public void setOldEndLine(int oldEndLine) {
		this.oldEndLine = oldEndLine;
	}

	public void setNewBeginLine(int newBeginLine) {
		this.newBeginLine = newBeginLine;
	}

	public void setNewEndLine(int newEndLine) {
		this.newEndLine = newEndLine;
	}
	
	
}
