package structure;

import gumtreediff.tree.ITree;

public class Boundary {

	private ITree root;
	private int beginLine;
	private int lastLine;
	private int beginCol;
	private int lastCol;

	public Boundary(ITree root, int beginLine, int lastLine, int beginCol, int lastCol) {
		this.root = root;
		this.beginLine = beginLine;
		this.lastLine = lastLine;
		this.beginCol = beginCol;
		this.lastCol = lastCol;
	}

	public Boundary() {}

	public ITree getRoot() {
		return root;
	}

	public void setRoot(ITree root) {
		this.root = root;
	}

	public int getBeginLine() {
		return beginLine;
	}

	public void setBeginLine(int beginLine) {
		this.beginLine = beginLine;
	}

	public int getLastLine() {
		return lastLine;
	}

	public void setLastLine(int lastLine) {
		this.lastLine = lastLine;
	}

	public int getBeginCol() {
		return beginCol;
	}

	public void setBeginCol(int beginCol) {
		this.beginCol = beginCol;
	}

	public int getLastCol() {
		return lastCol;
	}

	public void setLastCol(int lastCol) {
		this.lastCol = lastCol;
	}

}
