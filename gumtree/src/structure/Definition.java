package structure;

import java.util.List;

import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;

public class Definition {

	private int defLabelID;
	private ITree root;
	private String type;
	private String varName;
	private ITree block;
	private List<ITree> parBlocks;
	private List<ITree> parList;
	private TreeContext tc;

	public int getDefLabelID() {
		return defLabelID;
	}
	public ITree getRoot() {
		return root;
	}
	public String getType() {
		return type;
	}
	public String getVarName() {
		return varName;
	}
	public ITree getBlock() {
		return block;
	}
	public List<ITree> getParBlocks() {
		return parBlocks;
	}
	public List<ITree> getParList() {
		return parList;
	}
	public void setDefLabelID(int defLabelID) {
		this.defLabelID = defLabelID;
	}
	public void setRoot(ITree root) {
		this.root = root;
	}
	public void setType(String type) {
		this.type = type;
	}
	public void setVarName(String varName) {
		this.varName = varName;
	}
	public void setBlock(ITree block) {
		this.block = block;
	}
	public void setParBlocks(List<ITree> parBlocks) {
		this.parBlocks = parBlocks;
	}
	public void setParList(List<ITree> parList) {
		this.parList = parList;
	}
	public TreeContext getTc() {
		return tc;
	}
	public void setTc(TreeContext tc) {
		this.tc = tc;
	}

}
