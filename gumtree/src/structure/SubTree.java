package structure;

import java.util.ArrayList;
import java.util.List;

import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;

public class SubTree {

	private ITree root;
	private TreeContext tc;
	private int stNum;
	private String miName;
	private ArrayList<ITree> parBlocks;
	private List<ITree> pars;//因为subtree同父亲已断开，计算pars调用这个List

	public SubTree(ITree node, TreeContext context, int count, String name) {
		root = node;
		tc = context;
		stNum = count;
		miName = name;
	}

	public ITree getRoot() {
		return root;
	}

	public TreeContext getTC() {
		return tc;
	}

	public int getStNum() {
		return stNum;
	}

	public String getMiName() {
		return miName;
	}

	public ArrayList<ITree> getParBlocks() {
		return parBlocks;
	}

	public void setParBlocks(ArrayList<ITree> parBlocks) {
		this.parBlocks = parBlocks;
	}

	public List<ITree> getPars() {
		return pars;
	}

	public void setPars(List<ITree> pars) {
		this.pars = pars;
	}

}
