package structure;

import java.util.List;
import java.util.Stack;

import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;

public class DTree {

	private ITree root;
	private List<ITree> leaves;
	private TreeContext treeContext;
	private Stack<String> trace;

	public DTree(ITree root, List<ITree> leaves, Stack<String> trace, TreeContext treeContext) {
		this.root = root;
		this.leaves = leaves;
		this.trace = trace;
		this.treeContext = treeContext;
	}

	public ITree getRoot() {
		return root;
	}

	public String getRootType() {
		String type = treeContext.getTypeLabel(root);
		return type;
	}

	public Stack<String> getTrace() {
		return trace;
	}

	public List<ITree> getLeaves() {
		return leaves;
	}

	public TreeContext getTreeContext() {
		return treeContext;
	}

}
