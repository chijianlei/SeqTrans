package structure;

import java.util.ArrayList;
import java.util.HashMap;

import gumtreediff.actions.model.Action;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;

public class Transform {

	private SubTree sTree;
	private SubTree dTree;
	private HashMap<Integer, Integer> subMap;
	private HashMap<String, ArrayList<Action>> actMap;
	private String miName;


	public Transform(SubTree st, SubTree dt, HashMap<Integer, Integer> map,
			HashMap<String, ArrayList<Action>> actions, String name) {
		this.sTree = st;
		this.dTree = dt;
		this.subMap = map;
		this.actMap = actions;
		this.miName = name;
	}

	public ITree getSRoot() {
		return sTree.getRoot();
	}

	public ITree getDRoot() {
		return dTree.getRoot();
	}

	public SubTree getSTree() {
		return sTree;
	}

	public SubTree getDTree() {
		return dTree;
	}

	public TreeContext getSrcT() {
		return sTree.getTC();
	}

	public TreeContext getDstT() {
		return dTree.getTC();
	}

	public int getlineNum() {
		return sTree.getStNum();
	}

	public HashMap<Integer, Integer> getSubMap() {
		return subMap;
	}

	public HashMap<String, ArrayList<Action>> getActMap() {
		return actMap;
	}

	public String getMiName() {
		return miName;
	}


}
