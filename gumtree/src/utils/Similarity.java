package utils;

import java.util.ArrayList;
import java.util.List;

import apted.costmodel.StringUnitCostModel;
import apted.distance.APTED;
import apted.node.Node;
import apted.node.StringNodeData;
import apted.parser.BracketStringInputParser;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;
import structure.SubTree;

public class Similarity {
	private static TreeContext tCont;

	public static double getSimilarity(SubTree st1, SubTree st2) throws Exception {
		float editDistance = getEditDistance(st1, st2);
		ITree subRoot1 = st1.getRoot();
		ITree subRoot2 = st2.getRoot();
		List<ITree> nodes1 = new ArrayList<>();
		nodes1 = Utils.collectNode(subRoot1, nodes1);
		int nodeNum1 = nodes1.size();
		List<ITree> nodes2 = new ArrayList<>();
		nodes2 = Utils.collectNode(subRoot2, nodes2);
		int nodeNum2 = nodes2.size();
		int edgeNum1 = 0;
		edgeNum1 = Utils.collectEdge(subRoot1, edgeNum1);
		int edgeNum2 = 0;
		edgeNum2 = Utils.collectEdge(subRoot2, edgeNum2);
		double similarity = 1.0-editDistance/(nodeNum1+nodeNum2+edgeNum1+edgeNum2);
//		System.out.println(similarity);
		return similarity;
	}

	public static float getEditDistance(SubTree st1, SubTree st2) throws Exception {
		String srcTree = transfer2string(st1);
		String dstTree = transfer2string(st2);
//        System.out.println(srcTree);
//        System.out.println(dstTree);
		BracketStringInputParser parser = new BracketStringInputParser();
		Node<StringNodeData> t1 = parser.fromString(srcTree);
		Node<StringNodeData> t2 = parser.fromString(dstTree);
		// Initialise APTED.
		APTED<StringUnitCostModel, StringNodeData> apted = new APTED<>(new StringUnitCostModel());
		// Execute APTED.
		float result = apted.computeEditDistance(t1, t2);
//		System.out.println(result);
//		List<int[]> editMapping = apted.computeEditMapping();
//		for (int[] nodeAlignment : editMapping) {
//	        System.out.println(nodeAlignment[0] + "->" + nodeAlignment[1]);
//	    }
		return result;
	}

	public static String transfer2string(SubTree st) throws Exception {
		tCont = st.getTC();
		ITree root = st.getRoot();
		String bracketTree = "";
		List<ITree> children = root.getChildren();
		if(children.isEmpty())
			throw new Exception("check the root!");
		else {
			bracketTree = traverse(root, bracketTree, 0);
		}
		return bracketTree;
	}

	private static String traverse(ITree node, String bracketTree, int num) throws Exception {
		List<ITree> childs = node.getChildren();
		String type = tCont.getTypeLabel(node);
//		int size = node.getParent().getChildren().size()-1;
		bracketTree = bracketTree + "{"+tCont.getTypeLabel(node);
		if (node.hasLabel()&&childs.size()==0) {
			if(type.equals("name")) {
				if(node.isRoot())
					throw new Exception("why root???");
				ITree par = node.getParent();
				String parType = tCont.getTypeLabel(par);
				List<ITree> parChilds = par.getChildren();
				if(parType.equals("decl")&&parChilds.get(1).equals(node)) {//抽象掉decl的第二个节点也就是name部分的名字
					ITree firstChild = parChilds.get(0);
					String firstType = tCont.getTypeLabel(firstChild);
					if(firstType.equals("type")) {
						if(parChilds.size()==1)
							throw new Exception("unknown size");
						else
							bracketTree = bracketTree+"(declName)";
					}else
						bracketTree = bracketTree+"("+node.getLabel()+")";
				}else
					bracketTree = bracketTree+"("+node.getLabel()+")";
			}else
				bracketTree = bracketTree+"("+node.getLabel()+")";
		}

		for(int i=0;i<childs.size();i++) {
			ITree child = childs.get(i);
			bracketTree = traverse(child, bracketTree, i);
		}
		bracketTree = bracketTree +"}";
		return bracketTree;
	}//收集AST树中所有点

}
