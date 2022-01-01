package apted.util;

import java.util.List;

import apted.costmodel.StringUnitCostModel;
import apted.distance.APTED;
import apted.node.Node;
import apted.node.StringNodeData;
import apted.parser.BracketStringInputParser;

public class APTEDtest {

	public static void main (String args[]) throws Exception{
		String srcTree = "{decl_stmt{decl{type{name{name}{operator}{name}}}{name}}}";
		String dstTree = "{expr_stmt{expr{call{name{name}{operator}{name}}{argument_list{argument{expr{name}}}{argument{expr{name}}}}}}}";
		BracketStringInputParser parser = new BracketStringInputParser();
		Node<StringNodeData> t1 = parser.fromString(srcTree);
		Node<StringNodeData> t2 = parser.fromString(dstTree);
		// Initialise APTED.
		APTED<StringUnitCostModel, StringNodeData> apted = new APTED<>(new StringUnitCostModel());
		// Execute APTED.
		float result = apted.computeEditDistance(t1, t2);
		System.out.println(result);
		List<int[]> editMapping = apted.computeEditMapping();
	      for (int[] nodeAlignment : editMapping) {
	        System.out.println(nodeAlignment[0] + "->" + nodeAlignment[1]);
	      }
	}

}
