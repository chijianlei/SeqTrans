package split;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import gumtreediff.gen.srcml.SrcmlCppTreeGenerator;
import gumtreediff.matchers.MappingStore;
import gumtreediff.matchers.Matcher;
import gumtreediff.matchers.Matchers;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;
import utils.Utils;

/*
 * Prune the tree, clean up the data to extract accurate rules.
 */

public class Pruning {
	private TreeContext srcT;
	private TreeContext dstT;
	private MappingStore mappings;
	private ArrayList<PruneTuple> pts1 = new ArrayList<>();
	private ArrayList<PruneTuple> pts2 = new ArrayList<>();

	public static void main (String args[]) throws Exception{
		String path = "astra_driver.cpp";
		File cppfile = new File(path);
		TreeContext tc1 = new SrcmlCppTreeGenerator().generateFromFile(cppfile);
		ITree root1 = tc1.getRoot();
		String path2 = "astra_driver2.cpp";
		File cppfile2 = new File(path2);
		TreeContext tc2 = new SrcmlCppTreeGenerator().generateFromFile(cppfile2);
		ITree root2 = tc2.getRoot();
        Matcher m = Matchers.getInstance().getMatcher(root1, root2);
        m.match();
        MappingStore mappings = m.getMappings();
		Pruning pr = new Pruning(tc1, tc2, mappings);
		pr.pruneTree();
	}

	public Pruning(TreeContext tc1, TreeContext tc2, MappingStore mappings) {
		this.srcT = tc1;
		this.dstT = tc2;
		this.mappings = mappings;
	}

	public void pruneTree() throws Exception {
		System.out.println("Start Pruning");
		if(pts1.size()!=0||pts2.size()!=0) {
			throw new Exception("wrong action!");
		}
		ITree root1 = srcT.getRoot();
		ITree root2 = dstT.getRoot();
        List<ITree> before1 = root1.getDescendants();
        List<ITree> before2 = root2.getDescendants();
        ITree root_copy1 = root1.deepCopy();
		ITree root_copy2 = root2.deepCopy();
        List<ITree> before3 = root_copy1.getDescendants();
        List<ITree> before4 = root_copy2.getDescendants();
        System.out.println("Before Pruning:"+before1.size()+","+before2.size()
        +","+before3.size()+","+before4.size());
		List<ITree> results1 = new ArrayList<>();
		List<ITree> results2 = new ArrayList<>();
		results1 = traverse(root1, results1, "src");
		results2 = traverse(root2, results2, "dst");
		for(ITree node : results1) {
			int id = node.getId();
//			System.out.println("Prun1:"+id);
			ITree par = node.getParent();
			List<ITree> children = par.getChildren();
			int pos = children.indexOf(node);
			children.remove(node);//断开父亲和所有block node的连接
			PruneTuple pt = new PruneTuple(par, node, pos);
			pts1.add(pt);
			node.setParent(null);//是否需要断开block node跟父亲的连接呢?
		}
		for(ITree node : results2) {
			int id = node.getId();
//			System.out.println("Prun2:"+id);
			ITree par = node.getParent();
			List<ITree> children = par.getChildren();
			int pos = children.indexOf(node);
			children.remove(node);//断开父亲和所有block node的连接
			PruneTuple pt = new PruneTuple(par, node, pos);
			pts2.add(pt);
			node.setParent(null);//是否需要断开block node跟父亲的连接呢?
		}
		List<ITree> after1 = root1.getDescendants();
        List<ITree> after2 = root2.getDescendants();
        List<ITree> after3 = root_copy1.getDescendants();
        List<ITree> after4 = root_copy2.getDescendants();
		System.out.println("After Pruning:"+after1.size()+","+after2.size()
		+","+after3.size()+","+after4.size());
	}

	public void recoverTree() {
		for(PruneTuple pt : pts1) {
			ITree par = pt.getPar();
			ITree child = pt.getChild();
			int pos = pt.getPosition();
//			System.out.println("RecoverPrun1:"+pt.toString());
			List<ITree> children = par.getChildren();
			children.add(pos, child);
			child.setParent(par);
		}
		for(PruneTuple pt : pts2) {
			ITree par = pt.getPar();
			ITree child = pt.getChild();
			int pos = pt.getPosition();
//			System.out.println("RecoverPrun1:"+pt.toString());
			List<ITree> children = par.getChildren();
			children.add(pos, child);
			child.setParent(par);
		}
	}

	public List<ITree> traverse(ITree node, List<ITree> results, String source) throws Exception {
		if(node.isLeaf()) {
			Boolean fullMatch = false;
			if(source.equals("src")) {
				ITree target = mappings.getDst(node);
				if(target!=null) {
					fullMatch = isFullMatching(node, target);
				}else
					fullMatch = false;
			}else if(source.equals("dst")) {
				ITree target = mappings.getSrc(node);
				if(target!=null) {
					fullMatch = isFullMatching(target, node);

				}else
					fullMatch = false;
			}
			if(fullMatch) {
				ITree par = node.getParent();
				List<ITree> childs = par.getChildren();
				int height = node.getHeight();
				for(ITree tmp : childs) {
					int tmpH = tmp.getHeight();
					if(tmpH>height) {
						if(!results.contains(node)) {
							results.add(node);
						}
					}
				}
			}
		}else {
			List<ITree> childs = node.getChildren();
			if(source.equals("src")) {
				ITree target = mappings.getDst(node);
				Boolean fullMatch = false;
				if(target!=null) {
					fullMatch = isFullMatching(node, target);
				}
				if(fullMatch) {
					if(!results.contains(node)) {
						results.add(node);
					}
				}else {
					for(ITree tmp : childs) {
						traverse(tmp, results, "src");
					}
				}
			}else if(source.equals("dst")) {
				ITree target = mappings.getSrc(node);
				Boolean fullMatch = false;
				if(target!=null) {
					fullMatch = isFullMatching(target, node);
				}
				if(fullMatch) {
					if(!results.contains(node)) {
						results.add(node);
					}
				}else {
					for(ITree tmp : childs) {
						traverse(tmp, results, "dst");
					}
				}
			}else
				throw new Exception("error!");
		}
		return results;
	}//search for pruneable nodes

	public Boolean isFullMatching(ITree node1, ITree node2) {
		List<ITree> des1 = node1.getDescendants();
		List<ITree> des2 = node2.getDescendants();
		boolean result = true;
		String parsString1 = Utils.printParents(node1, srcT);
		String parsString2 = Utils.printParents(node2, dstT);
//		System.out.println(parsString1+","+parsString2);
		float sim = utils.Levenshtein.getSimilarityRatio(parsString1, parsString2);

		if(sim==1) {
			if(des1.size()==0&&des2.size()==0) {
				String type1 = srcT.getTypeLabel(node1);
				String type2 = dstT.getTypeLabel(node2);
				String value1 = node1.getLabel();
				String value2 = node2.getLabel();
				int pos1 = node1.positionInParent();
				int pos2 = node2.positionInParent();
				if(type1.equals(type2)&&value1.equals(value2)) {
					if(pos1==pos2)
						result = true;
				}else {
					result = false;
				}
			}else if(des1.size()!=0&&des2.size()!=0) {
				for(ITree tmp: des1) {
					ITree dstNode = mappings.getDst(tmp);
					if(dstNode!=null&&des2.contains(dstNode)) {
						String type1 = srcT.getTypeLabel(tmp);
						String type2 = dstT.getTypeLabel(dstNode);
						String value1 = tmp.getLabel();
						String value2 = dstNode.getLabel();
						int pos1 = tmp.positionInParent();
						int pos2 = dstNode.positionInParent();
						if(type1.equals(type2)&&value1.equals(value2)) {
							if(pos1==pos2)
								continue;
						}else {
							result = false;
							break;
						}
					}else {
						result = false;
						break;
					}
				}
				for(ITree tmp: des2) {
					ITree srcNode = mappings.getSrc(tmp);
					if(srcNode!=null&&des1.contains(srcNode)) {
						String type1 = srcT.getTypeLabel(srcNode);
						String type2 = dstT.getTypeLabel(tmp);
						String value1 = srcNode.getLabel();
						String value2 = tmp.getLabel();
						if(type1.equals(type2)&&value1.equals(value2)) {
							continue;
						}else {
							result = false;
							break;
						}
					}else {
						result = false;
						break;
					}
				}
			}else {
				result = false;
			}
		}else
			result = false;
		return result;
	}//FullMatching means all descendants and parents of the two nodes have the same type and value.
}
