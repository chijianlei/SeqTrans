package nodecluster;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import gumtreediff.actions.model.Action;
import gumtreediff.actions.model.Delete;
import gumtreediff.actions.model.Insert;
import gumtreediff.actions.model.Move;
import gumtreediff.actions.model.Update;
import gumtreediff.gen.srcml.SrcmlCppTreeGenerator;
import gumtreediff.matchers.Mapping;
import gumtreediff.matchers.MappingStore;
import gumtreediff.matchers.Matcher;
import gumtreediff.matchers.Matchers;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;
import structure.Migration;
import utils.Utils;

public class Cluster {
	private TreeContext tc1;
	private TreeContext tc2;
	private MappingStore mapping;
	private HashMap<ITree, ITree> node2rootMap = new HashMap<>();//insert专用
	private HashMap<ITree, ITree> parMap = new HashMap<>();//insert专用
	private Boolean hasBuildMap = false;

	public static void main (String args[]) throws Exception{
		String path = "talker.cpp";
		File cppfile = new File(path);
		TreeContext tc1 = new SrcmlCppTreeGenerator().generateFromFile(cppfile);
		String path2 = "talker2.cpp";
		File cppfile2 = new File(path2);
		TreeContext tc2 = new SrcmlCppTreeGenerator().generateFromFile(cppfile2);
		Cluster cl = new Cluster(tc1, tc2);
		cl.clusterActions(tc1, tc2);
	}

	public Cluster(TreeContext tC1, TreeContext tC2) {
		this.tc1 = tC1;
		this.tc2 = tC2;
		Matcher m = Matchers.getInstance().getMatcher(tc1.getRoot(), tc2.getRoot());
        m.match();
        this.mapping = m.getMappings();
	}

	public Cluster(TreeContext tC1, TreeContext tC2, MappingStore mappings) {
		this.tc1 = tC1;
		this.tc2 = tC2;
        this.mapping = mappings;
	}

	public ArrayList<String> extraceUPD(ArrayList<Migration> migrates) {
		ArrayList<String> commonUPD = new ArrayList<>();
		ArrayList<String> updStrings = new ArrayList<>();
		for(Migration m : migrates) {
			TreeContext tc1 = m.getSrcT();
			TreeContext tc2 = m.getDstT();
			HashMap<String, LinkedList<Action>> actions = Utils.collectAction(tc1, tc2, mapping);
			LinkedList<Action> updates = actions.get("update");
			for(Action a : updates) {
				String src = tc1.getTypeLabel(a.getNode());
				String dst = a.getName();
				String updString = src+"->"+dst;
				if(updStrings.contains(updString))
					commonUPD.add(updString);
				else
					updStrings.add(updString);
			}
		}
		return commonUPD;
	}

	public void clusterActions(TreeContext tC1, TreeContext tC2) throws Exception {
		HashMap<String, LinkedList<Action>> actions = Utils.collectAction(tc1, tc2, mapping);

        for(Mapping map : mapping) {
        	ITree src = map.getFirst();
        	ITree dst = map.getSecond();
        	System.out.println("Mapping:"+src.getId()+"->"+dst.getId());
        }

        LinkedList<Action> updates = actions.get("update");
		System.out.println("updsize:"+updates.size());
		LinkedList<Action> deletes = actions.get("delete");
		System.out.println("delsize:"+deletes.size());
		LinkedList<Action> inserts = actions.get("insert");
		System.out.println("addsize:"+inserts.size());
		LinkedList<Action> moves = actions.get("move");
		System.out.println("movsize:"+moves.size());
		HashMap<Integer, ArrayList<Action>> uptParentIds = new HashMap<>();
		HashMap<Integer, ArrayList<Action>> uptClusters = new HashMap<>();
		HashMap<Integer, ArrayList<Action>> delParentIds = new HashMap<>();
		HashMap<Integer, ArrayList<Action>> delClusters = new HashMap<>();
		HashMap<Integer, ArrayList<Action>> addParentIds = new HashMap<>();
		HashMap<Integer, ArrayList<Action>> addClusters = new HashMap<>();
		HashMap<Integer, ArrayList<Action>> movParentIds = new HashMap<>();
		HashMap<Integer, ArrayList<Action>> movClusters = new HashMap<>();

		for(Action a : updates) {
			System.out.println("-----findUptRoot-----");
			ITree sRoot = traverseSRoot(a);
			System.out.println("rootname:"+tc1.getTypeLabel(sRoot));
			int id = sRoot.getId();
			if(uptParentIds.get(id)==null) {
				ArrayList<Action> acts = new ArrayList<>();
				acts.add(a);
				uptParentIds.put(id, acts);
			}else {
				ArrayList<Action> acts = uptParentIds.get(id);
				acts.add(a);
				uptClusters.put(id, acts);
			}
		}
		for(Action a : deletes) {
			System.out.println("-----findDelRoot-----");
			ITree sRoot = traverseSRoot(a);
			System.out.println("rootname:"+tc1.getTypeLabel(sRoot));
			int id = sRoot.getId();
			if(delParentIds.get(id)==null) {
				ArrayList<Action> acts = new ArrayList<>();
				acts.add(a);
				delParentIds.put(id, acts);
			}else {
				ArrayList<Action> acts = delParentIds.get(id);
				acts.add(a);
				delClusters.put(id, acts);
			}
		}
		for(Action a : inserts) {
			System.out.println("-----findAddRoot-----");
			ITree sRoot = traverseSRoot(a);
			System.out.println("rootname:"+tc1.getTypeLabel(sRoot));
			int id = sRoot.getId();
			if(addParentIds.get(id)==null) {
				ArrayList<Action> acts = new ArrayList<>();
				acts.add(a);
				addParentIds.put(id, acts);
			}else {
				ArrayList<Action> acts = addParentIds.get(id);
				acts.add(a);
				addClusters.put(id, acts);
			}
		}
		for(Action a : moves) {
			System.out.println("-----findMovRoot-----");
			ITree sRoot = findMovRoot(a);
			System.out.println("rootname:"+tc1.getTypeLabel(sRoot));
			int id = sRoot.getId();
			if(movParentIds.get(id)==null) {
				ArrayList<Action> acts = new ArrayList<>();
				acts.add(a);
				movParentIds.put(id, acts);
			}else {
				ArrayList<Action> acts = movParentIds.get(id);
				acts.add(a);
				movClusters.put(id, acts);
			}
		}


		System.out.println("=====UPDCluster====="+uptClusters.size());
		for(Map.Entry<Integer, ArrayList<Action>> entry : uptClusters.entrySet()) {
			ArrayList<Action> acts = entry.getValue();
			ITree newRoot = downRoot(acts);
			System.out.println("downRootname:"+tc1.getTypeLabel(newRoot));
			printPath(newRoot);
		}
		System.out.println("=====DELCluster====="+delClusters.size());
		for(Map.Entry<Integer, ArrayList<Action>> entry : delClusters.entrySet()) {
			ArrayList<Action> acts = entry.getValue();
			ITree newRoot = downRoot(acts);
			System.out.println("downRootname:"+tc1.getTypeLabel(newRoot));
			printPath(newRoot);
		}
		System.out.println("=====ADDCluster====="+addClusters.size());
		for(Map.Entry<Integer, ArrayList<Action>> entry : addClusters.entrySet()) {
			ArrayList<Action> acts = entry.getValue();
			ITree newRoot = downRoot(acts);
			System.out.println("downRootname:"+tc1.getTypeLabel(newRoot));
			printPath(newRoot);
		}
		System.out.println("=====MOVCluster====="+movClusters.size());
		for(Map.Entry<Integer, ArrayList<Action>> entry : movClusters.entrySet()) {
			ArrayList<Action> acts = entry.getValue();
			ITree newRoot = downRoot(acts);
			System.out.println("downRootname:"+tc1.getTypeLabel(newRoot));
			printPath(newRoot);
		}
	}//需要debug

	public void printPath(ITree newRoot) {//打印从newRoot到SRoot
		ITree SRoot = newRoot;
		String typeLabel = tc1.getTypeLabel(newRoot);
		String allPath = typeLabel;
		while(!Utils.ifSRoot(typeLabel)) {//可能有问题，要注意循环条件
			typeLabel = tc1.getTypeLabel(SRoot.getParent());
			allPath = allPath+"<-"+typeLabel;
			SRoot = SRoot.getParent();
		}
		System.out.println(allPath);
	}

	public void buildInsertMap(LinkedList<Action> inserts) throws Exception {
		for(Action act : inserts) {//insert方法不涉及src节点，只在dst树中插入
			if(!(act instanceof Insert))
				throw new Exception("Action type error!");
			ITree dst = act.getNode();
			ITree par1 = null;
			ITree par2 = dst.getParent();
			ITree map_par2 = mapping.getSrc(par2);//the mapping node of par2 if exists
//			if(par2.getId()==3134) {
//				if(map_par2!=null)
//					System.err.println("exist!"+map_par2.getId());
//				else
//					System.err.println("not exist!");
//			}
			if(map_par2!=null) {//说明直接连接在insert_root上且有对应的src_root
				par1 = map_par2;
				parMap.put(dst, par2);
				node2rootMap.put(dst, par1);
			}else {//说明连接在insert_par上，不是insert_root
				if(parMap.get(dst)==null) {
					parMap.put(dst, par2);//映射链全部放入map，指向insert_root
				}else
					throw new Exception("error exist parMap!");
			}
		}

		for(Map.Entry<ITree, ITree> entry : parMap.entrySet()) {
			ITree dst = entry.getKey();
			ITree par = entry.getValue();
//			System.out.println("Insert:"+dst.getId()+","+par.getId());
			if(node2rootMap.get(dst)!=null) {
				continue;
			}else {//说明连接在insert_par上，不是insert_root
				ITree map_par = parMap.get(par);
				if(map_par==null)
					throw new Exception("check the null error!"+dst.getId()+","+par.getId());
				while(map_par!=null) {
					par = map_par;
					map_par = parMap.get(map_par);
				}//par is insert_root
//				System.out.println("mapped par:"+par.getId());
				ITree mapped_insert_root = mapping.getSrc(par);
				if(mapped_insert_root==null)
					throw new Exception("check the null error!");
				node2rootMap.put(dst, mapped_insert_root);
			}
		}
		hasBuildMap = true;
	}//insert方法有非常多节点连接的父亲节点属于dst树，需预先建立这些节点与src_root之间的map

	public ITree findMovRoot(Action a) throws Exception {
		if(!(a instanceof Move))
			throw new Exception("action is not move!");
		if(!hasBuildMap)
			throw new Exception("InsertMap should be built firstly!");
		ITree dst = ((Move)a).getParent();
		if(node2rootMap.get(dst)==null) {
			ITree sRoot = mapping.getDst(dst);
			if(sRoot == null)
				System.err.println("error id:"+dst.getId());
			return sRoot;
		}//发现另一种情况,move连接的节点不在insert结果中，直接从mapping中找

		ITree sRoot = node2rootMap.get(dst);
		//move连接的父亲是tc2中节点，直接从insert结果中找，必然因为insert插入到tc1中了
		if(sRoot==null)
			throw new Exception("sRoot is not exist!");
		return sRoot;
	}

	public ITree traverseRealParent(Action act) throws Exception {
		if(!hasBuildMap)
			throw new Exception("InsertMap should be built firstly!");
//		System.out.println("parMapSize:"+parMap.size());
		ITree dst = act.getNode();
		ITree mapped_insert_root = node2rootMap.get(dst);
		return mapped_insert_root;
	}//搜索该action根节点insert_root在src树上的映射, Insert专用

	public ITree traverseSRoot(Action a) throws Exception {
		ITree target = a.getNode();
		if(a instanceof Update||a instanceof Delete) {
			ITree src = a.getNode();
			String typeLabel = tc1.getTypeLabel(src);
			while(!Utils.ifSRoot(typeLabel)) {//可能有问题，要注意循环条件
				ITree par = src.getParent();
				typeLabel = tc1.getTypeLabel(par);
//				System.out.println("typeLabel:"+typeLabel);
				src = par;
			}
			target = src;
		}
		if(a instanceof Insert) {//insert 方法需要搜索tc2中的insert root节点
			ITree dst = ((Insert)a).getNode();
			String typeLabel = tc2.getTypeLabel(dst);
//			System.out.println(dst.getId()+"typeLabel:"+typeLabel);
			ITree par1 = null;
			ITree par2 = dst.getParent();
//			System.out.println(par2.getId());
//			int pos = ((Insert)a).getPosition();
	        for(Mapping map : mapping) {
	        	ITree first = map.getFirst();
	        	ITree second = map.getSecond();
	        	if(second.equals(par2)) {
	        		System.out.println("getMap:"+first.getId()+"->"+second.getId());
	        		par1 = first;
	        		typeLabel = tc1.getTypeLabel(par1);
	        		while(!Utils.ifSRoot(typeLabel)) {//可能有问题，要注意循环条件
//	        			System.out.println("LabelID:"+par1.getId());
	        			if(par1.isRoot())
	        				break;//发现有直接连接在总树根节点的情况
	        			else {
	        				ITree tmpPar = par1.getParent();
		        			typeLabel = tc1.getTypeLabel(tmpPar);
//		        			System.out.println("typeLabel:"+typeLabel);
		        			par1 = tmpPar;
	        			}
	        		}
	        		node2rootMap.put(dst, par1);//绑定子节点用
	        		break;
	        	}
	        }
	        if(par1 == null) {//仍然为-1说明不在mapping中，action为insert子树中的节点
	        	if(node2rootMap.get(par2)!=null) {
	        		par1 = node2rootMap.get(par2);
	        		node2rootMap.put(dst, par1);
	        	}else {
	        		System.err.println("error id:"+par2.getId());
//	        		throw new Exception("error childAction!");
	        	}

	        }
			target = par1;
		}
		if(a instanceof Move) {
			ITree src = ((Move) a).getNode();
			ITree dst = ((Move) a).getParent();
			String typeLabel = tc1.getTypeLabel(src);
//			System.out.println(dst.getId()+"typeLabel:"+typeLabel);
//			System.out.println("dstPar:"+dst.getParent().getId());
			while(!Utils.ifSRoot(typeLabel)) {//可能有问题，要注意循环条件
				ITree par = dst.getParent();
				typeLabel = tc1.getTypeLabel(par);
//				System.out.println(dst.getId()+"typeLabel:"+typeLabel);
				dst = par;
			}
			target = dst;
		}
		return target;
	}//搜索该action根语句root

	public ITree downRoot(ArrayList<Action> actions) throws Exception {//topdown or downtop?
		ITree sRoot = null;
		List<Integer> parents = new ArrayList<>();
		Boolean ifChild = true;
		Action exampleA = actions.get(0);
		if(exampleA instanceof Update) {
			sRoot = traverseSRoot(actions.get(0));
			for(Action a : actions) {
				parents.add(a.getNode().getParent().getId());
				Update act = (Update)a;
				System.out.println("Upt:"+act.getNode().getId()+","+act.getValue());
			}
		}else if(exampleA instanceof Delete) {
			sRoot = traverseSRoot(actions.get(0));
			for(Action a : actions) {
				parents.add(a.getNode().getParent().getId());
				Delete act = (Delete)a;
				System.out.println("Del:"+act.getNode().getId());
			}
		}else if(exampleA instanceof Insert) {
			sRoot = traverseSRoot(actions.get(0));
			for(Action a : actions) {
				ITree realPar = traverseRealParent(a);
				parents.add(realPar.getId());
				Insert act = (Insert)a;
				String out = "Add:"+act.getNode().getId()+"->"+act.getNode().getParent().getId();
				if(act.getParent().getId()!=act.getNode().getParent().getId())
					out = out+"("+act.getParent().getId()+")";
				out = out+","+act.getPosition();
				System.out.println(out);
			}
		}else if(exampleA instanceof Move) {
			ITree dst = ((Move)actions.get(0)).getParent();
			sRoot = node2rootMap.get(dst);
			String typeLabel = tc1.getTypeLabel(sRoot);
			System.out.println(sRoot.getId()+"sRootLabel:"+typeLabel);
			for(Action a : actions) {
				Move act = (Move)a;
				parents.add(act.getParent().getId());
				System.out.println("Mov:"+act.getNode().getId()+"->"+act.getParent().getId()
						+","+act.getPosition());
			}
		}

		while(ifChild) {//下降根节点必须保证覆盖所有actionNode
			System.out.println("par:"+sRoot.getId());
			if(parents.contains(sRoot.getId())) {
				break;//如果下降到只比action的父节点高一层,break
			}

			List<ITree> childs = sRoot.getChildren();
			if(childs.isEmpty())
				throw new Exception("Error!");
			else if(childs.size()==1)
				sRoot = childs.get(0);
			else {
				ITree tmpPar = null;
				for (ITree child : childs) {
					tmpPar = child;
					for(Action a : actions) {
						ITree target = a.getNode();
						ifChild = Utils.ifChild(tmpPar, target);
						System.out.println(Boolean.toString(ifChild)+" "+target.getId()+","+tmpPar.getId());
						if(!ifChild)
							break;
					}
					if(ifChild)
						break;
					else
						continue;
				}
				if(ifChild)
					sRoot = tmpPar;
			}
		}
		return sRoot;
	}//从根语句root下降rootnode,取所有action的公有最下方root

























}
