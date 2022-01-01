package split;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;
import structure.Migration;
import utils.Utils;

public class Statistic {

	public static void main (String args[]) throws Exception{
		String path = "migrations";
//		givenStatistic(path);
		collectSpecificLeaves(path, "call");
	}

	public static void givenStatistic(String path) throws Exception {
		String inputName = "";
		File[] dirs = (new File(path)).listFiles();
		for(File dir : dirs) {
			Split sp = new Split();
			ArrayList<Migration> migrats = new ArrayList<>();
			inputName = dir.getName();
			File[] files = dir.listFiles();
			String testName = files[0].getAbsolutePath();
			String miName = testName.split("\\\\")[testName.split("\\\\").length-1];//标记文件名
			String output = miName.substring(0, miName.length()-4)+".txt";
			System.out.println(output);
			File[] fileList = (new File("D:\\workspace\\eclipse2018\\gumtree")).listFiles();
			Boolean ifExist = false;
			for(File tmp : fileList) {
				String name = tmp.getName();
				if(name.equals(output)) {
					ifExist = true;
					break;
				}
			}
			if(ifExist)
				continue;
			migrats = sp.readMigration(path, inputName);
			sp.storeTrans(migrats);
			if(files.length!=2)
				throw new Exception("error dir!!");
			sp.suggestion(testName);
		}
	}

	public static void collectSpecificLeaves(String path, String parType) throws Exception {
		ArrayList<Migration> migrats = new ArrayList<>();
		Split sp = new Split();
		migrats = sp.readMigration(path, "");
		ArrayList<String> types1 = new ArrayList<>();
		ArrayList<String> types2 = new ArrayList<>();
		for(Migration m : migrats) {
			TreeContext tc1 = m.getSrcT();
			TreeContext tc2 = m.getDstT();
			ITree root1 = tc1.getRoot();
			ITree root2 = tc2.getRoot();
			List<ITree> leaves1 = new ArrayList<>();
			List<ITree> leaves2 = new ArrayList<>();
			leaves1 = Utils.traverse2Leaf(root1, leaves1);
			leaves2 = Utils.traverse2Leaf(root2, leaves2);
			for(ITree tmp : leaves1) {
				String parType1 = tc1.getTypeLabel(tmp.getParent());
				if(!parType1.equals(parType))
					continue;
				String type = tc1.getTypeLabel(tmp);
				if(!types1.contains(type)) {
					types1.add(type);
				}
			}
			for(ITree tmp : leaves2) {
				String parType2 = tc2.getTypeLabel(tmp.getParent());
				if(!parType2.equals(parType))
					continue;
				String type = tc2.getTypeLabel(tmp);
				if(!types2.contains(type)) {
					types2.add(type);
				}
			}
		}

		for(String type : types2) {
			if(!types1.contains(type)) {
				types1.add(type);
			}
		}
		for(String type : types1) {
			System.out.println(type);
		}
	}

}
