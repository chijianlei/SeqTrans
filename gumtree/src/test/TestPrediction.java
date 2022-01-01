package test;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;

public class TestPrediction {

	public static void main(String args[]) throws Exception{
		ArrayList<String> preds = new ArrayList<>();
		ArrayList<String> tgts = new ArrayList<>();
		BufferedWriter wr = new BufferedWriter(new FileWriter(new File("check.txt")));
		BufferedWriter wr1 = new BufferedWriter(new FileWriter(new File("validate.txt")));
		File pred = new File("pred.txt");
		BufferedReader br = new BufferedReader(new FileReader(pred));
		String tmpline = "";
		while((tmpline = br.readLine())!=null) {
			preds.add(tmpline);
		}
		File tgt = new File("tgt-train.txt");
		BufferedReader br1 = new BufferedReader(new FileReader(tgt));
		while((tmpline = br1.readLine())!=null) {
			tgts.add(tmpline);
		}
		for(int i=0;i<preds.size();i++) {
			String tmp1 = preds.get(i);
			String tmp2 = tgts.get(i);
			if(!tmp1.equals(tmp2)) {
				wr.append(tmp1);
				wr.newLine();
				wr1.append(tmp2);
				wr1.newLine();
			}
		}
		br.close();
		br1.close();
		wr.close();
		wr1.close();
	}
}
