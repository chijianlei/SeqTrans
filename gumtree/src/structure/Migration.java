package structure;

import gumtreediff.matchers.MappingStore;
import gumtreediff.tree.TreeContext;

public class Migration {

	private TreeContext srcT;
	private TreeContext dstT;
	private MappingStore mappings;
	private String repoName;
	private String miName_src;
	private String miName_dst;
	private String srcHash;
	private String dstHash;

	public Migration(TreeContext tc1, TreeContext tc2, MappingStore mappings, String src, String dst) {
		this.srcT = tc1;
		this.dstT = tc2;
		this.mappings = mappings;
		this.miName_src = src;
		this.miName_dst = dst;
	}

	public TreeContext getSrcT() {
		return srcT;
	}

	public TreeContext getDstT() {
		return dstT;
	}

	public MappingStore getMappings() {
		return mappings;
	}

	public String getRepoName() {
		return repoName;
	}

	public void setRepoName(String repoName) {
		this.repoName = repoName;
	}

	public String getMiName() {
		String[] tmps = miName_src.split("\\");
		return tmps[tmps.length-1];
	}

	public String getMiName_src() {
		return miName_src;
	}

	public String getMiName_dst() {
		return miName_dst;
	}

	public String getSrcHash() {
		return srcHash;
	}

	public String getDstHash() {
		return dstHash;
	}

	public void setSrcHash(String srcHash) {
		this.srcHash = srcHash;
	}

	public void setDstHash(String dstHash) {
		this.dstHash = dstHash;
	}

}
