package structure;

public class ChangeTuple {

	private String src;
	private String dst;

	public ChangeTuple() {

	}

	public ChangeTuple(String src, String dst) {
		this.src = src;
		this.dst = dst;
	}

	public String getSrc() {
		return src;
	}

	public String getDst() {
		return dst;
	}

	public void setSrc(String src) {
		this.src = src;
	}

	public void setDst(String dst) {
		this.dst = dst;
	}

	@Override
	public String toString() {
		String change = src+"->"+dst;
		return change;
	}

}
