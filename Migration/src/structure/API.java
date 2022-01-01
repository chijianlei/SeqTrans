package structure;

import java.util.ArrayList;

public class API {
	
	private String longName;
	private String className;
	private String methodName;
	private ArrayList<String> params;
	
	public API(String longName, String className, String methodName, ArrayList<String> params) {
		this.longName = longName;
		this.className = className;
		this.methodName = methodName;
		this.params = params;
	}
	

	public String getLongName() {
		return longName;
	}


	public String getClassName() {
		return className;
	}


	public String getMethodName() {
		return methodName;
	}


	public ArrayList<String> getParams() {
		return params;
	}


	public void setLongName(String longName) {
		this.longName = longName;
	}


	public void setClassName(String className) {
		this.className = className;
	}


	public void setMethodName(String methodName) {
		this.methodName = methodName;
	}


	public void setParams(ArrayList<String> params) {
		this.params = params;
	}

	@Override
	public int hashCode() {
		// TODO Auto-generated method stub
		return super.hashCode();
	}
	
	@Override
	public boolean equals(Object obj) {
		if(obj instanceof API) {			
			API targetAPI = (API) obj;
			String targetLongName = targetAPI.getLongName();
			ArrayList<String> params = targetAPI.getParams();
			if(targetLongName.equals(this.longName)) {
				if(params==null&&this.params==null) {
					return true;
				}else {
					if(this.params.size()!=params.size()) {
						return false;
					}else {
						for(int i=0;i<params.size();i++) {
							String param1 = params.get(i);
							String param2 = this.params.get(i);
							if(!param1.equals(param2)) {
								return false;
							}
						}
					}
				}
			}else {
				return false;
			}				
			return true;
		}
		return false;		
	}
	
	
	

}
