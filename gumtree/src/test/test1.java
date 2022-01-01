package test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public class test1 {

	public static void main(String args[]) throws Exception{
		Map<String, Integer> map = new TreeMap<>();
        map.put("a", 2);
        map.put("b", 3);
        map.put("c", 1);

        // 通过ArrayList构造函数把map.entrySet()转换成list
        List<Map.Entry<String, Integer>> list = new ArrayList<>(map.entrySet());
        // 通过比较器实现比较排序
        Collections.sort(list, new Comparator<Map.Entry<String, Integer>>() {
            @Override
			public int compare(Map.Entry<String, Integer> mapping1, Map.Entry<String, Integer> mapping2) {
                return mapping2.getValue().compareTo(mapping1.getValue());
            }
        });

        for(Map.Entry<String,Integer> mapping:list){
            System.out.println(mapping.getKey()+":"+mapping.getValue());
       }
	}




}
