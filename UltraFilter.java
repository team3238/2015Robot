package org.usfirst.frc.team3238.robot;

import java.util.Arrays;

public class UltraFilter 
{
	double m_data[];
	int m_index;
	
	
	UltraFilter(int length)
	{
		m_data = new double[length];
		m_index = 0;
	}
	
	public void addData(double data)
	{
		m_data[m_index] = data;
		m_index+=1;
		m_index = m_index % m_data.length;
	}
	
	public double getMedian()
	{
		double sortedArray[] = new double[m_data.length];
		
		for(int i = 0; i < sortedArray.length; i++)
		{
			sortedArray[i] = m_data[i];
		}
		
		Arrays.sort(sortedArray);
		
		return sortedArray[sortedArray.length/2];
	}
}
