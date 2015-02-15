package org.usfirst.frc.team3238.robot;

import java.util.Arrays;

public class UltraFilter 
{
	double m_data[];
	int m_index;
	
	
	UltraFilter()
	{
		m_data = new double[3];
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
		int zeroCount = 0;
		
		for(int i = 0; i < sortedArray.length; i++)
		{
			sortedArray[i] = m_data[i];
		}
		
		Arrays.sort(sortedArray);
		
		for(int i = 0; i < sortedArray.length; i++)
		{
			if(sortedArray[i] == 0)
			{
				zeroCount++;
			}
		}
		
		if(zeroCount == 0)
		{
			return sortedArray[sortedArray.length/2];
		}
		else
		{
			if(zeroCount == 1)
			{
				return sortedArray[1];
			}
			else
			{
				return sortedArray[2];
			}
		}
	}
}
