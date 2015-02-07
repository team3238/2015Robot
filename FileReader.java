package org.usfirst.frc.team3238.robot;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;

public class FileReader
{
    public static ArrayList<String> readFile(String fileName)
    {
        Scanner fileInput = null;
        File file = null;
        ArrayList<String> contents = null;
        try
        {
            file = new File("/" + fileName);
            fileInput = new Scanner(file);
            contents = new ArrayList<String>();
            String line = "";
            while(fileInput.hasNextLine())
            {
                line = fileInput.nextLine();
                contents.add(line);
            }
        } catch(IOException e)
        {
            System.out.println("Could not open " + fileName + "!");
        } finally
        {
            fileInput.close();
        }
        return contents;
    }
}