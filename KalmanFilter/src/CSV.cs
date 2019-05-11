using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KalmanFilter.src
{
    class CSV
    {
        public string[][] readCSVFile(string file)
        {
            List<String[]> tmpList = new List<string[]>();

            string line;
            StreamReader reader = new StreamReader(file);

            while ((line = reader.ReadLine()) != null)
            {
                var tmpLine = line.Split(',');
                tmpList.Add(tmpLine);
            }

            reader.Close();
            return tmpList.ToArray();
        }

        public void writeCSVFile(string path, string[][] str)
        {
            StreamWriter writer = new StreamWriter(path);

            var rows = str.GetLength(0);
            for (int row = 0; row < rows; row++)
            {
                StringBuilder sb = new StringBuilder();

                var cols = str[row].GetLength(0);
                for (int col = 0; col < cols; col++)
                {
                    sb.Append(str[row][col]);
                    if (col != (cols-1))
                        sb.Append(",");

                }
                var csvLine = sb.ToString();

                writer.WriteLine(csvLine);
            }

            writer.Close();
        }
    }
}
