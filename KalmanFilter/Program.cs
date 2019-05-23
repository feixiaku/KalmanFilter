using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Kalman;
using KalmanFilter.src;
using System.IO;

namespace KalmanFilter
{
    class Program
    {

        static void Main(string[] args)
        {
            string pathPrefix = "D:\\project\\kinect\\3d-pose-filtering\\data\\";
            string readFile = "csv_joined_raw.csv";
            string writeFile = "csv_joined_kf.csv";
            string consoleOutput = "Redirect.txt";

            FileStream ostream = null;
            StreamWriter oWriter = null;
            try
            {
                ostream = new FileStream(pathPrefix + consoleOutput, FileMode.OpenOrCreate, FileAccess.Write);
                oWriter = new StreamWriter(ostream);
            }
            catch (Exception e)
            {
                Console.WriteLine("Cannot open Redirect.txt for writing");
                Console.WriteLine(e.Message);
            }

            Console.SetOut(oWriter);

            CSV csv = new CSV();
            //read csv file
            string[][] content = csv.readCSVFile(pathPrefix + readFile);
            Console.WriteLine("read csv finish!");

            //kalman filter
            List<StandardKalmanFilter> filterList = new List<StandardKalmanFilter>();
            //fixed-lag smooth 
            int delayFrameNumber = 8;
            //List<FixedLagSmoother> filterList = new List<FixedLagSmoother>();
            var jointsNumber = (content[0].GetLength(0) - 1) / 3;
            for (int i = 0; i < jointsNumber; i++)
            {
                StandardKalmanFilter stdKF = new StandardKalmanFilter();
                filterList.Add(stdKF);

                //FixedLagSmoother fls = new FixedLagSmoother(8);
                //filterList.Add(fls);
            }

            var jointTableOffset = 1;
            for (var row = 0; row < content.GetLength(0); row++)
            {
                if (row == 0)
                {
                    continue;
                }

                List<Matrix> coordinates = new List<Matrix>();
                for (var col = 0; col < jointsNumber; col++)
                {
                    var x = float.Parse(content[row][jointTableOffset + col * 3]);
                    var y = float.Parse(content[row][jointTableOffset + col * 3 + 1]);
                    var z = float.Parse(content[row][jointTableOffset + col * 3 + 2]);
                    Vector3d joint = new Vector3d(x, y, z);
                    Matrix mat = joint.toMatrix();

                    coordinates.Add(mat);
                }

                //first frame position as kalman filter's initial state
                if (row == 1)
                {
                    for (var i = 0; i < jointsNumber; i++)
                    {
                        filterList[i].State[0, 0] = coordinates[i][0, 0];
                        filterList[i].State[2, 0] = coordinates[i][1, 0];
                        filterList[i].State[4, 0] = coordinates[i][2, 0];

                        //Console.WriteLine("i: {0}", i);
                        //Console.WriteLine("x: ");
                        //filterList[i].State.printMatrix();
                        //Console.WriteLine("F: ");
                        //filterList[i].F.printMatrix();
                        //Console.WriteLine("P: ");
                        //filterList[i].Coveriance.printMatrix();
                        //Console.WriteLine("Q: ");
                        //filterList[i].Q.printMatrix();
                        //Console.WriteLine("R: ");
                        //filterList[i].R.printMatrix();
                        //Console.WriteLine("H: ");
                        //filterList[i].H.printMatrix();
                    }
                }

                for (var i = 0; i < jointsNumber; i++)
                {
                    filterList[i].predict();
                    filterList[i].update(coordinates[i]);

                    content[row][jointTableOffset + i * 3] = filterList[i].State[0, 0].ToString();
                    content[row][jointTableOffset + i * 3 + 1] = filterList[i].State[2, 0].ToString();
                    content[row][jointTableOffset + i * 3 + 2] = filterList[i].State[4, 0].ToString();
                    if (row > (delayFrameNumber + jointTableOffset))
                    {
                        //content[row - delayFrameNumber - jointTableOffset][jointTableOffset + i * 3] = filterList[i].stateSmooth[row - delayFrameNumber - jointTableOffset - 1][0, 0].ToString();
                        //content[row - delayFrameNumber - jointTableOffset][jointTableOffset + i * 3 + 1] = filterList[i].stateSmooth[row - delayFrameNumber - jointTableOffset - 1][2, 0].ToString();
                        //content[row - delayFrameNumber - jointTableOffset][jointTableOffset + i * 3 + 2] = filterList[i].stateSmooth[row - delayFrameNumber - jointTableOffset - 1][4, 0].ToString();
                    }
                }
            }//end of row

            //for (var row = 0; row < content.GetLength(0); row++)
            //{
            //    if (row == 0)
            //        continue;

            //    for (var i = 0; i < jointsNumber; i++)
            //    {
            //        content[row][jointTableOffset + i * 3] = filterList[i].stateSmooth[row - 1][0, 0].ToString();
            //        content[row][jointTableOffset + i * 3 + 1] = filterList[i].stateSmooth[row - 1][2, 0].ToString();
            //        content[row][jointTableOffset + i * 3 + 2] = filterList[i].stateSmooth[row - 1][4, 0].ToString();
            //    }
            //}

            //write csv file
            csv.writeCSVFile(pathPrefix + writeFile, content);
            Console.WriteLine("write csv finish!");

            Console.SetOut(Console.Out);
            oWriter.Close();
            ostream.Close();
        }
    }
}
