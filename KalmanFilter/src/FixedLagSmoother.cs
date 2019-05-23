using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Kalman;

namespace KalmanFilter.src
{
    class FixedLagSmoother : StandardKalmanFilter
    {
        private int N;
        private int count;
        private Matrix Xk { get; set; }

        private Matrix Pk { get; set; }

        public List<Matrix> stateSmooth = new List<Matrix>();

        public FixedLagSmoother(int _N)
        {
            N = _N;
            this.defaultStdKF(this);
        }

        new public void update(Matrix z)
        {
            int k = count;

            Xk = F * State;
            Pk = F * Coveriance * F.Transpose() + Q;

            //Kalman Gain
            var SI = (H * Pk * H.Transpose() + R).Invert();
            var K = Pk * H.Transpose() * SI;

            var residual = z - (H * Xk);
            State = Xk + (K * residual);

            var I_KH = Matrix.IdentityMatrix(Pk.rows) - K * H;
            Coveriance = I_KH * Pk * I_KH.Transpose() + (K * R * K.Transpose());

            stateSmooth.Add(Xk);
            Console.WriteLine(count);
            //Xk.printMatrix();

            var HTSI = H.Transpose() * SI;
            var F_LH = (F - (K * H).Transpose());

            if (k >= N)
            {
                var PS = Coveriance;
                for (var i = 0; i < N; i++)
                {
                    K = PS * HTSI;
                    PS = PS * F_LH;

                    var si = k - i;
                    var tmp = K * residual;
                    stateSmooth[si] = stateSmooth[si] + tmp;
                    //Console.WriteLine("si: {0}", si);
                    stateSmooth[si].printMatrix();
                }
            }
            else {
                stateSmooth[k] = State;
                //Console.WriteLine("k: {0}", k);
                stateSmooth[k].printMatrix();
            }

            count++;

            //Console.WriteLine("State: ");
            State.printMatrix();
        }
    }
}
