using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Kalman;

namespace KalmanFilter.src
{
    class StandardKalmanFilter
    {

        public Matrix State { get; set; }

        public Matrix Coveriance { get; set; }

        public Matrix F { get; set; }

        public Matrix B { get; set; } = Matrix.ZeroMatrix(1, 1);

        public Matrix u { get; set; } = Matrix.ZeroMatrix(1,1);

        public Matrix H { get; set; }

        public Matrix Q { get; set; }

        public Matrix R { get; set; }

        private Matrix Xk { get; set; }

        private Matrix Pk { get; set; }

        public StandardKalmanFilter()
        {
            defaultStdKF(this);
        }

        public void initFilter(Matrix _F, Matrix _B, Matrix _u, Matrix _H, Matrix _Q, Matrix _R)
        {
            F = _F;
            B = _B;
            u = _u;
            H = _H;
            Q = _Q;
            R = _R;
        }

        public void setSetate(Matrix _State, Matrix _Coveraince)
        {
            State = _State;
            Coveriance = _Coveraince;
        }

        public void predict()
        {
            Xk = F * State;
            Pk = F * Coveriance * (F.Transpose()) + Q;
        }

        public void update(Matrix z)
        {
            //Kalman Gain
            var tmp = (H * Pk * H.Transpose() + R).Invert();
            var K = Pk * H.Transpose() * tmp;
            var residual = z - (H * Xk);
            State = Xk + (K * residual);

            var I_KH = Matrix.IdentityMatrix(Pk.rows) - K * H;
            Coveriance = I_KH * Pk * I_KH.Transpose() + (K * R * K.Transpose());
        }

        public void defaultStdKF(StandardKalmanFilter stdKF)
        {
            stdKF.State = Matrix.ZeroMatrix(6, 1);
            stdKF.State[0, 0] = 1.0;
            stdKF.State[1, 0] = 0.05;
            stdKF.State[2, 0] = 1.0;
            stdKF.State[3, 0] = 0.05;
            stdKF.State[4, 0] = 1.0;
            stdKF.State[5, 0] = 0.05;

            stdKF.F = Matrix.ZeroMatrix(6, 6);
            stdKF.F[0, 0] = 1.0;
            stdKF.F[0, 1] = 1.0;
            stdKF.F[1, 1] = 1.0;

            stdKF.F[2, 2] = 1.0;
            stdKF.F[2, 3] = 1.0;
            stdKF.F[3, 3] = 1.0;

            stdKF.F[4, 4] = 1.0;
            stdKF.F[4, 5] = 1.0;
            stdKF.F[5, 5] = 1.0;

            stdKF.H = Matrix.ZeroMatrix(3, 6);
            stdKF.H[0, 0] = 1.0;
            stdKF.H[1, 2] = 1.0;
            stdKF.H[2, 4] = 1.0;

            stdKF.Coveriance = Matrix.IdentityMatrix(6);

            stdKF.Q = Matrix.ZeroMatrix(6, 6);

            //for blender
            stdKF.Q[0, 0] = 0.000025;
            stdKF.Q[0, 1] = 0.0005;
            stdKF.Q[1, 0] = 0.0005;
            stdKF.Q[1, 1] = 0.01;
            //block 2
            stdKF.Q[2, 2] = 0.000025;
            stdKF.Q[2, 3] = 0.0005;
            stdKF.Q[3, 2] = 0.0005;
            stdKF.Q[3, 3] = 0.01;
            //block 3
            stdKF.Q[4, 4] = 0.000025;
            stdKF.Q[4, 5] = 0.0005;
            stdKF.Q[5, 4] = 0.0005;
            stdKF.Q[5, 5] = 0.01;

            //for kinect
            ////block 1
            //stdKF.Q[0, 0] = 0.0000000005;
            //stdKF.Q[0, 1] = 0.0000001;
            //stdKF.Q[1, 0] = 0.0000001;
            //stdKF.Q[1, 1] = 0.00002;
            ////block 2
            //stdKF.Q[2, 2] = 0.0000000005;
            //stdKF.Q[2, 3] = 0.0000001;
            //stdKF.Q[3, 2] = 0.0000001;
            //stdKF.Q[3, 3] = 0.00002;
            ////block 3
            //stdKF.Q[4, 4] = 0.0000000005;
            //stdKF.Q[4, 5] = 0.0000001;
            //stdKF.Q[5, 4] = 0.0000001;
            //stdKF.Q[5, 5] = 0.00002;

            stdKF.R = Matrix.IdentityMatrix(3);
        }
    }
}
