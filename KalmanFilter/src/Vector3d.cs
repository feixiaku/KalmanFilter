using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Kalman;

namespace KalmanFilter.src
{
    class Vector3d
    {
        public float x;
        public float y;
        public float z;

        public Vector3d(float _x, float _y, float _z)
        {
            x = _x;
            y = _y;
            z = _z;
        }

        public Matrix toMatrix()
        {
            Matrix mat = Matrix.ZeroMatrix(3, 1);
            mat[0, 0] = x;
            mat[1, 0] = y;
            mat[2, 0] = z;

            return mat;
        }
    }
}
