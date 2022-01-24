using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;

namespace surveillance_system
{
    public partial class Program
    {
        public static double InnerProduct(double[] a, double[] b)
        {
            if (a.Length != b.Length) return 0;

            double acc = 0;
            for (int i = 0; i < a.Length; i++)
            {
                acc += a[i] * b[i];
            }
            return acc;
        }

        public static double Norm(double[] a)
        {
            return Math.Sqrt(a[0] * a[0] + a[1] * a[1]);
        }

        public static double RadToDeg(double angle)
        {
            return (180 / Math.PI) * angle;
        }

        public static double DegToRad(double angle)
        {
            return (Math.PI / 180) * angle;
        }
    }

}
