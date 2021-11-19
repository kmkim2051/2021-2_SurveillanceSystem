using System;
using System.Collections.Generic;
using System.Text;

namespace surveillance_system
{
    public partial class Program
    {
        public class Pedestrian
        {
            public double X;

            public double Y;

            public double DST_X;

            public double DST_Y;

            public double Direction;

            public double Velocity;

            public double Unit_Travel_Dist;

            public double MAX_Dist_X;

            public double MAX_Dist_Y;

            /* ==================================
            /   추가
            /   line 67~ 87 변수 
            /   line 91~127 define_PED 함수 (깃허브 define_PED.m)
            /           라인 500~507에서 ped 위치 변수 사용하는데 이걸  define_PED에서 처리하는거 같아서 구현해놨습니다
            / ===================================*/
            public double W;

            public double H;

            public double D1;

            public double D2;

            public double W2;

            public double[] Pos_H1 = new double[2];

            public double[] Pos_H2 = new double[2];

            public double[] Pos_V1 = new double[2];

            public double[] Pos_V2 = new double[2];

            public int N_Surv; //number of surveillance camera viewing this target.

            //public int TTL;
            public void define_PED(
                double Width,
                double Height,
                double Direction,
                double X,
                double Y,
                double DST_X,
                double DST_Y,
                double Velocity
            )
            {
                Random rand = new Random();

                this.W = Width;
                this.H = Height;
                this.Direction = Direction;
                this.D1 = 90;
                this.D2 = 180 + 90 * rand.NextDouble();
                this.W2 = this.W / 2;

                //  PED.Pos_H1 = [PED.W2*cosd(PED.D1+PED.Dir) PED.W2*sind(PED.D1+PED.Dir)]+[X Y];
                this.Pos_H1[0] =
                    this.W2 * Math.Cos(D1 + this.Direction) + this.X;
                this.Pos_H1[1] =
                    this.W2 * Math.Sin(D1 + this.Direction) + this.Y;

                //  PED.Pos_H2 = [PED.W2*cosd(PED.D2+PED.Dir) PED.W2*sind(PED.D2+PED.Dir)]+[X Y];
                this.Pos_H2[0] =
                    this.W2 * Math.Cos(D2 + this.Direction) + this.X;
                this.Pos_H2[1] =
                    this.W2 * Math.Sin(D2 + this.Direction) + this.Y;

                // PED.Pos_V1 = [X Height];
                this.Pos_V1[0] = this.X;
                this.Pos_V1[1] = this.H;

                //  PED.Pos_V2 = [X 0];
                this.Pos_V2[0] = this.X;
                this.Pos_V2[1] = 0;

                this.X = X;
                this.Y = Y;
                this.DST_X = DST_X;
                this.DST_Y = DST_Y;
                this.Velocity = Velocity;

                // % for performace measure
                this.N_Surv = 0;
                //this.TTL = 0;
            }
        }
    }
}
