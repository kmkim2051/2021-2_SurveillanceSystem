﻿using System;
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

            public int TTL;
            public void define_PED(
                double Width,
                double Height,
                double DST_X,
                double DST_Y,
                double Velocity
            )
            {
                Random rand = new Random();

                this.W = Width;
                this.H = Height;
                this.D1 = 90;
                this.D2 = 180 + 90 * rand.NextDouble();
                this.W2 = this.W / 2;

                this.Pos_H1[0] =
                    Math.Round(this.W2 * Math.Cos(D1 + this.Direction) + this.X,2);
                this.Pos_H1[1] =
                    Math.Round(this.W2 * Math.Sin(D1 + this.Direction) + this.Y,2);
                this.Pos_H2[0] =
                    Math.Round(this.W2 * Math.Cos(D2 + this.Direction) + this.X,2);
                this.Pos_H2[1] =
                    Math.Round(this.W2 * Math.Sin(D2 + this.Direction) + this.Y,2);

                this.Pos_V1[0] = this.X;
                this.Pos_V1[1] = this.H;

                this.Pos_V2[0] = this.X;
                this.Pos_V2[1] = 0;

                this.DST_X = DST_X;
                this.DST_Y = DST_Y;
                this.Velocity = Velocity;

                this.Unit_Travel_Dist = Velocity * aUnitTime;

                // % for performace measure
                this.N_Surv = 0;
            }

            public void move()
            {               
                // 이동
                X += Unit_Travel_Dist * Math.Cos(Direction);
                Y += Unit_Travel_Dist * Math.Sin(Direction);

                Pos_H1[0] += Unit_Travel_Dist * Math.Cos(Direction);
                Pos_H1[1] += Unit_Travel_Dist * Math.Sin(Direction);

                Pos_H2[0] += Unit_Travel_Dist * Math.Cos(Direction);
                Pos_H2[1] += Unit_Travel_Dist * Math.Sin(Direction);

                Pos_V1[0] += Unit_Travel_Dist * Math.Cos(Direction);
                Pos_V2[0] += Unit_Travel_Dist * Math.Cos(Direction);

                // 목적지 도착 검사
                double[] dist = { X - DST_X, Y - DST_Y };
                if (Norm(dist) < 100)
                {
                    updateDestination();
                    setDirection();
                }
            }

            public void  updateDestination()
            {
                Random rand = new Random();
                DST_X = rand.Next(0, road.size);
                DST_Y = rand.Next(0, road.size);
            }

            public void setDirection()
            {
                double[] A = new double[2];
                A[0] = DST_X - X;
                A[1] = DST_Y - Y;

                double[] B = { 0.001, 0 };
                Direction = Math.Round(Math.Acos(InnerProduct(A, B) / (Norm(A) * Norm(B))), 8);
                if (Y > DST_Y)
                {
                    Direction = Math.Round(2 * Math.PI - Direction, 8);
                }
            }

            public void printPedInfo()
            {
                Console.WriteLine("======================Info======================");
                Console.WriteLine("출발지 : ({0},{1}) \n", this.X, this.Y);
                Console.WriteLine("목적지 : ({0},{1}) \n", this.DST_X, this.DST_Y);
                Console.WriteLine("방향 각도(라디안) : {0} \n", this.Direction);
                Console.WriteLine("속도 : {0} \n", this.Velocity);
                Console.WriteLine("단위이동거리 : {0} \n", this.Unit_Travel_Dist);
                Console.WriteLine("Pos_H1 : ({0},{1})   Pos_H2 : ({2},{3})  \n", 
                    this.Pos_H1[0], this.Pos_H1[1], this.Pos_H2[0], this.Pos_H2[1]);
                Console.WriteLine("Pos_V1 : ({0},{1})   Pos_V2 : ({2},{3}) \n",
                    this.Pos_V1[0], this.Pos_V1[1], this.Pos_V2[0], this.Pos_V2[1]);
                Console.WriteLine("TTL : {0} \n", this.TTL);
            }
        }
    }
}
