using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;/// <summary>

namespace CsharpCallC
{
    class Program
    {
        [DllImport("Project.dll", EntryPoint = "freeIndex", CharSet = CharSet.Ansi)]
        //private static extern int fnTestWin32();
        public static extern void freeIndex(IntPtr p);

        [DllImport("Project.dll", EntryPoint = "tri", CharSet = CharSet.Ansi)]
        //private static extern int fnTestWin32();
        public static extern bool tri(IntPtr p, int seglen, ref IntPtr index);
        static void Main(string[] args)
        {
            List<float> data = new List<float>();
            int N = 1000;
            Random rand = new Random();
            for (int i = 0; i < N; ++i)
            {
                data.Add(i);
                data.Add(0 + i%2/10.0f);
                data.Add(i+1);
                data.Add(0 + (i+1) % 2 / 10.0f);

                data.Add(i);
                data.Add(1 + i % 2 / 10.0f);
                data.Add(i + 1);
                data.Add(1 + (i+1) % 2 / 10.0f);
                //+ (float)rand.NextDouble() / 3
            }
            float[] raw = data.ToArray();
            if (raw.Length != 8 * N)
            {
                Console.WriteLine(raw.Length);
                Console.WriteLine( "error\n");
            }
            
            IntPtr buffer = Marshal.AllocHGlobal(8 * N * sizeof(float));
            Marshal.Copy(raw, 0, buffer, 8*N);


            IntPtr p = new IntPtr(0);

            while(true) {
                tri(buffer, 2 * N, ref p);
                freeIndex(p);
            }

            System.Console.WriteLine("end");
        }
    }
}
