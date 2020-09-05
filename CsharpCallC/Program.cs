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
            int N = 100;
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

            
                tri(buffer, 2 * N, ref p);
                

            int[] ans = new int[1];
            Marshal.Copy(p, ans, 0, 1);
            int c = ans[0];
            ans = new int[1 + c];
            Marshal.Copy(p, ans, 0, 1 + c);
            int tot = 0;
            for (int i= 0; i < ans[0]; ++i)
            {
                tot += ans[i + 1];
            }
            ans = new int[1 + c + 6 * tot];
            Marshal.Copy(p, ans, 0, 1 + c + 6*tot);
            freeIndex(p);


            System.Console.WriteLine("end");
        }
    }
}
