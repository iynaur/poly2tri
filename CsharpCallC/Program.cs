using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using System.Diagnostics;
/// <summary>

namespace CsharpCallC
{
    class Program
    {
        [DllImport("Project.dll", EntryPoint = "freeIndex", CharSet = CharSet.Ansi)]
        //private static extern int fnTestWin32();
        public static extern void freeIndex(IntPtr p);

        [DllImport("Project.dll", EntryPoint = "tri", CharSet = CharSet.Ansi)]
        //private static extern int fnTestWin32();
        public static extern int tri([In, MarshalAs(UnmanagedType.LPArray)] float[] seg, int seglen, ref IntPtr index);
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
            
            IntPtr p = new IntPtr(0);

            
            int len = tri(raw, 2 * N, ref p);
                

            int[] ans = new int[len];
            Marshal.Copy(p, ans, 0, len);
            int c = ans[0];
            
            int tot = 0;
            for (int i= 0; i < ans[0]; ++i)
            {
                tot += ans[i + 1];
            }
            Debug.Assert(6*tot + c + 1 == len);
            freeIndex(p);


            System.Console.WriteLine("end");
        }
    }
}
