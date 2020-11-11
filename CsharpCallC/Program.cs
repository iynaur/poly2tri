using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using System.Diagnostics;
using System.Web.Script.Serialization;
/// <summary>

namespace CsharpCallC
{
    class Program
    {
        [DllImport("libProject.so", EntryPoint = "freeIndex", CharSet = CharSet.Ansi)]
        public static extern void freeIndex(IntPtr p);

        [DllImport("libProject.so")]
        public static extern int tri([In, MarshalAs(UnmanagedType.LPArray)] float[] seg, int seglen, ref IntPtr index);

        [DllImport("libProject.so")]
        public static extern int qtGui();

        [DllImport ("/lib/x86_64-linux-gnu/libc.so.6")]
        private static extern int getpid ();

        static void Main(string[] args)
        {
            string input = "[{error: \"Account with that email exists\"}]";
            var jss = new JavaScriptSerializer();

            var array = jss.Deserialize<object[]>(input);
            var dict = array[0] as Dictionary<string, object>;
            Console.WriteLine(dict["error"]);

            // More short with dynamic
            dynamic d = jss.DeserializeObject(input);
            Console.WriteLine(d[0]["error"]);
            qtGui();
            Console.WriteLine(getpid ());
            List<float> data = new List<float>();
            int N = 100;
            Random rand = new Random();
            for (int i = 0; i < N; ++i)
            {
                data.Add(i);
                data.Add(0 );
                data.Add(i+1);
                data.Add(0 );

                data.Add(i);
                data.Add(1 );
                data.Add(i + 1);
                data.Add(1 );
                //+ (float)rand.NextDouble() / 3
            }
            float[] raw = data.ToArray();
            if (raw.Length != 8 * N)
            {
                Console.WriteLine(raw.Length);
                Console.WriteLine( "error\n");
            }
            
            IntPtr p = new IntPtr(0);

            int len = 0;
            while (true)
            {
              len = tri(raw, 2 * N, ref p);


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
            }


            System.Console.WriteLine("end");
        }
    }
}
