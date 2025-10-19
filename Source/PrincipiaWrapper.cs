using System;

namespace TargetInterceptPlanner
{
    public static class PrincipiaWrapper
    {
        public static bool Init()
        {
            Log("Attempting to find Principia....");

            foreach (AssemblyLoader.LoadedAssembly assembly in AssemblyLoader.loadedAssemblies)
            {
                try
                {
                    if (assembly.assembly.GetName().Name == "principia.ksp_plugin_adapter")
                    {
                        Log($"Principia found.");
                        return true;
                    }
                }
                catch (Exception ex)
                {
                    Log($"Error loading Principia: {ex}");
                }
            }

            Log($"Principia not found.");

            return false;
        }

        private static void Log(string message) => Util.Log(message, "[TIP-PrincipiaWrapper]");
    }
}
