// Inspired by the documentation in https://github.com/mockingbirdnest/Principia/wiki/Interface-for-other-KSP-mods


using System;
using System.Collections.Generic;
using System.Reflection;

namespace TargetInterceptPlanner
{
    public static class PrincipiaWrapper
    {
        private static Type _principiaType;

        public static object Principia { get; private set; }

        public static bool APIReady => Principia != null;

        public static bool Init()
        {
            _principiaType = null;
            Principia = null;

            Log("Attempting to grab Principia types...");

            // Search for Principia's type
            AssemblyLoader.loadedAssemblies.TypeOperation(type =>
            {
                if (type.FullName == "principia.ksp_plugin_adapter.ExternalInterface")
                    _principiaType = type;
            });

            if (_principiaType == null)
            {
                Log("Principia type not found.");
                return false;
            }

            Log($"Found Principia version: {_principiaType.Assembly.GetName().Version}");

            try
            {
                Principia = _principiaType.GetMethod("Get", BindingFlags.Public | BindingFlags.Static)?.Invoke(null, null);
            }
            catch (Exception ex)
            {
                Log($"Error accessing Principia API: {ex.Message}");
                return false;
            }

            if (Principia == null)
            {
                Log("Failed to retrieve Principia instance.");
                return false;
            }

            Log("Successfully initialized Principia wrapper.");
            return true;
        }

        private static void Log(string message) => Util.Log(message, "[TIP-PrincipiaWrapper]");
    }
}
