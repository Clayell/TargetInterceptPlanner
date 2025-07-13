using System;
using System.Reflection;

namespace LunarTransferPlanner
{
    public static class PrincipiaWrapper
    {
        private static Type _principiaType;
        public static object Principia { get; private set; }

        public static bool AssemblyExists => _principiaType != null;
        public static bool APIReady => Principia != null;

        public static bool InitPrincipiaWrapper()
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

        private static void Log(string message) => Util.Log(message, "[LTP-PrincipiaWrapper]");

        private static void LogWarning(string message) => Util.LogWarning(message, "[LTP-PrincipiaWrapper]");

        public static class Reflection
        {
            private const BindingFlags Flags = BindingFlags.Public | BindingFlags.Instance;

            public static T GetMemberValue<T>(object obj, string name)
            {
                if (obj == null) throw new ArgumentNullException(nameof(obj));

                try
                {
                    var type = obj.GetType();
                    var value = type.GetField(name, Flags)?.GetValue(obj) ?? type.GetProperty(name, Flags)?.GetValue(obj);

                    if (value is T typedValue)
                        return typedValue;

                    throw new InvalidCastException($"Cannot cast '{name}' to {typeof(T)}.");
                }
                catch (Exception ex)
                {
                    LogWarning($"[PrincipiaWrapper] Error getting member '{name}' from type '{obj.GetType()}': {ex.Message}");
                    throw new InvalidOperationException($"Failed to get member '{name}' from Principia object.", ex);
                }
            }

            public delegate T BoundMethod<T>(params object[] args);

            public static BoundMethod<T> BindMethod<T>(object obj, string methodName)
            {
                if (obj == null) throw new ArgumentNullException(nameof(obj));

                var method = obj.GetType().GetMethod(methodName, Flags)
                          ?? throw new MissingMethodException($"Method '{methodName}' not found on type {obj.GetType()}.");

                return args =>
                {
                    var result = method.Invoke(obj, args);
                    return result is T typedResult ? typedResult :
                        throw new InvalidCastException($"Cannot cast result of '{methodName}' to {typeof(T)}.");
                };
            }
        }
    }
}