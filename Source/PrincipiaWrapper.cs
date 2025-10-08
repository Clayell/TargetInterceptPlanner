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

        // This class provides the following methods:
        // — Reflection.Call(obj, "name")(args);
        // — Reflection.GetFieldOrPropertyValue(obj, "name");
        // — Reflection.SetFieldOrPropertyValue(obj, "name", value).
        // The following generics are equivalent to casting the result of the
        // non-generic versions, with better error messages:
        // — Reflection.Call<T>(obj, "name")(args) for (T)Reflection.Call(obj, "name")(args);
        // — Reflection.GetFieldOrPropertyValue<T>(obj, "name") for
        //   (T)Reflection.GetFieldOrPropertyValue(obj, "name").
        public static class Reflection
        {
            // Returns the value of the property or field of |obj| with the given name.
            public static T GetFieldOrPropertyValue<T>(object obj, string name)
            {
                if (obj == null)
                {
                    throw new NullReferenceException($"Cannot access {typeof(T).FullName} {name} on null object");
                }
                Type type = obj.GetType();
                object result = null;
                FieldInfo field = type.GetField(name, public_instance);
                PropertyInfo property = type.GetProperty(name, public_instance);
                if (field != null)
                {
                    result = field.GetValue(obj);
                }
                else if (property != null)
                {
                    result = property.GetValue(obj, index: null);
                }
                else
                {
                    throw new MissingMemberException($"No public instance field or property {name} in {type.FullName}");
                }
                try
                {
                    return (T)result;
                }
                catch (Exception exception)
                {
                    throw new InvalidCastException(
                        $@"Could not convert the value of {(field == null ? "property" : "field")} {(field?.FieldType ?? property.PropertyType).FullName} {type.FullName}.{name}, {result}, to {typeof(T).FullName}",
                        exception);
                }
            }

            public static void SetFieldOrPropertyValue<T>(object obj, string name, T value)
            {
                if (obj == null)
                {
                    throw new NullReferenceException(
                        $"Cannot set {typeof(T).FullName} {name} on null object");
                }
                Type type = obj.GetType();
                FieldInfo field = type.GetField(name, public_instance);
                PropertyInfo property = type.GetProperty(name, public_instance);
                if (field == null && property == null)
                {
                    throw new MissingMemberException(
                        $"No public instance field or property {name} in {type.FullName}");
                }
                try
                {
                    field?.SetValue(obj, value);
                    property?.SetValue(obj, value, index: null);
                }
                catch (Exception exception)
                {
                    throw new ArgumentException(
                        $@"Could not set {(field == null ? "property" : "field")} {(field?.FieldType ?? property.PropertyType).FullName} {type.FullName}.{name} to {typeof(T).FullName} {value?.GetType().FullName ?? "null"} {value}",
                        exception);
                }
            }

            public static object GetFieldOrPropertyValue(object obj, string name)
            {
                return GetFieldOrPropertyValue<object>(obj, name);
            }

            public delegate T BoundMethod<T>(params object[] args);

            public static BoundMethod<T> Call<T>(object obj, string name)
            {
                if (obj == null)
                {
                    throw new NullReferenceException($"Cannot call {name} on null object");
                }
                Type type = obj.GetType();
                MethodInfo method = type.GetMethod(name, public_instance);
                if (method == null)
                {
                    throw new KeyNotFoundException(
                        $"No public instance method {name} in {type.FullName}");
                }
                return args =>
                {
                    object result = method.Invoke(obj, args);
                    try
                    {
                        return (T)result;
                    }
                    catch (Exception exception)
                    {
                        throw new InvalidCastException(
                            $@"Could not convert the result of {method.ReturnType.FullName} {type.FullName}.{name}(), {result}, to {typeof(T).FullName}",
                            exception);
                    }
                };
            }

            public static BoundMethod<object> Call(object obj, string name)
            {
                return Call<object>(obj, name);
            }

            private const BindingFlags public_instance =
                BindingFlags.Public | BindingFlags.Instance;
        }
    }
}
