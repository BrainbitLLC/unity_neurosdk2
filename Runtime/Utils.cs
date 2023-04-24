#if UNITY_IOS
#define __IOS__
#endif

using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;

namespace NeuroSDK
{
    #region Marshaller
    internal class NativeArrayMarshaler<T>
    {
        private readonly IMarshaler _marshaler;

        public NativeArrayMarshaler()
        {
            _marshaler = typeof(T).IsPrimitive ? (IMarshaler)new PrimitiveMarshaller() : new StructEnumMarshaller();
        }

        public T[] MarshalArray(IntPtr firstElementPtr, IntPtr elementCount)
        {
            return _marshaler.MarshalArray(firstElementPtr, elementCount);
        }

        private interface IMarshaler
        {
            T[] MarshalArray(IntPtr firstElementPtr, IntPtr elementCount);
        }

        private class PrimitiveMarshaller : IMarshaler
        {
            private delegate T[] PrimitiveArrayCopyDelegate(IntPtr firstElementPtr, IntPtr elementCount);

            private readonly PrimitiveArrayCopyDelegate _copyFunction;

            public PrimitiveMarshaller()
            {
                _copyFunction = GetCopyFunctionForType(typeof(T));
            }

            public T[] MarshalArray(IntPtr firstElementPtr, IntPtr elementCount)
            {
                var resultArray = _copyFunction(firstElementPtr, elementCount);
                return resultArray ??
                       throw new ArrayTypeMismatchException("Wrong result buffer type after native buffer marshaling");
            }

            private static PrimitiveArrayCopyDelegate GetCopyFunctionForType(Type type)
            {
                if (type == typeof(int))
                {
                    return CopyInt;
                }
                else if (type == typeof(double))
                {
                    return CopyDouble;
                }

                throw new ArgumentException($"There is no reader function for type {type.Name}");
            }

            private static T[] CopyDouble(IntPtr firstElementPtr, IntPtr elementCount)
            {
                double[] buffer = new double[elementCount.ToInt32()];
                Marshal.Copy(firstElementPtr, buffer, 0, (int)elementCount);
                return (buffer as T[])!;
            }

            private static T[] CopyInt(IntPtr firstElementPtr, IntPtr elementCount)
            {
                int[] buffer = new int[elementCount.ToInt32()];
                Marshal.Copy(firstElementPtr, buffer, 0, (int)elementCount);
                return (buffer as T[])!;
            }

        }

        private class StructEnumMarshaller : IMarshaler
        {
            private delegate T NativePtrReader(IntPtr nativePtr);
            private readonly NativePtrReader _ptrReaderFunc;

            public StructEnumMarshaller()
            {
                _ptrReaderFunc = typeof(T).IsEnum ? NativeEnumPtrReader : NativeStructPtrReader;
            }

            public T[] MarshalArray(IntPtr firstElementPtr, IntPtr elementCount)
            {
                int cnt = elementCount.ToInt32();
                var result = new T[cnt];
                var type = typeof(T).IsEnum ? typeof(T).GetEnumUnderlyingType() : typeof(T);
                int dataSz = Marshal.SizeOf(type);
                for (uint i = 0; i < cnt; ++i)
                {
                    result[i] = _ptrReaderFunc(firstElementPtr);
                    firstElementPtr = IntPtr.Add(firstElementPtr, dataSz);
                }

                return result;
            }

            private static T NativeStructPtrReader(IntPtr nativePtr)
            {
                return Marshal.PtrToStructure<T>(nativePtr)!;
            }

            private static T NativeEnumPtrReader(IntPtr nativePtr)
            {
                return (T)(object)Marshal.ReadInt32(nativePtr);
            }
        }
    }
    #endregion

    #region Platform
    internal enum PlatformType
    {
        WinX86,
        WinX64,
        WinArm,
        WinArm64,
        OSX,
        iOS,
        AndroidARMv7,
        AndroidARMv8,
        AndroidX86,
        AndroidX64,
        LinuxX86,
        LinuxX64,
        LinuxArm,
        LinuxArm64,
    }

    internal static class Platform
    {
        private static PlatformType? _platformType;

        public static PlatformType Type
        {
            get
            {
                if (_platformType == null)
                {
                    _platformType = DetectPlatform();
                }

                return _platformType.Value;
            }
        }

        private static PlatformType DetectPlatform()
        {
#if __IOS__
            return PlatformType.iOS;
#elif UNITY_5_3_OR_NEWER
            switch (UnityEngine.Application.platform)
            {
                case UnityEngine.RuntimePlatform.WindowsEditor:
                case UnityEngine.RuntimePlatform.WindowsPlayer:
                    {
                        return DetectWindowsArch();
                    }
                case UnityEngine.RuntimePlatform.LinuxEditor:
                case UnityEngine.RuntimePlatform.LinuxPlayer:
                    {
                        return DetectLinuxArch();
                    }
                case UnityEngine.RuntimePlatform.Android:
                    {
                        return DetectAndroidArch();
                    }
                case UnityEngine.RuntimePlatform.OSXEditor:
                case UnityEngine.RuntimePlatform.OSXPlayer:
                    {
                        return PlatformType.OSX;
                    }
                case UnityEngine.RuntimePlatform.IPhonePlayer:
                    {
                        return PlatformType.iOS;
                    }
            }
#elif NET5_0_OR_GREATER
            if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
            {
                return DetectWindowsArch();
            }
            else if (OperatingSystem.IsAndroid())
            {
                return DetectAndroidArch();
            }
            else if (OperatingSystem.IsIOS())
            {
                return PlatformType.iOS;
            }
            else if (OperatingSystem.IsMacOS())
            {
                return PlatformType.OSX;
            }
            else if (OperatingSystem.IsLinux())
            {
                return DetectLinuxArch();
            }
#elif WINDOWS_UWP
            return DetectWindowsArch();
#elif __MACOS__
            return PlatformType.OSX;
#elif __ANDROID__
            return DetectAndroidArch();
#else
            if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
            {
                return DetectWindowsArch();
            }
            else if (RuntimeInformation.IsOSPlatform(OSPlatform.OSX))
            {
                return CheckiOS();
            }
            else if (RuntimeInformation.IsOSPlatform(OSPlatform.Linux))
            {
                if (CheckIsAndroid())
                {
                    return GetAndroidABI();
                }
                else
                {
                    // TODO: Пока нет возможности для IL2CPP определить Android или нет, но при Mono сборке код выше корректен
                    return DetectAndroidArch();
                }
            }
#endif
            throw new PlatformNotSupportedException("Current platform is not supported by the SDK");
        }
#if !__IOS__
        private static PlatformType DetectWindowsArch()
        {
            switch (RuntimeInformation.ProcessArchitecture)
            {
                case Architecture.X86:
                    return PlatformType.WinX86;
                case Architecture.X64:
                    return PlatformType.WinX64;
                case Architecture.Arm:
                    return PlatformType.WinArm;
                case Architecture.Arm64:
                    return PlatformType.WinArm64;
                default:
                    throw new PlatformNotSupportedException("Current architecture is not supported by the SDK for Windows.");
            }
        }

        private static PlatformType DetectLinuxArch()
        {
            switch (RuntimeInformation.ProcessArchitecture)
            {
                case Architecture.X86:
                    return PlatformType.LinuxX86;
                case Architecture.X64:
                    return PlatformType.LinuxX64;
                case Architecture.Arm:
                    return PlatformType.LinuxArm;
                case Architecture.Arm64:
                    return PlatformType.LinuxArm64;
                default:
                    throw new PlatformNotSupportedException("Current architecture is not supported by the SDK for Linux.");
            }
        }
        private static PlatformType DetectAndroidArch()
        {
            switch (RuntimeInformation.ProcessArchitecture)
            {
                case Architecture.X86:
                    return PlatformType.AndroidX86;
                case Architecture.X64:
                    return PlatformType.AndroidX64;
                case Architecture.Arm:
                    return PlatformType.AndroidARMv7;
                case Architecture.Arm64:
                    return PlatformType.AndroidARMv8;
                default:
                    throw new PlatformNotSupportedException("Current architecture is not supported by the SDK for Android.");
            }
        }
        private static bool CheckIsAndroid()
        {
            using (var process = new System.Diagnostics.Process())
            {
                process.StartInfo.FileName = "getprop";
                process.StartInfo.Arguments = "ro.build.user";
                process.StartInfo.RedirectStandardOutput = true;
                process.StartInfo.UseShellExecute = false;
                process.StartInfo.CreateNoWindow = true;
                try
                {
                    process.Start();
                    var output = process.StandardOutput.ReadToEnd();
                    return !string.IsNullOrEmpty(output);
                }
                catch
                {
                    return false;
                }
            }
        }

        private static PlatformType GetAndroidABI()
        {
            using (var process = new System.Diagnostics.Process())
            {
                process.StartInfo.FileName = "getprop";
                process.StartInfo.Arguments = "ro.product.cpu.abi";
                process.StartInfo.RedirectStandardOutput = true;
                process.StartInfo.UseShellExecute = false;
                process.StartInfo.CreateNoWindow = true;
                try
                {
                    process.Start();
                    var output = process.StandardOutput.ReadToEnd();
                    if (output.Trim().Equals("arm64-v8a")) return PlatformType.AndroidARMv8;
                    if (output.Trim().Equals("armeabi-v7a")) return PlatformType.AndroidARMv7;
                    if (output.Trim().Equals("x86")) return PlatformType.AndroidX86;
                    if (output.Trim().Equals("x86_64")) return PlatformType.AndroidX64;
                    throw new InvalidOperationException($"Unrecognized platform string: {output}");
                }
                catch (Exception e)
                {
                    throw new InvalidOperationException($"Can't determine Android ABI: {e.Message}");
                }
            }
        }
#endif
        private static PlatformType CheckiOS()
        {
            var pLen = Marshal.AllocHGlobal(sizeof(int));
            sysctlbyname("hw.machine", IntPtr.Zero, pLen, IntPtr.Zero, 0);

            var length = Marshal.ReadInt32(pLen);

            var pStr = Marshal.AllocHGlobal(length);
            sysctlbyname("hw.machine", pStr, pLen, IntPtr.Zero, 0);

            var hardwareStr = Marshal.PtrToStringAnsi(pStr);

            Marshal.FreeHGlobal(pLen);
            Marshal.FreeHGlobal(pStr);

            if (hardwareStr.Contains("iPhone") || hardwareStr.Contains("iPad"))
                return PlatformType.iOS;
            else
                return PlatformType.OSX;
        }
        [DllImport("libc")]
        static internal extern int sysctlbyname([MarshalAs(UnmanagedType.LPStr)] string property, IntPtr output, IntPtr oldLen, IntPtr newp, uint newlen);
    }
    #endregion

    #region ApiFactory
    internal sealed class SDKApiFactory
    {
        private static string GetApplicationRoot()
        {
            return Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location);
        }

        private static readonly Lazy<ISDKApi> _api = new Lazy<ISDKApi>(() =>
        {
            var platform = Platform.Type;
            try
            {
                switch (platform)
                {
                    case PlatformType.WinArm:
                    case PlatformType.WinArm64:
                    case PlatformType.WinX64:
                    case PlatformType.WinX86:
                        Environment.SetEnvironmentVariable("PATH", $"{Path.Combine(GetApplicationRoot(), @"libs\windows")};{Environment.GetEnvironmentVariable("PATH")}"); // Not embed
                        break;
                    case PlatformType.LinuxArm:
                    case PlatformType.LinuxArm64:
                    case PlatformType.LinuxX64:
                    case PlatformType.LinuxX86:
                        Environment.SetEnvironmentVariable("PATH", $"{Path.Combine(GetApplicationRoot(), @"libs\linux")};{Environment.GetEnvironmentVariable("PATH")}"); // Not embed
                        break;
                }
            }
            catch (Exception)
            {
                // ignore for IL2CPP
            }
            switch (platform)
            {
#if !__IOS__
                case PlatformType.WinArm:
                    return new SDKApiArm();
                case PlatformType.WinArm64:
                    return new SDKApiArm64();
                case PlatformType.WinX64:
                    return new SDKApiX64();
                case PlatformType.WinX86:
                    return new SDKApiX32();
                case PlatformType.AndroidARMv7:
                case PlatformType.AndroidARMv8:
                case PlatformType.AndroidX86:
                case PlatformType.AndroidX64:
                case PlatformType.OSX:
                case PlatformType.LinuxX64:
                case PlatformType.LinuxX86:
                    return new SDKApiAllArch();
#endif
                case PlatformType.iOS:
#if __IOS__
                    return new SDKApiIOS();
#else
                    return new SDKApiAllArch();
#endif
            }
            throw new NotSupportedException($"[OSDescription]:[{RuntimeInformation.OSDescription}]");
        });
        private SDKApiFactory() { }
        public static ISDKApi Inst
        {
            get => _api.Value;
        }

        public static void ThrowIfError(OpStatus status, byte error)
        {
            if (!status.Success || error == 0)
                throw new SDKException($"[Error]:[{status.Error}] [{status.ErrorMsg}]", status.Error);
        }
    }
    #endregion
}
