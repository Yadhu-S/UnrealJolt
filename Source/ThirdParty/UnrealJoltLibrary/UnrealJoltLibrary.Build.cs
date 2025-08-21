// Fill out your copyright notice in the Description page of Project Settings.

using System;
using System.Diagnostics;
using System.IO;
using System.Text;
using UnrealBuildTool;
using System.Linq;
using System.Text.RegularExpressions;


public class UnrealJoltLibrary : ModuleRules
{

	private void BuildJolt(string BuildType, ReadOnlyTargetRules Target)
	{

		var ThirdPartyJoltPath = Path.Combine(ModuleDirectory, "JoltPhysics");
		var ModulePath = Path.Combine( ModuleDirectory, "JoltPhysics", "Build");
		var JoltBuildDir = MBuildUtils.GetJoltBuildDir(ModuleDirectory, Target);
		var EngineDir = Path.GetFullPath(Target.RelativeEnginePath);;

		Console.WriteLine("Jolt third party directory: " + ThirdPartyJoltPath);
		//-- Generation step
		string cmakeGeneratorType;
		var buildTypeGenerator = "";
		var buildTypeCompilator = "";
		var cmakeOptions = "";

		// NOTE: for big worlds, larger than 5KM. https://jrouwe.github.io/JoltPhysics/index.html#big-worlds 
		cmakeOptions += " -DDOUBLE_PRECISION=ON"; 
		cmakeOptions += " -DCROSS_PLATFORM_DETERMINISTIC=ON ";
		cmakeOptions += " -DOBJECT_LAYER_BITS=32 ";
		cmakeOptions += " -DINTERPROCEDURAL_OPTIMIZATION=ON ";
		cmakeOptions += " -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ";
		cmakeOptions += " -DTARGET_SAMPLES=OFF ";
		cmakeOptions += " -DTARGET_HELLO_WORLD=OFF";
		cmakeOptions += " -DTARGET_VIEWER=OFF ";
		cmakeOptions += " -DTARGET_UNIT_TESTS=OFF ";
		cmakeOptions += " -DTARGET_PERFORMANCE_TEST=OFF ";

		// NOTE: AVX is turned OFF in unreal engine, so this need to be OFF in Jolt to properly load the lib.
		cmakeOptions += " -DUSE_AVX=OFF ";
		cmakeOptions += " -DUSE_AVX2=OFF ";
		cmakeOptions += " -DUSE_F16C=OFF ";


		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			cmakeGeneratorType = "\"Visual Studio 17 2022\"";
			switch (BuildType)
			{
				case "Debug":
					cmakeOptions += " -DDEBUG_RENDERER_IN_DISTRIBUTION=ON ";
					cmakeOptions += " -DPROFILER_IN_DISTRIBUTION=OFF ";
					cmakeOptions += " -DPROFILER_IN_DEBUG_AND_RELEASE =OFF ";
					buildTypeCompilator = "--config Debug";
					break;
				case "Release":
					cmakeOptions += " -DDEBUG_RENDERER_IN_DISTRIBUTION=ON ";
					cmakeOptions += " -DPROFILER_IN_DISTRIBUTION=OFF ";
					cmakeOptions += " -DPROFILER_IN_DEBUG_AND_RELEASE=OFF ";
					buildTypeCompilator = "--config Release";
					break;
				default:
					buildTypeCompilator = "--config Distribution";
					break;
			}

			// This is needed to make sure the library generated is not a Static Library MT (but it's 'MD')
			// making it compatible with Unreal.
			cmakeOptions += " -DUSE_STATIC_MSVC_RUNTIME_LIBRARY=0";
		}
		else if (Target.Platform == UnrealTargetPlatform.Mac)
		{
			cmakeGeneratorType = "\"Unix Makefiles\"";
			// Build for M1/M2/M3 processors.
			cmakeOptions += " -DCMAKE_OSX_ARCHITECTURES=\"arm64\" ";
			cmakeOptions += " -DCMAKE_OSX_DEPLOYMENT_TARGET=\"11.0\" ";
			cmakeOptions += " -DBUILD_SHARED_LIBS=ON ";
			switch (BuildType)
			{
				case "Debug":
					cmakeOptions += " -DDEBUG_RENDERER_IN_DISTRIBUTION=ON ";
					buildTypeGenerator = " -DCMAKE_CONFIGURATION_TYPES=Debug";
					break;
				case "Release":
					cmakeOptions += " -DDEBUG_RENDERER_IN_DISTRIBUTION=ON ";
					buildTypeGenerator = " -DCMAKE_CONFIGURATION_TYPES=Release";
					break;
				default:
					buildTypeGenerator = "-DCMAKE_CONFIGURATION_TYPES=Distribution ";
					break;
			}
		}
		else if (Target.Platform == UnrealTargetPlatform.Linux)
		{

			cmakeGeneratorType = "\"Unix Makefiles\"";
			cmakeOptions += " -DBUILD_SHARED_LIBS=OFF ";
			cmakeOptions += " -DCMAKE_POSITION_INDEPENDENT_CODE=ON ";
			cmakeOptions += " -DGENERATE_DEBUG_SYMBOLS=ON";
			cmakeOptions += " -DCMAKE_CXX_FLAGS=\"-ffp-model=precise -ffp-contract=off\" ";


			var toolChain = MBuildUtils.GetLatestBundledClangToolchain(EngineDir);
			cmakeOptions += " -DCMAKE_SYSROOT=\"" + toolChain + "/x86_64-unknown-linux-gnu\" ";
			cmakeOptions += " -DCMAKE_CXX_COMPILER=\"" + toolChain + "/x86_64-unknown-linux-gnu/bin/clang++\" ";
			cmakeOptions += " -DCMAKE_CXX_FLAGS=\"-nostdinc++ -I " + EngineDir + "Source/ThirdParty/Unix/LibCxx/include/ -I " + EngineDir + "Source/ThirdParty/Unix/LibCxx/include/c++/v1/\" ";
			switch (BuildType)
			{
				case "Debug":
					cmakeOptions += " -DUSE_ASSERTS=ON " ;
					cmakeOptions += " -DDEBUG_RENDERER_IN_DISTRIBUTION=ON ";
					cmakeOptions += " -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ";
					buildTypeGenerator = "-DCMAKE_CONFIGURATION_TYPES=Debug";
					break;
				case "Release":
					cmakeOptions += " -DUSE_ASSERTS=ON " ;
					cmakeOptions += " -DDEBUG_RENDERER_IN_DISTRIBUTION=ON ";
					cmakeOptions += " -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ";
					buildTypeGenerator = "-DCMAKE_CONFIGURATION_TYPES=Release";
					break;
				default:
					buildTypeGenerator = "-DCMAKE_CONFIGURATION_TYPES=Distribution ";
					break;
			}
		}else
		{
			Console.WriteLine("[ERROR] You are trying to build Jolt on an unsupported platform: `" + Target.Platform);
			return;
		}


		var generateCommand = "";
		generateCommand += MBuildUtils.GetCMakeExe() + " ";
		generateCommand += " -S " + Path.Combine(ThirdPartyJoltPath, "Build") + " ";
		generateCommand += "-B " + JoltBuildDir + " ";
		generateCommand += "-G " + cmakeGeneratorType + " ";
		generateCommand += buildTypeGenerator;
		generateCommand += cmakeOptions;
		var configureCode = MBuildUtils.ExecuteCommandSync(generateCommand,Path.GetFullPath(ModulePath));
		if (configureCode != 0)
		{
			Console.WriteLine("Jolt lib configure CMake project failed with code: " + configureCode);
			return;
		}
		// Compilation step
		var buildCommand = "";
		buildCommand += MBuildUtils.GetCMakeExe() + " ";
		buildCommand += " --build " + JoltBuildDir + " ";
		buildCommand += " -j " + Environment.ProcessorCount + " ";
		buildCommand += buildTypeCompilator;


		var buildExitCode = MBuildUtils.ExecuteCommandSync (buildCommand, Path.GetFullPath(ModulePath));
		if (buildExitCode != 0)
		{
			Console.WriteLine("Jolt lib build failed with code: " + buildExitCode);
		}
	}

	public UnrealJoltLibrary(ReadOnlyTargetRules Target) : base(Target)
	{
				// DOC: https://docs.unreal engine.com/4.27/en-US/Production Pipelines/BuildTools/UnrealBuild Tool/Third Party Libraries/
		// Useful resource: https://github.com/caseymcc/UE4CMake/
		// The ModuleType. External setting tells the engine not to look for (or compile) source code.
		string JoltBuildDir = MBuildUtils.GetJoltBuildDir(ModuleDirectory, Target);
		string ThirdPartyJoltPath= Path.Combine(ModuleDirectory, "JoltPhysics");
		Type = ModuleType.External;
		string buildType;
		if (Target.Configuration == UnrealTargetConfiguration.Debug ||
				Target.Configuration == UnrealTargetConfiguration.Debug)
		{
			buildType = "Debug";
		}
		else if (Target.Configuration == UnrealTargetConfiguration.Development)
		{
			buildType = "Release";
		}
		else
		{
			buildType = "Distribution";
		}

		BuildJolt(buildType, Target);
		// Specify these files should trigger a rebuild
		ExternalDependencies.Add(Path.Combine(ThirdPartyJoltPath, "Jolt"));
		//--- Now add the Jolt include paths
		PublicIncludePaths.Add(ThirdPartyJoltPath);

		// Now add the Jolt macros on the Module: for well-formed ABI ---
		// Available defines https://jrouwe.github.io/JoltPhysics/md__build_2_r_e_a_d_m_e.html#autotoc_md70
		PublicDefinitions.Add("JPH_DOUBLE_PRECISION");
		PublicDefinitions.Add("JPH_CROSS_PLATFORM_DETERMINISTIC");
		PublicDefinitions.Add("JPH_OBJECT_LAYER_BITS=32");
		PublicDefinitions.Add("JPH_OBJECT_STREAM");

		switch (buildType)
		{
			case "Debug":
				{
					// DEBUG
					Console.WriteLine("Building Jolt: DEBUG");
					PublicDefinitions.Add("_DEBUG=1");
					PublicDefinitions.Add("JPH_DEBUG_RENDERER");
					// Only on windows when compiling in debug mode this is enabled.
					PublicDefinitions.Add(Target.Platform == UnrealTargetPlatform.Win64
							? "JPH_FLOATING_POINT_EXCEPTIONS_ENABLED=1"
							: "JPH_ENABLE_ASSERTS");

					break;
				}
			case "Release":
				{
					// RELEASE
					Console.WriteLine("Building Jolt: RELEASE");
					PublicDefinitions.Add("JPH_DEBUG_RENDERER");
					PublicDefinitions.Add(Target.Platform == UnrealTargetPlatform.Win64
							? "JPH_FLOATING_POINT_EXCEPTIONS_ENABLED"
							: "JPH_ENABLE_ASSERTS");

					break;
				}
			default:
				// DISTRIBUTION
				Console.WriteLine("Building Jolt: DISTRIBUTION");
				//PublicDefinitions.Add("NDEBUG=1");
				break;
		}

		// --- Now it's time to link the compiler library
		string libPath;
		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			switch (buildType)
			{
				case "Debug":
					libPath = Path.Combine(JoltBuildDir, "Debug", "Jolt.lib");
					break;
				case "Release":
					libPath = Path.Combine(JoltBuildDir, "Release", "Jolt.lib");
					break;
				default:
					libPath = Path.Combine(JoltBuildDir, "Distribution", "Jolt.lib");
					break;
			}
		} else {
			libPath = Path.Combine(JoltBuildDir, "libJolt.a");
		}

		PublicAdditionalLibraries.Add(libPath);


	}
}

// from UE4CMAKE:  https://github.com/caseymcc/UE4CMake/blob/main/Source/CMakeTarget.Build.cs
public class MBuildUtils {
	public static Tuple<string, string> GetExecuteCommandSync()
	{
		string cmd = "";
		string options = "";

		if((BuildHostPlatform.Current.Platform == UnrealTargetPlatform.Win64) 
#if !UE_5_0_OR_LATER
				|| (BuildHostPlatform.Current.Platform == UnrealTargetPlatform.Win32)
#endif//!UE_5_0_OR_LATER
		  )
		{
			cmd="cmd.exe";
			options="/c ";
		}
		else if(IsUnixPlatform(BuildHostPlatform.Current.Platform)) 
		{
			cmd="bash";
			options="-c ";
		}
		return Tuple.Create(cmd, options);
	}

	public static int ExecuteCommandSync(string Command, string MModulePath)
	{
		var cmdInfo=GetExecuteCommandSync();

		if(IsUnixPlatform(BuildHostPlatform.Current.Platform)) 
		{
			Command=" \""+Command.Replace("\"", "\\\"")+" \"";
		}

		Console.WriteLine("Calling: "+cmdInfo.Item1+" "+cmdInfo.Item2+Command);

		var processInfo = new ProcessStartInfo(cmdInfo.Item1, cmdInfo.Item2+Command)
		{
			CreateNoWindow=true,
			UseShellExecute=false,
			RedirectStandardError=true,
			RedirectStandardOutput=true,
			WorkingDirectory=MModulePath
		};

		var outputString = new StringBuilder();
		var p = Process.Start(processInfo);

		p.OutputDataReceived+=(_, args) => {outputString.Append(args.Data); Console.WriteLine(args.Data);};
		p.ErrorDataReceived+=(_, args) => {outputString.Append(args.Data); Console.WriteLine(args.Data);};
		p.BeginOutputReadLine();
		p.BeginErrorReadLine();
		p.WaitForExit();

		if(p.ExitCode != 0)
		{
			Console.WriteLine(outputString);
		}
		return p.ExitCode;
	}

	private static bool IsUnixPlatform(UnrealTargetPlatform Platform) {
		return Platform == UnrealTargetPlatform.Linux || Platform == UnrealTargetPlatform.Mac;
	}

	public static string GetCMakeExe()
	{
		var program = "cmake";

		if((BuildHostPlatform.Current.Platform == UnrealTargetPlatform.Win64) 
#if !UE_5_0_OR_LATER
				|| (BuildHostPlatform.Current.Platform == UnrealTargetPlatform.Win32)
#endif//!UE_5_0_OR_LATER
		  )
		{
			program+=".exe";
		}
		return program;
	}

	
	public static string GetJoltBuildDir(string ModuleDirectory, ReadOnlyTargetRules Target)
	{
		string configFolder = "lib";

		// Convert the build configuration enum to string and lower-case it for folder naming.
		switch (Target.Configuration)
		{
			case UnrealTargetConfiguration.Debug:
				configFolder = "lib-debug";
				break;
			case UnrealTargetConfiguration.Development:
				configFolder = "lib-development";
				break;
			case UnrealTargetConfiguration.Shipping:
				configFolder = "lib-shipping";
				break;
			case UnrealTargetConfiguration.Test:
				configFolder = "lib-test";
				break;
			default:
				configFolder = "lib";
				break;
		}

		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			return Path.Combine(ModuleDirectory, configFolder, "Win64");
		}
		else if (Target.Platform == UnrealTargetPlatform.Mac)
		{
			return Path.Combine(ModuleDirectory, configFolder, "Mac");
		}
		else if (Target.Platform == UnrealTargetPlatform.Linux)
		{
			return Path.Combine(ModuleDirectory, configFolder, "Linux");
		}
		return "invalid platform";
	}


	/* Tacky function that chatGPT wrote. Meh... don't care. it works for now
	   basically it finds the toolchain directory for linux systems
	   the clang version will change based on engine version and this will use regex to find it
	   */
	public static string GetLatestBundledClangToolchain(string engineDir)
	{
		string toolchainBase = Path.Combine(engineDir, "Extras", "ThirdPartyNotUE", "SDKs", "HostLinux", "Linux_x64");

		var toolchains = Directory.GetDirectories(toolchainBase)
			.Select(dir => new
					{
					Path = dir,
					Name = Path.GetFileName(dir),
					Match = Regex.Match(Path.GetFileName(dir), @"^v(\d+)_clang-.*$")
					})
		.Where(t => t.Match.Success)
			.OrderByDescending(t => int.Parse(t.Match.Groups[1].Value))
			.ToList();

		if (toolchains.Count == 0)
		{
			throw new BuildException("No Clang toolchain found in: " + toolchainBase);
		}

		return toolchains.First().Path;
	}

}


