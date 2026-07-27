#pragma once

#include "CoreMinimal.h"

/*
 * To follow rules of deterministic simulation
 * https://jrouwe.github.io/JoltPhysicsDocs/5.6.0/index.html#deterministic-simulation
 */
struct UNREALJOLT_API FJoltMath
{
	// Input in radians.
	static float Sin(float X);
	static float Cos(float X);
	static float Tan(float X);

	// Inputs clamped to [-1, 1] by Jolt — never returns NaN, unlike std::asin/acos.
	static float ASin(float X);
	static float ACos(float X);

	static float ATan(float X);
	static float ATan2(float Y, float X);
};
