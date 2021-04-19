// Copyright (c) 2020-2021 Julian "Jibb" Smart
// Released under the MIT license. See https://github.com/JibbSmart/GamepadMotionHelpers/blob/main/LICENSE for more info
// Revision 4

#define _USE_MATH_DEFINES
#include <math.h>

// You don't need to look at these. These will just be used internally by the GamepadMotion class declared below.
// You can ignore anything in namespace GamepadMotionHelpers.

namespace GamepadMotionHelpers
{
	struct GyroCalibration
	{
		float X;
		float Y;
		float Z;
		float AccelMagnitude;
		int NumSamples;
	};

	struct Quat
	{
		float w;
		float x;
		float y;
		float z;

		Quat();
		Quat(float inW, float inX, float inY, float inZ);
		void Set(float inW, float inX, float inY, float inZ);
		Quat& operator*=(const Quat& rhs);
		friend Quat operator*(Quat lhs, const Quat& rhs);
		void Normalize();
		Quat Normalized() const;
		void Invert();
		Quat Inverse() const;
	};

	struct Vec
	{
		float x;
		float y;
		float z;

		Vec();
		Vec(float inValue);
		Vec(float inX, float inY, float inZ);
		void Set(float inX, float inY, float inZ);
		float Length() const;
		void Normalize();
		Vec Normalized() const;
		float Dot(const Vec& other) const;
		Vec Cross(const Vec& other) const;
		Vec Min(const Vec& other) const;
		Vec Max(const Vec& other) const;
		Vec Abs() const;
		Vec Lerp(const Vec& other, float factor) const;
		Vec Lerp(const Vec& other, const Vec& factor) const;
		Vec& operator+=(const Vec& rhs);
		friend Vec operator+(Vec lhs, const Vec& rhs);
		Vec& operator-=(const Vec& rhs);
		friend Vec operator-(Vec lhs, const Vec& rhs);
		Vec& operator*=(const float rhs);
		friend Vec operator*(Vec lhs, const float rhs);
		Vec& operator/=(const float rhs);
		friend Vec operator/(Vec lhs, const float rhs);
		Vec& operator*=(const Quat& rhs);
		friend Vec operator*(Vec lhs, const Quat& rhs);
		Vec operator-() const;
	};

	struct SensorMinMaxWindow
	{
		Vec MinGyro;
		Vec MaxGyro;
		Vec MeanGyro;
		Vec MinAccel;
		Vec MaxAccel;
		Vec MeanAccel;
		int NumSamples;
		float TimeSampled;

		SensorMinMaxWindow();
		void Reset(float remainder);
		void AddSample(const Vec& inGyro, const Vec& inAccel, float deltaTime);
		Vec GetMidGyro();
	};

	// we want pairs because the balance will tell us if we have a gradient
	struct SensorWindowPair
	{
		SensorMinMaxWindow A;
		SensorMinMaxWindow B;
		SensorMinMaxWindow* Current;

		SensorWindowPair();
		void Reset(float remainder);
		void ChangeCurrent();
		SensorMinMaxWindow* GetOther();
	};

	struct AutoCalibration
	{
		const int NumWindows = 2;
		SensorWindowPair MinMaxWindows[2];
		Vec SmoothedAngularVelocityGyro;
		Vec SmoothedAngularVelocityAccel;
		Vec SmoothedPreviousAccel;
		Vec PreviousAccel;

		AutoCalibration();
		bool AddSampleStillness(const Vec& inGyro, const Vec& inAccel, Vec& inOutVecMask, float deltaTime);
		void NoSampleStillness();
		bool AddSampleSensorFusion(const Vec& inGyro, const Vec& inAccel, Vec& inOutVecMask, float deltaTime);
		void NoSampleSensorFusion();
		void SetCalibrationData(GyroCalibration* calibrationData);

	private:
		Vec MinDeltaGyro = Vec(10.f);
		Vec MinDeltaAccel = Vec(10.f);
		float RecalibrateThreshold = 1.f;

		const int MinAutoWindowSamples = 10;
		const float MinAutoWindowTime = 2.f;
		const float MaxRecalibrateThreshold = 1.1f;
		const float MinClimbRate = 0.2f;
		const float RecalibrateClimbRate = 0.1f;
		const float RecalibrateDrop = 0.1f;
		const float MaxMeanError = 0.4f;

		const float SmoothingQuickness = 5.f;
		const float AngularAccelerationThreshold = 20.f;
		const float SensorFusionCalibrationEaseInTime = 3.f;
		const float SensorFusionCalibrationQuickness = 10.f;
		
		float SensorFusionSkippedTime = 0.f;
		float TimeSteady = 0.f;

		GyroCalibration* CalibrationData;
	};

	struct Motion
	{
		Quat Quaternion;
		Vec Accel;
		Vec Grav;

		const int NumGravDirectionSamples = 10;
		Vec GravDirectionSamples[10];
		int LastGravityIdx = 9;
		int NumGravDirectionSamplesCounted = 0;
		float TimeCorrecting = 0.0f;

		Motion();
		void Reset();
		void Update(float inGyroX, float inGyroY, float inGyroZ, float inAccelX, float inAccelY, float inAccelZ, float gravityLength, float deltaTime);
	};
}

// Note that I'm using a Y-up coordinate system. This is to follow the convention set by the motion sensors in
// PlayStation controllers, which was what I was using when writing in this. But for the record, Z-up is
// better for most games (XY ground-plane in 3D games simplifies using 2D vectors in navigation, for example).

// Gyro units should be degrees per second. Accelerometer should be Gs (approx. 9.8m/s^2 = 1G). If you're using
// radians per second, meters per second squared, etc, conversion should be simple.

enum CalibrationMode
{
	Manual = 0,
	Stillness = 1,
	SensorFusion = 2,
};

// https://stackoverflow.com/a/1448478/1130520
inline CalibrationMode operator|(CalibrationMode a, CalibrationMode b)
{
    return static_cast<CalibrationMode>(static_cast<int>(a) | static_cast<int>(b));
}

inline CalibrationMode operator&(CalibrationMode a, CalibrationMode b)
{
    return static_cast<CalibrationMode>(static_cast<int>(a) & static_cast<int>(b));
}

inline CalibrationMode operator~(CalibrationMode a)
{
	return static_cast<CalibrationMode>(~static_cast<int>(a));
}

// https://stackoverflow.com/a/23152590/1130520
inline CalibrationMode& operator|=(CalibrationMode& a, CalibrationMode b)
{
	return (CalibrationMode&)((int&)(a) |= static_cast<int>(b));
}

inline CalibrationMode& operator&=(CalibrationMode& a, CalibrationMode b)
{
	return (CalibrationMode&)((int&)(a) &= static_cast<int>(b));
}

class GamepadMotion
{
public:
	GamepadMotion();

	void Reset();

	void ProcessMotion(float gyroX, float gyroY, float gyroZ,
		float accelX, float accelY, float accelZ, float deltaTime);

	// reading the current state
	void GetCalibratedGyro(float& x, float& y, float& z);
	void GetGravity(float& x, float& y, float& z);
	void GetProcessedAcceleration(float& x, float& y, float& z);
	void GetOrientation(float& w, float& x, float& y, float& z);

	// gyro calibration functions
	void StartContinuousCalibration();
	void PauseContinuousCalibration();
	void ResetContinuousCalibration();
	void GetCalibrationOffset(float& xOffset, float& yOffset, float& zOffset);
	void SetCalibrationOffset(float xOffset, float yOffset, float zOffset, int weight);

	CalibrationMode GetCalibrationMode();
	void SetCalibrationMode(CalibrationMode calibrationMode);

	void ResetMotion();

private:
	GamepadMotionHelpers::Vec Gyro;
	GamepadMotionHelpers::Vec RawAccel;
	GamepadMotionHelpers::Motion Motion;
	GamepadMotionHelpers::GyroCalibration GyroCalibration;
	GamepadMotionHelpers::AutoCalibration AutoCalibration;
	CalibrationMode CurrentCalibrationMode;

	bool IsCalibrating;
	void PushSensorSamples(float gyroX, float gyroY, float gyroZ, float accelMagnitude);
	void GetCalibratedSensor(float& gyroOffsetX, float& gyroOffsetY, float& gyroOffsetZ, float& accelMagnitude);
};

///////////// Everything below here are just implementation details /////////////

namespace GamepadMotionHelpers
{
	Quat::Quat()
	{
		w = 1.0f;
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	}

	Quat::Quat(float inW, float inX, float inY, float inZ)
	{
		w = inW;
		x = inX;
		y = inY;
		z = inZ;
	}

	static Quat AngleAxis(float inAngle, float inX, float inY, float inZ)
	{
		Quat result = Quat(cosf(inAngle * 0.5f), inX, inY, inZ);
		result.Normalize();
		return result;
	}

	void Quat::Set(float inW, float inX, float inY, float inZ)
	{
		w = inW;
		x = inX;
		y = inY;
		z = inZ;
	}

	Quat& Quat::operator*=(const Quat& rhs)
	{
		Set(w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
			w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
			w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
			w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w);
		return *this;
	}

	Quat operator*(Quat lhs, const Quat& rhs)
	{
		lhs *= rhs;
		return lhs;
	}

	void Quat::Normalize()
	{
		//printf("Normalizing: %.4f, %.4f, %.4f, %.4f\n", w, x, y, z);
		const float length = sqrtf(x * x + y * y + z * z);
		float targetLength = 1.0f - w * w;
		if (targetLength <= 0.0f || length <= 0.0f)
		{
			Set(1.0f, 0.0f, 0.0f, 0.0f);
			return;
		}
		targetLength = sqrtf(targetLength);
		const float fixFactor = targetLength / length;

		x *= fixFactor;
		y *= fixFactor;
		z *= fixFactor;

		//printf("Normalized: %.4f, %.4f, %.4f, %.4f\n", w, x, y, z);
		return;
	}

	Quat Quat::Normalized() const
	{
		Quat result = *this;
		result.Normalize();
		return result;
	}

	void Quat::Invert()
	{
		x = -x;
		y = -y;
		z = -z;
		return;
	}

	Quat Quat::Inverse() const
	{
		Quat result = *this;
		result.Invert();
		return result;
	}

	Vec::Vec()
	{
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	}

	Vec::Vec(float inValue)
	{
		x = inValue;
		y = inValue;
		z = inValue;
	}

	Vec::Vec(float inX, float inY, float inZ)
	{
		x = inX;
		y = inY;
		z = inZ;
	}

	void Vec::Set(float inX, float inY, float inZ)
	{
		x = inX;
		y = inY;
		z = inZ;
	}

	float Vec::Length() const
	{
		return sqrtf(x * x + y * y + z * z);
	}

	void Vec::Normalize()
	{
		const float length = Length();
		if (length == 0.0)
		{
			return;
		}
		const float fixFactor = 1.0f / length;

		x *= fixFactor;
		y *= fixFactor;
		z *= fixFactor;
		return;
	}

	Vec Vec::Normalized() const
	{
		Vec result = *this;
		result.Normalize();
		return result;
	}

	Vec& Vec::operator+=(const Vec& rhs)
	{
		Set(x + rhs.x, y + rhs.y, z + rhs.z);
		return *this;
	}

	Vec operator+(Vec lhs, const Vec& rhs)
	{
		lhs += rhs;
		return lhs;
	}

	Vec& Vec::operator-=(const Vec& rhs)
	{
		Set(x - rhs.x, y - rhs.y, z - rhs.z);
		return *this;
	}

	Vec operator-(Vec lhs, const Vec& rhs)
	{
		lhs -= rhs;
		return lhs;
	}

	Vec& Vec::operator*=(const float rhs)
	{
		Set(x * rhs, y * rhs, z * rhs);
		return *this;
	}

	Vec operator*(Vec lhs, const float rhs)
	{
		lhs *= rhs;
		return lhs;
	}

	Vec& Vec::operator/=(const float rhs)
	{
		Set(x / rhs, y / rhs, z / rhs);
		return *this;
	}

	Vec operator/(Vec lhs, const float rhs)
	{
		lhs /= rhs;
		return lhs;
	}

	Vec& Vec::operator*=(const Quat& rhs)
	{
		Quat temp = rhs * Quat(0.0f, x, y, z) * rhs.Inverse();
		Set(temp.x, temp.y, temp.z);
		return *this;
	}

	Vec operator*(Vec lhs, const Quat& rhs)
	{
		lhs *= rhs;
		return lhs;
	}

	Vec Vec::operator-() const
	{
		Vec result = Vec(-x, -y, -z);
		return result;
	}

	float Vec::Dot(const Vec& other) const
	{
		return x * other.x + y * other.y + z * other.z;
	}

	Vec Vec::Cross(const Vec& other) const
	{
		return Vec(y * other.z - z * other.y,
			z * other.x - x * other.z,
			x * other.y - y * other.x);
	}

	Vec Vec::Min(const Vec& other) const
	{
		return Vec(x < other.x ? x : other.x,
			y < other.y ? y : other.y,
			z < other.z ? z : other.z);
	}
	
	Vec Vec::Max(const Vec& other) const
	{
		return Vec(x > other.x ? x : other.x,
			y > other.y ? y : other.y,
			z > other.z ? z : other.z);
	}

	Vec Vec::Abs() const
	{
		return Vec(x > 0 ? x : -x,
			y > 0 ? y : -y,
			z > 0 ? z : -z);
	}

	Vec Vec::Lerp(const Vec& other, float factor) const
	{
		return *this + (other - *this) * factor;
	}

	Vec Vec::Lerp(const Vec& other, const Vec& factor) const
	{
		return Vec(this->x + (other.x - this->x) * factor.x,
			this->y + (other.y - this->y) * factor.y,
			this->z + (other.z - this->z) * factor.z);
	}

	Motion::Motion()
	{
		Reset();
	}

	void Motion::Reset()
	{
		Quaternion.Set(1.0f, 0.0f, 0.0f, 0.0f);
		Accel.Set(0.0f, 0.0f, 0.0f);
		Grav.Set(0.0f, 0.0f, 0.0f);
		NumGravDirectionSamplesCounted = 0;
	}

	/// <summary>
	/// The gyro inputs should be calibrated degrees per second but have no other processing. Acceleration is in G units (1 = approx. 9.8m/s^2)
	/// </summary>
	void Motion::Update(float inGyroX, float inGyroY, float inGyroZ, float inAccelX, float inAccelY, float inAccelZ, float gravityLength, float deltaTime)
	{
		const Vec axis = Vec(inGyroX, inGyroY, inGyroZ);
		const Vec accel = Vec(inAccelX, inAccelY, inAccelZ);
		float angle = axis.Length() * (float)M_PI / 180.0f;
		angle *= deltaTime;

		// rotate
		Quat rotation = AngleAxis(angle, axis.x, axis.y, axis.z);
		Quaternion *= rotation; // do it this way because it's a local rotation, not global
		//printf("Quat: %.4f %.4f %.4f %.4f _",
		//	Quaternion.w, Quaternion.x, Quaternion.y, Quaternion.z);
		float accelMagnitude = accel.Length();
		if (accelMagnitude > 0.0f)
		{
			const Vec accelNorm = accel / accelMagnitude;
			LastGravityIdx = (LastGravityIdx + NumGravDirectionSamples - 1) % NumGravDirectionSamples;
			// for comparing and perhaps smoothing gravity samples, we need them to be global
			Vec absoluteAccel = accel * Quaternion;
			//printf("Absolute Accel: %.4f %.4f %.4f\n",
			//	absoluteAccel.x, absoluteAccel.y, absoluteAccel.z);
			GravDirectionSamples[LastGravityIdx] = absoluteAccel;
			Vec gravityMin = absoluteAccel;
			Vec gravityMax = absoluteAccel;
			const float steadyGravityThreshold = 0.05f;
			NumGravDirectionSamplesCounted++;
			const int numGravSamples = NumGravDirectionSamplesCounted < NumGravDirectionSamples ? NumGravDirectionSamplesCounted : NumGravDirectionSamples;
			for (int idx = 1; idx < numGravSamples; idx++)
			{
				Vec thisSample = GravDirectionSamples[(LastGravityIdx + idx) % NumGravDirectionSamples];
				gravityMin = gravityMin.Min(thisSample);
				gravityMax = gravityMax.Max(thisSample);
			}
			const Vec gravityBoxSize = gravityMax - gravityMin;
			//printf(" Gravity Box Size: %.4f _ ", gravityBoxSize.Length());
			if (gravityBoxSize.x <= steadyGravityThreshold &&
				gravityBoxSize.y <= steadyGravityThreshold &&
				gravityBoxSize.z <= steadyGravityThreshold)
			{
				absoluteAccel = gravityMin + (gravityBoxSize * 0.5f);
				const Vec gravityDirection = -absoluteAccel.Normalized();
				const Vec expectedGravity = Vec(0.0f, -1.0f, 0.0f) * Quaternion.Inverse();
				const float errorAngle = acosf(Vec(0.0f, -1.0f, 0.0f).Dot(gravityDirection)) * 180.0f / (float)M_PI;

				const Vec flattened = gravityDirection.Cross(Vec(0.0f, -1.0f, 0.0f)).Normalized();

				if (errorAngle > 0.0f)
				{
					const float EaseInTime = 0.25f;
					TimeCorrecting += deltaTime;

					const float tighteningThreshold = 5.0f;

					float confidentSmoothCorrect = errorAngle;
					confidentSmoothCorrect *= 1.0f - exp2f(-deltaTime * 4.0f);

					if (TimeCorrecting < EaseInTime)
					{
						confidentSmoothCorrect *= TimeCorrecting / EaseInTime;
					}

					Quaternion = AngleAxis(confidentSmoothCorrect * (float)M_PI / 180.0f, flattened.x, flattened.y, flattened.z) * Quaternion;
				}
				else
				{
					TimeCorrecting = 0.0f;
				}

				Grav = Vec(0.0f, -gravityLength, 0.0f) * Quaternion.Inverse();
				Accel = accel + Grav; // gravity won't be shaky. accel might. so let's keep using the quaternion's calculated gravity vector.
			}
			else
			{
				TimeCorrecting = 0.0f;
				Grav = Vec(0.0f, -gravityLength, 0.0f) * Quaternion.Inverse();
				Accel = accel + Grav;
			}
		}
		else
		{
			TimeCorrecting = 0.0f;
			Accel.Set(0.0f, 0.0f, 0.0f);
		}
		Quaternion.Normalize();
	}

	SensorMinMaxWindow::SensorMinMaxWindow()
	{
		Reset(0.f);
	}

	void SensorMinMaxWindow::Reset(float remainder)
	{
		NumSamples = 0;
		TimeSampled = remainder;
	}

	void SensorMinMaxWindow::AddSample(const Vec& inGyro, const Vec& inAccel, float deltaTime)
	{
		if (NumSamples == 0)
		{
			MaxGyro = inGyro;
			MinGyro = inGyro;
			MeanGyro = inGyro;
			MaxAccel = inAccel;
			MinAccel = inAccel;
			MeanAccel = inAccel;
			NumSamples = 1;
			TimeSampled += deltaTime;
			return;
		}

		MaxGyro = MaxGyro.Max(inGyro);
		MinGyro = MinGyro.Min(inGyro);
		MaxAccel = MaxAccel.Max(inAccel);
		MinAccel = MinAccel.Min(inAccel);

		NumSamples++;
		TimeSampled += deltaTime;

		// https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
		Vec delta = inGyro - MeanGyro;
		MeanGyro += delta * (1.f / NumSamples);
		delta = inAccel - MeanAccel;
		MeanAccel += delta * (1.f / NumSamples);
	}

	Vec SensorMinMaxWindow::GetMidGyro()
	{
		//return (MaxGyro + MinGyro) * 0.5f;
		return MeanGyro;
	}

	SensorWindowPair::SensorWindowPair()
	{
		Reset(0.f);
	}

	void SensorWindowPair::Reset(float remainder)
	{
		Current = &A;
		A.Reset(remainder);
		B.Reset(0.f);
	}

	void SensorWindowPair::ChangeCurrent()
	{
		Current = GetOther();
	}

	SensorMinMaxWindow* SensorWindowPair::GetOther()
	{
		return Current == &A ? &B : &A;
	}

	AutoCalibration::AutoCalibration()
	{
		CalibrationData = nullptr;
		for (int Idx = 0; Idx < NumWindows; Idx++)
		{
			// -1/x, -2/x, -3/x, etc
			MinMaxWindows[Idx].A.TimeSampled = MinAutoWindowTime * (-Idx / (float)NumWindows);
			MinMaxWindows[Idx].B.TimeSampled = 0.f;
		}
	}

	bool AutoCalibration::AddSampleStillness(const Vec& inGyro, const Vec& inAccel, Vec& inOutVecMask, float deltaTime)
	{
		if ((inGyro.x == 0.f && inGyro.y == 0.f && inGyro.z == 0.f) ||
			(inAccel.x == 0.f && inAccel.y == 0.f && inAccel.z == 0.f))
		{
			// zeroes are almost certainly not valid inputs
			return false;
		}

		bool calibrated = false;
		const Vec climbThisTick = Vec(MinClimbRate * deltaTime);
		MinDeltaGyro += climbThisTick;
		MinDeltaAccel += climbThisTick;

		RecalibrateThreshold += RecalibrateClimbRate * deltaTime;
		if (RecalibrateThreshold > MaxRecalibrateThreshold) RecalibrateThreshold = MaxRecalibrateThreshold;

		for (int Idx = 0; Idx < NumWindows; Idx++)
		{
			SensorWindowPair* thisPair = &MinMaxWindows[Idx];
			const SensorWindowPair* otherPair = &MinMaxWindows[(Idx + NumWindows - 1) % NumWindows];
			SensorMinMaxWindow* thisSample = thisPair->Current;
			thisSample->AddSample(inGyro, inAccel, deltaTime);
			if (thisSample == &thisPair->A &&
				(thisSample->NumSamples >= MinAutoWindowSamples / 2 && thisSample->TimeSampled >= MinAutoWindowTime / 2.f))
			{
				thisPair->ChangeCurrent();
				continue;
			}
			else if (thisSample->NumSamples < MinAutoWindowSamples || thisSample->TimeSampled < MinAutoWindowTime)
			{
				continue;
			}

			const SensorMinMaxWindow* otherSample = thisPair->GetOther();

			const Vec maxGyro = thisSample->MaxGyro.Max(otherSample->MaxGyro);
			const Vec minGyro = thisSample->MinGyro.Min(otherSample->MinGyro);
			const Vec maxAccel = thisSample->MaxAccel.Max(otherSample->MaxAccel);
			const Vec minAccel = thisSample->MinAccel.Min(otherSample->MinAccel);

			// get deltas
			const Vec gyroDelta = maxGyro - minGyro;
			const Vec accelDelta = maxAccel - minAccel;

			MinDeltaGyro = MinDeltaGyro.Min(gyroDelta);
			MinDeltaAccel = MinDeltaAccel.Min(accelDelta);

			// each window is split into 2 halves. If the controller is truly still, each half will have similar properties
			Vec gyroMeanDelta = (thisSample->MeanGyro - otherSample->MeanGyro).Abs();
			Vec accelMeanDelta = (thisSample->MeanAccel - otherSample->MeanAccel).Abs();

			const Vec gyroMeanError = gyroDelta * MaxMeanError;
			const Vec accelMeanError = accelDelta * MaxMeanError;

			bool isFlat = true;
			bool isNarrow = true;

			//printf("\tgyro mean error: %.4f, %.4f, %.4f | gyro mean delta: %.4f, %.4f, %.4f\n",
			//		accelMeanError.x, accelMeanError.y, accelMeanError.z,
			//		gyroMeanDelta.x, gyroMeanDelta.y, gyroMeanDelta.z);

			// check for consistency within the window
			if (gyroMeanDelta.x > gyroMeanError.x || gyroMeanDelta.y > gyroMeanError.y || gyroMeanDelta.z > gyroMeanError.z ||
				accelMeanDelta.x > accelMeanError.x || accelMeanDelta.y > accelMeanError.y || accelMeanDelta.z > accelMeanError.z)
			{
				//printf("Too much flux across window: with gyro deltas: %.2f, %.2f, %.2f and accel deltas: %.3f, %.3f, %.3f\n",
				//	gyroDelta.x, gyroDelta.y, gyroDelta.z,
				//	accelDelta.x, accelDelta.y, accelDelta.z);
				isFlat = false;
			}

			// check that all inputs are below appropriate thresholds to be considered "still"
			if (gyroDelta.x > MinDeltaGyro.x * RecalibrateThreshold ||
				gyroDelta.y > MinDeltaGyro.y * RecalibrateThreshold ||
				gyroDelta.z > MinDeltaGyro.z * RecalibrateThreshold ||
				accelDelta.x > MinDeltaAccel.x * RecalibrateThreshold ||
				accelDelta.y > MinDeltaAccel.y * RecalibrateThreshold ||
				accelDelta.z > MinDeltaAccel.z * RecalibrateThreshold)
			{
				if (isFlat)
				{
					//printf("Too shaky: with gyro deltas: %.2f, %.2f, %.2f and accel deltas: %.3f, %.3f, %.3f\n",
					//	gyroDelta.x, gyroDelta.y, gyroDelta.z,
					//	accelDelta.x, accelDelta.y, accelDelta.z);
				}
				isNarrow = false;
			}

			if (isFlat && isNarrow)
			{
				//printf("Recalibrating... with gyro deltas: %.2f, %.2f, %.2f and accel deltas: %.3f, %.3f, %.3f\n",
				//	gyroDelta.x, gyroDelta.y, gyroDelta.z,
				//	accelDelta.x, accelDelta.y, accelDelta.z);

				RecalibrateThreshold -= RecalibrateDrop;
				if (RecalibrateThreshold < 1.f) RecalibrateThreshold = 1.f;

				if (CalibrationData != nullptr)
				{
					Vec calibratedGyro = (thisPair->A.GetMidGyro() + thisPair->B.GetMidGyro()) * 0.5f;

					Vec oldGyroBias = Vec(CalibrationData->X, CalibrationData->Y, CalibrationData->Z) / max((float)CalibrationData->NumSamples, 1.f);

					CalibrationData->X = (inOutVecMask.x != 0) ? calibratedGyro.x : oldGyroBias.x;
					CalibrationData->Y = (inOutVecMask.y != 0) ? calibratedGyro.y : oldGyroBias.y;
					CalibrationData->Z = (inOutVecMask.z != 0) ? calibratedGyro.z : oldGyroBias.z;

					CalibrationData->AccelMagnitude = (thisPair->A.MeanAccel + thisPair->B.MeanAccel).Length() * 0.5f;

					CalibrationData->NumSamples = 1;

					calibrated = true;
				}
			}

			const float otherTimeSampled = otherPair->A.TimeSampled + otherPair->B.TimeSampled;
			if (otherTimeSampled + deltaTime >= MinAutoWindowTime)
			{
				thisPair->Reset(MinAutoWindowTime / (float)NumWindows);
			}
			else
			{
				thisPair->Reset(otherTimeSampled - (MinAutoWindowTime / (float)NumWindows)); // keep in sync with other windows
			}
		}

		return calibrated;
	}

	void AutoCalibration::NoSampleStillness()
	{
		for (int Idx = 0; Idx < NumWindows; Idx++)
		{
			MinMaxWindows[Idx].Reset(0.f);
		}
	}

	bool AutoCalibration::AddSampleSensorFusion(const Vec& inGyro, const Vec& inAccel, Vec& inOutVecMask, float deltaTime)
	{
		if (deltaTime <= 0.f)
		{
			return false;
		}

		if ((inGyro.x == 0.f && inGyro.y == 0.f && inGyro.z == 0.f) ||
			(inAccel.x == 0.f && inAccel.y == 0.f && inAccel.z == 0.f))
		{
			// all zeroes are almost certainly not valid inputs
			TimeSteady = 0.f;
			SensorFusionSkippedTime = 0.f;
			PreviousAccel = inAccel;
			SmoothedPreviousAccel = inAccel;
			SmoothedAngularVelocityGyro = GamepadMotionHelpers::Vec();
			SmoothedAngularVelocityAccel = GamepadMotionHelpers::Vec();
			return false;
		}

		if (PreviousAccel.x == 0.f && PreviousAccel.y == 0.f && PreviousAccel.z == 0.f)
		{
			TimeSteady = 0.f;
			SensorFusionSkippedTime = 0.f;
			PreviousAccel = inAccel;
			SmoothedPreviousAccel = inAccel;
			SmoothedAngularVelocityGyro = GamepadMotionHelpers::Vec();
			SmoothedAngularVelocityAccel = GamepadMotionHelpers::Vec();
			return false;
		}

		// in case the controller state hasn't updated between samples
		if (inAccel.x == PreviousAccel.x && inAccel.y == PreviousAccel.y && inAccel.z == PreviousAccel.z)
		{
			SensorFusionSkippedTime += deltaTime;
			return false;
		}

		deltaTime += SensorFusionSkippedTime;
		SensorFusionSkippedTime = 0.f;
		bool calibrated = false;
		
		// TODO: soft-tiered smoothing
		// framerate independent lerp smoothing: https://www.gamasutra.com/blogs/ScottLembcke/20180404/316046/Improved_Lerp_Smoothing.php
		const float smoothingLerpFactor = exp2f(-SmoothingQuickness * deltaTime);
		//SmoothedAngularVelocityGyro += (inGyro - SmoothedAngularVelocityGyro) * (deltaForSmoothing);
		// velocity from smoothed accel matches better if we also smooth gyro
		//SmoothedAngularVelocityGyro = inGyro;
		Vec previousGyro = SmoothedAngularVelocityGyro;
		//const float gyroAccelerationMag = (inGyro - SmoothedAngularVelocityGyro).Length() / deltaTime;
		// soft-tired smoothing: http://gyrowiki.jibbsmart.com/blog:tight-and-smooth:soft-tiered-smoothing
		//const float directness = min(max(0.f, (gyroAccelerationMag - AngularAccelerationThreshold * 0.5f) / (AngularAccelerationThreshold * 0.5f)), 1.f);
		//SmoothedAngularVelocityGyro = inGyro.Lerp(SmoothedAngularVelocityGyro, directness); // non-smoothed portion
		SmoothedAngularVelocityGyro = inGyro.Lerp(SmoothedAngularVelocityGyro, smoothingLerpFactor); // smooth what remains
		const float gyroAccelerationMag = (SmoothedAngularVelocityGyro - previousGyro).Length() / deltaTime;
		const float directness = min(max(0.f, (gyroAccelerationMag - AngularAccelerationThreshold * 0.5f) / (AngularAccelerationThreshold * 0.5f)), 1.f);
		// get angle between old and new accel
		const Vec previousNormal = SmoothedPreviousAccel.Normalized();
		const Vec thisAccel = inAccel.Lerp(SmoothedPreviousAccel, smoothingLerpFactor);
		const Vec thisNormal = thisAccel.Normalized();
		Vec angularVelocity = thisNormal.Cross(previousNormal);
		const float crossLength = angularVelocity.Length();
		//const float angleChange = asinf(crossLength);
		const float thisDotPrev = min(max(-1.f, thisNormal.Dot(previousNormal)), 1.f);
		const float angleChange = acosf(thisDotPrev) * 180.0f / (float)M_PI;
		const float anglePerSecond = angleChange / deltaTime;
		if (crossLength > 0.f)
		{
			angularVelocity *= anglePerSecond / crossLength;
		}
		// smoothed
		//SmoothedAngularVelocityAccel += (angularVelocity - SmoothedAngularVelocityAccel) * (deltaForSmoothing);
		SmoothedAngularVelocityAccel = angularVelocity;

		// apply corrections
		if (directness > 0.f || CalibrationData == nullptr)
		{
			TimeSteady = 0.f;
			//printf("No calibration due to acceleration of %.4f\n", gyroAccelerationMag);
		}
		else
		{
			TimeSteady = min(TimeSteady + deltaTime, SensorFusionCalibrationEaseInTime);
			const float calibrationEaseIn = TimeSteady / SensorFusionCalibrationEaseInTime;
			const Vec oldGyroBias = Vec(CalibrationData->X, CalibrationData->Y, CalibrationData->Z) / max((float)CalibrationData->NumSamples, 1.f);
			// recalibrate over time proportional to the difference between the calculated bias and the current assumed bias
			Vec newGyroBias = (SmoothedAngularVelocityGyro - SmoothedAngularVelocityAccel).Lerp(oldGyroBias, exp2f(-SensorFusionCalibrationQuickness * calibrationEaseIn * deltaTime));
			// don't change bias in axes that can't be affected by the gravity direction
			Vec axisCalibrationStrength = thisNormal.Abs();
			if (axisCalibrationStrength.x > 0.7f)
			{
				axisCalibrationStrength.x = 1.f;
			}
			if (axisCalibrationStrength.y > 0.7f)
			{
				axisCalibrationStrength.y = 1.f;
			}
			if (axisCalibrationStrength.z > 0.7f)
			{
				axisCalibrationStrength.z = 1.f;
			}
			newGyroBias = newGyroBias.Lerp(oldGyroBias, axisCalibrationStrength.Min(Vec(1.f)));

			CalibrationData->X = (inOutVecMask.x != 0) ? newGyroBias.x : oldGyroBias.x;
			CalibrationData->Y = (inOutVecMask.y != 0) ? newGyroBias.y : oldGyroBias.y;
			CalibrationData->Z = (inOutVecMask.z != 0) ? newGyroBias.z : oldGyroBias.z;

			if (axisCalibrationStrength.x <= 0.7f)
			{
				inOutVecMask.x = 0;
			}
			if (axisCalibrationStrength.y <= 0.7f)
			{
				inOutVecMask.y = 0;
			}
			if (axisCalibrationStrength.z <= 0.7f)
			{
				inOutVecMask.z = 0;
			}

			CalibrationData->AccelMagnitude = thisAccel.Length();

			CalibrationData->NumSamples = 1;

			calibrated = true;

			//printf("Recalibrating at a strength of %.4f\n", calibrationEaseIn);
		}

		SmoothedPreviousAccel = thisAccel;
		PreviousAccel = inAccel;

		//printf("Gyro: %.4f, %.4f, %.4f | Accel: %.4f, %.4f, %.4f\n",
		//	SmoothedAngularVelocityGyro.x, SmoothedAngularVelocityGyro.y, SmoothedAngularVelocityGyro.z,
		//	SmoothedAngularVelocityAccel.x, SmoothedAngularVelocityAccel.y, SmoothedAngularVelocityAccel.z);

		return calibrated;
	}

	void AutoCalibration::NoSampleSensorFusion()
	{
		TimeSteady = 0.f;
		SensorFusionSkippedTime = 0.f;
		PreviousAccel = GamepadMotionHelpers::Vec();
		SmoothedPreviousAccel = GamepadMotionHelpers::Vec();
		SmoothedAngularVelocityGyro = GamepadMotionHelpers::Vec();
		SmoothedAngularVelocityAccel = GamepadMotionHelpers::Vec();
	}

	void AutoCalibration::SetCalibrationData(GyroCalibration* calibrationData)
	{
		CalibrationData = calibrationData;
	}

} // namespace GamepadMotionHelpers

GamepadMotion::GamepadMotion()
{
	IsCalibrating = false;
	CurrentCalibrationMode = CalibrationMode::Manual;
	Reset();
	AutoCalibration.SetCalibrationData(&GyroCalibration);
}

void GamepadMotion::Reset()
{
	GyroCalibration = {};
	Gyro = {};
	RawAccel = {};
	Motion.Reset();
}

void GamepadMotion::ProcessMotion(float gyroX, float gyroY, float gyroZ,
	float accelX, float accelY, float accelZ, float deltaTime)
{
	float accelMagnitude = sqrtf(accelX * accelX + accelY * accelY + accelZ * accelZ);

	if (IsCalibrating)
	{
		// manual calibration
		PushSensorSamples(gyroX, gyroY, gyroZ, accelMagnitude);
		AutoCalibration.NoSampleSensorFusion();
		AutoCalibration.NoSampleStillness();
	}
	else
	{
		// we only calibrate in axes that haven't already been calibrated by a previous step. To start, we're calibrating in All axes.
		GamepadMotionHelpers::Vec vecMask = GamepadMotionHelpers::Vec(1.f);
		
		if (CurrentCalibrationMode & CalibrationMode::SensorFusion)
		{
			AutoCalibration.AddSampleSensorFusion(GamepadMotionHelpers::Vec(gyroX, gyroY, gyroZ), GamepadMotionHelpers::Vec(accelX, accelY, accelZ), vecMask, deltaTime);
		}
		else
		{
			AutoCalibration.NoSampleSensorFusion();
		}

		if (CurrentCalibrationMode & CalibrationMode::Stillness)
		{
			AutoCalibration.AddSampleStillness(GamepadMotionHelpers::Vec(gyroX, gyroY, gyroZ), GamepadMotionHelpers::Vec(accelX, accelY, accelZ), vecMask, deltaTime);
		}
		else
		{
			AutoCalibration.NoSampleStillness();
		}
	}

	float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
	GetCalibratedSensor(gyroOffsetX, gyroOffsetY, gyroOffsetZ, accelMagnitude);

	gyroX -= gyroOffsetX;
	gyroY -= gyroOffsetY;
	gyroZ -= gyroOffsetZ;

	Motion.Update(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, accelMagnitude, deltaTime);

	Gyro.x = gyroX;
	Gyro.y = gyroY;
	Gyro.z = gyroZ;
	RawAccel.x = accelX;
	RawAccel.y = accelY;
	RawAccel.z = accelZ;
}

// reading the current state
void GamepadMotion::GetCalibratedGyro(float& x, float& y, float& z)
{
	x = Gyro.x;
	y = Gyro.y;
	z = Gyro.z;
}

void GamepadMotion::GetGravity(float& x, float& y, float& z)
{
	x = Motion.Grav.x;
	y = Motion.Grav.y;
	z = Motion.Grav.z;
}

void GamepadMotion::GetProcessedAcceleration(float& x, float& y, float& z)
{
	x = Motion.Accel.x;
	y = Motion.Accel.y;
	z = Motion.Accel.z;
}

void GamepadMotion::GetOrientation(float& w, float& x, float& y, float& z)
{
	w = Motion.Quaternion.w;
	x = Motion.Quaternion.x;
	y = Motion.Quaternion.y;
	z = Motion.Quaternion.z;
}

// gyro calibration functions
void GamepadMotion::StartContinuousCalibration()
{
	IsCalibrating = true;
}

void GamepadMotion::PauseContinuousCalibration()
{
	IsCalibrating = false;
}

void GamepadMotion::ResetContinuousCalibration()
{
	GyroCalibration = {};
}

void GamepadMotion::GetCalibrationOffset(float& xOffset, float& yOffset, float& zOffset)
{
	float accelMagnitude;
	GetCalibratedSensor(xOffset, yOffset, zOffset, accelMagnitude);
}

void GamepadMotion::SetCalibrationOffset(float xOffset, float yOffset, float zOffset, int weight)
{
	if (GyroCalibration.NumSamples > 1)
	{
		GyroCalibration.AccelMagnitude *= ((float)weight) / GyroCalibration.NumSamples;
	}
	else
	{
		GyroCalibration.AccelMagnitude = (float)weight;
	}

	GyroCalibration.NumSamples = weight;
	GyroCalibration.X = xOffset * weight;
	GyroCalibration.Y = yOffset * weight;
	GyroCalibration.Z = zOffset * weight;
}

CalibrationMode GamepadMotion::GetCalibrationMode()
{
	return CurrentCalibrationMode;
}

void GamepadMotion::SetCalibrationMode(CalibrationMode calibrationMode)
{
	CurrentCalibrationMode = calibrationMode;
}

void GamepadMotion::ResetMotion()
{
	Motion.Reset();
}

// Private Methods

void GamepadMotion::PushSensorSamples(float gyroX, float gyroY, float gyroZ, float accelMagnitude)
{
	// accumulate
	GyroCalibration.NumSamples++;
	GyroCalibration.X += gyroX;
	GyroCalibration.Y += gyroY;
	GyroCalibration.Z += gyroZ;
	GyroCalibration.AccelMagnitude += accelMagnitude;
}

void GamepadMotion::GetCalibratedSensor(float& gyroOffsetX, float& gyroOffsetY, float& gyroOffsetZ, float& accelMagnitude)
{
	if (GyroCalibration.NumSamples <= 0)
	{
		gyroOffsetX = 0.f;
		gyroOffsetY = 0.f;
		gyroOffsetZ = 0.f;
		accelMagnitude = 0.f;
		return;
	}

	const float inverseSamples = 1.f / GyroCalibration.NumSamples;
	gyroOffsetX = GyroCalibration.X * inverseSamples;
	gyroOffsetY = GyroCalibration.Y * inverseSamples;
	gyroOffsetZ = GyroCalibration.Z * inverseSamples;
	accelMagnitude = GyroCalibration.AccelMagnitude * inverseSamples;
}
