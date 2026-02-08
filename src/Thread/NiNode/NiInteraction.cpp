#include "NiInteraction.h"

#include "NiMath.h"

namespace Thread::NiNode
{
	namespace 
	{
		template <typename MotionDescriptorType>
		void AddAngleScores(MotionDescriptorType* descriptor, const RE::NiPoint3& vecA, const RE::NiPoint3& vecB)
		{
			const auto scoreXY = NiMath::GetAngleXY(vecA, vecB);
			const auto scoreXZ = NiMath::GetAngleXZ(vecA, vecB);
			const auto scoreYZ = NiMath::GetAngleYZ(vecA, vecB);

			descriptor->AddValue(INiDescriptor::Feature::AngleXY, scoreXY);
			descriptor->AddValue(INiDescriptor::Feature::AngleXZ, scoreXZ);
			descriptor->AddValue(INiDescriptor::Feature::AngleYZ, scoreYZ);
		}
	}
	NiInteraction EvaluateKissing(const NiMotion& a_motionA, const NiMotion& a_motionB)
	{
		NiInteraction result{};
		assert(a_motionA.HasSufficientData() && a_motionB.HasSufficientData());

		const auto mouthA = a_motionA.DescribeMotion(NiMotion::pMouth);
		const auto mouthB = a_motionB.DescribeMotion(NiMotion::pMouth);
		const float mouthDistance = mouthA.Mean().GetDistance(mouthB.Mean());
		if (mouthDistance > Settings::fDistanceMouth * 2.0f) {
			return result;
		}

		const float duration = std::min(mouthA.duration, mouthB.duration);
		const float avgVelocity = 0.5f * (mouthA.avgSpeed + mouthB.avgSpeed);
		const auto vHeadYA = a_motionA.GetLatestMoment(NiMotion::vHeadY);
		const auto vHeadYB = a_motionB.GetLatestMoment(NiMotion::vHeadY);

		const float distanceScore = mouthDistance / Settings::fDistanceMouth;
		const float velocityScore = avgVelocity / Settings::fMaxKissSpeed;
		const float timeScore = duration / Settings::fMinTypeDuration;
		const float oscillationScore = 0.5f * (mouthA.oscillation + mouthB.oscillation);
		const float impulseScore = 0.5f * (mouthA.impulse + mouthB.impulse);
		const float stability = 0.5f * (mouthA.positionalVariance + mouthB.positionalVariance);

		auto descriptor = NiDescriptor<NiType::Kissing>();
		AddAngleScores(&descriptor, vHeadYA, vHeadYB);
		descriptor.AddValue(INiDescriptor::Feature::Distance, distanceScore);
		descriptor.AddValue(INiDescriptor::Feature::Time, timeScore);
		descriptor.AddValue(INiDescriptor::Feature::Velocity, velocityScore);
		descriptor.AddValue(INiDescriptor::Feature::Oscillation, oscillationScore);
		descriptor.AddValue(INiDescriptor::Feature::Impulse, impulseScore);
		descriptor.AddValue(INiDescriptor::Feature::Stability, stability);

		result.descriptor = std::make_unique<NiDescriptor<NiType::Kissing>>(std::move(descriptor));
		result.velocity = avgVelocity;

		return result;
	}

}  // namespace Thread::NiNode
