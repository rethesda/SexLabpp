#pragma once

namespace Registry::Collision
{
	struct NodeUpdate
	{
		static void Install();

		static void AddOrUpdateSkew(const std::pair<RE::NiPointer<RE::NiNode*>, RE::NiMatrix3>& a_skew);
		static void DeleteSkew(const RE::NiPointer<RE::NiNode*>& a_skew);

	private:
		static void thunk(RE::NiAVObject* a_obj, RE::NiUpdateData* updateData);
		static inline REL::Relocation<decltype(thunk)> func;
		static inline constexpr std::size_t size{ 5 };

		static inline std::vector<std::pair<RE::NiPointer<RE::NiNode>, RE::NiMatrix3>> skews;
	};
} // namespace Registry::Collision