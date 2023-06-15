#include "Papyrus/sslThreadLibrary.h"

#include "Registry/Animation.h"
#include "Registry/Define/Furniture.h"
#include "Registry/Define/RaceKey.h"
#include "Registry/Library.h"
#include "Registry/Validation.h"

namespace Papyrus
{
	std::vector<RE::TESObjectREFR*> FindBeds(VM* a_vm, RE::VMStackID a_stackID, RE::TESQuest*, RE::TESObjectREFR* a_center, float a_radius, float a_radiusZ)
	{
		if (!a_center) {
			a_vm->TraceStack("Cannot find refs from a none center", a_stackID);
			return {};
		} else if (a_radius < 0.0f) {
			a_vm->TraceStack("Cannot find refs within a negative radius", a_stackID);
			return {};
		}
		return Registry::BedHandler::GetBedsInArea(a_center, a_radius, a_radiusZ);
	}

	int32_t GetBedType(VM* a_vm, StackID a_stackID, RE::TESQuest*, RE::TESObjectREFR* a_reference)
	{
		if (!a_reference) {
			a_vm->TraceStack("Reference is none", a_stackID);
			return false;
		}
		return static_cast<int32_t>(Registry::BedHandler::GetBedType(a_reference));
	}

	bool IsBed(VM* a_vm, RE::VMStackID a_stackID, RE::TESQuest*, RE::TESObjectREFR* a_reference)
	{
		if (!a_reference) {
			a_vm->TraceStack("Reference is none", a_stackID);
			return false;
		}
		return Registry::BedHandler::IsBed(a_reference);
	}

	std::vector<RE::Actor*> FindAvailableActors(VM* a_vm, StackID a_stackID, RE::TESQuest*, RE::TESObjectREFR* a_center, float a_radius, LegacySex a_targetsex,
		RE::Actor* ignore_ref01, RE::Actor* ignore_ref02, RE::Actor* ignore_ref03, RE::Actor* ignore_ref04)
	{
		if (!a_center) {
			a_vm->TraceStack("Cannot find actor from a none reference", a_stackID);
			return {};
		} else if (a_targetsex < LegacySex::None || a_targetsex > LegacySex::CrtFemale) {
			a_vm->TraceStack(fmt::format("Invalid target sex. Argument should be in [{}; {}]", LegacySex::None, LegacySex::CrtFemale).c_str(), a_stackID);
			return {};
		} else if (a_radius < 0) {
			a_vm->TraceStack("Cannot find actor in negative radius", a_stackID);
			return {};
		}
		std::vector<RE::Actor*> ret{};
		const auto& highactors = RE::ProcessLists::GetSingleton()->highActorHandles;
		for (auto&& handle : highactors) {
			const auto& actor = handle.get();
			if (!actor ||
					actor.get() == ignore_ref01 ||
					actor.get() == ignore_ref02 ||
					actor.get() == ignore_ref03 ||
					actor.get() == ignore_ref04)
				continue;

			if (!Registry::IsValidActor(actor.get()) || a_targetsex != LegacySex::None && GetLegacySex(actor.get()) != a_targetsex)
				continue;

			ret.push_back(actor.get());
		}
		return ret;
	}

	inline LegacySex ValidateSex(LegacySex a_targetsex, const RE::BSFixedString& a_targetrace)
	{
		if (a_targetsex >= LegacySex::CrtMale || !a_targetrace.empty() && a_targetrace != "humans") {
			if (a_targetsex == LegacySex::Male || !Settings::bCreatureGender) {
				return LegacySex::CrtMale;
			} else if (a_targetsex == LegacySex::Female) {
				return LegacySex::CrtFemale;
			}
		}
		return a_targetsex;
	}

	RE::Actor* FindAvailableActor(VM* a_vm, StackID a_stackID, RE::TESQuest*, RE::TESObjectREFR* a_center, float a_radius, LegacySex a_targetsex,
		RE::Actor* ignore_ref01, RE::Actor* ignore_ref02, RE::Actor* ignore_ref03, RE::Actor* ignore_ref04, RE::BSFixedString a_targetrace)
	{
		const auto targetsex = ValidateSex(a_targetsex, a_targetrace);
		auto valids = FindAvailableActors(a_vm, a_stackID, nullptr, a_center, a_radius, targetsex, ignore_ref01, ignore_ref02, ignore_ref03, ignore_ref04);
		if (!valids.empty()) {
			const auto targetrace = a_targetrace.empty() ? "humans" : a_targetrace;
			const auto where = std::remove_if(valids.begin(), valids.end(), [&](auto a_actor) {
				return !Registry::RaceHandler::HasRaceKey(a_actor, targetrace);
			});
			valids.erase(where, valids.end());
		}
		return valids.empty() ? nullptr : valids[0];
	}

	RE::Actor* FindAvailableActorInFaction(VM* a_vm, StackID a_stackID, RE::TESQuest*, RE::TESFaction* a_faction, RE::TESObjectREFR* a_center, float a_radius, LegacySex a_targetsex,
		RE::Actor* ignore_ref01, RE::Actor* ignore_ref02, RE::Actor* ignore_ref03, RE::Actor* ignore_ref04, bool a_hasfaction, RE::BSFixedString a_targetrace, bool a_samefloor)
	{
		if (!a_faction) {
			a_vm->TraceStack("Cannot find actor in none faction", a_stackID);
			return nullptr;
		}
		const auto targetsex = ValidateSex(a_targetsex, a_targetrace);
		auto valids = FindAvailableActors(a_vm, a_stackID, nullptr, a_center, a_radius, targetsex, ignore_ref01, ignore_ref02, ignore_ref03, ignore_ref04);
		for (auto&& actor : valids) {
			if (a_samefloor && std::fabs(a_center->GetPositionZ() - actor->GetPositionZ()) > 200)
				continue;
			if (actor->IsInFaction(a_faction) != a_hasfaction)
				continue;
			if (!Registry::RaceHandler::HasRaceKey(actor, a_targetrace))
				continue;

			return actor;
		}
		return nullptr;
	}

	RE::Actor* FindAvailableActorWornForm(VM* a_vm, StackID a_stackID, RE::TESQuest*, uint32_t a_slotmask, RE::TESObjectREFR* a_center, float a_radius, LegacySex a_targetsex,
		RE::Actor* ignore_ref01, RE::Actor* ignore_ref02, RE::Actor* ignore_ref03, RE::Actor* ignore_ref04, bool a_recognizenostrip, bool a_shouldwear, RE::BSFixedString a_targetrace,
		bool a_samefloor)
	{
		if (a_slotmask == 0) {
			a_vm->TraceStack("Cannot find actor from worn form without slotmask", a_stackID);
			return nullptr;
		}
		const auto slotmask = RE::BGSBipedObjectForm::BipedObjectSlot(a_slotmask);
		const auto targetsex = ValidateSex(a_targetsex, a_targetrace);
		const auto valids = FindAvailableActors(a_vm, a_stackID, nullptr, a_center, a_radius, targetsex, ignore_ref01, ignore_ref02, ignore_ref03, ignore_ref04);
		for (auto&& actor : valids) {
			if (a_samefloor && std::fabs(a_center->GetPositionZ() - actor->GetPositionZ()) > 200)
				continue;
			const auto armo = actor->GetWornArmor(slotmask);
			if (static_cast<bool>(armo) != a_shouldwear)
				continue;
			if (a_recognizenostrip) {
				const auto kywdform = armo ? armo->As<RE::BGSKeywordForm>() : nullptr;
				if (kywdform && kywdform->ContainsKeywordString("NoStrip"))
					continue;
			}
			if (!Registry::RaceHandler::HasRaceKey(actor, a_targetrace))
				continue;

			return actor;
		}
		return nullptr;
	}

	std::vector<RE::Actor*> FindAvailablePartners(VM* a_vm, StackID a_stackID, RE::TESQuest*,
		std::vector<RE::Actor*> a_positions, int a_total, int a_males, int a_females, float a_radius)
	{
		if (a_positions.size() >= a_total) {
			return a_positions;
		}
		const auto valids = FindAvailableActors(a_vm, a_stackID, nullptr,
			a_positions.empty() ? RE::PlayerCharacter::GetSingleton() : a_positions[0],
			a_radius,
			LegacySex::None,
			a_positions.size() > 0 ? a_positions[0] : nullptr,
			a_positions.size() > 1 ? a_positions[1] : nullptr,
			a_positions.size() > 2 ? a_positions[2] : nullptr,
			a_positions.size() > 3 ? a_positions[3] : nullptr);

		if (valids.empty()) {
			return a_positions;
		}
		auto genders = GetLegacySex(a_positions);
		for (auto&& actor : valids) {
			int targetsex;
			if (genders[LegacySex::Male] < a_males) {
				targetsex = LegacySex::Male;
			} else if (genders[LegacySex::Female] < a_females) {
				targetsex = LegacySex::Female;
			} else {
				targetsex = LegacySex::None;
			}
			const auto sex = GetLegacySex(actor);
			if (targetsex == LegacySex::None || sex == targetsex) {
				a_positions.push_back(actor);
				if (a_positions.size() == a_total) {
					return a_positions;
				} else {
					genders[sex]++;
				}
			}
		}
		return a_positions;
	}

	std::vector<RE::Actor*> FindAnimationPartnersImpl(VM* a_vm, StackID a_stackID, RE::TESQuest*,
		RE::BSFixedString a_sceneid, RE::TESObjectREFR* a_center, float a_radius, std::vector<RE::Actor*> a_includes)
	{
		const auto scene = Registry::Library::GetSingleton()->GetSceneByID(a_sceneid);
		if (!scene) {
			a_vm->TraceStack("Cannot find actors for a none scene", a_stackID);
			return {};
		}
		std::vector<RE::Actor*> ret{};
		auto valids = FindAvailableActors(a_vm, a_stackID, nullptr,
			a_center,
			a_radius,
			LegacySex::None,
			a_includes.size() > 0 ? a_includes[0] : nullptr,
			a_includes.size() > 1 ? a_includes[1] : nullptr,
			a_includes.size() > 2 ? a_includes[2] : nullptr,
			a_includes.size() > 3 ? a_includes[3] : nullptr);

		for (auto&& position : scene->positions) {
			RE::Actor* fill = nullptr;
			for (auto& include : a_includes) {
				if (include && position.CanFillPosition(include)) {
					include = nullptr;
					fill = include;
					break;
				}
			}
			if (fill) {
				continue;
			}
			for (auto& valid : valids) {
				if (valid && position.CanFillPosition(valid)) {
					valid = nullptr;
					fill = valid;
					break;
				}
			}
			if (!fill) {
				return {};
			}
		}
		return ret;
	}

	std::vector<RE::Actor*> SortActorsByAnimationImpl(VM* a_vm, StackID a_stackID, RE::TESQuest*,
		RE::BSFixedString a_sceneid, std::vector<RE::Actor*> a_positions, std::vector<RE::Actor*> a_submissives)
	{
		const auto library = Registry::Library::GetSingleton();
		const auto scene = library->GetSceneByID(a_sceneid);
		if (!scene) {
			a_vm->TraceStack("Cannot sort actors by a none scene", a_stackID);
			return a_positions;
		}
		auto subcount = scene->GetSubmissiveCount();
		std::vector<std::pair<RE::Actor*, Registry::PositionFragment>> argFrag;
		for (auto&& actor : a_positions) {
			const auto submissive = subcount > 0 && std::find(a_submissives.begin(), a_submissives.end(), actor) != a_submissives.end();
			if (submissive) {
				subcount--;
			}
			argFrag.emplace_back(
				actor,
				Registry::MakePositionFragment(
					actor,
					submissive).get());
		}
		auto ret = library->SortByScene(argFrag, scene);
		return ret.empty() ? a_positions : ret;
	}

}	 // namespace Papyrus
