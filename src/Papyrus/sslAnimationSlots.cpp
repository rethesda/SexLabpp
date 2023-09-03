#include "sslAnimationSlots.h"

#include "Registry/Animation.h"
#include "Registry/Library.h"

namespace Papyrus::AnimationSlots
{
	inline std::vector<RE::BSFixedString> ScenesToString(std::vector<Registry::Scene*> a_scenes)
	{
		std::vector<RE::BSFixedString> ret{};
		ret.reserve(a_scenes.size());
		for (auto&& scene : a_scenes) {
			ret.push_back(scene->id);
		}
		return ret;
	}

	std::vector<RE::BSFixedString> GetByTagsImpl(VM* a_vm, StackID a_stackID, RE::TESQuest* a_qst, int32_t a_actorcount, std::vector<std::string_view> a_tags)
	{
		if (!a_qst) {
			a_vm->TraceStack("Cannot call GetByTagsImpl on a none object", a_stackID);
			return {};
		}
		if (a_actorcount <= 0 || a_actorcount > Registry::MAX_ACTOR_COUNT) {
			a_vm->TraceStack("Actor count should be between 0 and 5", a_stackID);
      return {};
		}
		const auto lib = Registry::Library::GetSingleton();
    const auto scenes = lib->GetByTags(a_actorcount, a_tags);
		return ScenesToString(scenes);
	}

	std::vector<RE::BSFixedString> GetByTypeImpl(VM* a_vm, StackID a_stackID, RE::TESQuest* a_qst,
		int32_t a_actorcount, int32_t a_males, int32_t a_females, std::vector<std::string_view> a_tags)
	{
		if (!a_qst) {
			a_vm->TraceStack("Cannot call GetByTypeImpl on a none object", a_stackID);
			return {};
		}
		if (a_actorcount <= 0 || a_actorcount > Registry::MAX_ACTOR_COUNT) {
			a_vm->TraceStack("Actor count should be between 0 and 5", a_stackID);
			return {};
		}
		const auto lib = Registry::Library::GetSingleton();
		auto scenes = lib->GetByTags(a_actorcount, a_tags);
		const auto end = std::remove_if(scenes.begin(), scenes.end(), [&](Registry::Scene* a_scene) {
			return !a_scene->Legacy_IsCompatibleSexCount(a_males, a_females);
		});
		const auto size = scenes.size();
		scenes.erase(end, scenes.end());
		logger::info("Get by tags found {} scenes; {}/{} match sex arguments", size, scenes.size(), size);
		return ScenesToString(scenes);
	}

	std::vector<RE::BSFixedString> PickByActorsImpl(VM* a_vm, StackID a_stackID, RE::TESQuest* a_qst, std::vector<RE::Actor*> a_positions, std::vector<std::string_view> a_tags)
	{
		if (!a_qst) {
			a_vm->TraceStack("Cannot call PickByActorsImpl on a none object", a_stackID);
			return {};
		}
		if (a_positions.empty() || std::find(a_positions.begin(), a_positions.end(), nullptr) != a_positions.end()) {
			a_vm->TraceStack("Cannot find scenes for a none position", a_stackID);
			return {};
		}
    const auto lib = Registry::Library::GetSingleton();
		std::vector<RE::Actor*> vic{};
		if (std::find(a_tags.begin(), a_tags.end(), "Forced") != a_tags.end()) {
			vic.push_back(a_positions[0]);
		}
		const auto scenes = lib->LookupScenes(a_positions, a_tags, vic);
		return ScenesToString(scenes);
	}

	std::vector<RE::BSFixedString> CreateProxyArray(RE::TESQuest*, uint32_t a_returnsize, uint32_t crt_specifier)
	{
		std::vector<RE::BSFixedString> ret{};
		ret.reserve(a_returnsize);
		Registry::Library::GetSingleton()->ForEachScene([&](const Registry::Scene* a_scene) {
			if (crt_specifier == 0 && a_scene->HasCreatures())
				return false;
			else if (crt_specifier == 1 && !a_scene->HasCreatures())
				return false;

			ret.push_back(a_scene->id);
			return ret.size() == a_returnsize;
		});
		return ret;
	}

}	 // namespace Papyrus::AnimationSlots
