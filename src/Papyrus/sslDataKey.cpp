#include "Papyrus/sslDataKey.h"

namespace SLPP
{
	int32_t DataKey::BuildDataKeyNative(VM* a_vm, StackID a_stackID, RE::StaticFunctionTag*, RE::Actor* a_ref, bool a_isvictim, uint32_t a_raceid)
	{
		if (!a_ref) {
			a_vm->TraceStack("Cannot build data key from a none reference", a_stackID);
			return 0;
		}
		return static_cast<int32_t>(SexLab::DataKey::BuildKey(a_ref, a_isvictim, a_raceid));
	}

	int32_t DataKey::BuildCustomKey(VM* a_vm, StackID a_stackID, RE::StaticFunctionTag*, uint32_t a_gender, uint32_t a_raceid, std::vector<bool> a_extradata)
	{
		if (a_gender < 0 || a_gender > 4) {
			a_vm->TraceStack("Custom keys require a gender", a_stackID);
			return 0;
		}
		return static_cast<int32_t>(SexLab::DataKey::BuildCustomKey(a_gender, a_raceid, a_extradata));
	}

	std::vector<int32_t> DataKey::SortDataKeys(RE::StaticFunctionTag*, std::vector<int32_t> a_keys)
	{
		std::vector<uint32_t> tmp{ a_keys.begin(), a_keys.end() };
		SexLab::DataKey::SortKeys(tmp);
		return std::vector<int32_t>{ tmp.begin(), tmp.end() };
	}

	bool DataKey::IsLess(RE::StaticFunctionTag*, uint32_t a_key, uint32_t a_cmp)
	{
		return SexLab::DataKey::IsLess(a_key, a_cmp);
	}

	bool DataKey::Match(RE::StaticFunctionTag*, uint32_t a_key, uint32_t a_cmp)
	{
		return SexLab::DataKey::MatchKey(a_key, a_cmp);
	}

	bool DataKey::MatchArray(VM* a_vm, StackID a_stackID, RE::StaticFunctionTag*, std::vector<int32_t> a_key, std::vector<int32_t> a_cmp)
	{
		if (a_key.size() != a_cmp.size()) {
			a_vm->TraceStack("Cannot match two arrays of unequal size", a_stackID);
			return false;
		}
		std::vector<uint32_t> key{ a_key.begin(), a_key.end() }, cmp{ a_cmp.begin(), a_cmp.end() };
		return SexLab::DataKey::MatchArray(key, cmp);
	}
	
	int32_t DataKey::GetLegacyGenderByKey(RE::StaticFunctionTag*, uint32_t a_key)
	{
		if (a_key & Key::Male)
			return 0;
		else if (a_key & Key::Female)
			return 1;
		else if (a_key & Key::Crt_Male)
			return 2;
		else if (a_key & Key::Crt_Female)
			return 3;
		return 0;
	}

	int32_t DataKey::BuildByLegacyGenderNative(RE::StaticFunctionTag*, int32_t a_legacygender, int a_raceid)
	{
		uint32_t g;
		switch (a_legacygender) {
		case 0:
			g = Key::Male;
			break;
		case 1:
			g = Key::Female;
			break;
		case 2:
			g = Key::Crt_Male;
			break;
		case 3:
			g = Key::Crt_Female;
			break;
		case -1:
			return Key::Male | Key::Female | Key::Futa;
		default:
			g = Key::Male;
			break;
		}
		return g | (a_raceid << 8);
	}

	int32_t DataKey::AddGenderToKey(RE::StaticFunctionTag*, uint32_t a_key, uint32_t a_gender)
	{
		switch (a_gender) {
		case 0:
			return a_key | Key::Male;
		case 1:
			return a_key | Key::Female;
		case 2:
			return a_key | Key::Female;
		case 3:
			return a_key | Key::Crt_Male;
		case 4:
			return a_key | Key::Crt_Female;
		case 5:
			return a_key | Key::Overwrite_Male;
		case 6:
			return a_key | Key::Overwrite_Female;
		default:
			return a_key;
		}
	}

	int32_t DataKey::RemoveGenderFromKey(RE::StaticFunctionTag*, uint32_t a_key, uint32_t a_gender)
	{
		switch (a_gender) {
		case 0:
			return a_key & (~Key::Male);
		case 1:
			return a_key & (~Key::Female);
		case 2:
			return a_key & (~Key::Female);
		case 3:
			return a_key & (~Key::Crt_Male);
		case 4:
			return a_key & (~Key::Crt_Female);
		case 5:
			return a_key & (~Key::Overwrite_Male);
		case 6:
			return a_key & (~Key::Overwrite_Female);
		default:
			return a_key;
		}
	}

	void DataKey::NeutralizeCreatureGender(RE::StaticFunctionTag*, std::vector<int32_t> a_keys)
	{
		for (auto&& k : a_keys) {
			k |= Key::Crt_Male | Key::Crt_Female;
		}
	}
}