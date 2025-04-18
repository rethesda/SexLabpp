#include "Furniture.h"

#include "Registry/Util/RayCast.h"
#include "Registry/Util/RayCast/ObjectBound.h"
#include "Util/StringUtil.h"

namespace Registry
{

#define MAPENTRY(value)             \
	{                                 \
		#value, FurnitureType::Value::value \
	}

	static inline const std::map<RE::BSFixedString, FurnitureType::Value, FixedStringCompare> FurniTable = {
		MAPENTRY(BedRoll),
		MAPENTRY(BedSingle),
		MAPENTRY(BedDouble),
		MAPENTRY(Wall),
		MAPENTRY(Railing),
		MAPENTRY(CraftCookingPot),
		MAPENTRY(CraftAlchemy),
		MAPENTRY(CraftEnchanting),
		MAPENTRY(CraftSmithing),
		MAPENTRY(CraftAnvil),
		MAPENTRY(CraftWorkbench),
		MAPENTRY(CraftGrindstone),
		MAPENTRY(Table),
		MAPENTRY(TableCounter),
		MAPENTRY(Chair),
		MAPENTRY(ChairCommon),
		MAPENTRY(ChairWood),
		MAPENTRY(ChairBar),
		MAPENTRY(ChairNoble),
		MAPENTRY(ChairMisc),
		MAPENTRY(Bench),
		MAPENTRY(BenchNoble),
		MAPENTRY(BenchMisc),
		MAPENTRY(Throne),
		MAPENTRY(ThroneRiften),
		MAPENTRY(ThroneNordic),
		MAPENTRY(XCross),
		MAPENTRY(Pillory),
	};

#undef MAPENTRY

	FurnitureType::FurnitureType(const RE::BSFixedString& a_value)
	{
		const auto where = FurniTable.find(a_value);
		if (where != FurniTable.end()) {
			value = where->second;
		} else {
			// throw std::runtime_error(std::format("Unrecognized Furniture: {}", a_value.c_str()));
			logger::error("Unrecognized Furniture: {}", a_value.c_str());
			value = Value::None;
		}
	}

	RE::BSFixedString FurnitureType::ToString() const
	{
		for (auto&& [key, value] : FurniTable) {
			if (value == this->value) {
				return key;
			}
		}
		return "";
	}

	FurnitureType FurnitureType::GetBedType(const RE::TESObjectREFR* a_reference)
	{
		if (a_reference->HasKeyword(GameForms::FurnitureBedRoll)) {
			return FurnitureType::BedRoll;
		}
		if (std::string name{ a_reference->GetName() }; name.empty() || Util::CastLower(name).find("bed") == std::string::npos)
			return FurnitureType::None;
		const auto root = a_reference->Get3D();
		const auto extra = root ? root->GetExtraData("FRN") : nullptr;
		const auto node = extra ? netimmerse_cast<RE::BSFurnitureMarkerNode*>(extra) : nullptr;
		if (!node) {
			return FurnitureType::None;
		}

		size_t sleepmarkers = 0;
		for (auto&& marker : node->markers) {
			if (marker.animationType.all(RE::BSFurnitureMarker::AnimationType::kSleep)) {
				sleepmarkers += 1;
			}
		}
		switch (sleepmarkers) {
		case 0:
			return FurnitureType::None;
		case 1:
			return FurnitureType::BedSingle;
		default:
			return FurnitureType::BedDouble;
		}
	}

	bool FurnitureType::IsBedType(const RE::TESObjectREFR* a_reference)
	{
		return GetBedType(a_reference).IsBed();
	}

	FurnitureDetails::FurnitureDetails(const YAML::Node& a_node)
	{
		const auto parse_node = [&](const YAML::Node& it) {
			const auto typenode = it["Type"];
			if (!typenode.IsDefined()) {
				logger::error("Missing 'Type' for node {}", a_node.Mark());
				return;
			}
			const auto typestr = typenode.as<std::string>();
			const auto furniture = FurnitureType(typestr);
			if (!furniture.IsValid()) {
				logger::error("Unrecognized Furniture: '{}' {}", typestr, typenode.Mark());
				return;
			}
			const auto offsetnode = it["Offset"];
			if (!offsetnode.IsDefined() || !offsetnode.IsSequence()) {
				logger::error("Missing 'Offset' node for type '{}' {}", typestr, typenode.Mark());
				return;
			}
			std::vector<Coordinate> offsets{};
			const auto push = [&](const YAML::Node& node) {
				const auto vec = node.as<std::vector<float>>();
				if (vec.size() != 4) {
					logger::error("Invalid offset size. Expected 4 but got {} {}", vec.size(), node.Mark());
					return;
				}
				Coordinate coords(vec);
				offsets.push_back(coords);
			};
			if (offsetnode[0].IsSequence()) {
				offsets.reserve(offsetnode.size());
				for (auto&& offset : offsetnode) {
					push(offset);
				}
			} else {
				push(offsetnode);
			}
			if (offsets.empty()) {
				logger::error("Type '{}' is defined but has no valid offsets {}", typestr, typenode.Mark());
				return;
			}
			_data.emplace_back(furniture, std::move(offsets));
		};
		if (a_node.IsSequence()) {
			for (auto&& it : a_node) {
				parse_node(it);
			}
		} else {
			parse_node(a_node);
		}
	}

	std::vector<std::pair<FurnitureType, std::vector<Coordinate>>> FurnitureDetails::GetCoordinatesInBound(RE::TESObjectREFR* a_ref, REX::EnumSet<FurnitureType::Value> a_filter) const
	{
		const auto niobj = a_ref->Get3D();
		const auto ninode = niobj ? niobj->AsNode() : nullptr;
		if (!ninode)
			return {};
		const auto boundingbox = ObjectBound::MakeBoundingBox(ninode);
		if (!boundingbox)
			return {};
		auto centerstart = boundingbox->GetCenterWorld();
		centerstart.z = boundingbox->worldBoundMax.z;

		const glm::vec4 tracestart{
			centerstart.x,
			centerstart.y,
			centerstart.z,
			0.0f
		};
		const auto ref_coords = Coordinate(a_ref);
		std::vector<std::pair<FurnitureType, std::vector<Coordinate>>> ret{};
		for (auto&& [type, offsetlist] : _data) {
			if (!a_filter.any(type.value)) {
				continue;
			}
			std::vector<Coordinate> vec{};
			for (auto&& offset : offsetlist) {
				auto coordinates = ref_coords;
				offset.Apply(coordinates);
				// check the place surrounding the center to see if there is anything occupying it (walls, actors, etc)
				// add some height to the base coordinate to avoid failing from hitting a rug or gold coin. 128 is roughly a NPC height in units
				constexpr auto radius = 32.0f;
				constexpr auto step = 8.0f;
				const auto& center = coordinates.location;
				const glm::vec4 traceend_base{ center.x, center.y, center.z + 16.0f, 0.0f };
				const glm::vec4 traceend_up{ center.x, center.y, center.z + 128.0f, 0.0f };
				for (float x = traceend_base.x - radius; x <= traceend_base.x + radius; x += step) {
					for (float y = traceend_base.y - radius; y <= traceend_base.y + radius; y += step) {
						glm::vec4 point(x, y, traceend_base.z, 0.0f);
						if (glm::distance(point, traceend_base) > radius)
							continue;
						const auto resA = Raycast::hkpCastRay(tracestart, point, { a_ref });
						if (resA.hit && glm::distance(resA.hitPos, point) > 8.0) {
							goto __L_NEXT;
						}
						const auto resB = Raycast::hkpCastRay(point, traceend_up, { a_ref });
						if (resB.hit) {
							goto __L_NEXT;
						}
					}
				}
				// if we got here then the area around coordinates, incl a cone upwards is free of obstacles
				vec.push_back(coordinates);
__L_NEXT:;
			}
			if (!vec.empty()) {
				ret.emplace_back(type, vec);
			}
		}
		return ret;
	}

	std::vector<std::pair<FurnitureType, Coordinate>> FurnitureDetails::GetClosestCoordinateInBound(
		RE::TESObjectREFR* a_ref,
		REX::EnumSet<FurnitureType::Value> a_filter,
		const RE::TESObjectREFR* a_center) const
	{
		const auto centercoordinates = Coordinate(a_center);
		const auto valids = GetCoordinatesInBound(a_ref, a_filter);
		std::vector<std::pair<FurnitureType, Coordinate>> ret{};
		for (auto&& [type, coordinates] : valids) {
			float distance = std::numeric_limits<float>::max();
			Coordinate distance_coords;

			for (auto&& coords : coordinates) {
				const auto d = glm::distance(coords.location, centercoordinates.location);
				if (d < distance) {
					distance_coords = coords;
					distance = d;
				}
			}
			if (distance != std::numeric_limits<float>::max()) {
				ret.emplace_back(type, distance_coords);
			}
		}
		return ret;
	}

}	 // namespace Registry
