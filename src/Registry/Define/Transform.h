#pragma once

namespace Registry
{
	enum CoordinateType : uint8_t
	{
		X = 0,
		Y,
		Z,
		R,

		Total
	};

	struct Coordinate
	{
		Coordinate(const RE::TESObjectREFR* a_ref);
		Coordinate(const RE::NiPoint3& a_point, float a_rotation);
		Coordinate(const std::vector<float>& a_coordinates);
		Coordinate(std::ifstream& a_stream);
		Coordinate() = default;
		~Coordinate() = default;

		void Apply(Coordinate& a_coordinate) const;

		template <typename T>
		void ToContainer(T& a_out) const
		{
			assert(a_out.size() >= 4);
			std::copy_n(&location.x, location.length(), a_out.begin());
			a_out[3] = rotation;
		}
		RE::NiPoint3 AsNiPoint() const { return { location.x, location.y, location.z }; }
		float GetDistance(const Coordinate& a_other) const { return glm::distance(location, a_other.location); }

	public:
		bool operator==(const Coordinate& a_rhs) const { return location == a_rhs.location && rotation == a_rhs.rotation; }

	public:
		glm::vec3 location;
		float rotation;
	};

	class Transform
	{
	public:
		Transform(const Coordinate& a_rawcoordinates);
		Transform(std::ifstream& a_binarystream);
		Transform() = default;
		~Transform() = default;

	public:
		void Apply(Coordinate& a_coordinate) const { _offset.Apply(a_coordinate); }
		Coordinate ApplyCopy(const Coordinate& a_coordinate) const;
		bool HasChanges() const;

		const Coordinate& GetRawOffset() const;
		const Coordinate& GetOffset() const;
		float GetOffset(CoordinateType a_type) const;
		void SetOffset(float a_value, CoordinateType a_type);
		void UpdateOffset(const Coordinate& a_coordinate);
		void UpdateOffset(float x, float y, float z, float rot);
		void UpdateOffset(float a_value, CoordinateType a_where);
		void ResetOffset();

		void Save(YAML::Node& a_node) const;
		void Load(const YAML::Node& a_node);

	private:
		Coordinate _raw;
		Coordinate _offset;
	};

}	 // namespace Registry
