#pragma once

using std::array;
using std::vector;

namespace TEN::Input
{
	constexpr unsigned int NUM_DEFAULT_BINDINGS = 1;
	constexpr unsigned int MAX_BINDINGS			= 4;
	constexpr unsigned int MAX_KEY_MAPPINGS		= 4;

	// Contains multiple BINDINGS which may be composed of multiple KEY MAPPINGS.
	class InputBinding
	{
	public:
		InputBinding();
		InputBinding(array<array<int, MAX_KEY_MAPPINGS>, NUM_DEFAULT_BINDINGS> defaultBinding);

		array<int, MAX_KEY_MAPPINGS> Get(int bindingIndex);
		void Set(int bindingIndex, array<int, MAX_KEY_MAPPINGS> binding);
		void Clear(int bindingIndex);

	private:
		array<array<int, MAX_KEY_MAPPINGS>, MAX_BINDINGS> Bindings;

		void Initialize();
		bool IsIndexWithinRange(int bindingIndex);
	};
}
