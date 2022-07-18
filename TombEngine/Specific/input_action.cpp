#include "framework.h"
#include "Specific/input_action.h"

#include "Game/control/control.h"
#include "Renderer/Renderer11.h"
#include "Specific/clock.h"

using TEN::Renderer::g_Renderer;

namespace TEN::Input
{
	InputAction::InputAction(ActionID actionID)
	{
		this->ID = actionID;
	}

	void InputAction::Update(float value)
	{
		this->UpdateValue(value);

		if (this->IsClicked())
		{
			this->PrevTimeActive = 0.0f;
			this->TimeActive = 0.0f;
			this->TimeInactive += DELTA_TIME;
		}
		else if (this->IsReleased())
		{
			this->PrevTimeActive = TimeActive;
			this->TimeActive += DELTA_TIME;
			this->TimeInactive = 0.0f;
		}
		else if (this->IsHeld())
		{
			this->PrevTimeActive = TimeActive;
			this->TimeActive += DELTA_TIME;
			this->TimeInactive = 0.0f;
		}
		else
		{
			this->PrevTimeActive = 0.0f;
			this->TimeActive = 0.0f;
			this->TimeInactive += DELTA_TIME;
		}
	}

	void InputAction::Clear()
	{
		this->Value = 0.0f;
		this->PrevValue = 0.0f;
		this->TimeActive = 0.0f;
		this->PrevTimeActive = 0.0f;
		this->TimeInactive = 0.0f;
	}

	void InputAction::PrintDebugInfo()
	{
		g_Renderer.PrintDebugMessage("ID: %d", (int)ID);
		g_Renderer.PrintDebugMessage("IsClicked: %d", this->IsClicked());
		g_Renderer.PrintDebugMessage("IsPulsed (.2s, .6s): %d", this->IsPulsed(0.2f, 0.6f));
		g_Renderer.PrintDebugMessage("IsHeld: %d", this->IsHeld());
		g_Renderer.PrintDebugMessage("IsReleased: %d", this->IsReleased());
		g_Renderer.PrintDebugMessage("");
		g_Renderer.PrintDebugMessage("Value: %.3f", Value);
		g_Renderer.PrintDebugMessage("PrevValue: %.3f", PrevValue);
		g_Renderer.PrintDebugMessage("TimeActive: %.3f", TimeActive);
		g_Renderer.PrintDebugMessage("PrevTimeActive: %.3f", PrevTimeActive);
		g_Renderer.PrintDebugMessage("TimeInactive: %.3f", TimeInactive);
	}

	bool InputAction::IsClicked()
	{
		return (Value && !PrevValue);
	}

	// To avoid desync on the second pulse, ensure initialDelayInSeconds is a multiple of delayInSeconds.
	bool InputAction::IsPulsed(float delayInSeconds, float initialDelayInSeconds)
	{
		if (this->IsClicked())
			return true;

		if (!this->IsHeld() || !PrevTimeActive || TimeActive == PrevTimeActive)
			return false;

		float syncedTimeActive = TimeActive - std::fmod(TimeActive, DELTA_TIME);
		float activeDelay = (TimeActive > initialDelayInSeconds) ? delayInSeconds : initialDelayInSeconds;
		float delayTime = std::floor(syncedTimeActive / activeDelay) * activeDelay;
		if (delayTime >= PrevTimeActive)
			return true;

		return false;
	}

	bool InputAction::IsHeld()
	{
		return Value;
	}

	bool InputAction::IsReleased()
	{
		return (!Value && PrevValue);
	}

	ActionID InputAction::GetID()
	{
		return ID;
	}

	float InputAction::GetValue()
	{
		return Value;
	}

	float InputAction::GetTimeActive()
	{
		return TimeActive;
	}

	float InputAction::GetTimeInactive()
	{
		return TimeInactive;
	}

	void InputAction::UpdateValue(float value)
	{
		this->PrevValue = Value;
		this->Value = value;
	}
}
