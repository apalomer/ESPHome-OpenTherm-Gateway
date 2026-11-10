#include "opentherm_climate.h"
#include "esphome/core/log.h"
#include "esphome/core/version.h"
#include <cstring>

namespace esphome {
namespace opentherm {

static const char *const TAG = "opentherm.climate";

// Room override presets, in RoomControl order
static const char *const ROOM_PRESETS[] = {"Thermostat", "Temporary override", "Permanent override"};

void OpenthermClimate::setup() {
  this->mode = climate::CLIMATE_MODE_HEAT;
  // Don't set target_temperature to 0 - leave it as NAN until initialized
  // from boiler or set by user
  if (default_target_temperature_ > 0) {
    this->target_temperature = default_target_temperature_;
    target_temperature_initialized_ = true;
  }
}

void OpenthermClimate::dump_config() {
  LOG_CLIMATE("", "OpenTherm Climate", this);
}

climate::ClimateTraits OpenthermClimate::traits() {
  auto traits = climate::ClimateTraits();
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  traits.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_HEAT});
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_ACTION);

  if (read_only_ || override_presets_) {
    // Room temperature range
    traits.set_visual_min_temperature(5);
    traits.set_visual_max_temperature(30);
    traits.set_visual_temperature_step(0.5);
  } else {
    traits.set_visual_min_temperature(5);
    traits.set_visual_max_temperature(80);
    traits.set_visual_temperature_step(1);
  }

  return traits;
}

void OpenthermClimate::set_override_presets(bool enabled) {
#if ESPHOME_VERSION_CODE >= VERSION_CODE(2026, 5, 0)
  override_presets_ = enabled;
  if (enabled) {
    this->set_supported_custom_presets(ROOM_PRESETS);
    this->set_custom_preset_(ROOM_PRESETS[0]);
  }
#else
  if (enabled)
    ESP_LOGE(TAG, "Room override needs ESPHome 2026.5.0 or newer");
#endif
}

void OpenthermClimate::set_room_control_preset(RoomControl control) {
#if ESPHOME_VERSION_CODE >= VERSION_CODE(2026, 5, 0)
  if (override_presets_)
    this->set_custom_preset_(ROOM_PRESETS[static_cast<uint8_t>(control)]);
#endif
}

optional<RoomControl> OpenthermClimate::requested_room_control(const climate::ClimateCall &call) const {
#if ESPHOME_VERSION_CODE >= VERSION_CODE(2026, 5, 0)
  if (call.has_custom_preset()) {
    const char *preset = call.get_custom_preset().c_str();
    for (uint8_t i = 0; i < sizeof(ROOM_PRESETS) / sizeof(ROOM_PRESETS[0]); i++) {
      if (strcmp(preset, ROOM_PRESETS[i]) == 0)
        return static_cast<RoomControl>(i);
    }
  }
#endif
  return {};
}

void OpenthermClimate::control(const climate::ClimateCall &call) {
  if (control_callback_) {
    control_callback_(call);
    return;
  }

  if (call.get_mode().has_value()) {
    esphome::climate::ClimateMode mode = *call.get_mode();
    ESP_LOGD(TAG, "Setting mode to %d", mode);
    this->mode = mode;
    this->publish_state();
  }

  if (call.get_target_temperature().has_value() && !read_only_) {
    float temp = *call.get_target_temperature();
    ESP_LOGD(TAG, "Setting target temperature to %.1f", temp);
    this->target_temperature = temp;
    target_temperature_initialized_ = true;  // Mark as user-set

    if (target_temperature_setter_) {
      target_temperature_setter_(temp);
    }

    this->publish_state();
  }
}

}  // namespace opentherm
}  // namespace esphome
