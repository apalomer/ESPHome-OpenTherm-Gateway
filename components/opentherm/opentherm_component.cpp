#include "opentherm_component.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cmath>

namespace esphome
{
  namespace opentherm
  {

    static const char *const TAG = "opentherm.component";

    // Initialize static members
    OpenthermComponent *OpenthermComponent::instance_ = nullptr;
    unsigned long OpenthermComponent::last_status_response_ = 0;
    unsigned long OpenthermComponent::last_intercepted_response_ = 0;
    OpenThermMessageID OpenthermComponent::last_intercepted_id_ = static_cast<OpenThermMessageID>(0);
    bool OpenthermComponent::has_new_intercepted_response_ = false;

    OpenthermComponent::OpenthermComponent(uint32_t update_interval) : PollingComponent(update_interval)
    {
      instance_ = this;
    }

    void OpenthermComponent::add_read_sensor(sensor::Sensor *sensor, uint8_t message_id)
    {
      GenericReadSensor read_sensor;
      read_sensor.sensor = sensor;
      read_sensor.id = static_cast<OpenThermMessageID>(message_id);
      read_sensors_.push_back(read_sensor);
    }

    void OpenthermComponent::setup()
    {
      ESP_LOGD(TAG, "Setting up OpenTherm component");

      // Initialize OpenTherm instances
      ot_ = new OpenTherm(in_pin_, out_pin_, false);                  // Master
      slave_ot_ = new OpenTherm(slave_in_pin_, slave_out_pin_, true); // Slave

      // Start OpenTherm communication
      ot_->begin(handleInterrupt);
      slave_ot_->begin(slaveHandleInterrupt, processRequest);

      // Setup climate controllers
      if (hot_water_climate_ != nullptr)
      {
        hot_water_climate_->set_target_temperature_callback([this](float temperature)
                                                            { return this->setHotWaterTemperature(temperature); });
      }

      if (heating_water_climate_ != nullptr)
      {
        heating_water_climate_->set_target_temperature_callback([this](float temperature)
                                                                { return this->setHeatingTargetTemperature(temperature); });
      }

      // Read Phase 1 values once at startup (these don't change)
      delay(1000); // Give OpenTherm time to initialize

      // Read max CH setpoint (Data-ID 57)
      if (max_ch_setpoint_sensor_ != nullptr)
      {
        unsigned long response = ot_->sendRequest(ot_->buildRequest(OpenThermRequestType::READ, OpenThermMessageID::MaxTSet, 0));
        if (ot_->isValidResponse(response))
        {
          float value = ot_->getFloat(response);
          max_ch_setpoint_sensor_->publish_state(value);
          ESP_LOGI(TAG, "Max CH setpoint: %.1f°C", value);
        }
      }

      // Note: Min CH setpoint (Data-ID 58) is not in standard OpenTherm spec
      // Most boilers don't support it, so we skip it

      // Read max relative modulation (Data-ID 14)
      if (max_modulation_sensor_ != nullptr)
      {
        unsigned long response = ot_->sendRequest(ot_->buildRequest(OpenThermRequestType::READ, OpenThermMessageID::MaxRelModLevelSetting, 0));
        if (ot_->isValidResponse(response))
        {
          float value = ot_->getFloat(response);
          max_modulation_sensor_->publish_state(value);
          ESP_LOGI(TAG, "Max modulation: %.1f%%", value);
        }
      }

      // Read OpenTherm versions (Data-ID 124, 125)
      if (master_ot_version_sensor_ != nullptr)
      {
        unsigned long response = ot_->sendRequest(ot_->buildRequest(OpenThermRequestType::READ, OpenThermMessageID::OpenThermVersionMaster, 0));
        if (ot_->isValidResponse(response))
        {
          float value = ot_->getFloat(response);
          master_ot_version_sensor_->publish_state(value);
          ESP_LOGI(TAG, "Master OT version: %.2f", value);
        }
      }

      if (slave_ot_version_sensor_ != nullptr)
      {
        unsigned long response = ot_->sendRequest(ot_->buildRequest(OpenThermRequestType::READ, OpenThermMessageID::OpenThermVersionSlave, 0));
        if (ot_->isValidResponse(response))
        {
          float value = ot_->getFloat(response);
          slave_ot_version_sensor_->publish_state(value);
          ESP_LOGI(TAG, "Slave OT version: %.2f", value);
        }
      }

      if (override_enabled_ && room_climate_ != nullptr)
      {
        room_climate_->set_override_presets(true);
        room_climate_->set_control_callback([this](const climate::ClimateCall &call)
                                            { this->room_control(call); });
        ESP_LOGI(TAG, "Room override enabled: flow %.0f-%.0f°C, kp %.1f, ki %.2f/min, hysteresis %.1f K",
                 ovr_min_flow_, ovr_max_flow_, ovr_kp_, ovr_ki_, ovr_hysteresis_);
      }
    }

    void OpenthermComponent::loop()
    {
      slave_ot_->process();

      // A thermostat setpoint change ends a temporary override
      if (thermostat_setpoint_changed_)
      {
        thermostat_setpoint_changed_ = false;
        if (room_control_ == RoomControl::TEMPORARY)
        {
          ESP_LOGI(TAG, "Thermostat setpoint changed to %.1f°C, ending temporary override", cached_room_setpoint_.value);
          setRoomControl(RoomControl::THERMOSTAT);
          runRoomControl();
          publishRoomClimate(ot_->isCentralHeatingActive(last_status_response_));
        }
      }

      // Process intercepted responses (moved from interrupt context)
      if (has_new_intercepted_response_)
      {
        processCachedResponse(last_intercepted_response_, last_intercepted_id_);
        has_new_intercepted_response_ = false;
      }

      fetchPendingValue();
    }

    void OpenthermComponent::update()
    {
      // Read and publish sensor values

      // Binary sensors from status
      bool is_flame_on = ot_->isFlameOn(last_status_response_);
      bool is_central_heating_active = ot_->isCentralHeatingActive(last_status_response_);
      bool is_hot_water_active = ot_->isHotWaterActive(last_status_response_);
      bool is_fault = ot_->isFault(last_status_response_);
      bool is_diagnostic = ot_->isDiagnostic(last_status_response_);

      if (flame_ != nullptr)
        flame_->publish_state(is_flame_on);

      if (ch_active_ != nullptr)
        ch_active_->publish_state(is_central_heating_active);

      if (dhw_active_ != nullptr)
        dhw_active_->publish_state(is_hot_water_active);

      if (fault_ != nullptr)
        fault_->publish_state(is_fault);

      if (diagnostic_ != nullptr)
        diagnostic_->publish_state(is_diagnostic);

      if (dropped_frames_sensor_ != nullptr)
        dropped_frames_sensor_->publish_state(dropped_frames_);

      // Temperature and other sensors (using cache with timeout).
      // Only values something consumes are read, so unsupported IDs don't cost bus time.
      float ext_temperature = external_temperature_sensor_ != nullptr ? getExternalTemperature() : NAN;
      float return_temperature = return_temperature_sensor_ != nullptr ? getReturnTemperature() : NAN;
      float boiler_temperature = (boiler_temperature_ != nullptr || heating_water_climate_ != nullptr)
                                     ? getCachedOrFetch(cached_boiler_temp_, OpenThermMessageID::Tboiler)
                                     : NAN;
      float exhaust_temperature = exhaust_temperature_sensor_ != nullptr
                                      ? getCachedOrFetch(cached_exhaust_temp_, OpenThermMessageID::Texhaust)
                                      : NAN;
      float pressure = pressure_sensor_ != nullptr ? getPressure() : NAN;
      float modulation = modulation_sensor_ != nullptr ? getModulation() : NAN;
      float heating_target_temp = (heating_target_temperature_sensor_ != nullptr || heating_water_climate_ != nullptr)
                                      ? getHeatingTargetTemperature()
                                      : NAN;
      float hot_water_temp = hot_water_climate_ != nullptr ? getHotWaterTemperature() : NAN;

      if (external_temperature_sensor_ != nullptr && !std::isnan(ext_temperature))
        external_temperature_sensor_->publish_state(ext_temperature);

      if (return_temperature_sensor_ != nullptr && !std::isnan(return_temperature))
        return_temperature_sensor_->publish_state(return_temperature);

      if (boiler_temperature_ != nullptr && !std::isnan(boiler_temperature))
        boiler_temperature_->publish_state(boiler_temperature);

      if (exhaust_temperature_sensor_ != nullptr && !std::isnan(exhaust_temperature))
        exhaust_temperature_sensor_->publish_state(exhaust_temperature);

      if (pressure_sensor_ != nullptr && !std::isnan(pressure))
        pressure_sensor_->publish_state(pressure);

      if (modulation_sensor_ != nullptr && !std::isnan(modulation))
        modulation_sensor_->publish_state(modulation);

      if (heating_target_temperature_sensor_ != nullptr && !std::isnan(heating_target_temp) && heating_target_temp > 0)
        heating_target_temperature_sensor_->publish_state(heating_target_temp);
      if (room_temperature_sensor_ != nullptr && !std::isnan(cached_room_temp_.value))
        room_temperature_sensor_->publish_state(cached_room_temp_.value);
      if (room_setpoint_sensor_ != nullptr && !std::isnan(cached_room_setpoint_.value))
        room_setpoint_sensor_->publish_state(cached_room_setpoint_.value);

      for (auto &read_sensor : read_sensors_)
      {
        if (read_sensor.sensor == nullptr)
          continue;

        float value = readGenericSensorValue(read_sensor);
        if (!std::isnan(value))
          read_sensor.sensor->publish_state(value);
      }

      // Read OEM diagnostic codes (Data-ID 5 and 115) - only if fault or diagnostic active
      if (is_fault || is_diagnostic)
      {
        // OEM fault code (Data-ID 5) - Application-specific fault flags
        if (oem_fault_code_sensor_ != nullptr)
        {
          unsigned long response = ot_->sendRequest(ot_->buildRequest(OpenThermRequestType::READ, OpenThermMessageID::ASFflags, 0));
          if (ot_->isValidResponse(response))
          {
            uint16_t fault_code = response & 0xFF; // Low byte contains OEM fault code
            oem_fault_code_sensor_->publish_state(fault_code);
            if (fault_code != 0)
            {
              ESP_LOGW(TAG, "OEM Fault Code: %d", fault_code);
            }
          }
        }

        // OEM diagnostic code (Data-ID 115)
        if (oem_diagnostic_code_sensor_ != nullptr)
        {
          unsigned long response = ot_->sendRequest(ot_->buildRequest(OpenThermRequestType::READ, OpenThermMessageID::OEMDiagnosticCode, 0));
          if (ot_->isValidResponse(response))
          {
            uint16_t diag_code = response & 0xFFFF; // Full 16-bit diagnostic code
            oem_diagnostic_code_sensor_->publish_state(diag_code);
            if (diag_code != 0)
            {
              ESP_LOGW(TAG, "OEM Diagnostic Code: %d", diag_code);
            }
          }
        }
      }
      else
      {
        // No fault - publish 0
        if (oem_fault_code_sensor_ != nullptr)
          oem_fault_code_sensor_->publish_state(0);
        if (oem_diagnostic_code_sensor_ != nullptr)
          oem_diagnostic_code_sensor_->publish_state(0);
      }

      // Update climate controllers
      if (hot_water_climate_ != nullptr)
      {
        hot_water_climate_->current_temperature = hot_water_temp;
        hot_water_climate_->action = is_hot_water_active ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_OFF;
        float hot_water_target = getHotWaterTargetTemperature();
        if (!std::isnan(hot_water_target))
          hot_water_climate_->target_temperature = hot_water_target;
        hot_water_climate_->publish_state();
      }

      if (heating_water_climate_ != nullptr)
      {
        heating_water_climate_->current_temperature = boiler_temperature;
        heating_water_climate_->action = is_central_heating_active ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_OFF;
        // Initialize target temperature from boiler on first update only
        // After that, keep the user's set value and don't overwrite it
        // The heating_target_temperature_sensor will show what the boiler is actually using
        heating_water_climate_->initialize_target_temperature(heating_target_temp);
        heating_water_climate_->publish_state();
      }

      runRoomControl();
      if (room_climate_ != nullptr)
        publishRoomClimate(is_central_heating_active);
    }

    void OpenthermComponent::publishRoomClimate(bool ch_active)
    {
      float room_temperature = NAN;
      if (room_climate_external_sensor_ != nullptr && room_climate_external_sensor_->has_state())
      {
        room_temperature = room_climate_external_sensor_->state;
      }
      else if (!std::isnan(cached_room_temp_.value))
      {
        room_temperature = cached_room_temp_.value;
      }

      if (!std::isnan(room_temperature))
        room_climate_->current_temperature = room_temperature;

      // Thermostat preset follows the thermostat; an override shows the HA setpoint
      float target = room_control_ == RoomControl::THERMOSTAT ? cached_room_setpoint_.value : override_setpoint_;
      if (!std::isnan(target))
        room_climate_->target_temperature = target;
      if (override_enabled_)
        room_climate_->set_room_control_preset(room_control_);
      room_climate_->action = ch_active ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_OFF;
      room_climate_->publish_state();
    }

    void OpenthermComponent::set_room_override(float min_flow, float max_flow, float kp, float ki, float hysteresis)
    {
      override_enabled_ = true;
      ovr_min_flow_ = min_flow;
      ovr_max_flow_ = max_flow;
      ovr_kp_ = kp;
      ovr_ki_ = ki;
      ovr_hysteresis_ = hysteresis;
    }

    void OpenthermComponent::room_control(const climate::ClimateCall &call)
    {
      if (call.get_mode().has_value())
      {
        climate::ClimateMode mode = *call.get_mode();
        bool off = mode == climate::CLIMATE_MODE_OFF;
        if (off != room_off_)
        {
          ESP_LOGI(TAG, "Room heating %s from Home Assistant", off ? "switched off" : "switched back on");
          room_off_ = off;
          ovr_active_ = false; // restart the controller cleanly
        }
        room_climate_->mode = mode;
      }

      optional<RoomControl> preset = room_climate_->requested_room_control(call);
      if (preset.has_value())
        setRoomControl(*preset);

      if (call.get_target_temperature().has_value())
      {
        override_setpoint_ = *call.get_target_temperature();
        ESP_LOGI(TAG, "Room setpoint from Home Assistant: %.1f°C", override_setpoint_);
        // Changing the temperature while following the thermostat starts a temporary override
        if (room_control_ == RoomControl::THERMOSTAT)
          setRoomControl(RoomControl::TEMPORARY);
      }

      runRoomControl();
      publishRoomClimate(ot_->isCentralHeatingActive(last_status_response_));
    }

    void OpenthermComponent::setRoomControl(RoomControl control)
    {
      if (control == room_control_)
        return;

      static const char *const names[] = {"thermostat", "temporary override", "permanent override"};
      ESP_LOGI(TAG, "Room control: %s -> %s", names[static_cast<int>(room_control_)], names[static_cast<int>(control)]);

      if (control == RoomControl::THERMOSTAT)
      {
        override_setpoint_ = NAN; // the entity shows the thermostat setpoint again
      }
      else
      {
        // Keep the temperature the entity shows (the thermostat's when coming from the thermostat preset)
        if (std::isnan(override_setpoint_))
          override_setpoint_ = !std::isnan(cached_room_setpoint_.value) ? cached_room_setpoint_.value
                                                                        : room_climate_->target_temperature;
        if (control == RoomControl::TEMPORARY)
          override_reference_trset_ = cached_room_setpoint_.value;
      }

      if (room_control_ == RoomControl::THERMOSTAT || control == RoomControl::THERMOSTAT)
        ovr_active_ = false; // start or stop rewriting cleanly

      room_control_ = control;
    }

    float OpenthermComponent::overrideRoomTemperature()
    {
      if (room_climate_external_sensor_ != nullptr && room_climate_external_sensor_->has_state() &&
          !std::isnan(room_climate_external_sensor_->state))
        return room_climate_external_sensor_->state;

      if (!std::isnan(cached_room_temp_.value) && millis() - cached_room_temp_.last_update < ROOM_TEMP_TIMEOUT_)
        return cached_room_temp_.value;

      return NAN;
    }

    void OpenthermComponent::runRoomControl()
    {
      if (!override_enabled_)
        return;

      if (!room_off_ && room_control_ == RoomControl::THERMOSTAT)
      {
        if (ovr_active_)
          ESP_LOGI(TAG, "Room override stopped, thermostat frames pass through unchanged");
        ovr_active_ = false;
        return;
      }

      unsigned long now = millis();

      if (room_off_)
      {
        if (!ovr_active_)
          ESP_LOGI(TAG, "Room override: heating off");
        ovr_active_ = true;
        ovr_demand_ = false;
        ovr_flow_ = NAN;
        last_control_ms_ = now;
        return;
      }

      float room = overrideRoomTemperature();
      if (std::isnan(room) || std::isnan(override_setpoint_))
      {
        if (ovr_active_)
          ESP_LOGW(TAG, "Room override paused: no room temperature, thermostat back in control");
        ovr_active_ = false;
        return;
      }

      float error = override_setpoint_ - room;
      float span = ovr_max_flow_ - ovr_min_flow_;

      if (!ovr_active_)
      {
        // Bumpless start: continue from the flow temperature the thermostat was asking for
        pi_integral_ = 0.0f;
        if (thermostat_ch_enable_ && !std::isnan(thermostat_tset_))
          pi_integral_ = thermostat_tset_ - ovr_min_flow_ - ovr_kp_ * error;
        pi_integral_ = std::max(0.0f, std::min(span, pi_integral_));
        ovr_demand_ = error > 0.0f;
        last_control_ms_ = now;
        ovr_active_ = true;
        ESP_LOGI(TAG, "Room override active: setpoint %.1f°C, room %.1f°C", override_setpoint_, room);
      }

      float dt_min = (now - last_control_ms_) / 60000.0f;
      last_control_ms_ = now;

      // Hysteresis: stop above setpoint + hysteresis, start again once below the setpoint
      if (error <= -ovr_hysteresis_)
        ovr_demand_ = false;
      else if (error > 0.0f)
        ovr_demand_ = true;

      if (!ovr_demand_)
      {
        ovr_flow_ = NAN;
        ESP_LOGD(TAG, "Room override: room %.2f°C, setpoint %.1f°C, no heat demand", room, override_setpoint_);
        return;
      }

      float proportional = ovr_kp_ * error;
      float output = ovr_min_flow_ + proportional + pi_integral_;
      float delta = ovr_ki_ * error * dt_min;
      // Anti-windup: don't integrate further into a saturated output
      bool winding_up = (output >= ovr_max_flow_ && delta > 0.0f) || (output <= ovr_min_flow_ && delta < 0.0f);
      if (!winding_up)
        pi_integral_ = std::max(0.0f, std::min(span, pi_integral_ + delta));

      output = ovr_min_flow_ + proportional + pi_integral_;
      ovr_flow_ = std::max(ovr_min_flow_, std::min(ovr_max_flow_, output));
      ESP_LOGD(TAG, "Room override: room %.2f°C, setpoint %.1f°C, flow %.1f°C (P %.1f, I %.1f)",
               room, override_setpoint_, ovr_flow_, proportional, pi_integral_);
    }

    void OpenthermComponent::noteThermostatFrame(unsigned long request)
    {
      OpenThermMessageType type = ot_->getMessageType(request);
      OpenThermMessageID id = ot_->getDataID(request);

      if (id == OpenThermMessageID::Status && type == OpenThermMessageType::READ_DATA)
      {
        thermostat_ch_enable_ = (request & 0x0100) != 0;
      }
      else if (id == OpenThermMessageID::TSet && type == OpenThermMessageType::WRITE_DATA)
      {
        thermostat_tset_ = ot_->getFloat(request);
      }
      else if (id == OpenThermMessageID::TrSet && type == OpenThermMessageType::WRITE_DATA &&
               room_control_ == RoomControl::TEMPORARY)
      {
        float trset = ot_->getFloat(request);
        if (std::isnan(override_reference_trset_))
          override_reference_trset_ = trset; // first value seen since the override started
        else if (std::fabs(trset - override_reference_trset_) >= 0.05f)
          thermostat_setpoint_changed_ = true; // handled in loop()
      }
    }

    unsigned long OpenthermComponent::rewriteForBoiler(unsigned long request)
    {
      if (!ovr_active_)
        return request;

      OpenThermMessageType type = ot_->getMessageType(request);
      OpenThermMessageID id = ot_->getDataID(request);
      unsigned int data = request & 0xFFFF;

      // Data-ID 0: master status, CH enable is bit 0 of the high byte. DHW and the other bits stay as sent.
      if (id == OpenThermMessageID::Status && type == OpenThermMessageType::READ_DATA)
      {
        unsigned int new_data = ovr_demand_ ? (data | 0x0100) : (data & ~0x0100u);
        return new_data == data ? request : ot_->buildRequest(type, id, new_data);
      }

      // Data-ID 1: CH flow setpoint
      if (id == OpenThermMessageID::TSet && type == OpenThermMessageType::WRITE_DATA)
      {
        float flow = ovr_demand_ && !std::isnan(ovr_flow_) ? ovr_flow_ : OVERRIDE_OFF_FLOW_;
        return ot_->buildRequest(type, id, ot_->temperatureToData(flow));
      }

      return request;
    }

    unsigned long OpenthermComponent::restoreForThermostat(unsigned long request, unsigned long response)
    {
      // Answer the thermostat with its own values, so it doesn't see what the gateway changed
      OpenThermMessageType type = ot_->getMessageType(response);
      OpenThermMessageID id = ot_->getDataID(response);
      unsigned int data = response & 0xFFFF;

      if (id == OpenThermMessageID::Status)
        data = (request & 0xFF00) | (data & 0x00FF); // echo the thermostat's master status, keep the boiler status
      else if (id == OpenThermMessageID::TSet && type == OpenThermMessageType::WRITE_ACK)
        data = request & 0xFFFF;
      else
        return response;

      return slave_ot_->buildResponse(type, id, data);
    }

    void OpenthermComponent::register_climate(OpenthermClimate *climate)
    {
      ClimateType type = climate->get_climate_type();

      if (type == ClimateType::HOT_WATER)
      {
        hot_water_climate_ = climate;
      }
      else if (type == ClimateType::HEATING_WATER)
      {
        heating_water_climate_ = climate;
      }
      else if (type == ClimateType::ROOM)
      {
        room_climate_ = climate;
      }
    }

    float OpenthermComponent::getExternalTemperature()
    {
      return getCachedOrFetch(cached_external_temp_, OpenThermMessageID::Toutside);
    }

    float OpenthermComponent::getHeatingTargetTemperature()
    {
      return getCachedOrFetch(cached_heating_target_, OpenThermMessageID::TSet);
    }

    float OpenthermComponent::getReturnTemperature()
    {
      return getCachedOrFetch(cached_return_temp_, OpenThermMessageID::Tret);
    }

    float OpenthermComponent::getHotWaterTargetTemperature()
    {
      return getCachedOrFetch(cached_dhw_target_, OpenThermMessageID::TdhwSet);
    }

    float OpenthermComponent::getHotWaterTemperature()
    {
      return getCachedOrFetch(cached_dhw_temp_, OpenThermMessageID::Tdhw);
    }

    float OpenthermComponent::getRoomTemperature()
    {
      return cached_room_temp_.value;
    }

    float OpenthermComponent::getRoomSetpoint()
    {
      return cached_room_setpoint_.value;
    }

    bool OpenthermComponent::setTemperatureWithVerification(
        float temperature,
        OpenThermMessageID write_msg_id,
        OpenThermMessageID read_msg_id,
        OpenthermClimate *climate,
        const char *name)
    {
      ESP_LOGI(TAG, "Setting %s temperature to %.1f°C", name, temperature);

      unsigned int data = ot_->temperatureToData(temperature);
      unsigned long request = ot_->buildRequest(OpenThermRequestType::WRITE, write_msg_id, data);
      unsigned long response = ot_->sendRequest(request);

      if (!ot_->isValidResponse(response))
      {
        ESP_LOGE(TAG, "Failed to set %s temperature - invalid response", name);
        return false;
      }

      // Small delay to allow boiler to process the write command
      delay(100);

      // Verify the setpoint was accepted by reading it back (with retry)
      const int max_retries = 3;
      for (int retry = 0; retry < max_retries; retry++)
      {
        unsigned long read_response = ot_->sendRequest(
            ot_->buildRequest(OpenThermRequestType::READ, read_msg_id, 0));

        if (ot_->isValidResponse(read_response))
        {
          float actual_setpoint = ot_->getFloat(read_response);

          if (!std::isnan(actual_setpoint))
          {
            ESP_LOGI(TAG, "%s setpoint verified: %.1f°C (requested: %.1f°C)",
                     name, actual_setpoint, temperature);

            // Update climate entity immediately with verified value
            if (climate != nullptr)
            {
              climate->target_temperature = actual_setpoint;
              climate->publish_state();
            }

            // Check if setpoint was clamped by boiler (e.g., min/max limits)
            if (std::abs(actual_setpoint - temperature) > 1.0f)
            {
              ESP_LOGW(TAG, "%s setpoint was adjusted by boiler from %.1f°C to %.1f°C (min/max limits?)",
                       name, temperature, actual_setpoint);
            }

            return true;
          }
        }

        // Retry with exponential backoff
        if (retry < max_retries - 1)
        {
          unsigned long backoff = 50 * (1 << retry); // 50ms, 100ms, 200ms
          ESP_LOGW(TAG, "Failed to verify %s setpoint, retry %d/%d after %lu ms",
                   name, retry + 1, max_retries, backoff);
          delay(backoff);
        }
      }

      ESP_LOGW(TAG, "%s setpoint write succeeded but verification failed after %d retries", name, max_retries);
      return true; // Write succeeded even if verification failed
    }

    bool OpenthermComponent::setHotWaterTemperature(float temperature)
    {
      return setTemperatureWithVerification(
          temperature,
          OpenThermMessageID::TdhwSet,
          OpenThermMessageID::TdhwSet,
          hot_water_climate_,
          "DHW");
    }

    bool OpenthermComponent::setHeatingTargetTemperature(float temperature)
    {
      return setTemperatureWithVerification(
          temperature,
          OpenThermMessageID::TSet,
          OpenThermMessageID::TSet,
          heating_water_climate_,
          "CH");
    }

    float OpenthermComponent::getModulation()
    {
      return getCachedOrFetch(cached_modulation_, OpenThermMessageID::RelModLevel);
    }

    float OpenthermComponent::getPressure()
    {
      return getCachedOrFetch(cached_pressure_, OpenThermMessageID::CHPressure);
    }

    void OpenthermComponent::processRequest(unsigned long request, OpenThermResponseStatus status)
    {
      if (instance_ == nullptr || instance_->ot_ == nullptr || instance_->slave_ot_ == nullptr)
        return;

      // The library also calls back for partial (INVALID) and stale (TIMEOUT) frames.
      // Forwarding those sends random type/id/data to the boiler, so drop them.
      if (status != OpenThermResponseStatus::SUCCESS)
      {
        instance_->dropped_frames_++;
        ESP_LOGW(TAG, "Dropped thermostat frame 0x%08lX (%s), total dropped: %u",
                 request, instance_->slave_ot_->statusToString(status), static_cast<unsigned>(instance_->dropped_frames_));
        return;
      }

      instance_->noteThermostatFrame(request);

      // During a room override the CH enable bit and the flow setpoint are replaced before they reach the boiler
      unsigned long forwarded = instance_->rewriteForBoiler(request);
      unsigned long response = instance_->ot_->sendRequest(forwarded);
      // 0 = boiler did not answer, bad parity = noise; let the thermostat time out instead of relaying garbage
      if (response != 0 && !instance_->ot_->parity(response))
      {
        unsigned long reply = forwarded != request ? instance_->restoreForThermostat(request, response) : response;
        instance_->slave_ot_->sendResponse(reply);
      }
      instance_->last_relay_ms_ = millis();

      OpenThermMessageID id = instance_->ot_->getDataID(request);
      OpenThermMessageType msg_type = instance_->ot_->getMessageType(request);
      bool valid = instance_->ot_->isValidResponse(response);

      if (forwarded != request)
        ESP_LOGV(TAG, "Relayed 0x%08lX rewritten to 0x%08lX -> 0x%08lX", request, forwarded, response);
      else
        ESP_LOGV(TAG, "Relayed 0x%08lX -> 0x%08lX", request, response);

      if (msg_type == OpenThermMessageType::WRITE_DATA)
        instance_->logThermostatWrite(request);

      // Update status response (critical for binary sensors)
      if (id == OpenThermMessageID::Status && valid)
      {
        last_status_response_ = response;
      }

      // Store response for processing in loop()
      if (valid)
      {
        last_intercepted_response_ = response;
        last_intercepted_id_ = id;
        has_new_intercepted_response_ = true;
      }
      // Also cache WRITE-DATA requests (thermostat setting values)
      else if (msg_type == OpenThermMessageType::WRITE_DATA)
      {
        // For WRITE requests, cache the REQUEST data (what thermostat wants to set)
        last_intercepted_response_ = request; // Use request, not response
        last_intercepted_id_ = id;
        has_new_intercepted_response_ = true;
      }
    }

    void OpenthermComponent::logThermostatWrite(unsigned long request)
    {
      uint8_t id = (request >> 16) & 0xFF;
      uint16_t data = request & 0xFFFF;
      auto it = thermostat_writes_.find(id);
      if (it == thermostat_writes_.end())
      {
        // Expected once per data-id after boot; later, a new id points to a corrupted frame
        ESP_LOGW(TAG, "Thermostat WRITE to new data-id %u: 0x%04X", id, data);
        thermostat_writes_[id] = data;
      }
      else if (it->second != data)
      {
        ESP_LOGD(TAG, "Thermostat WRITE data-id %u: 0x%04X -> 0x%04X", id, it->second, data);
        it->second = data;
      }
    }

    void OpenthermComponent::processCachedResponse(unsigned long response, OpenThermMessageID id)
    {
      // This runs in loop(), not interrupt context - safe to do complex operations
      unsigned long now = millis();
      const CachedMessageSpec *spec = cachedMessageSpec(id);

      if (spec != nullptr)
      {
        CachedValue &cache = this->*(spec->cache);
        cache.value = decodeMessageValue(response, id);
        cache.last_update = now;
        ESP_LOGV(TAG, "Cached %s (msg_id %d): %.2f", spec->log_name, static_cast<int>(id), cache.value);
        return;
      }

      if (id == OpenThermMessageID::Status)
      {
        // Already handled in processRequest for immediate binary sensor updates
        ESP_LOGD(TAG, "Updated status response: %lu", response);
      }
    }

    float OpenthermComponent::decodeMessageValue(unsigned long response, OpenThermMessageID id)
    {
      switch (valueEncodingForMessage(id))
      {
        case ValueEncoding::S16:
          return static_cast<float>(static_cast<int16_t>(ot_->getUInt(response)));

        case ValueEncoding::U16:
        case ValueEncoding::RAW:
          return static_cast<float>(ot_->getUInt(response));

        case ValueEncoding::F88:
        default:
          return ot_->getFloat(response);
      }
    }

    OpenthermComponent::ValueEncoding OpenthermComponent::valueEncodingForMessage(OpenThermMessageID id) const
    {
      switch (static_cast<uint8_t>(id))
      {
        case 0:   // Status: flag8/flag8
        case 2:   // MConfigMMemberIDcode: flag8/u8
        case 3:   // SConfigSMemberIDcode: flag8/u8
        case 4:   // RemoteRequest: u8/u8
        case 5:   // ASFflags: flag8/u8
        case 6:   // RBPflags: flag8/flag8
        case 10:  // TSP: u8/u8
        case 11:  // TSPindexTSPvalue: u8/u8
        case 12:  // FHBsize: u8/u8
        case 13:  // FHBindexFHBvalue: u8/u8
        case 15:  // MaxCapacityMinModLevel: u8/u8
        case 20:  // DayTime: special/u8
        case 21:  // Date: u8/u8
        case 35:  // BoilerFanSpeedSetpointAndActual: u8/u8
        case 48:  // TdhwSetUBTdhwSetLB: s8/s8
        case 49:  // MaxTSetUBMaxTSetLB: s8/s8
        case 70:  // StatusVentilationHeatRecovery: flag8/flag8
        case 72:  // ASFflagsOEMfaultCodeVentilationHeatRecovery: flag8/u8
        case 74:  // SConfigSMemberIDCodeVentilationHeatRecovery: flag8/u8
        case 76:  // VentilationHeatRecoveryVersion: u8/u8
        case 86:  // RBPflagsVentilationHeatRecovery: flag8/flag8
        case 87:  // NominalVentilationValue: u8/-
        case 88:  // TSPventilationHeatRecovery: u8/u8
        case 89:  // TSPindexTSPvalueVentilationHeatRecovery: u8/u8
        case 90:  // FHBsizeVentilationHeatRecovery: u8/u8
        case 91:  // FHBindexFHBvalueVentilationHeatRecovery: u8/u8
        case 93:  // Brand: u8/u8
        case 94:  // BrandVersion: u8/u8
        case 95:  // BrandSerialNumber: u8/u8
        case 98:  // RFsensorStatusInformation: special/special
        case 99:  // RemoteOverrideOperatingModeHeatingDHW: special/special
        case 100: // RemoteOverrideFunction: flag8/-
        case 101: // StatusSolarStorage: flag8/flag8
        case 102: // ASFflagsOEMfaultCodeSolarStorage: flag8/u8
        case 103: // SConfigSMemberIDcodeSolarStorage: flag8/u8
        case 104: // SolarStorageVersion: u8/u8
        case 105: // TSPSolarStorage: u8/u8
        case 106: // TSPindexTSPvalueSolarStorage: u8/u8
        case 107: // FHBsizeSolarStorage: u8/u8
        case 108: // FHBindexFHBvalueSolarStorage: u8/u8
        case 126: // MasterVersion: u8/u8
        case 127: // SlaveVersion: u8/u8
          return ValueEncoding::RAW;

        case 33: // Texhaust: s16
          return ValueEncoding::S16;

        case 71: // Vset: -/u8
        case 77: // RelVentLevel: -/u8
        case 78: // RHexhaust: -/u8
        case 22:  // Year
        case 73:  // OEMDiagnosticCodeVentilationHeatRecovery
        case 79:  // CO2exhaust
        case 84:  // RPMexhaust
        case 85:  // RPMsupply
        case 96:  // CoolingOperationHours
        case 97:  // PowerCycles
        case 109: // ElectricityProducerStarts
        case 110: // ElectricityProducerHours
        case 111: // ElectricityProduction
        case 112: // CumulativElectricityProduction
        case 113: // UnsuccessfulBurnerStarts
        case 114: // FlameSignalTooLowNumber
        case 115: // OEMDiagnosticCode
        case 116: // SuccessfulBurnerStarts
        case 117: // CHPumpStarts
        case 118: // DHWPumpValveStarts
        case 119: // DHWBurnerStarts
        case 120: // BurnerOperationHours
        case 121: // CHPumpOperationHours
        case 122: // DHWPumpValveOperationHours
        case 123: // DHWBurnerOperationHours
          return ValueEncoding::U16;

        default:
          return ValueEncoding::F88;
      }
    }

    const OpenthermComponent::CachedMessageSpec *OpenthermComponent::cachedMessageSpec(OpenThermMessageID id) const
    {
      static const CachedMessageSpec specs[] = {
          {OpenThermMessageID::Toutside, &OpenthermComponent::cached_external_temp_, "external temp"},
          {OpenThermMessageID::Tret, &OpenthermComponent::cached_return_temp_, "return temp"},
          {OpenThermMessageID::Tboiler, &OpenthermComponent::cached_boiler_temp_, "boiler temp"},
          {OpenThermMessageID::Texhaust, &OpenthermComponent::cached_exhaust_temp_, "exhaust temp"},
          {OpenThermMessageID::CHPressure, &OpenthermComponent::cached_pressure_, "pressure"},
          {OpenThermMessageID::RelModLevel, &OpenthermComponent::cached_modulation_, "modulation"},
          {OpenThermMessageID::TSet, &OpenthermComponent::cached_heating_target_, "heating target"},
          {OpenThermMessageID::Tdhw, &OpenthermComponent::cached_dhw_temp_, "DHW temp"},
          {OpenThermMessageID::TdhwSet, &OpenthermComponent::cached_dhw_target_, "DHW target"},
          {OpenThermMessageID::Tr, &OpenthermComponent::cached_room_temp_, "room temp"},
          {OpenThermMessageID::TrSet, &OpenthermComponent::cached_room_setpoint_, "room setpoint"},
      };

      for (const auto &spec : specs)
      {
        if (spec.id == id)
          return &spec;
      }

      return nullptr;
    }

    float OpenthermComponent::getCachedOrFetch(CachedValue &cache, OpenThermMessageID msg_id)
    {
      // Unsigned arithmetic handles millis() overflow correctly (wraps at 2^32)
      unsigned long age = millis() - cache.last_update;
      bool never_read = cache.last_update == 0;
      bool fresh = !never_read && !std::isnan(cache.value) && age < CACHE_TIMEOUT_;
      // Rate limit: a failed read also sets last_update, so unsupported IDs aren't retried every update
      bool recently_tried = !never_read && age < MIN_FETCH_INTERVAL_;

      if (!fresh && !recently_tried)
      {
        bool queued = false;
        for (const auto &pending : pending_fetches_)
          queued |= pending.cache == &cache;
        if (!queued)
          pending_fetches_.push_back({&cache, msg_id});
      }

      return cache.value; // Fresh, stale or NAN; a queued read updates it for the next update()
    }

    void OpenthermComponent::fetchPendingValue()
    {
      if (pending_fetches_.empty() || !slave_ot_->isReady())
        return;

      // The thermostat sends about one frame per second. Reading right after a relayed frame keeps our
      // blocking request out of the way of the next one; with a silent thermostat, read anyway.
      unsigned long since_relay = millis() - last_relay_ms_;
      if (since_relay > FETCH_WINDOW_ && since_relay < THERMOSTAT_SILENT_)
        return;

      // Drop entries an intercepted thermostat exchange already refreshed since they were queued
      while (!pending_fetches_.empty())
      {
        const CachedValue *cache = pending_fetches_.front().cache;
        if (std::isnan(cache->value) || millis() - cache->last_update >= CACHE_TIMEOUT_)
          break;
        pending_fetches_.erase(pending_fetches_.begin());
      }
      if (pending_fetches_.empty())
        return;

      PendingFetch fetch = pending_fetches_.front();
      pending_fetches_.erase(pending_fetches_.begin());

      unsigned long response = ot_->sendRequest(ot_->buildRequest(OpenThermRequestType::READ, fetch.id, 0));
      fetch.cache->last_update = millis(); // Also on failure, to rate limit retries

      if (ot_->isValidResponse(response))
      {
        fetch.cache->value = decodeMessageValue(response, fetch.id);
        ESP_LOGV(TAG, "Fetched msg_id %d: %.2f", static_cast<int>(fetch.id), fetch.cache->value);
      }
      else
      {
        ESP_LOGW(TAG, "Failed to fetch value for msg_id %d (response 0x%08lX), using stale cache if available",
                 static_cast<int>(fetch.id), response);
      }

      // Forward a thermostat frame that arrived while we were blocked
      slave_ot_->process();
    }

    float OpenthermComponent::readGenericSensorValue(GenericReadSensor &read_sensor)
    {
      if (read_sensor.id == OpenThermMessageID::Status)
      {
        if (last_status_response_ == 0)
          return NAN;

        read_sensor.cache.value = decodeMessageValue(last_status_response_, read_sensor.id);
        read_sensor.cache.last_update = millis();
        return read_sensor.cache.value;
      }

      return getCachedOrFetch(read_sensor.cache, read_sensor.id);
    }

    void IRAM_ATTR OpenthermComponent::handleInterrupt()
    {
      if (instance_ != nullptr && instance_->ot_ != nullptr)
      {
        instance_->ot_->handleInterrupt();
      }
    }

    void IRAM_ATTR OpenthermComponent::slaveHandleInterrupt()
    {
      if (instance_ != nullptr && instance_->slave_ot_ != nullptr)
      {
        instance_->slave_ot_->handleInterrupt();
      }
    }

    bool OpenthermComponent::sendBoilerReset()
    {
      ESP_LOGW(TAG, "Sending Boiler Lock-Out Reset (BLOR) command");

      // Build WRITE-DATA command with Command-Code 1 (BLOR) as per OpenTherm spec section 5.3.3
      unsigned long request = ot_->buildRequest(
          OpenThermRequestType::WRITE,
          OpenThermMessageID::Command,  // Data ID 4
          0x0100);                      // HB=1 (BLOR command), LB=0

      ESP_LOGD(TAG, "BLOR request: 0x%08lX", request);
      unsigned long response = ot_->sendRequest(request);
      ESP_LOGD(TAG, "BLOR response: 0x%08lX", response);

      if (ot_->isValidResponse(response))
      {
        OpenThermMessageType resp_type = ot_->getMessageType(response);

        // Extract full response data
        uint16_t response_data = response & 0xFFFF;
        uint8_t high_byte = (response_data >> 8) & 0xFF;
        uint8_t low_byte = response_data & 0xFF;

        ESP_LOGD(TAG, "BLOR response data: HB=0x%02X (%d), LB=0x%02X (%d), type=%d",
                 high_byte, high_byte, low_byte, low_byte, static_cast<int>(resp_type));

        if (resp_type == OpenThermMessageType::WRITE_ACK)
        {
          ESP_LOGI(TAG, "Boiler reset command acknowledged (WRITE-ACK)");
          return true;
        }
        else
        {
          ESP_LOGW(TAG, "BLOR unexpected response type: %d", static_cast<int>(resp_type));
          return false;
        }
      }

      ESP_LOGE(TAG, "Boiler reset command - no valid response");
      return false;
    }

  } // namespace opentherm
} // namespace esphome
