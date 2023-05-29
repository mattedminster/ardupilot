port = serial:find_serial(0)
port:begin(115200)
port:set_flow_control(0)
-- gcs:send_text(0, "OKOKOKOKOKOKOKOKOKO")

local function receive_byte()
  if port:available() > 0 then
    local byte = port:read()
    local value = string.byte(byte)  -- Get the decimal value of the byte

    -- Perform decoding logic based on the received byte value
    local reload_state = (value & 0x04) ~= 0
    local trigger_state = (value & 0x02) ~= 0
    local top_state = (value & 0x01) ~= 0

    -- Print button states
    -- gcs:send_text(6, "Reload State: " .. (reload_state and "Pressed" or "Released"))
    -- gcs:send_text(6, "Trigger State: " .. (trigger_state and "Pressed" or "Released"))
    -- gcs:send_text(6, "Top State: " .. (top_state and "Pressed" or "Released"))

    -- Concatenate button states into a string
    local button_states = tostring("1" .. (reload_state and "1" or "0") .. (trigger_state and "1" or "0") .. (top_state and "1" or "0"))

    local msg_type = 33 --message type this is specfic to renegade labs

    -- Send button states to GCS as a named float value
    gcs:send_named_float(button_states, msg_type)
  end

  return receive_byte, 100
end

return receive_byte, 100
