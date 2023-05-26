-- Lua script to write and read from a serial

local port = serial:find_serial(4)

port:begin(115200)
port:set_flow_control(0)

local step = 65

function spit ()
  gcs:send_text(0, "spit")
  if port:available() > 0 then
    read = port:read()
    gcs:send_text(0, read .. " = " .. step)
  end
  if step > 122 then
    step = 65
  else
    step = step + 1
  end
  port:write(step)
  return spit, 1000
end

return spit, 1000