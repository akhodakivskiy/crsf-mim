local CRSF_FRAME_CUSTOM_TELEM = 0x80
local CRSF_FRAME_CUSTOM_TELEM_LEGACY = 0x7F
local CRSF_PAYLOAD_ARDUPILOT_SUBTYPE_SINGLE = 0xF0
local CRSF_PAYLOAD_ARDUPILOT_SUBTYPE_MULTI = 0xF2
local CRSF_PAYLOAD_ARDUPILOT_SUBTYPE_TEXT = 0xF1

local APPID_STATUS = 0x6000
local APPID_ACCEL_LAT = 0x6001
local APPID_ACCEL_VER = 0x6002
local APPID_DIST_HOR = 0x6003
local APPID_DIST_VER = 0x6004
local APPID_ZEM = 0x6005
local APPID_TTGO = 0x6006
local APPID_ADDR_MIM = 0x6007
local APPID_ADDR_SKYMAP = 0x6008

local function doGarbageCollect()
  collectgarbage()
  collectgarbage()
end

local function telemInit()
  return {
    is_connected = 0,
    is_engaging = 0,
    is_target_ready = 0,
    is_ceptor_ready = 0,
    is_location_ready = 0,
    command_type = 0,
    accel_lat = 0,
    accel_ver = 0,
    dist_hor = 0,
    dist_ver = 0,
    time_to_go = 0,
    zero_effort_miss = 0,
    addr_mim = "0.0.0.0",
    addr_skymap = "0.0.0.0",
  }
end

local function formatIpAddress(value)
  return string.format(
    "%d.%d.%d.%d",
    bit32.band(bit32.rshift(value, 24), 256),
    bit32.band(bit32.rshift(value, 16), 256),
    bit32.band(bit32.rshift(value, 8), 256),
    bit32.band(value, 24, 256)
  )
end

local function telemProcess(tel, appid, value)
  if appid == APPID_STATUS then
    tel.is_connected = bit32.band(value, 1)
    tel.is_engaging = bit32.band(bit32.rshift(value, 1), 1)
    tel.is_target_ready = bit32.band(bit32.rshift(value, 2), 1)
    tel.is_ceptor_ready = bit32.band(bit32.rshift(value, 3), 1)
    tel.is_location_ready = bit32.band(bit32.rshift(value, 4), 1)
    tel.command_type = bit32.band(bit32.rshift(value, 5), 3)
    setTelemetryValue(APPID_STATUS, 0, 0, tel.is_connected, 0, 0, "_con")
    setTelemetryValue(APPID_STATUS, 1, 0, tel.is_engaging, 0, 0, "_eng")
    setTelemetryValue(APPID_STATUS, 2, 0, tel.is_target_ready, 0, 0, "_tgt")
    setTelemetryValue(APPID_STATUS, 3, 0, tel.is_ceptor_ready, 0, 0, "_int")
    setTelemetryValue(APPID_STATUS, 4, 0, tel.is_location_ready, 0, 0, "_loc")
  elseif appid == APPID_ACCEL_LAT then
    tel.accel_lat = value
    setTelemetryValue(APPID_ACCEL_LAT, 0, 0, value / 98.1, 19, 2, "_acl")
  elseif appid == APPID_ACCEL_VER then
    tel.accel_ver = value
    setTelemetryValue(APPID_ACCEL_VER, 0, 0, value / 98.1, 19, 2, "_acv")
  elseif appid == APPID_DIST_HOR then
    tel.dist_hor = value
    setTelemetryValue(APPID_DIST_HOR, 0, 0, value, 9, 0, "_dh")
  elseif appid == APPID_DIST_VER then
    tel.dist_ver = value
    setTelemetryValue(APPID_DIST_VER, 0, 0, value, 9, 0, "_dv")
  elseif appid == APPID_ZEM then
    tel.zero_effort_miss = value
    setTelemetryValue(APPID_ZEM, 0, 0, value, 9, 0, "_zem")
  elseif appid == APPID_TTGO then
    tel.time_to_go = value
    setTelemetryValue(APPID_TTGO, 0, 0, value, 37, 0, "_tgo")
  elseif appid == APPID_ADDR_MIM then
    tel.addr_mim = formatIpAddress(value)
  elseif appid == APPID_ADDR_SKYMAP then
    tel.addr_skymap = formatIpAddress(value)
  end
end

local function crossfirePop(tel)
  local command, data = crossfireTelemetryPop()

  if command == nil and data == nil then
    return false
  end

  if (command == CRSF_FRAME_CUSTOM_TELEM or command == CRSF_FRAME_CUSTOM_TELEM_LEGACY) and data ~= nil then
    -- actual payload starts at data[2]
    if #data >= 7 and data[1] == CRSF_PAYLOAD_ARDUPILOT_SUBTYPE_SINGLE then
      local appid = bit32.lshift(data[3], 8) + data[2]
      local value = bit32.lshift(data[7], 24) + bit32.lshift(data[6], 16) + bit32.lshift(data[5], 8) + data[4]
      telemProcess(tel, appid, value)
    elseif #data >= 8 and data[1] == CRSF_PAYLOAD_ARDUPILOT_SUBTYPE_MULTI then
      -- passthrough array
      local appid, value
      local size = data[2]
      for i = 0, size - 1 do
        appid = bit32.lshift(data[4 + (6 * i)], 8) + data[3 + (6 * i)]
        value = bit32.lshift(data[8 + (6 * i)], 24)
          + bit32.lshift(data[7 + (6 * i)], 16)
          + bit32.lshift(data[6 + (6 * i)], 8)
          + data[5 + (6 * i)]
        telemProcess(tel, appid, value)
      end
    end
  end

  return true
end

local function drawWidget(widget)
  local tel = widget.tel

  lcd.setColor(CUSTOM_COLOR, BLACK)
  lcd.clear(CUSTOM_COLOR)

  lcd.setColor(CUSTOM_COLOR, WHITE)

  local text = "---------------------"
  local w, h = lcd.sizeText(text, 0 + SMLSIZE)

  text = "Man in the Middle"
  lcd.drawText(1, 0, text, 0 + CUSTOM_COLOR + SMLSIZE)

  text = string.format("skymap: %s", tel.is_connected ~= 0 and "+" or "-")
  lcd.drawText(1, h * 1, text, 0 + CUSTOM_COLOR + SMLSIZE)

  text = string.format("engaging: %s", tel.is_engaging ~= 0 and "+" or "-")
  lcd.drawText(1, h * 2, text, 0 + CUSTOM_COLOR + SMLSIZE)

  text = string.format("target: %s", tel.is_target_ready ~= 0 and "+" or "-")
  lcd.drawText(1, h * 3, text, 0 + CUSTOM_COLOR + SMLSIZE)

  text = string.format("i-ceptor: %s", tel.is_ceptor_ready ~= 0 and "+" or "-")
  lcd.drawText(1, h * 4, text, 0 + CUSTOM_COLOR + SMLSIZE)

  text = string.format("i-loc: %s", tel.is_location_ready ~= 0 and "+" or "-")
  lcd.drawText(1, h * 5, text, 0 + CUSTOM_COLOR + SMLSIZE)

  cmd_str = "none"
  if tel.command_type == 1 then
    cmd_str = "pursuit"
  elseif tel.command_type == 2 then
    cmd_str = "pronav"
  end
  text = string.format("cmd: %s", cmd_str)
  lcd.drawText(1, h * 6, text, 0 + CUSTOM_COLOR + SMLSIZE)

  -- right column

  text = string.format("acc lat: %.01f", tel.accel_lat / 1000)
  lcd.drawText(w, h * 1, text, 0 + CUSTOM_COLOR + SMLSIZE)

  text = string.format("acc ver: %.01f", tel.accel_ver / 1000)
  lcd.drawText(w, h * 2, text, 0 + CUSTOM_COLOR + SMLSIZE)

  text = string.format("dist hor: %d", tel.dist_hor)
  lcd.drawText(w, h * 3, text, 0 + CUSTOM_COLOR + SMLSIZE)

  text = string.format("dist ver: %d", tel.dist_ver)
  lcd.drawText(w, h * 4, text, 0 + CUSTOM_COLOR + SMLSIZE)

  text = string.format("ttgo: %d", tel.time_to_go)
  lcd.drawText(w, h * 5, text, 0 + CUSTOM_COLOR + SMLSIZE)

  text = string.format("zem: %d", tel.zero_effort_miss)
  lcd.drawText(w, h * 6, text, 0 + CUSTOM_COLOR + SMLSIZE)

  if widget.message ~= nil then
    lcd.drawText(10, h * 7, widget.message, 0 + CUSTOM_COLOR + SMLSIZE)
  end
end

local function telemetryPop(widget, count)
  local success = true
  local result = nil
  local i = 0

  while success and count > i do
    success, result = pcall(crossfirePop, widget.tel)

    if not success then
      widget.message = result
      break
    end

    if not result then
      break
    end

    if not result then
      break
    end

    i = i + 1
  end
end

local mim_options = {}

local function mim_create(zone, options)
  return {
    zone = zone,
    options = options,
    message = nil,

    tel = telemInit(),
  }
end

local function mim_update(widget, options)
  widget.options = options
end

local function mim_background(widget)
  telemetryPop(widget, 10)
  doGarbageCollect()
end

local function mim_refresh(widget, event, touchState)
  telemetryPop(widget, 10)

  drawWidget(widget)
  doGarbageCollect()
end

return {
  name = "mim",
  options = mim_options,
  create = mim_create,
  update = mim_update,
  background = mim_background,
  refresh = mim_refresh,
}
