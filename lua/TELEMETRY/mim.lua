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

local tel = {}

tel.is_connected = 0
tel.is_engaging = 0
tel.is_target_ready = 0
tel.is_ceptor_ready = 0
tel.is_location_ready = 0
tel.accel_lat = 0
tel.accel_ver = 0
tel.dist_hor = 0
tel.dist_ver = 0
tel.time_to_go = 0
tel.zero_effort_miss = 0

local function doGarbageCollect()
  collectgarbage()
  collectgarbage()
end

local function processTelemetry(appid, value)
  if appid == APPID_STATUS then
    tel.is_connected = bit32.band(value, 1)
    tel.is_engaging = bit32.band(bit32.rshift(value, 1), 1)
    tel.is_target_ready = bit32.band(bit32.rshift(value, 2), 1)
    tel.is_ceptor_ready = bit32.band(bit32.rshift(value, 3), 1)
    tel.is_location_ready = bit32.band(bit32.rshift(value, 4), 1)
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
  end
end

local function crossfirePop()
  local command, data = crossfireTelemetryPop()
  -- command is 0x80 CRSF_FRAMETYPE_ARDUPILOT
  if
    (
      command == CRSF_FRAME_CUSTOM_TELEM
      or command == CRSF_FRAME_CUSTOM_TELEM_LEGACY
    ) and data ~= nil
  then
    -- actual payload starts at data[2]
    if #data >= 7 and data[1] == CRSF_PAYLOAD_ARDUPILOT_SUBTYPE_SINGLE then
      local appid = bit32.lshift(data[3], 8) + data[2]
      local value = bit32.lshift(data[7], 24)
        + bit32.lshift(data[6], 16)
        + bit32.lshift(data[5], 8)
        + data[4]
      processTelemetry(appid, value)
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
        processTelemetry(appid, value)
      end
    end
  end
end

-- init is called once when model is loaded
local function mim_init()
  doGarbageCollect()
end

local function mim_background()
  pcall(crossfirePop)

  doGarbageCollect()
end

-- run is called periodically only when screen is visible
local function mim_run(event)
  mim_background()
  lcd.clear()

  lcd.drawText(1, 1, "Man in the Middle", SMLSIZE)
  local text = ""

  -- left column

  text = string.format("skymap: %s", tel.is_connected ~= 0 and "+" or "-")
  lcd.drawText(1, 11, text, SMLSIZE)

  text = string.format("engaging: %s", tel.is_engaging ~= 0 and "+" or "-")
  lcd.drawText(1, 20, text, SMLSIZE)

  text = string.format("target: %s", tel.is_target_ready ~= 0 and "+" or "-")
  lcd.drawText(1, 29, text, SMLSIZE)

  text = string.format("i-ceptor: %s", tel.is_ceptor_ready ~= 0 and "+" or "-")
  lcd.drawText(1, 38, text, SMLSIZE)

  text = string.format("i-loc: %s", tel.is_location_ready ~= 0 and "+" or "-")
  lcd.drawText(1, 47, text, SMLSIZE)

  -- right column

  text = string.format("acc lat: %.01f", tel.accel_lat / 1000)
  lcd.drawText(55, 11, text, SMLSIZE)

  text = string.format("acc ver: %.01f", tel.accel_ver / 1000)
  lcd.drawText(55, 20, text, SMLSIZE)

  text = string.format("dist hor: %d", tel.dist_hor)
  lcd.drawText(55, 29, text, SMLSIZE)

  text = string.format("dist ver: %d", tel.dist_ver)
  lcd.drawText(55, 38, text, SMLSIZE)

  text = string.format("ttgo: %d", tel.time_to_go)
  lcd.drawText(55, 47, text, SMLSIZE)

  text = string.format("zem: %d", tel.zero_effort_miss)
  lcd.drawText(55, 56, text, SMLSIZE)

  return 0
end

return {
  run = mim_run,
  background = mim_background,
  init = mim_init,
}
