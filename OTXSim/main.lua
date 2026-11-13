--[[
    OTXSIM model simulator - EdgeTX colour-screen widget edition
    Original OpenTX telemetry script Copyright (C) 2024  Mike Shellim
    Port to EdgeTX colour/touch radios (RadioMaster TX15, EdgeTX 3.0)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details. A copy of the GNU General
    Public License is available at <https://www.gnu.org/licenses/>

    Original instructions: https://rc-soar.com/opentx/lua/otxsim/index.htm

INSTALL
    Copy this file to  /WIDGETS/OTXSim/main.lua  on the radio SD card / flash.
    On the main view: long-press ENTER (or tap) a zone -> Setup widgets ->
    pick "OTXSim". A large zone (or a 1x1 full-screen layout) works best.
    To use the controls, select the widget and choose "Full screen".

USAGE (full screen)
    Page 1 (Model):  live 3D aircraft with labelled surface deflections
                     (100% output = 30 degrees), timers, flight mode,
                     logical switches, GV1-GV9.
        touch : [3/4] [Rear] [Side] [Top] select the camera view,
                [Tags on/off] shows or hides the surface labels,
                tap a GV to select it, use [-] [+] to change it
        keys  : wheel/+- select GV, ENTER = edit, wheel/+- = change,
                EXIT = stop editing / leave full screen, PAGE = page 2
    Page 2 (Assign): channel -> surface assignment editor.
        touch : tap Surface / Side / Reverse cells to cycle them,
                [^] [v] buttons (or swipe) scroll the list,
                [Auto] re-runs auto detection and forgets saved changes
        Side "Both" = one channel drives both wings (Y-lead / single servo).
        Changes are saved per model in /WIDGETS/OTXSim/<model>.cfg

AUTO DETECTION
    Surface type comes from the output (channel) name, then mixer line names,
    then the mixer source (Ail / Ele / Rud stick or input).
    Side comes from R/L, Right/Left in those names. Where it can't be told,
    paired channels are split right/left, and a lone channel is set to Both.
        keys  : as the original script (ENTER, EXIT, wheel/+-), PAGE = page 1
    Long-press RTN always leaves full screen.
    When not full screen, the camera view is set by the widget's "View"
    option: 1 = 3/4, 2 = Rear, 3 = Side, 4 = Top.
    Widget option "Labels" turns the surface tags on or off.

HISTORY
         13/11/2026    Ported to EdgeTX colour widget (TX15 480x320),
                       touch support, 16 channels, improved L/R detection
                       Detection from mixer names/sources, 'Both' side,
                       saved per-model assignments
                       3D shaded aircraft with 3/4, Rear, Side, Top views
    v1.3 29/12/2021 MS Fixed drawing of verticals (e.g. rudder) if inverted output
    v1.2 30/04/2020 MS Added timers ...
    v0.9 12/04/2020 MS 1st release
--]]

-- ===========================================================================
-- Constants
-- ===========================================================================
local N_CHANS = 16
local N_GVARS = 9
local N_LS    = 64
local N_TMR   = 3

local FLD_CHAN_NUM     = 0
local FLD_SURFACE_TYPE = 1
local FLD_LR           = 2
local FLD_ROT          = 3

local LEFT, RIGHT, BOTH = -1, 1, 0   -- BOTH = one channel drives both wings
local ROT_NORMAL  = 1

-- Model dimensions, arbitrary 'model' units
local SPAN_LEN = 3.6
local ELE_LEN  = 1
local RUD_LEN  = 0.6
local MAX_DEFL = 0.22

local modWidth   = SPAN_LEN
local modHeight  = RUD_LEN + 2 * MAX_DEFL
local modYOffset = RUD_LEN / 2

-- Text box alignment: bits 0-1 : 0=centre, 1=right, 2=left ; bit 2: 0=top, 1=bottom
local TOP_CNTR, TOP_RT, TOP_LT = 0x00, 0x01, 0x02
local BOT_CNTR, BOT_RT, BOT_LT = 0x04, 0x05, 0x06

local SURF_OTHER = 7

-- Surface type template (copied per widget instance, as wing x0/x1 are mutated)
local function newSurfTypes()
  return {
    [1] = {Name='Flap', draw=true,  hasLR=true,  x0=0, y0=0, x1=0, y1=0, dx=0, dy=MAX_DEFL, txtPos=0.5, txtAlign=BOT_CNTR},
    [2] = {Name='Ail',  draw=true,  hasLR=true,  x0=0, y0=0, x1=0, y1=0, dx=0, dy=MAX_DEFL, txtPos=0.5, txtAlign=BOT_CNTR},
    [3] = {Name='Tip',  draw=true,  hasLR=true,  x0=0, y0=0, x1=0, y1=0, dx=0, dy=MAX_DEFL, txtPos=0.5, txtAlign=BOT_CNTR},
    [4] = {Name='Ele',  draw=true,  hasLR=false, x0=-ELE_LEN/2, y0=RUD_LEN, x1=ELE_LEN/2, y1=RUD_LEN, dx=0, dy=MAX_DEFL, txtPos=0.9, txtAlign=TOP_LT},
    [5] = {Name='Rud',  draw=true,  hasLR=false, x0=0, y0=0, x1=0, y1=RUD_LEN, dx=MAX_DEFL, dy=0, txtPos=0.65, txtAlign=TOP_RT},
    [6] = {Name='Vee',  draw=true,  hasLR=true,  x0=0, y0=0, x1=2*RUD_LEN/4, y1=RUD_LEN, dx=-2*MAX_DEFL/3, dy=MAX_DEFL/3, txtPos=1, txtAlign=TOP_LT},
    [7] = {Name='---',  draw=false, hasLR=false},
  }
end

-- ===========================================================================
-- Colours & drawing helpers
-- ===========================================================================
local C = nil
local function initColours()
  if C then return end
  C = {
    bg      = lcd.RGB(18, 20, 25),
    lblBg   = lcd.RGB(40, 44, 52),
    lblEdge = lcd.RGB(70, 76, 90),
    lblVal  = lcd.RGB(80, 165, 255),
    btnBg   = lcd.RGB(30, 34, 42),
    grid    = lcd.RGB(34, 39, 48),
    blade   = lcd.RGB(55, 62, 78),
    panel   = lcd.RGB(34, 44, 66),
    panelHi = lcd.RGB(52, 66, 96),
    text    = lcd.RGB(235, 240, 250),
    dim     = lcd.RGB(140, 150, 172),
    base    = lcd.RGB(120, 130, 150),
    surfPos = lcd.RGB(90, 225, 130),
    surfNeg = lcd.RGB(255, 160, 70),
    surfZero= lcd.RGB(200, 210, 225),
    lsOn    = lcd.RGB(90, 225, 130),
    lsOff   = lcd.RGB(110, 120, 140),
    accent  = lcd.RGB(60, 140, 255),
    edit    = lcd.RGB(255, 200, 0),
    editTxt = lcd.RGB(20, 20, 20),
  }
end

-- Colours are lcd.RGB() values, which EdgeTX accepts directly as colour flags

local floor = math.floor
local function rnd(v) return floor(v + 0.5) end

local function textSize(s, flags)
  if lcd.sizeText then
    return lcd.sizeText(s, flags)
  end
  local h = (flags == SMLSIZE) and 12 or 16
  return #s * floor(h * 0.6), h
end

local function txt(x, y, s, flags, c)
  lcd.drawText(x, y, s, (flags or 0) + (c or C.text))
end

local function fillRect(x, y, w, h, c)
  lcd.drawFilledRectangle(x, y, w, h, c)
end

local function rect(x, y, w, h, c)
  lcd.drawRectangle(x, y, w, h, c)
end


-- Draw a button and register it as a touch target
local function button(w, x, y, bw, bh, label, action, active)
  fillRect(x, y, bw, bh, active and C.accent or C.panelHi)
  local tw, th = textSize(label, 0)
  txt(x + (bw - tw) / 2, y + (bh - th) / 2, label, 0, C.text)
  w.hits[#w.hits + 1] = {x = x, y = y, w = bw, h = bh, fn = action}
end

local function addHit(w, x, y, bw, bh, fn)
  w.hits[#w.hits + 1] = {x = x, y = y, w = bw, h = bh, fn = fn}
end

-- ===========================================================================
-- Event helpers (nil-safe: constants missing on a build simply never match)
-- ===========================================================================
local function isEvt(event, ...)
  if not event or event == 0 then return false end
  for i = 1, select('#', ...) do
    local e = select(i, ...)
    if e ~= nil and event == e then return true end
  end
  return false
end

local function evtInc(event)
  if isEvt(event, EVT_VIRTUAL_NEXT, EVT_VIRTUAL_NEXT_REPT, EVT_VIRTUAL_INC, EVT_VIRTUAL_INC_REPT,
                  EVT_PLUS_BREAK, EVT_PLUS_REPT, EVT_ROT_RIGHT) then return 1 end
  if isEvt(event, EVT_VIRTUAL_PREV, EVT_VIRTUAL_PREV_REPT, EVT_VIRTUAL_DEC, EVT_VIRTUAL_DEC_REPT,
                  EVT_MINUS_BREAK, EVT_MINUS_REPT, EVT_ROT_LEFT) then return -1 end
  return 0
end

local function evtEnter(e) return isEvt(e, EVT_VIRTUAL_ENTER, EVT_ENTER_BREAK) end
local function evtExit(e)  return isEvt(e, EVT_VIRTUAL_EXIT, EVT_EXIT_BREAK) end
local function evtPage(e)
  return isEvt(e, EVT_VIRTUAL_NEXT_PAGE, EVT_VIRTUAL_PREV_PAGE, EVT_PAGEDN_FIRST, EVT_PAGEUP_FIRST,
                  EVT_PAGE_BREAK, EVT_MENU_BREAK)
end

-- ===========================================================================
-- Model data
-- ===========================================================================
local function refreshLSDefs(w)
  for i = 0, N_LS - 1 do
    local ls = model.getLogicalSwitch(i)
    w.lsDef[i] = (ls ~= nil) and (ls.func or 0) > 0
  end
  w.outName = {}
  for i = 1, N_CHANS do
    local out = model.getOutput(i - 1)
    w.outName[i] = (out and out.name) or ''
  end
  w.lsLastRefresh = getTime()
end

local function cacheSourceIds(w)
  w.lsId, w.chId, w.tmrId = {}, {}, {}
  for i = 0, N_LS - 1 do
    local fi = getFieldInfo('ls' .. (i + 1))
    w.lsId[i] = fi and fi.id or ('ls' .. (i + 1))
  end
  for i = 1, N_CHANS do
    local fi = getFieldInfo('ch' .. i)
    w.chId[i] = fi and fi.id or ('ch' .. i)
  end
  for i = 1, N_TMR do
    local fi = getFieldInfo('timer' .. i)
    w.tmrId[i] = fi and fi.id or nil
  end
end

-- Distribute active wing panels (types 1-3) across the half span
local function updateSurfaces(w)
  local present, cnt = {}, 0
  for j = 1, N_CHANS do
    local t = w.chAssign[j].type
    if t >= 1 and t <= 3 and not present[t] then
      present[t] = true
      cnt = cnt + 1
    end
  end
  w.meshDirty = true
  if cnt == 0 then return end
  local step = SPAN_LEN / (2 * cnt)
  local x = 0
  for t = 1, 3 do
    if present[t] then
      w.surf[t].x0 = x
      x = x + step
      w.surf[t].x1 = x
    end
  end
end

local function modAssignment(w, iChan, sType, lr, rot)
  local a = w.chAssign[iChan]
  if not a then
    w.chAssign[iChan] = {type = sType or SURF_OTHER, side = lr or RIGHT, rot = rot or ROT_NORMAL}
  else
    if lr then a.side = lr end
    if rot then a.rot = rot end
    if sType then a.type = sType end
  end
end

-- ---------------------------------------------------------------------------
-- Auto-detection
-- ---------------------------------------------------------------------------
local function clean(s)
  return string.lower(string.gsub(s or '', '[%s%-_%.%[%]]', ''))
end

-- Surface type from a name (output name, mix name or source name). nil = unknown
local function typeFromText(raw)
  local n = clean(raw)
  if n == '' then return nil end
  if string.find(n, 'tip') then return 3 end
  if string.find(n, 'ail') or string.find(n, 'flaperon') then return 2 end
  if string.find(n, 'fl') then return 1 end
  if string.find(n, 'vee') or string.find(n, 'vtail') or string.find(n, 'rvat') or string.find(n, 'ruddervator')
     or (string.sub(n, 1, 1) == 'v' and #n <= 3) then return 6 end
  if string.find(n, 'el') then return 4 end   -- ele, elev, elevator
  if string.find(n, 'rud') or n == 'rd' then return 5 end
  if string.find(n, 'right') or string.find(n, 'left') then return 2 end
  return nil
end

-- Side from a name. nil = can't tell
local function sideOf(raw)
  local s = clean(raw)
  if s == '' then return nil end
  if string.find(s, 'right') or string.find(s, 'rght') then return RIGHT end
  if string.find(s, 'left') or string.find(s, 'lft') then return LEFT end
  -- remove the surface words so their own letters (aiL, fLap, ...) don't count
  for _, kw in ipairs({'flaperon', 'aileron', 'ailrn', 'ail', 'flap', 'flp', 'fl', 'tip',
                       'ruddervator', 'vtail', 'vee', 'elevator', 'elev', 'ele', 'rudder', 'rud'}) do
    s = string.gsub(s, kw, '')
  end
  local hasR, hasL = string.find(s, 'r') ~= nil, string.find(s, 'l') ~= nil
  if hasR and not hasL then return RIGHT end
  if hasL and not hasR then return LEFT end
  return nil
end

-- Names to inspect for a channel: output name first, then its mixer lines
local function channelTexts(ch0)
  local out = model.getOutput(ch0)
  local names = {(out and out.name) or ''}
  local srcs = {}
  local cnt = (model.getMixesCount and model.getMixesCount(ch0)) or 0
  for k = 0, cnt - 1 do
    local mx = model.getMix(ch0, k)
    if mx then
      if mx.name and mx.name ~= '' then names[#names + 1] = mx.name end
      if mx.source and getSourceName then
        local ok, sn = pcall(getSourceName, mx.source)
        if ok and sn then srcs[#srcs + 1] = sn end
      end
    end
  end
  return names, srcs
end

local function autoAssign(w)
  w.chAssign = {}
  local groups = {}
  for i = 1, N_CHANS do
    local names, srcs = channelTexts(i - 1)
    local t, side
    -- type: names first, then (only if none) the first recognisable mix source
    for _, n in ipairs(names) do t = t or typeFromText(n) end
    if not t then
      for _, s in ipairs(srcs) do
        local st = typeFromText(s)
        if st == 2 or st == 4 or st == 5 then t = st break end
      end
    end
    t = t or SURF_OTHER
    for _, n in ipairs(names) do side = side or sideOf(n) end
    w.chAssign[i] = {type = t, side = side or RIGHT, rot = ROT_NORMAL, sideKnown = side ~= nil}
    if w.surf[t].hasLR then
      groups[t] = groups[t] or {}
      table.insert(groups[t], i)
    end
  end

  -- Resolve sides that couldn't be read from names
  for _, chans in pairs(groups) do
    if #chans == 1 then
      local a = w.chAssign[chans[1]]
      if not a.sideKnown then a.side = BOTH end   -- single servo/Y-lead: show both wings
    else
      local hasR, hasL = false, false
      for _, c in ipairs(chans) do
        local a = w.chAssign[c]
        if a.sideKnown then
          if a.side == RIGHT then hasR = true else hasL = true end
        end
      end
      local nextSide = RIGHT
      for _, c in ipairs(chans) do
        local a = w.chAssign[c]
        if not a.sideKnown then
          if hasR and not hasL then nextSide = LEFT
          elseif hasL and not hasR then nextSide = RIGHT end
          a.side = nextSide
          if nextSide == RIGHT then hasR = true else hasL = true end
          nextSide = -nextSide
        end
      end
    end
  end
  for i = 1, N_CHANS do w.chAssign[i].sideKnown = nil end
  updateSurfaces(w)
end

-- ---------------------------------------------------------------------------
-- Saved assignments (one file per model, next to this script)
-- ---------------------------------------------------------------------------
local function cfgPath()
  local info = model.getInfo and model.getInfo()
  local nm = (info and info.name and info.name ~= '') and info.name or 'model'
  return '/WIDGETS/OTXSim/' .. string.gsub(nm, '[^%w]', '_') .. '.cfg'
end

local function saveAssign(w)
  local f = io.open(cfgPath(), 'w')
  if not f then return end
  for i = 1, N_CHANS do
    local a = w.chAssign[i]
    io.write(f, string.format('%d,%d,%d\n', a.type, a.side, a.rot))
  end
  io.close(f)
end

local function clearSaved()
  local f = io.open(cfgPath(), 'w')
  if f then io.close(f) end
end

local function loadAssign(w)
  local f = io.open(cfgPath(), 'r')
  if not f then return false end
  local data = io.read(f, 1024)
  io.close(f)
  if not data or #data == 0 then return false end
  local tmp, i = {}, 0
  for t, s, r in string.gmatch(data, '(%-?%d+),(%-?%d+),(%-?%d+)') do
    i = i + 1
    t, s, r = tonumber(t), tonumber(s), tonumber(r)
    if i > N_CHANS or not w.surf[t] then return false end
    tmp[i] = {type = t, side = s, rot = (r < 0) and -1 or 1}
  end
  if i ~= N_CHANS then return false end
  w.chAssign = tmp
  updateSurfaces(w)
  return true
end

--[[ Root flight mode of a GVAR (follows FM links; value <= 1024 = own value).
     Workaround for getGlobalVariable not resolving linked GVs. ]]
local function getfmGVRoot(igv, fmNow)
  local fm = fmNow or getFlightMode()
  local fm0 = fm
  for _ = 1, 10 do   -- guard against link loops
    local val = model.getGlobalVariable(igv - 1, fm)
    if val == nil or val <= 1024 then return fm end
    fm = val - 1025
    if fm >= fm0 then fm = fm + 1 end  -- link index skips the FM's own slot
    fm0 = fm
  end
  return getFlightMode()
end

local function getGV(igv, fmNow)
  return model.getGlobalVariable(igv - 1, getfmGVRoot(igv, fmNow)) or 0
end

local function setGV(igv, val)
  if val > 1024 then val = 1024 elseif val < -1024 then val = -1024 end
  model.setGlobalVariable(igv - 1, getfmGVRoot(igv), val)
end

local function chanPercent(w, i)
  return (getValue(w.chId[i]) or 0) / 10.24
end

-- ===========================================================================
-- 3D model view (lightweight)
--   Model space: x = right wing, y = up, z = rearward (nose -z, tail +z)
--   Flat, double-sided panels; painter's-algorithm depth sort; mild perspective.
--   The fixed airframe is projected + sorted once per view and cached; only the
--   moving surfaces are processed each frame, keeping well inside the Lua
--   CPU limit of EdgeTX.
-- ===========================================================================
local rad = math.rad
local sin, cos, abs, sqrt = math.sin, math.cos, math.abs, math.sqrt

local MAX_DEG = 30     -- surface deflection (degrees) shown for 100% output
local CAM_D   = 9      -- camera distance (perspective strength)

local VIEWS = {
  {name = '3/4',  yaw = rad(38), pitch = rad(26)},
  {name = 'Rear', yaw = 0,       pitch = rad(8)},
  {name = 'Side', yaw = rad(90), pitch = rad(4)},
  {name = 'Top',  yaw = 0,       pitch = rad(90)},
}
for _, v in ipairs(VIEWS) do
  v.cb, v.sb, v.ca, v.sa = cos(v.yaw), sin(v.yaw), cos(v.pitch), sin(v.pitch)
end

-- base colours {r,g,b}
local K_AIRFRAME = {238, 241, 247}
local K_CANOPY   = {28, 44, 66}
local K_WHEEL    = {48, 56, 70}
local K_FIN      = {165, 172, 186}
local K_SURF     = {95, 240, 255}
local K_FLAP     = {175, 250, 255}
local K_RUDDER   = {70, 150, 240}

-- light direction in view space, normalised
local LX, LY, LZ = -0.30, 0.55, 0.78
do local l = sqrt(LX*LX + LY*LY + LZ*LZ); LX, LY, LZ = LX/l, LY/l, LZ/l end

local function V(x, y, z) return {x, y, z} end
local function quad(list, a, b, c, d, k) list[#list + 1] = {a, b, c, d, k = k} end
local function tri(list, a, b, c, k)     list[#list + 1] = {a, b, c, k = k} end

-- n-sided polygon centred at c in the plane of unit vectors u, v (one item)
local function ngon(list, c, u, v, r, n, k)
  local poly = {k = k}
  for i = 0, n - 1 do
    local a = 2 * math.pi * i / n
    local ca, sa = cos(a) * r, sin(a) * r
    poly[i + 1] = {c[1] + u[1]*ca + v[1]*sa, c[2] + u[2]*ca + v[2]*sa, c[3] + u[3]*ca + v[3]*sa}
  end
  list[#list + 1] = poly
end

-- ---------------------------------------------------------------------------
-- Geometry
-- ---------------------------------------------------------------------------
local WING_Y, WING_T = 0.24, 0.05     -- wing top surface, thickness (front face)
local WING_LE, WING_HINGE = -0.38, 0.18
local SURF_CHORD, SURF_T = 0.17, 0.03
local ROOT_X, TIP_X = 0.24, 1.95
local STAB_HINGE, FIN_HINGE = 1.95, 1.98

local function wingSlots(w)
  local has = {}
  for i = 1, N_CHANS do has[w.chAssign[i].type] = true end
  local s = {}
  s[1] = {ROOT_X, 1.00}
  if has[3] then s[2] = {1.03, 1.50}; s[3] = {1.53, TIP_X}
  else s[2] = {1.03, TIP_X} end
  if not has[2] and has[3] then s[3] = {1.03, TIP_X} end
  return s, has
end

-- horizontal plate in x0..x1, z0..z1 at height y
local function hplate(list, x0, x1, y, z0, z1, k)
  quad(list, V(x0, y, z0), V(x1, y, z0), V(x1, y, z1), V(x0, y, z1), k)
end

local buildHingeDefs   -- defined below

local function buildStatic(w)
  local P = {}
  local slots, has = wingSlots(w)
  w.slots, w.has = slots, has

  -- fuselage: 6-sided, 3 segments
  local rings = {{-1.15, 0.0, 0}, {-0.98, 0.19, 0}, {0.45, 0.19, 0}, {2.05, 0.045, 0.03}}
  local N, prev = 6, nil
  for _, rg in ipairs(rings) do
    local ring = {}
    for i = 0, N - 1 do
      local a = 2 * math.pi * (i + 0.5) / N
      ring[i] = V(rg[2] * cos(a), rg[3] + rg[2] * sin(a), rg[1])
    end
    if prev then
      for i = 0, N - 1 do
        local j = (i + 1) % N
        quad(P, prev[i], prev[j], ring[j], ring[i], K_AIRFRAME)
      end
    end
    prev = ring
  end

  -- canopy: top + sloped windscreen
  quad(P, V(-0.11, 0.29, -0.70), V(0.11, 0.29, -0.70), V(0.11, 0.29, -0.32), V(-0.11, 0.29, -0.32), K_CANOPY)
  quad(P, V(-0.15, 0.17, -0.88), V(0.15, 0.17, -0.88), V(0.11, 0.29, -0.70), V(-0.11, 0.29, -0.70), K_CANOPY)

  -- wing: 3 spanwise plates (helps depth sorting) + leading-edge face
  for _, sp in ipairs({{-2.0, -0.25}, {-0.25, 0.25}, {0.25, 2.0}}) do
    hplate(P, sp[1], sp[2], WING_Y, WING_LE, WING_HINGE, K_AIRFRAME)
  end
  quad(P, V(-2.0, WING_Y, WING_LE), V(2.0, WING_Y, WING_LE), V(2.0, WING_Y - WING_T, WING_LE),
       V(-2.0, WING_Y - WING_T, WING_LE), K_AIRFRAME)

  -- fixed trailing-edge strips where nothing is assigned
  local function fixedTE(x0, x1)
    hplate(P, x0, x1, WING_Y - 0.01, WING_HINGE, WING_HINGE + SURF_CHORD, K_AIRFRAME)
    hplate(P, -x1, -x0, WING_Y - 0.01, WING_HINGE, WING_HINGE + SURF_CHORD, K_AIRFRAME)
  end
  fixedTE(0.0, ROOT_X)
  if not has[1] then fixedTE(slots[1][1], slots[1][2]) end
  if not has[2] and not has[3] then fixedTE(1.03, TIP_X) end
  if has[3] and not has[2] then fixedTE(1.03, 1.50) end

  -- struts / gear legs (lines)
  w.lines = {}
  local function ln(a, b) w.lines[#w.lines + 1] = {a, b} end
  for _, s in ipairs({1, -1}) do
    ln(V(s*0.15, -0.12, -0.15), V(s*1.05, WING_Y - WING_T, -0.15))
    ln(V(s*0.12, -0.15, -0.10), V(s*0.42, -0.58, -0.05))
  end
  ln(V(0, -0.18, -0.85), V(0, -0.60, -0.88))

  -- wheels: hexagons in the y-z plane
  local U, W2 = V(0, 1, 0), V(0, 0, 1)
  ngon(P, V(0.44, -0.62, -0.05), U, W2, 0.13, 6, K_WHEEL)
  ngon(P, V(-0.44, -0.62, -0.05), U, W2, 0.13, 6, K_WHEEL)
  ngon(P, V(0.0, -0.64, -0.88), U, W2, 0.11, 6, K_WHEEL)

  -- tail
  w.vtail = has[6] and not has[4] and not has[5]
  if w.vtail then
    local L, c35, s35 = 0.75, cos(rad(35)), sin(rad(35))
    for _, s in ipairs({1, -1}) do
      quad(P, V(s*0.03, 0.04, 1.62), V(s*(0.03 + L*c35), 0.04 + L*s35, 1.72),
           V(s*(0.03 + L*c35), 0.04 + L*s35, STAB_HINGE), V(s*0.03, 0.04, STAB_HINGE), K_AIRFRAME)
    end
  else
    hplate(P, -0.70, 0.70, 0.02, 1.70, STAB_HINGE, K_AIRFRAME)
    quad(P, V(0, 0.04, 1.50), V(0, 0.62, 1.82), V(0, 0.62, FIN_HINGE), V(0, 0.04, FIN_HINGE), K_FIN)
    if not has[4] then hplate(P, -0.68, 0.68, 0.02, STAB_HINGE, STAB_HINGE + 0.15, K_AIRFRAME) end
    if not has[5] then
      quad(P, V(0, 0.04, FIN_HINGE), V(0, 0.62, FIN_HINGE), V(0, 0.62, FIN_HINGE + 0.13),
           V(0, 0.04, FIN_HINGE + 0.13), K_FIN)
    end
  end

  w.static = P
  buildHingeDefs(w)
  w.cache, w.cacheKey, w.dyn, w.dynKey = nil, nil, nil, nil
  w.meshDirty = false
end

local SURF_LABEL = {[1] = 'flap', [2] = 'aileron', [3] = 'tip', [6] = 'vee'}

-- Hinge definitions for every movable surface slot, built with the static mesh:
-- defs[type][side] = list of {a = hinge start, b = hinge end, n = deflection dir,
--                             chord, k = colour, name}
buildHingeDefs = function(w)
  local defs = {}
  local function add(t, side, a, b, n, chord, k, name)
    defs[t] = defs[t] or {}
    defs[t][side] = defs[t][side] or {}
    local l = defs[t][side]
    l[#l + 1] = {a = a, b = b, n = n, chord = chord, k = k, name = name}
  end
  local up = V(0, 1, 0)
  for t = 1, 3 do
    local sl = w.slots[t]
    if sl then
      for _, sd in ipairs({RIGHT, LEFT}) do
        add(t, sd, V(sd*sl[1], WING_Y - 0.01, WING_HINGE), V(sd*sl[2], WING_Y - 0.01, WING_HINGE), up,
            SURF_CHORD, (t == 1) and K_FLAP or K_SURF, ((sd == RIGHT) and 'R ' or 'L ') .. SURF_LABEL[t])
      end
    end
  end
  add(4, RIGHT, V(0.03, 0.02, STAB_HINGE), V(0.68, 0.02, STAB_HINGE), up, 0.15, K_SURF, nil)
  add(4, RIGHT, V(-0.03, 0.02, STAB_HINGE), V(-0.68, 0.02, STAB_HINGE), up, 0.15, K_SURF, 'Elevator')
  add(5, RIGHT, V(0, 0.04, FIN_HINGE), V(0, 0.62, FIN_HINGE), V(1, 0, 0), 0.13, K_RUDDER, 'Rudder')
  local L, c35, s35 = 0.75, cos(rad(35)), sin(rad(35))
  for _, sd in ipairs({RIGHT, LEFT}) do
    add(6, sd, V(sd*0.03, 0.05, STAB_HINGE), V(sd*(0.03 + L*c35), 0.05 + L*s35, STAB_HINGE),
        V(-sd*s35, c35, 0), 0.15, K_SURF, ((sd == RIGHT) and 'R ' or 'L ') .. 'vee')
  end
  w.hdefs = defs
end

-- Project one deflected surface straight to a screen item (no intermediate polygon)
local function surfaceItem(vw, f, d, theta)
  local cb, sb, ca, sa, s, ox, oy = vw.cb, vw.sb, vw.ca, vw.sa, f.s, f.ox, f.oy
  local c, sn = cos(theta), sin(theta)
  local n, ch = d.n, d.chord
  -- chord vector: rearward (0,0,1) rotated towards n
  local dx, dy, dz = n[1]*sn*ch, n[2]*sn*ch, c*ch
  local a, b = d.a, d.b
  local px = {a[1], b[1], b[1] + dx, a[1] + dx}
  local py = {a[2], b[2], b[2] + dy, a[2] + dy}
  local pz = {a[3], b[3], b[3] + dz, a[3] + dz}
  local X, Y = {}, {}
  local depth = 0
  for j = 1, 4 do
    local x, z0 = px[j]*cb + pz[j]*sb, -px[j]*sb + pz[j]*cb
    local y, z = py[j]*ca - z0*sa, py[j]*sa + z0*ca
    local k = CAM_D / (CAM_D - z) * s
    X[j], Y[j] = floor(ox + x*k + 0.5), floor(oy - y*k + 0.5)
    depth = depth + z
  end
  -- lighting: normal = hinge x chord (model space), rotated into view space
  local hx, hy, hz = b[1] - a[1], b[2] - a[2], b[3] - a[3]
  local nx, ny, nz = hy*dz - hz*dy, hz*dx - hx*dz, hx*dy - hy*dx
  local vx, vz0 = nx*cb + nz*sb, -nx*sb + nz*cb
  local vy, vz = ny*ca - vz0*sa, ny*sa + vz0*ca
  local l = sqrt(vx*vx + vy*vy + vz*vz)
  local br = 0.75
  if l > 1e-9 then br = 0.36 + 0.64 * abs((vx*LX + vy*LY + vz*LZ) / l) end
  local K = d.k
  return {d = depth / 4, x = X, y = Y,
          c  = lcd.RGB(floor(K[1]*br), floor(K[2]*br), floor(K[3]*br)),
          te = lcd.RGB(floor(K[1]*br*0.55), floor(K[2]*br*0.55), floor(K[3]*br*0.6))}
end

-- Moving surfaces for this frame: screen items + labels.
-- Each channel's projected surfaces are cached and only recomputed when its
-- output changes, at most MAX_RECALC surfaces per frame (bounds the CPU cost).
local MAX_RECALC = 6

local function buildDynamic(w, vw, f, key)
  local items, labels = {}, {}
  local defs = w.hdefs
  if w.dynKey ~= key then w.dyn, w.dynKey = {}, key end
  local cache = w.dyn
  local budget = MAX_RECALC
  for i = 1, N_CHANS do
    local a = w.chAssign[i]
    local t = a.type
    local dt = defs[t]
    if dt then
      local pct = chanPercent(w, i)
      if pct > 125 then pct = 125 elseif pct < -125 then pct = -125 end
      local e = cache[i]
      local same = e and e.t == t and e.side == a.side and e.rot == a.rot
      if same and (abs(e.pct - pct) < 0.25 or budget <= 0) then
        for _, it in ipairs(e.items) do items[#items + 1] = it end
        for _, lb in ipairs(e.labels) do labels[#labels + 1] = lb end
      elseif budget <= 0 then
        -- out of budget this frame and nothing cached yet: appears next frame
      else
      local myItems, myLabels = {}, {}
      local deg = a.rot * pct * MAX_DEG / 100
      local s1, d1, s2, d2 = RIGHT, deg, nil, nil
      if w.surf[t].hasLR then
        if a.side == BOTH then
          s2, d2 = LEFT, (t == 2 or t == 3) and -deg or deg
        elseif a.side == LEFT then
          s1 = LEFT
        end
      end
      for q = 1, (s2 and 2 or 1) do
        local sd, dg = s1, d1
        if q == 2 then sd, dg = s2, d2 end
        local list = dt[sd]
        if list then
          local th = rad(dg)
          for _, d in ipairs(list) do
            local it = surfaceItem(vw, f, d, th)
            myItems[#myItems + 1] = it
            budget = budget - 1
            if d.name then
              myLabels[#myLabels + 1] = {x = (it.x[3] + it.x[4]) / 2, y = (it.y[3] + it.y[4]) / 2,
                                         name = d.name, deg = dg}
            end
          end
        end
      end
      cache[i] = {t = t, side = a.side, rot = a.rot, pct = pct, items = myItems, labels = myLabels}
      for _, it in ipairs(myItems) do items[#items + 1] = it end
      for _, lb in ipairs(myLabels) do labels[#labels + 1] = lb end
      end
    end
  end
  return items, labels
end

-- ---------------------------------------------------------------------------
-- Projection helpers
-- ---------------------------------------------------------------------------
local function xform(vw, p)
  local x = p[1]*vw.cb + p[3]*vw.sb
  local z = -p[1]*vw.sb + p[3]*vw.cb
  local y = p[2]
  return x, y*vw.ca - z*vw.sa, y*vw.sa + z*vw.ca
end

local function clampPt(x, y, r)
  if x < r.x then x = r.x elseif x > r.x + r.w - 1 then x = r.x + r.w - 1 end
  if y < r.y then y = r.y elseif y > r.y + r.h - 1 then y = r.y + r.h - 1 end
  return floor(x + 0.5), floor(y + 0.5)
end

-- Liang-Barsky line clip against rect r
local function clipLine(x0, y0, x1, y1, r)
  local dx, dy = x1 - x0, y1 - y0
  local t0, t1 = 0, 1
  local p = {-dx, dx, -dy, dy}
  local q = {x0 - r.x, r.x + r.w - 1 - x0, y0 - r.y, r.y + r.h - 1 - y0}
  for i = 1, 4 do
    if p[i] == 0 then
      if q[i] < 0 then return nil end
    else
      local t = q[i] / p[i]
      if p[i] < 0 then if t > t1 then return nil elseif t > t0 then t0 = t end
      else if t < t0 then return nil elseif t < t1 then t1 = t end end
    end
  end
  return x0 + t0*dx, y0 + t0*dy, x0 + t1*dx, y0 + t1*dy
end

-- project a polygon list into drawable items {d, x[], y[], c}
-- (clamping to r is only needed for the cached static items)
local function projectPolys(list, vw, f, r, out, clamp, i0, i1)
  local s, ox, oy = f.s, f.ox, f.oy
  local cb, sb, ca, sa = vw.cb, vw.sb, vw.ca, vw.sa
  for pi = i0 or 1, i1 or #list do
    local poly = list[pi]
    local n = #poly
    local sx, sy, vx, vy, vz = {}, {}, {}, {}, {}
    local depth = 0
    for j = 1, n do
      local p = poly[j]
      local x = p[1]*cb + p[3]*sb
      local z0 = -p[1]*sb + p[3]*cb
      local y = p[2]*ca - z0*sa
      local z = p[2]*sa + z0*ca
      local k = CAM_D / (CAM_D - z) * s
      if clamp then
        sx[j], sy[j] = clampPt(ox + x*k, oy - y*k, r)
      else
        sx[j], sy[j] = floor(ox + x*k + 0.5), floor(oy - y*k + 0.5)
      end
      vx[j], vy[j], vz[j] = x, y, z
      depth = depth + z
    end
    local ax, ay, az = vx[2] - vx[1], vy[2] - vy[1], vz[2] - vz[1]
    local bx, by, bz = vx[n] - vx[1], vy[n] - vy[1], vz[n] - vz[1]
    local nx, ny, nz = ay*bz - az*by, az*bx - ax*bz, ax*by - ay*bx
    local l = sqrt(nx*nx + ny*ny + nz*nz)
    local b = 0.75
    if l > 1e-9 then b = 0.36 + 0.64 * abs((nx*LX + ny*LY + nz*LZ) / l) end
    local k = poly.k
    local it = {d = depth / n, x = sx, y = sy, c = lcd.RGB(floor(k[1]*b), floor(k[2]*b), floor(k[3]*b))}
    if poly.te then it.te = lcd.RGB(floor(k[1]*b*0.55), floor(k[2]*b*0.55), floor(k[3]*b*0.6)) end
    out[#out + 1] = it
  end
  return out
end

local function byDepth(a, b) return a.d < b.d end

-- Build the per-view cache in stages (one stage per refresh) so no single
-- refresh exceeds the CPU limit. Returns the cache once complete.
local function buildCache(w, vi, r)
  local key = vi .. ':' .. r.x .. ':' .. r.y .. ':' .. r.w .. ':' .. r.h
  if w.cacheKey ~= key then
    w.cacheKey, w.cache = key, {stage = 0}
  end
  local c = w.cache
  if c.stage >= 3 then return c end
  local vw = VIEWS[vi]

  if c.stage == 0 then
    -- fit: bounding box of the projected airframe
    local minx, maxx, miny, maxy = 1e9, -1e9, 1e9, -1e9
    local function acc(p)
      local x, y, z = xform(vw, p)
      local k = CAM_D / (CAM_D - z)
      x, y = x * k, y * k
      if x < minx then minx = x end; if x > maxx then maxx = x end
      if y < miny then miny = y end; if y > maxy then maxy = y end
    end
    for _, poly in ipairs(w.static) do for j = 1, #poly do acc(poly[j]) end end
    acc(V(0, 0.55, -1.18)); acc(V(0, -0.55, -1.18)); acc(V(0.55, 0, -1.18)); acc(V(-0.55, 0, -1.18))
    acc(V(0, 0.85, 2.1)); acc(V(0, -0.2, 2.2))
    local s = math.min((r.w - 16) / (maxx - minx), (r.h - 28) / (maxy - miny))
    c.f = {s = s, ox = r.x + r.w / 2 - (minx + maxx) / 2 * s,
           oy = r.y + 18 + (r.h - 18) / 2 + (miny + maxy) / 2 * s}
    c.stage = 1
  elseif c.stage == 1 then
    -- static airframe, projected in chunks, then sorted back to front
    c.items = c.items or {}
    c.next = c.next or 1
    local last = math.min(#w.static, c.next + 11)
    projectPolys(w.static, vw, c.f, r, c.items, true, c.next, last)
    c.next = last + 1
    if c.next <= #w.static then return nil end
    local items = c.items
    for _, ln in ipairs(w.lines) do
      local x0, y0, z0 = xform(vw, ln[1])
      local x1, y1, z1 = xform(vw, ln[2])
      local k0 = CAM_D / (CAM_D - z0) * c.f.s
      local k1 = CAM_D / (CAM_D - z1) * c.f.s
      local ax, ay = clampPt(c.f.ox + x0*k0, c.f.oy - y0*k0, r)
      local bx, by = clampPt(c.f.ox + x1*k1, c.f.oy - y1*k1, r)
      items[#items + 1] = {d = (z0 + z1) / 2, line = true, x = {ax, bx}, y = {ay, by},
                           c = lcd.RGB(60, 70, 88)}
    end
    table.sort(items, byDepth)
    c.items = items
    c.stage = 2
  else
    -- background: ground grid lines and propeller disc
    local f = c.f
    local function pr(p)
      local x, y, z = xform(vw, p)
      local k = CAM_D / (CAM_D - z) * f.s
      return f.ox + x*k, f.oy - y*k
    end
    local grid, gy = {}, -0.78
    for i = -4, 4 do
      local g = i * 0.75
      for _, seg in ipairs({{V(g, gy, -3), V(g, gy, 3.5)}, {V(-3, gy, g + 0.25), V(3, gy, g + 0.25)}}) do
        local x0, y0 = pr(seg[1])
        local x1, y1 = pr(seg[2])
        local a, b, cc, d = clipLine(x0, y0, x1, y1, r)
        if a then grid[#grid + 1] = {rnd(a), rnd(b), rnd(cc), rnd(d)} end
      end
    end
    c.grid = grid
    local dp = {}
    ngon(dp, V(0, 0, -1.19), V(1, 0, 0), V(0, 1, 0), 0.55, 12, nil)
    local function prc(p) local x, y = pr(p); return clampPt(x, y, r) end
    local disc = {x = {}, y = {}, c = lcd.RGB(30, 34, 42)}
    for j, p in ipairs(dp[1]) do disc.x[j], disc.y[j] = prc(p) end
    c.disc = disc
    local blades = {}
    local hx, hy = prc(V(0, 0, -1.2))
    for bl = 0, 2 do
      local a = rad(20 + bl * 120)
      local ex, ey = prc(V(0.52 * cos(a), 0.52 * sin(a), -1.2))
      blades[#blades + 1] = {hx, hy, ex, ey}
    end
    c.blades = blades
    c.stage = 3
  end
  return nil
end

local function drawItem(it)
  local x, y, c = it.x, it.y, it.c
  local n = #x
  if n == 2 then
    lcd.drawLine(x[1], y[1], x[2], y[2], SOLID, c)
    return
  end
  local x1, y1 = x[1], y[1]
  if n == 4 then
    lcd.drawFilledTriangle(x1, y1, x[2], y[2], x[3], y[3], c)
    lcd.drawFilledTriangle(x1, y1, x[3], y[3], x[4], y[4], c)
  else
    for k = 2, n - 1 do
      lcd.drawFilledTriangle(x1, y1, x[k], y[k], x[k + 1], y[k + 1], c)
    end
  end
  if it.te then lcd.drawLine(x[3], y[3], x[4], y[4], SOLID, it.te) end
end

-- Draw the aircraft in rectangle r using view vi
local function drawModel(w, r, vi, showLabels)
  if w.meshDirty or not w.static then
    buildStatic(w)
    return                 -- geometry built this frame; draw from the next one
  end
  local vw = VIEWS[vi] or VIEWS[1]
  local c = buildCache(w, vi, r)
  if not c then
    local tw, th = textSize('...', 0)
    txt(r.x + (r.w - tw) / 2, r.y + (r.h - th) / 2, '...', 0, C.dim)
    return
  end
  local gk = C.grid
  for _, g in ipairs(c.grid) do lcd.drawLine(g[1], g[2], g[3], g[4], SOLID, gk) end
  drawItem(c.disc)
  local bk = C.blade
  for _, b in ipairs(c.blades) do lcd.drawLine(b[1], b[2], b[3], b[4], SOLID, bk) end

  -- moving surfaces: project, sort, then merge with the cached static list
  local dyn, labels = buildDynamic(w, vw, c.f, w.cacheKey)
  table.sort(dyn, byDepth)
  local S = c.items
  local i, j, ns, nd = 1, 1, #S, #dyn
  while i <= ns or j <= nd do
    if j > nd or (i <= ns and S[i].d < dyn[j].d) then
      drawItem(S[i]); i = i + 1
    else
      drawItem(dyn[j]); j = j + 1
    end
  end

  -- labels
  if showLabels then
    local f = c.f
    if not w.lblW then w.lblW = {}; w.lblDigitW = textSize('0', SMLSIZE) end
    local placed = {}
    local function overlaps(x, y, bw, bh)
      for _, p in ipairs(placed) do
        if x < p[1] + p[3] and p[1] < x + bw and y < p[2] + p[4] and p[2] < y + bh then return true end
      end
      return false
    end
    for _, lb in ipairs(labels) do
      local ax, ay = lb.x, lb.y
      local val = string.format('%.1f', lb.deg)
      local lw = w.lblW[lb.name]
      if not lw then lw = {textSize(lb.name .. ' ', SMLSIZE)}; w.lblW[lb.name] = lw end
      local nw, th = lw[1], lw[2]
      local vwid = #val * w.lblDigitW
      local bw, bh = nw + vwid + 10, th + 4
      local bx, by = ax - bw / 2, ay - bh - 6
      if bx < r.x then bx = r.x elseif bx + bw > r.x + r.w then bx = r.x + r.w - bw end
      if by < r.y then by = r.y elseif by + bh > r.y + r.h then by = r.y + r.h - bh end
      local y0 = by
      for t = 1, 6 do
        if not overlaps(bx, by, bw, bh) then break end
        local step = math.ceil(t / 2) * (bh + 2)
        by = (t % 2 == 1) and (y0 - step) or (y0 + step)
        if by < r.y then by = r.y elseif by + bh > r.y + r.h then by = r.y + r.h - bh end
      end
      placed[#placed + 1] = {bx, by, bw, bh}
      fillRect(bx, by, bw, bh, C.lblBg)
      rect(bx, by, bw, bh, C.lblEdge)
      txt(bx + 3, by + 2, lb.name, SMLSIZE, C.text)
      txt(bx + 3 + nw, by + 2, val, SMLSIZE, C.lblVal)
      rect(bx + 4 + nw + vwid, by + 3, 3, 3, C.lblVal)   -- degree mark
    end
  end
end

-- View selection buttons (top-left of the model area)
local function drawViewButtons(w, r)
  local bx, by = r.x + 4, r.y + 4
  for i, v in ipairs(VIEWS) do
    local tw, th = textSize(v.name, SMLSIZE)
    local bw, bh = tw + 16, th + 10
    fillRect(bx, by, bw, bh, C.btnBg)
    rect(bx, by, bw, bh, (w.view == i) and C.accent or C.lblEdge)
    if w.view == i then rect(bx + 1, by + 1, bw - 2, bh - 2, C.accent) end
    txt(bx + 8, by + 5, v.name, SMLSIZE, C.text)
    addHit(w, bx, by, bw, bh, function() w.view = i end)
    bx = bx + bw + 6
  end
  -- labels on/off
  local lbl = w.showLabels and 'Tags on' or 'Tags off'
  local tw, th = textSize('Tags off', SMLSIZE)
  local bw, bh = tw + 16, th + 10
  bx = bx + 6
  fillRect(bx, by, bw, bh, C.btnBg)
  rect(bx, by, bw, bh, w.showLabels and C.accent or C.lblEdge)
  txt(bx + 8, by + 5, lbl, SMLSIZE, w.showLabels and C.text or C.dim)
  addHit(w, bx, by, bw, bh, function() w.showLabels = not w.showLabels end)
end

-- ===========================================================================
-- Page 1 parts
-- ===========================================================================
local function hms(secs)
  local neg = secs < 0
  if neg then secs = -secs end
  local hh = floor(secs / 3600)
  local mm = floor((secs % 3600) / 60)
  local ss = floor(secs % 60)
  return neg, hh, mm, ss
end

local function drawTopBar(w, X, Y, W, H, interactive)
  fillRect(X, Y, W, H, C.panel)
  local fx = X + 6
  if interactive then
    button(w, X + 2, Y + 2, 70, H - 4, 'Model', function() w.page = 1; w.editing = false end, w.page == 1)
    button(w, X + 76, Y + 2, 70, H - 4, 'Assign', function() w.page = 2; w.editing = false end, w.page == 2)
    fx = X + 156
  end

  local fmNo, fmName = getFlightMode()
  local fmStr = 'FM' .. fmNo .. ' ' .. (fmName or '')
  local _, th = textSize(fmStr, 0)
  txt(fx, Y + (H - th) / 2, fmStr, 0, C.edit)

  -- timers (right aligned)
  local tx = X + W - 4
  for i = N_TMR, 1, -1 do
    local tm = model.getTimer(i - 1)
    if tm and tm.mode and tm.mode ~= 0 then
      local v = w.tmrId[i] and getValue(w.tmrId[i]) or tm.value or 0
      local neg, hh, mm, ss = hms(v)
      local s = string.format('T%d %s%d:%02d:%02d', i, neg and '-' or '', hh, mm, ss)
      local tw, sh = textSize(s, SMLSIZE)
      tx = tx - tw
      txt(tx, Y + (H - sh) / 2, s, SMLSIZE, C.text)
      tx = tx - 10
    end
  end
end

local function drawLogicalSwitches(w, X, Y, W, H)
  local cols = 8
  local rows = N_LS / cols
  local _, th = textSize('L', SMLSIZE)
  txt(X, Y, 'Logical switches', SMLSIZE, C.dim)
  local gy = Y + th + 2
  local cell = floor(math.min((W - 2) / cols, (H - th - 4) / rows))
  if cell < 5 then return gy end
  local box = cell - 3
  local half = floor(box / 2) - 1
  local on, off = C.lsOn, C.lsOff
  local def, ids = w.lsDef, w.lsId
  local fr, dr, gv = lcd.drawFilledRectangle, lcd.drawRectangle, getValue
  X, gy = floor(X), floor(gy)
  local i = 0
  for row = 0, rows - 1 do
    local cy = gy + row * cell
    local cx = X
    for col = 1, cols do
      if def[i] then
        if (gv(ids[i]) or 0) > 0 then fr(cx, cy, box, box, on) else dr(cx, cy, box, box, off) end
      else
        fr(cx + half, cy + half, 2, 2, off)
      end
      cx = cx + cell
      i = i + 1
    end
  end
  return gy + rows * cell
end

local function drawOtherChannels(w, X, Y, W, H)
  local _, th = textSize('C', SMLSIZE)
  local y = Y
  for i = 1, N_CHANS do
    local a = w.chAssign[i]
    if not w.surf[a.type].draw then
    local nm = w.outName[i]
    local v = rnd(chanPercent(w, i) * a.rot)
    -- skip unused channels (no name and centred) to save space
    if nm ~= '' or v ~= 0 then
      if y + th > Y + H then break end
      if nm == '' then nm = 'CH' .. i end
      txt(X, y, nm, SMLSIZE, C.dim)
      local s = tostring(v)
      local tw = textSize(s, SMLSIZE)
      txt(X + W - tw - 2, y, s, SMLSIZE, C.text)
      y = y + th + 1
    end
    end
  end
end

local function drawGVars(w, X, Y, W, H, interactive)
  local btnW = interactive and 52 or 0
  local cellW = (W - 2 * btnW - (interactive and 8 or 0)) / N_GVARS
  local blink = (floor(getTime() / 40) % 2) == 0
  local fmNow = getFlightMode()
  if not w.gvLblW then
    w.gvLblW, w.gvLblH = textSize('GV0', SMLSIZE)
    w.digitW = textSize('0', 0)
  end
  local lw, lh, dw = w.gvLblW, w.gvLblH, w.digitW
  for i = 1, N_GVARS do
    local cx = X + (i - 1) * cellW
    local sel = interactive and (i == w.activeGV)
    local bg = C.panel
    if sel then bg = (w.editing and blink) and C.edit or C.panelHi end
    fillRect(cx + 1, Y, cellW - 2, H, bg)
    local fg = (sel and w.editing and blink) and C.editTxt or C.text
    txt(cx + (cellW - lw) / 2, Y + 2, 'GV' .. i, SMLSIZE, (sel and w.editing and blink) and C.editTxt or C.dim)
    local v = tostring(getGV(i, fmNow))
    txt(cx + (cellW - #v * dw) / 2, Y + lh + 3, v, 0, fg)
    if interactive then
      addHit(w, cx, Y, cellW, H, function()
        if w.activeGV == i then w.editing = not w.editing
        else w.activeGV = i end
      end)
    end
  end
  if interactive then
    local bx = X + W - 2 * btnW - 4
    button(w, bx, Y, btnW, H, '-', function() setGV(w.activeGV, getGV(w.activeGV) - 1) end, false)
    button(w, bx + btnW + 4, Y, btnW, H, '+', function() setGV(w.activeGV, getGV(w.activeGV) + 1) end, false)
  end
end

local function page1Keys(w, event)
  if evtExit(event) then
    if w.editing then w.editing = false
    elseif lcd.exitFullScreen then lcd.exitFullScreen() end
  elseif evtEnter(event) then
    w.editing = not w.editing
  elseif evtPage(event) then
    w.editing = false
    w.page = 2
  else
    local inc = evtInc(event)
    if inc ~= 0 then
      if w.editing then
        setGV(w.activeGV, getGV(w.activeGV) + inc)
      else
        w.activeGV = (w.activeGV - 1 + inc) % N_GVARS + 1
      end
    end
  end
end

local function drawPage1(w, X, Y, W, H, interactive)
  local _, fh = textSize('0', 0)
  local _, sh = textSize('0', SMLSIZE)
  local gvH = fh + sh + 8
  local rpW = math.max(96, floor(W * 0.2))
  local top = Y
  local bottom = Y + H - gvH - 2

  -- right panel: logical switches + unassigned channel values
  local rpX = X + W - rpW
  fillRect(rpX, top, rpW, bottom - top, C.panel)
  local lsBottom = drawLogicalSwitches(w, rpX + 4, top + 2, rpW - 8, floor((bottom - top) * 0.62))
  drawOtherChannels(w, rpX + 4, lsBottom + 4, rpW - 8, bottom - lsBottom - 6)

  -- model
  local mr = {x = X, y = top, w = W - rpW - 2, h = bottom - top}
  drawModel(w, mr, w.view, w.showLabels)
  if interactive then
    drawViewButtons(w, mr)
  else
    -- controls only work in full screen (EdgeTX sends no input to a widget in a zone)
    local hint = 'Full screen: Assign, views, GV edit'
    local _, hh = textSize(hint, SMLSIZE)
    txt(mr.x + 4, mr.y + mr.h - hh - 2, hint, SMLSIZE, C.dim)
  end

  -- gvars
  drawGVars(w, X, bottom + 2, W, gvH, interactive)
end

-- ===========================================================================
-- Page 2: assignment editor
-- ===========================================================================
local function isSubfieldVisible(w, iChan, f)
  if f == FLD_LR then return w.surf[w.chAssign[iChan].type].hasLR end
  return true
end

local function cycleType(w, iChan, inc)
  local t = w.chAssign[iChan].type
  modAssignment(w, iChan, (t + inc - 1) % #w.surf + 1)
  updateSurfaces(w)
  saveAssign(w)
end

local SIDE_NAME = {[RIGHT] = 'Right', [LEFT] = 'Left', [BOTH] = 'Both'}

local function cycleSide(w, iChan)
  local a = w.chAssign[iChan]
  if a.side == RIGHT then a.side = LEFT
  elseif a.side == LEFT then a.side = BOTH
  else a.side = RIGHT end
  saveAssign(w)
end

local function flipRot(w, iChan)
  local a = w.chAssign[iChan]
  a.rot = -a.rot
  saveAssign(w)
end

local function page2Keys(w, event)
  if evtExit(event) then
    if w.activeField == FLD_CHAN_NUM then
      w.page = 1
    else
      w.editing = false
      w.activeField = FLD_CHAN_NUM
    end
  elseif evtEnter(event) then
    if w.activeField == FLD_ROT then flipRot(w, w.activeChan)
    elseif w.activeField == FLD_LR then cycleSide(w, w.activeChan)
    elseif w.activeField == FLD_CHAN_NUM then w.activeField = FLD_SURFACE_TYPE
    elseif w.activeField == FLD_SURFACE_TYPE then w.editing = not w.editing end
  elseif evtPage(event) then
    w.editing = false
    w.activeField = FLD_CHAN_NUM
    w.page = 1
  else
    local inc = evtInc(event)
    if inc ~= 0 then
      if w.editing then
        if w.activeField == FLD_SURFACE_TYPE then cycleType(w, w.activeChan, inc) end
      elseif w.activeField == FLD_CHAN_NUM then
        w.activeChan = (w.activeChan - 1 + inc) % N_CHANS + 1
      else
        repeat
          w.activeField = (w.activeField - 1 + inc) % 3 + 1
        until isSubfieldVisible(w, w.activeChan, w.activeField)
      end
    end
  end
end

local function drawCell(w, x, y, cw, ch, s, iChan, field, onTap)
  local isActive = (iChan == w.activeChan) and (field == w.activeField)
  local blink = (floor(getTime() / 40) % 2) == 0
  local editingThis = isActive and w.editing and field == FLD_SURFACE_TYPE
  if isActive then
    fillRect(x, y + 1, cw - 2, ch - 2, (editingThis and blink) and C.edit or C.accent)
  elseif onTap then
    fillRect(x, y + 1, cw - 2, ch - 2, C.panelHi)
  end
  local _, th = textSize(s, 0)
  txt(x + 4, y + (ch - th) / 2, s, 0, (editingThis and blink) and C.editTxt or C.text)
  if onTap then
    addHit(w, x, y, cw, ch, function()
      w.activeChan = iChan
      w.activeField = field
      w.editing = false
      onTap()
    end)
  end
end

local function drawPage2(w, X, Y, W, H)
  local _, fh = textSize('0', 0)
  local rowH = fh + 8
  local scrollW = 52
  local tableW = W - scrollW - 4
  local colX = {X + 2, X + 60, X + 60 + floor(tableW * 0.24), X + 60 + floor(tableW * 0.46), X + 60 + floor(tableW * 0.66)}
  local colW = {58, colX[3] - colX[2], colX[4] - colX[3], colX[5] - colX[4], X + tableW - colX[5]}

  -- header
  local hdr = {'CH', 'Output', 'Surface', 'Side', 'Reverse'}
  for c = 1, 5 do txt(colX[c] + 4, Y + 2, hdr[c], SMLSIZE, C.dim) end
  local _, sh = textSize('0', SMLSIZE)
  local listY = Y + sh + 6
  local visible = math.max(1, floor((Y + H - listY) / rowH))

  -- keep active channel visible
  if w.activeChan <= w.scroll then w.scroll = w.activeChan - 1 end
  if w.activeChan > w.scroll + visible then w.scroll = w.activeChan - visible end
  w.scroll = math.max(0, math.min(w.scroll, N_CHANS - visible))
  w.p2visible = visible

  for row = 1, visible do
    local i = w.scroll + row
    if i > N_CHANS then break end
    local y = listY + (row - 1) * rowH
    if row % 2 == 0 then fillRect(X, y, tableW, rowH, C.panel) end
    local a = w.chAssign[i]
    local st = w.surf[a.type]
    local out = model.getOutput(i - 1)

    drawCell(w, colX[1], y, colW[1], rowH, 'CH' .. i, i, FLD_CHAN_NUM, nil)
    txt(colX[2] + 4, y + (rowH - fh) / 2, (out and out.name) or '', 0, C.dim)
    drawCell(w, colX[3], y, colW[3], rowH, st.Name, i, FLD_SURFACE_TYPE,
             function() cycleType(w, i, 1) end)
    if st.hasLR then
      drawCell(w, colX[4], y, colW[4], rowH, SIDE_NAME[a.side] or 'Right', i, FLD_LR,
               function() cycleSide(w, i) end)
    end
    drawCell(w, colX[5], y, colW[5], rowH, (a.rot == ROT_NORMAL) and 'No' or 'Yes', i, FLD_ROT,
             function() flipRot(w, i) end)
    -- tapping the CH / output columns just selects the row
    addHit(w, colX[1], y, colX[3] - colX[1], rowH, function()
      w.activeChan = i; w.activeField = FLD_CHAN_NUM; w.editing = false
    end)
  end

  -- scroll buttons + re-run auto detection
  local sx = X + W - scrollW
  local autoH = 40
  local bh = floor((H - autoH - 12) / 2)
  button(w, sx, Y, scrollW, bh, '^', function()
    w.scroll = math.max(0, w.scroll - visible)
    if w.activeChan > w.scroll + visible then w.activeChan = w.scroll + visible end
  end, false)
  button(w, sx, Y + bh + 6, scrollW, autoH, 'Auto', function()
    clearSaved()
    autoAssign(w)
  end, false)
  button(w, sx, Y + bh + autoH + 12, scrollW, bh, 'v', function()
    w.scroll = math.min(N_CHANS - visible, w.scroll + visible)
    if w.activeChan <= w.scroll then w.activeChan = w.scroll + 1 end
  end, false)
end

-- ===========================================================================
-- Touch dispatch
-- ===========================================================================
local function handleTouch(w, event, ts)
  if not ts then return end
  if isEvt(event, EVT_TOUCH_TAP) then
    local tx, ty = ts.x, ts.y
    -- first match wins: editable cells are registered before the row-select fallback
    for _, h in ipairs(w.hits) do
      if tx >= h.x and tx < h.x + h.w and ty >= h.y and ty < h.y + h.h then
        h.fn()
        return
      end
    end
  elseif isEvt(event, EVT_TOUCH_SLIDE) and w.page == 2 then
    -- swipe up/down scrolls the assignment list
    local dy = ts.slideY or 0
    w.slideAcc = (w.slideAcc or 0) + dy
    local step = 24
    while w.slideAcc <= -step do
      w.slideAcc = w.slideAcc + step
      w.scroll = math.min(N_CHANS - (w.p2visible or 1), w.scroll + 1)
    end
    while w.slideAcc >= step do
      w.slideAcc = w.slideAcc - step
      w.scroll = math.max(0, w.scroll - 1)
    end
    if w.activeChan <= w.scroll then w.activeChan = w.scroll + 1 end
    local last = w.scroll + (w.p2visible or 1)
    if w.activeChan > last then w.activeChan = last end
  elseif isEvt(event, EVT_TOUCH_FIRST) then
    w.slideAcc = 0
  end
end

-- ===========================================================================
-- Widget interface
-- ===========================================================================
local function create(zone, options)
  initColours()
  local w = {
    zone = zone, options = options,
    page = 1, editing = false,
    activeChan = 1, activeField = FLD_CHAN_NUM,
    activeGV = 1, scroll = 0,
    view = math.max(1, math.min(4, (options and options.View) or 1)),
    showLabels = not (options and options.Labels == 0),
    surf = newSurfTypes(),
    chAssign = {}, lsDef = {}, hits = {},
  }
  cacheSourceIds(w)
  refreshLSDefs(w)
  if not loadAssign(w) then autoAssign(w) end
  return w
end

local function update(w, options)
  w.options = options
  w.view = math.max(1, math.min(4, (options and options.View) or 1))
  w.showLabels = not (options and options.Labels == 0)
end

local function background(w)
end

local function refresh(w, event, touchState)
  initColours()
  -- logical switch definitions can change while editing the model
  if getTime() - (w.lsLastRefresh or 0) > 200 then refreshLSDefs(w) end

  local full = (event ~= nil)
  local X, Y, W, H
  if full then
    X, Y, W, H = 0, 0, LCD_W, LCD_H
  else
    X, Y, W, H = w.zone.x or 0, w.zone.y or 0, w.zone.w, w.zone.h
  end

  if full then
    -- input first, using the touch targets from the previous frame
    if touchState then
      handleTouch(w, event, touchState)
    elseif w.page == 1 then
      page1Keys(w, event)
    else
      page2Keys(w, event)
    end
  end

  w.hits = {}
  fillRect(X, Y, W, H, C.bg)

  if not full then
    -- Embedded in a main-view zone: model view (plus GVs if there's room)
    local _, fh = textSize('0', 0)
    local _, sh = textSize('0', SMLSIZE)
    local gvH = fh + sh + 8
    if H >= 200 and W >= 360 then
      -- large zone: same layout as the full-screen Model page, without touch controls
      local topH = 28
      drawTopBar(w, X, Y, W, topH, false)
      drawPage1(w, X, Y + topH + 2, W, H - topH - 2, false)
    elseif H >= 150 and W >= 300 then
      drawModel(w, {x = X, y = Y, w = W, h = H - gvH - 2}, w.view, w.showLabels)
      drawGVars(w, X, Y + H - gvH, W, gvH, false)
    else
      drawModel(w, {x = X, y = Y, w = W, h = H}, w.view, w.showLabels and H >= 120)
    end
    return
  end

  local topH = 34
  drawTopBar(w, X, Y, W, topH, true)
  if w.page == 1 then
    drawPage1(w, X, Y + topH + 2, W, H - topH - 2, true)
  else
    drawPage2(w, X, Y + topH + 4, W, H - topH - 6)
  end
end

return {
  name = 'OTXSim',
  options = {
    {'View', VALUE, 1, 1, 4},   -- 1=3/4 2=Rear 3=Side 4=Top (view when not full screen)
    {'Labels', BOOL, 1},        -- show surface name/deflection tags
  },
  create = create,
  update = update,
  background = background,
  refresh = refresh,
}
