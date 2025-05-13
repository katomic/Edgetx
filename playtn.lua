
local function init()
    -- Called once when the script is loaded
end
  
local function run()
    -- Called periodically while the Special Function switch is on
  local thrpos = getOutputValue(9)
    --rawtone = thrpos + 200
  local tone = (thrpos + 1800) * 2
  local dur = 180 - thrpos
  local pause = math.abs(thrpos) / 900
  playTone(tone, dur, pause, PLAY_NOW, -5, 5) -- freq, dur, pause, flags, freqIncr, vol
  --playTone(6150, 560, 120, 3, 10, 5)  

end
  
local function background()
    -- Called periodically while the Special Function switch is off
   --playTone(6150, 560, 120, 3, 10, 5)  
end
  
return { run=run, background=background, init=init }