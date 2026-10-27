local H = dofile('harness.lua')
local S, P, run = H.S, H.P, H.run
print("== 1) Autotune in HOLD with switch high")
run(4)
S.sw = 2
run(80, function() return P.SCR_USER2 ~= 0 end)
run(S.t + 1)
assert(P.SCR_USER2 > 0, "autotune did not save gains")
print(string.format("   saved P=%.3f I=%.3f", P.SCR_USER2, P.SCR_USER3))
S.sw = 0; run(S.t + 1)

local function item(z) return { command = function() return 16 end, z = function() return z end } end
for _, deg in ipairs({ 20, 90 }) do
  print(string.format("== 2) Yaw alignment to %d deg via waypoint", deg))
  S.yaw = 0; S.rate = 0; S.mode = 10
  S.items[S.nav] = item(deg)
  run(S.t + 0.5)
  S.nav = S.nav + 1
  local t0, peak = S.t, 0
  run(S.t + 20, function()
    peak = math.max(peak, math.deg(S.yaw)); return S.mode ~= 3 and S.t - t0 > 0.5
  end)
  print(string.format("   end mode=%d (4=HOLD ok, 10=AUTO timeout) t=%.1fs overshoot=%.1fdeg final=%.1fdeg",
    S.mode, S.t - t0, peak - deg, math.deg(S.yaw)))
end

print("== 3) CIRCLE mode keeps script alive")
S.mode = 9; run(S.t + 2); print("   ok")
print("== 4) Unknown mode (LOITER=5) keeps script alive")
S.mode = 5; run(S.t + 2); print("   ok")
