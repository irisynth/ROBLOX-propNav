
local PropNav = {}

local G = 9.80665
PropNav.G = G

PropNav.Law = {
	TruePN = 1,
	PurePN = 2,
	ZEM    = 3,
	ATPN   = 4,
	APPN   = 5,
	AZEM   = 6,
}
local Law = PropNav.Law

local EPS = 1e-9

local function unit(v)
	local m = v.Magnitude
	if m < EPS then return Vector3.zero end
	return v / m
end
PropNav.unit = unit


function PropNav.losRate(Rlos, Vtm)
	local d = Rlos:Dot(Rlos)
	if d < EPS then return Vector3.zero end
	return Rlos:Cross(Vtm) / d
end

function PropNav.closingSpeed(Vt, Vm, Ulos)
	return -(Vt - Vm):Dot(Ulos)
end

function PropNav.collisionClosingSpeed(Vt, Vm, Ulos)
	local vt, vm = Vt.Magnitude, Vm.Magnitude
	if vm < EPS then return 0 end
	if vt < EPS then return vm end
	local betat = math.acos(math.clamp(Vt:Dot(Ulos) / vt, -1, 1))
	local betam = math.asin(math.clamp((vt / vm) * math.sin(betat), -1, 1))
	return vm * math.cos(betam) - vt * math.cos(betat)
end

function PropNav.timeToGo(Rlos, Vt, Vm)
	local vcc = PropNav.collisionClosingSpeed(Vt, Vm, unit(Rlos))
	if vcc <= EPS then return math.huge end
	return Rlos.Magnitude / vcc
end

local function applyGCP(Ac, Ulos, Uvm)
	local d = Ulos:Dot(Uvm)
	if math.abs(d) > 1e-6 then
		return Ac - (Ac:Dot(Uvm) / d) * Ulos
	end
	return Ac - Ac:Dot(Uvm) * Uvm
end
PropNav.applyGCP = applyGCP


function PropNav.guidanceAccel(law, Rlos, Vt, At, Vm, N)
	local Ulos = unit(Rlos)
	local Uvm  = unit(Vm)
	local Vtm  = Vt - Vm
	local Ws   = PropNav.losRate(Rlos, Vtm)

	if law == Law.PurePN then
		return N * Ws:Cross(Vm)

	elseif law == Law.TruePN then
		local Vc = PropNav.closingSpeed(Vt, Vm, Ulos)
		return applyGCP(N * Vc * Ws:Cross(Ulos), Ulos, Uvm)

	elseif law == Law.APPN then
		local Atn  = At - At:Dot(Ulos) * Ulos
		local Atsi = Atn - Atn:Dot(Uvm) * Uvm
		return N * Ws:Cross(Vm) + (N / 2) * Atsi

	elseif law == Law.ATPN then
		local Atn = At - At:Dot(Ulos) * Ulos
		local Vc  = PropNav.closingSpeed(Vt, Vm, Ulos)
		return applyGCP(N * Vc * Ws:Cross(Ulos) + (N / 2) * Atn, Ulos, Uvm)

	elseif law == Law.ZEM or law == Law.AZEM then
		local tgo = PropNav.timeToGo(Rlos, Vt, Vm)
		if tgo == math.huge or tgo < 1e-4 then return Vector3.zero end
		local ZEM = Rlos + Vtm * tgo
		if law == Law.AZEM then
			ZEM = ZEM + (At / 2) * tgo * tgo
		end
		local ZEMn = ZEM - ZEM:Dot(Ulos) * Ulos
		return applyGCP(N * ZEMn / (tgo * tgo), Ulos, Uvm)
	end

	return Vector3.zero
end

function PropNav.limitAccel(Acmd, Amax)
	if Acmd.Magnitude > Amax then
		return Amax * unit(Acmd)
	end
	return Acmd
end


PropNav.Maneuver = {
	Straight = 0,
	Turn     = 1,
	Weave    = 2,
	Beam     = 3,
	Break    = 4,
}
local Maneuver = PropNav.Maneuver

function PropNav.turnAccel(UWt, Vt, nG)
	if nG == 0 then return Vector3.zero end
	local v = Vt.Magnitude
	if v < EPS then return Vector3.zero end
	return (nG * G / v) * UWt:Cross(Vt)
end

local EASE_ANGLE = math.rad(10)
local function steerToward(Vt, Vdes, nG)
	local axis = Vt:Cross(Vdes)
	if axis.Magnitude < 1e-6 then return Vector3.zero end
	local err = math.atan2(axis.Magnitude, Vt:Dot(Vdes))
	local demand = nG * math.min(1, err / EASE_ANGLE)
	return PropNav.turnAccel(unit(axis), Vt, demand)
end

local function beamAccel(Pm, Pt, Vt, nG, keepLevel)
	local L = unit(Pm - Pt)
	local across = Vt - Vt:Dot(L) * L
	if keepLevel then
		across = Vector3.new(across.X, 0, across.Z)
	end
	if across.Magnitude < EPS then return Vector3.zero end
	return steerToward(Vt, unit(across), nG)
end

local function breakAccel(Pm, Pt, Vt, nG)
	local L = unit(Pm - Pt)
	local dir = L:Cross(Vt)
	if dir.Magnitude < EPS then
		return PropNav.turnAccel(Vector3.yAxis, Vt, nG)
	end
	return (nG * G) * unit(dir)
end

function PropNav.targetAccel(cfg, s)
	local man = cfg.TargetManeuver or Maneuver.Straight

	if (cfg.BreakTgo or 0) > 0 and man ~= Maneuver.Break then
		local tgo = PropNav.timeToGo(s.Pt - s.Pm, s.Vt, s.Vm)
		if tgo <= cfg.BreakTgo then
			man = Maneuver.Break
		end
	end

	if man == Maneuver.Turn then
		return PropNav.turnAccel(cfg.TurnAxis, s.Vt, cfg.TargetG)
	elseif man == Maneuver.Weave then
		return PropNav.turnAccel(cfg.TurnAxis, s.Vt,
			cfg.TargetG * math.sin(cfg.WeaveRate * s.t))
	elseif man == Maneuver.Beam then
		return beamAccel(s.Pm, s.Pt, s.Vt, cfg.TargetG, cfg.KeepLevel)
	elseif man == Maneuver.Break then
		return breakAccel(s.Pm, s.Pt, s.Vt, cfg.BreakG or cfg.TargetG)
	end
	return Vector3.zero
end


function PropNav.availableAccel(v, cfg)
	local amax = cfg.MaxG * G
	if (cfg.CornerSpeed or 0) <= 0 then return amax end
	local r = math.min(1, v / cfg.CornerSpeed)
	return amax * r * r
end

function PropNav.axialAccel(t, Vm, cfg, Alat)
	local v = Vm.Magnitude
	local a = 0
	if t < cfg.BoostTime and v < cfg.MaxSpeed then
		a = cfg.BoostAccel
	end
	a = a - cfg.Drag * v * v
	if (cfg.InducedDrag or 0) > 0 and v > EPS and Alat then
		local alat = Alat.Magnitude
		a = a - cfg.InducedDrag * (alat * alat) / (v * v)
	end
	return a * unit(Vm)
end


local Engagement = {}
Engagement.__index = Engagement

function PropNav.newEngagement(cfg)
	local self = setmetatable({}, Engagement)
	self.cfg = cfg
	self.s = {
		t  = 0,
		Pt = cfg.Pt0, Vt = cfg.Vt0,
		Pm = cfg.Pm0, Vm = cfg.Vm0,
	}
	self.Amax = cfg.MaxG * G
	self.lastAmax = self.Amax
	self.finished = false
	self.acc = 0
	self.maxG = 0
	self.lastAm = Vector3.zero
	self.lastAt = Vector3.zero
	return self
end

function Engagement:derivative(s)
	local cfg = self.cfg
	local At = PropNav.targetAccel(cfg, s)
	local amax = PropNav.availableAccel(s.Vm.Magnitude, cfg)
	self.lastAmax = amax
	local Am
	if s.t < cfg.GuidanceDelay then
		Am = Vector3.zero
	else
		local Rlos = s.Pt - s.Pm
		Am = PropNav.limitAccel(
			PropNav.guidanceAccel(cfg.Law, Rlos, s.Vt, At, s.Vm, cfg.N),
			amax)
	end
	self.lastAt, self.lastAm = At, Am
	Am = Am + PropNav.axialAccel(s.t, s.Vm, cfg, Am)
	return { Pt = s.Vt, Vt = At, Pm = s.Vm, Vm = Am }
end

local function advanceState(s, d, h)
	return {
		t  = s.t  + h,
		Pt = s.Pt + d.Pt * h, Vt = s.Vt + d.Vt * h,
		Pm = s.Pm + d.Pm * h, Vm = s.Vm + d.Vm * h,
	}
end

function Engagement:rk4(h)
	local s = self.s
	local k1 = self:derivative(s)
	local k2 = self:derivative(advanceState(s, k1, h / 2))
	local k3 = self:derivative(advanceState(s, k2, h / 2))
	local k4 = self:derivative(advanceState(s, k3, h))
	local w = h / 6
	self.s = {
		t  = s.t + h,
		Pt = s.Pt + (k1.Pt + 2 * k2.Pt + 2 * k3.Pt + k4.Pt) * w,
		Vt = s.Vt + (k1.Vt + 2 * k2.Vt + 2 * k3.Vt + k4.Vt) * w,
		Pm = s.Pm + (k1.Pm + 2 * k2.Pm + 2 * k3.Pm + k4.Pm) * w,
		Vm = s.Vm + (k1.Vm + 2 * k2.Vm + 2 * k3.Vm + k4.Vm) * w,
	}
	if self:range() > 100 then
		local g = self.lastAm.Magnitude / G
		if g > self.maxG then self.maxG = g end
	end
end

function Engagement:range()
	return (self.s.Pt - self.s.Pm).Magnitude
end

function Engagement:stepSize()
	local h = self.cfg.TimeStep
	local d = self:range()
	if d < 100 then
		if d < 10 then
			if d < self.cfg.MinMissDist then return h / 50 end
			return h / 20
		end
		return h / 10
	end
	return h / 2
end

function Engagement:shouldStop()
	local s = self.s
	if s.t >= self.cfg.StopTime then return true, "timeout" end
	if s.t > self.cfg.MinFlightTime then
		local Ulos = unit(s.Pt - s.Pm)
		if (s.Vm - s.Vt):Dot(Ulos) < 0 then return true, "closest approach" end
	end
	return false, nil
end

function Engagement:Advance(dt)
	if self.finished then return end
	self.acc = self.acc + dt
	local guard = 0
	while self.acc > 0 and not self.finished do
		guard = guard + 1
		if guard > 4000 then break end
		local h = math.min(self:stepSize(), self.acc)
		self:rk4(h)
		self.acc = self.acc - h
		local stop, why = self:shouldStop()
		if stop then
			self.finished = true
			self.result = {
				timeOfFlight = self.s.t,
				missDistance = self:range(),
				reason       = why,
				hit          = self:range() <= self.cfg.MinMissDist,
				maxG         = self.maxG,
				finalSpeed   = self.s.Vm.Magnitude,
			}
		end
	end
end

function Engagement:Telemetry()
	local s = self.s
	local Rlos = s.Pt - s.Pm
	local Ulos = unit(Rlos)
	return {
		t        = s.t,
		range    = Rlos.Magnitude,
		closing  = PropNav.closingSpeed(s.Vt, s.Vm, Ulos),
		tgo      = PropNav.timeToGo(Rlos, s.Vt, s.Vm),
		losRate  = PropNav.losRate(Rlos, s.Vt - s.Vm).Magnitude,
		mslSpeed = s.Vm.Magnitude,
		mslG     = self.lastAm.Magnitude / G,
		tgtG     = self.lastAt.Magnitude / G,
		mslAvailG = self.lastAmax / G,
	}
end

return PropNav
