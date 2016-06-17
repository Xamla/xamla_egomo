ExponentialSmoothing = {
  numberOfTimeSeries,
  method,
  results,
  parameter,
  trend,
}

function ExponentialSmoothing:new(method, parameterTable, numberOfTimeSeries, initialValues, trend)
  o = {}
  setmetatable(o, self)
  self.__index = self

  self.method = method

  if self.method == "exp1" or self.method == "exp2" or self.method == "brown" then
    self.parameter = parameterTable or {["alpha"] = 0.5}
  elseif self.method == "exp-holt-winters" then
    self.parameter = parameterTable or {["alpha"] = 0.5, ["beta"] = 0.5}
  end

  self.numberOfTimeSeries = numberOfTimeSeries

  if initialValues == nil then
    self.results = {}
    for i=1,numberOfTimeSeries do
      self.results[#self.results+1] = 0.0
    end
  else
    self.results = initialValues
  end

  if trend == nil then
    self.trend = {}
    for i=1,numberOfTimeSeries do
      self.trend[#self.trend+1] = 0.0
    end
  else
    self.trend = trend
  end

  return o
end

-- TODO: Check equations
function ExponentialSmoothing:compute(label, value)
  local alpha = self.parameter["alpha"]
  if self.method == "exp1" then
    return (alpha * value) + ((1 - alpha) * self.results[label])
  elseif self.method == "exp2" then
    local s_t = (alpha * value) + ((1 - alpha) * self.results[label])
    local b_t = (alpha * s_t) + ((1 - alpha) * self.trend[label])
    local old_b_t = self.trend[label]
    self.trend[label] = b_t
    return (2 * s_t) - old_b_t
  elseif self.method == "brown" then
    local s_t = (alpha * value) + ((1 - alpha) * self.results[label])
    local b_t = alpha * (s_t - self.results[label]) + (1 - alpha) * self.trend[label]
    self.trend[label] = b_t
    return s_t + b_t + ((1 - alpha) / alpha) * b_t
  elseif self.method == "exp-holt-winters" then
    local beta = self.parameter["beta"]
    local s_t = (alpha * value) + ((1 - alpha) * (self.results[label] + self.trend[label]))
    local b_t = beta * (s_t - self.results[label]) + (1 - beta) * self.trend[label]
    self.trend[label] = b_t
    return s_t + b_t
  end
end

function ExponentialSmoothing:updateValues(newValues)
  for label,value in pairs(newValues) do
    local newValue = self:compute(label,value)
    self.results[label] = newValue
  end
end

return ExponentialSmoothing
