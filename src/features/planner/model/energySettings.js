const parseLooseNumber = (rawValue) => {
  const normalized = String(rawValue ?? "").trim().replace(",", ".");
  if (!normalized) return null;
  const parsed = Number(normalized);
  return Number.isFinite(parsed) ? parsed : null;
};

export const formatEnergyNumber = (value, digits) =>
  Number(value.toFixed(digits)).toString();

export const normalizeBatteryRange = (rawValue) => {
  const parsed = parseLooseNumber(rawValue);
  return parsed == null ? null : Math.max(1, Math.min(10000, Math.round(parsed)));
};

export const normalizeCruiseSpeed = (rawValue) => {
  const parsed = parseLooseNumber(rawValue);
  return parsed == null
    ? null
    : Math.max(0.05, Math.min(0.8, Number(parsed.toFixed(3))));
};

export const normalizePayload = (rawValue) => {
  const parsed = parseLooseNumber(rawValue);
  return parsed == null
    ? null
    : Math.max(0, Math.min(500, Number(parsed.toFixed(2))));
};
