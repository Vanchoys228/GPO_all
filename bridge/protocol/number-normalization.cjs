const clamp = (value, min, max) => Math.max(min, Math.min(value, max));

const clampInt = (value, min, max) =>
  Math.round(clamp(Number.isFinite(value) ? value : min, min, max));

const normalizeNumber = (value, fallback) => {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
};

module.exports = {
  clamp,
  clampInt,
  normalizeNumber,
};
