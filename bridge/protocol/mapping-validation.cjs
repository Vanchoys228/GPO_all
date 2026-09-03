const sanitizeMappingSurveyMode = (rawMode) => {
  const mode = String(rawMode || "snake").trim().toLowerCase();
  if (["snake", "double"].includes(mode)) return mode;
  return "snake";
};

module.exports = {
  sanitizeMappingSurveyMode,
};
