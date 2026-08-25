export const MAPPING_SURVEY_MODES = [
  { key: "snake", label: "Змейка" },
  { key: "double", label: "Двойной объезд" },
];

export const getMappingSurveyModeLabel = (modeKey) =>
  MAPPING_SURVEY_MODES.find((mode) => mode.key === modeKey)?.label ||
  MAPPING_SURVEY_MODES[0].label;

export const buildMappingSurveyPayload = ({
  batteryRangeMeters,
  commandId,
  field,
  mode,
  payloadKg,
}) => ({
  type: "start_mapping_survey",
  commandId,
  clearMap: true,
  mode,
  field,
  motion: {
    cruiseSpeedMps: 0.8,
    payloadKg,
    batteryRange: batteryRangeMeters,
  },
});
