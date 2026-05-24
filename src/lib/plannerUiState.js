const STORAGE_KEY = "gpo.planner.ui.v1";

const defaultState = () => ({
  leftCollapsed: false,
  rightCollapsed: false,
  sections: {
    "left.legend": true,
    "left.points": true,
    "left.routeAlgo": true,
    "left.energy": true,
    "left.surfaces": false,
    "left.algoParams": true,
    "right.telemetry": false,
    "right.constraints": true,
  },
});

export function loadPlannerUiState() {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return defaultState();
    const parsed = JSON.parse(raw);
    return {
      ...defaultState(),
      ...parsed,
      sections: { ...defaultState().sections, ...(parsed.sections || {}) },
    };
  } catch {
    return defaultState();
  }
}

export function savePlannerUiState(next) {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(next));
  } catch {
    // ignore quota / private mode
  }
}

export function getSectionOpen(sectionId, fallback = true) {
  const s = loadPlannerUiState();
  if (s.sections && Object.prototype.hasOwnProperty.call(s.sections, sectionId)) {
    return s.sections[sectionId];
  }
  return fallback;
}

export function setSectionOpenInStorage(sectionId, open) {
  const s = loadPlannerUiState();
  const sections = { ...s.sections, [sectionId]: open };
  savePlannerUiState({ ...s, sections });
}
