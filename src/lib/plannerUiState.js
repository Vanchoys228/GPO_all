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
    // Ignore quota/private mode limitations.
  }
}

export function getSectionOpen(sectionId, fallback = true) {
  const state = loadPlannerUiState();
  if (state.sections && Object.prototype.hasOwnProperty.call(state.sections, sectionId)) {
    return state.sections[sectionId];
  }
  return fallback;
}

export function setSectionOpenInStorage(sectionId, open) {
  const state = loadPlannerUiState();
  const sections = { ...state.sections, [sectionId]: open };
  savePlannerUiState({ ...state, sections });
}
