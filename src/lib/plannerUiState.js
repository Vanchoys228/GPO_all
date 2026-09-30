const STORAGE_KEY = "gpo.planner.ui.v1";

const defaultState = () => ({
  leftCollapsed: false,
  rightCollapsed: false,
});

export function loadPlannerUiState() {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return defaultState();
    const parsed = JSON.parse(raw);
    return {
      ...defaultState(),
      ...parsed,
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
