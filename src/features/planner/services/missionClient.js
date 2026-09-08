import { MISSION_API_BASE_URL } from "../../../lib/runtimeConfig";
export const getMission = async (missionId,signal) => {
  const response = await fetch(`${MISSION_API_BASE_URL}/api/missions/${encodeURIComponent(missionId)}`,{signal});
  const result = await response.json();
  if (!response.ok || !result.ok) throw new Error(result.error || "Состояние миссии недоступно.");
  return result;
};
