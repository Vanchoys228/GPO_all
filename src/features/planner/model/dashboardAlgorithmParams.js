import { ALGORITHM_OPTIONS, getDefaultAlgorithmParams } from "../../../lib/routeAlgorithms";

export const createDashboardAlgorithmParams = () =>
  Object.fromEntries(
    ALGORITHM_OPTIONS.map((option) => [option.key, getDefaultAlgorithmParams(option.key)])
  );
