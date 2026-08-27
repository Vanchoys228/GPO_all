const defaultConfig = require("../config/runtime-config.cjs");
const defaultCoordinateContract = require("../../shared/coordinate-contract.json");
const { createNativeSolver: defaultCreateNativeSolver } = require("../solver/native-solver.cjs");
const { createSolverHttpServer: defaultCreateSolverHttpServer } = require("../servers/solver-http-server.cjs");

const startPlanningProcess = ({
  config = defaultConfig,
  coordinateContract = defaultCoordinateContract,
  createNativeSolver = defaultCreateNativeSolver,
  createSolverHttpServer = defaultCreateSolverHttpServer,
} = {}) => {
  const nativeSolver = createNativeSolver({ solverPath: config.SOLVER_PATH });
  const planningServer = createSolverHttpServer({
    coordinateContract,
    host: config.BRIDGE_HOST,
    nativeSolver,
    port: config.SOLVER_PORT,
    solverPath: config.SOLVER_PATH,
  });

  return planningServer.start();
};

module.exports = { startPlanningProcess };
