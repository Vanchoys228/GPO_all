const { createHash, randomUUID } = require("crypto");
const { validateMissionId, terminalStates } = require("../protocol/mission-contract.cjs");
const { validatePoints } = require("../protocol/route-validation.cjs");
const { prepareRoute } = require("./route-planning.cjs");
const canonical = value => Array.isArray(value) ? value.map(canonical) : value && typeof value === "object"
  ? Object.fromEntries(Object.keys(value).sort().map(key => [key,canonical(value[key])])) : value;

// One command owner per robot. All effects go through repository and adapter ports.
const createMissionService = ({repository, adapter, now = () => new Date().toISOString()}) => {
  let pending = Promise.resolve();
  const serial = action => {
    const result = pending.then(action);
    pending = result.catch(() => {});
    return result;
  };
  const submit = (payload, {requestId = randomUUID()} = {}) => serial(async () => {
    const missionId = validateMissionId(requestId || randomUUID());
    const fingerprint = createHash("sha256").update(JSON.stringify(canonical(payload))).digest("hex");
    let record = await repository.get(missionId);
    if (record && record.fingerprint !== fingerprint) throw Object.assign(new Error("Mission id already belongs to another command."),{statusCode:409});
    if (record && record.status !== "prepared") return record;
    if (!record) {
      let command = {...payload,route:validatePoints(payload.route),commandId:missionId};
      if (payload.scene) {
        const result = await prepareRoute({seedRoute:payload.seedRoute || payload.route,scene:payload.scene});
        command = {...command,route:result.route,scene:result.scene,motion:result.scene.motion,sceneRevision:result.sceneRevision,planning:result.planning};
      }
      if (command.route.length < 2) throw Object.assign(new Error("At least two route points are required."),{statusCode:400});
      record = {missionId,fingerprint,status:"prepared",createdAt:now(),updatedAt:now(),command};
      await repository.save(record);
    }
    // Retrying an uncertain delivery reuses commandId; it never creates a new launch.
    await adapter.submit(record.command);
    record = {...record,status:"persisted",updatedAt:now()};
    await repository.save(record);
    return record;
  });
  const get = missionId => serial(async () => {
    const record = await repository.get(validateMissionId(missionId));
    if (!record || record.status === "prepared" || terminalStates.has(record.status)) return record;
    const feedback = await adapter.getFeedback(missionId);
    const rank = {persisted:0,accepted:1,running:2,completed:3,failed:3,cancelled:3};
    if (!feedback || feedback.missionId !== missionId || !(feedback.status in rank) || rank[feedback.status] < rank[record.status]) return record;
    if (record.status === feedback.status) return record;
    const updated = {...record,status:feedback.status,updatedAt:now()};
    await repository.save(updated);
    return updated;
  });
  return {submit,get,drain:() => pending};
};
module.exports = {createMissionService};
