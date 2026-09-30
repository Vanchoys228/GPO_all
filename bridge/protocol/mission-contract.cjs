const validateMissionId = id => {
  if (typeof id !== "string" || !/^[a-zA-Z0-9_-]{1,63}$/.test(id)) throw Object.assign(new Error("Invalid mission id."), {statusCode:400});
  return id;
};
const terminalStates = new Set(["completed", "cancelled", "failed"]);
module.exports = {validateMissionId, terminalStates};
