const fs = require("fs/promises");
const path = require("path");
const { randomUUID } = require("crypto");
const { validateMissionId } = require("../protocol/mission-contract.cjs");
const createMissionRepository = ({directory}) => {
  const filename = id => path.join(directory, `${validateMissionId(id)}.json`);
  return {
    async ready() { await fs.mkdir(directory,{recursive:true}); await fs.access(directory,require("fs").constants.W_OK); },
    async get(id) {
      try { return JSON.parse(await fs.readFile(filename(id), "utf8")); }
      catch (error) { if (error.code === "ENOENT") return null; throw error; }
    },
    async list() {
      await fs.mkdir(directory,{recursive:true});
      const names = (await fs.readdir(directory)).filter(name => /^[a-zA-Z0-9_-]{1,63}\.json$/.test(name));
      return Promise.all(names.map(name => this.get(name.slice(0,-5))));
    },
    async save(record) {
      const target = filename(record.missionId);
      await fs.mkdir(directory,{recursive:true});
      const temporary = `${target}.${randomUUID()}.tmp`;
      try { await fs.writeFile(temporary,JSON.stringify(record,null,2)); await fs.rename(temporary,target); }
      finally { await fs.unlink(temporary).catch(() => {}); }
    },
  };
};
module.exports = {createMissionRepository, validateMissionId};
