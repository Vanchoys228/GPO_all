import {useCallback,useEffect,useState} from "react";
import {getMission,listMissions,cancelMission} from "../services/missionClient";
const terminal = new Set(["completed","failed","cancelled"]);
export const useMissionTracking = ({onStarted,fetchMission = getMission}) => {
  const [missionId,setMissionId] = useState(null);
  const [state,setState] = useState(null);
  const cancel = useCallback(async () => {
    if (!missionId) return;
    try {setState(await cancelMission(missionId));}
    catch(error) {setState(previous => ({...previous,connectionError:error.message}));}
  },[missionId]);
  const track = useCallback(ack => {
    if (!ack?.missionId) return;
    setState({missionId:ack.missionId,status:ack.status || "persisted"});
    setMissionId(ack.missionId);
  },[]);
  useEffect(() => {
    if (missionId || fetchMission !== getMission) return;
    const controller = new AbortController();
    let timer;
    const restore = async () => {
      try {
        const records = await listMissions(controller.signal);
        if(controller.signal.aborted)return;
        const active=records.find(record=>!terminal.has(record.status));
        if(active){track(active);return;}
      } catch { /* Retry discovery after the mission service returns. */ }
      if(!controller.signal.aborted)timer=setTimeout(restore,1000);
    };
    restore();
    return ()=>{controller.abort();clearTimeout(timer);};
  },[missionId,fetchMission,track]);
  useEffect(() => {
    if (!missionId) return;
    const controller = new AbortController();
    let timer;
    let started = false;
    const poll = async () => {
      try {
        const result = await fetchMission(missionId,controller.signal);
        if (controller.signal.aborted) return;
        if (result.missionId !== missionId) throw new Error("Ответ относится к другой миссии.");
        setState(result);
        if (result.status === "running" && !started) {started=true;onStarted?.();}
        if (terminal.has(result.status)) {setMissionId(null);return;}
      } catch(error) {
        if (controller.signal.aborted) return;
        setState(previous => ({...previous,connectionError:error.message}));
      }
      timer = setTimeout(poll,1000);
    };
    poll();
    return () => {controller.abort();clearTimeout(timer);};
  },[missionId,fetchMission,onStarted]);
  return {state,track,cancel};
};
